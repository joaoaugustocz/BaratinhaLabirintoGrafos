#include <Arduino.h>
#include <FastLED.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // Para WebSockets
#include <FS.h>               // Necessário para o sistema de arquivos
#include <LittleFS.h>         // Usando LittleFS



// --- Configurações de Wi-Fi ---
const char* ssid = "Joao_2G";
const char* password = "Naotemsenha";

// --- Servidor Web HTTP ---
WebServer httpServer(80);

// --- WebSocket Server ---
WebSocketsServer webSocketServer = WebSocketsServer(81); // Porta 81 para WebSockets


// === ESTADOS DO ROBÔ PARA CONTROLE WEB ===
enum EstadoRobo {
  PARADO_WEB,
  INICIANDO_EXPLORACAO_WEB,
  SEGUINDO_LINHA_WEB,
  PAUSADO_WEB,
  EM_NO_WEB,
  CALIBRANDO_LINHA_WEB
};
EstadoRobo estadoRoboAtual = PARADO_WEB;
bool exploracaoWebIniciada = false;

//--------------------------- Motores (Suas Definições)
#define in1 GPIO_NUM_39 // Motor Esquerdo Dir1
#define in2 GPIO_NUM_45 // Motor Esquerdo Dir2
#define pwmM1 GPIO_NUM_46 // Motor Esquerdo PWM (LEDC Chan 0)

#define in3 GPIO_NUM_40 // Motor Direito Dir1
#define in4 GPIO_NUM_41 // Motor Direito Dir2
#define pwmM2 GPIO_NUM_42 // Motor Direito PWM (LEDC Chan 1)

// Canais LEDC
#define LEDC_CANAL_MOTOR_ESQUERDO 0
#define LEDC_CANAL_MOTOR_DIREITO  1
#define LEDC_FREQ 1000 
#define LEDC_RESOLUTION 8

//--------------------------- PID (Suas Constantes e Variáveis Globais)
int posicao = 0; 

#define KP_3 0.1
#define KI_3 0//0.0003
#define KD_3 0//1.75
#define M1_BASE_3 30 
#define M2_BASE_3 30  
#define Mm1_RETA_3 30 
#define Mm2_RETA_3 30 
#define MMAX_CURVA_3 80
#define MMAX2_REVERSO_3 -90


int error = 0;
int lastError = 0;
int second_lastError = 0;


// Constantes PID e velocidades ativas (inicializadas com o conjunto _3)
float current_KP = KP_3;
float current_KI = KI_3;
float current_KD = KD_3;
int current_M1_base = M1_BASE_3;  // Direito
int current_M2_base = M2_BASE_3;  // Esquerdo
int current_Mm1_reta = Mm1_RETA_3; // Direito
int current_Mm2_reta = Mm2_RETA_3; // Esquerdo
int current_MMAX_curva = MMAX_CURVA_3;
int current_MMAX2_reverso = MMAX2_REVERSO_3;

#define setPoint 3000 // Ponto central para 7 sensores ( (7-1)*1000 / 2 )
int I_pid = 0; 

//--------------------------- MPU6050 (Suas Variáveis)
const int MPU_ADDR = 0x68;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw, accX_raw, accY_raw, accZ_raw; // Renomeado para _raw
float angleX = 0, angleY = 0, angleZ = 0;
float gyroRateX, gyroRateY, gyroRateZ;
float accAngleX, accAngleY;
float elapsedTime_mpu; // Renomeado
unsigned long currentTime_mpu, previousTime_mpu; // Renomeado
float accAngleCorrectionFactor = 0.98;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
int calibracoes_mpu = 1000; 
const int EEPROM_OFFSET_ADDR = 0;
float gyroScaleFactor;
float gyroScaleFactors[] = {131.0, 65.5, 32.8, 16.4};

//--------------------------- Sensores Linha (Suas Definições)
QTRSensors qtr;
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];
#define tempoCalibracaoLinha 240 

#define s1 GPIO_NUM_1
#define s2 GPIO_NUM_2
#define s3 GPIO_NUM_3
#define s4 GPIO_NUM_4
#define s5 GPIO_NUM_5 // Pino 5
#define s6 GPIO_NUM_6 // Pino 6
#define s7 GPIO_NUM_7 // Pino 4 (Conforme seu código anterior, S7 era GPIO_NUM_4)
                      // Se S7 é GPIO_NUM_6 e S6 é GPIO_NUM_5, e S5 é GPIO_NUM_4, ajuste conforme sua placa.
                      // Vou manter a ordem que você listou por último: s5=GPIO_NUM_4, s6=GPIO_NUM_5, s7=GPIO_NUM_6
                      // No seu último código base: s5=GPIO_NUM_5, s6=GPIO_NUM_6, s7=GPIO_NUM_7.
                      // Vou usar: s1=7, s2=1, s3=2, s4=3, s5=4, s6=5, s7=6 (precisa confirmar!)


//--------------------------- LEDs (Suas Definições)
#define NUM_LEDS 4
#define DATA_PIN GPIO_NUM_48
CRGB leds[NUM_LEDS];

// --- Protótipos ---
void setColor(char sensor, int h, int s, int v);
void configurarEscalaGiroscopio();
void calibrarGiroscopio();
void salvarCalibracaoEEPROM(); // Renomeado para clareza
bool carregarCalibracaoEEPROM(); // Renomeado para clareza
float getAngleMPU(char eixo); // Renomeado para clareza
void lerSens();
void motorE_PWM(int vel);
void motorD_PWM(int vel);
void motor(char lado, char dir, int pwm);
void pid_controlado_web();
void executarCalibracaoLinhaWebService();
void pararMotoresWebService();
void pid_seguelinha_original(); // Sua função PID original, se quiser usá-la


void broadcastSerial(const String &message) { 
    Serial.print(message);
    String nonConstMessage = message; // Cria uma cópia não-constante
    webSocketServer.broadcastTXT(nonConstMessage); 
}

void broadcastSerialLn(const String &message) {
    Serial.println(message);
    String nonConstMessage = message + "\n"; // Cria uma cópia não-constante com newline
    webSocketServer.broadcastTXT(nonConstMessage);
}


void broadcastSerialF(const char *format, ...) { 
    char buf[256]; 
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    Serial.print(buf);
    String messageToSend = String(buf); // Crie uma variável String nomeada
    webSocketServer.broadcastTXT(messageToSend); // Passe a variável
}



// --- Funções de Controle Web e Lógica do Robô ---
void pararMotoresWebService() {
    motor('a', 'f', 0); 
    Serial.println("Motores Parados (Controle Web)");
}

void pid_controlado_web() {
    if (estadoRoboAtual != SEGUINDO_LINHA_WEB) return;

    lerSens(); 
    int m1Speed_web, m2Speed_web; // Direito, Esquerdo

    second_lastError = lastError;
    lastError = error;
    error = posicao - setPoint;

    

    bool condicao_ree_especial = (abs(lastError) < 1100 && abs(second_lastError) < 1100);

    if (abs(error) >= setPoint - 100 && !condicao_ree_especial) { 
        //Serial.print("NÓ WEB Detectado! Erro: "); Serial.println(error);
        broadcastSerialLn("NÓ WEB Detectado! Erro: " + String(error));
        pararMotoresWebService();
        estadoRoboAtual = EM_NO_WEB;
        I_pid = 0; 
        return; 
    }
    
    if (abs(error) == 3000 && condicao_ree_especial) {
        error = 0; 
        Serial.println("WEB: Condição REEEEEEEEE especial do PID original, erro zerado.");
        // A lógica de "REEEEEEEEE" de ir pra frente rápido está na sua pid_seguelinha_original.
        // Aqui, apenas zeramos o erro para o cálculo do PID desta iteração.
        // Se quiser replicar o impulso, adicione aqui, mas pode conflitar com "parar no nó".
    }

    I_pid = I_pid + error;
    I_pid = constrain(I_pid, -10000, 10000); 
    
    int motorSpeedCorrection = current_KP * error + current_KD * (error - lastError) + current_KI * I_pid;
    
    

    if (abs(error) <= 200) {
        m1Speed_web = current_Mm1_reta - motorSpeedCorrection; 
        m2Speed_web = current_Mm2_reta + motorSpeedCorrection; 
    } else {
        m1Speed_web = current_M1_base - motorSpeedCorrection; 
        m2Speed_web = current_M2_base + motorSpeedCorrection; 
    }
    
    m1Speed_web = constrain(m1Speed_web, current_MMAX2_reverso, current_MMAX_curva);
    m2Speed_web = constrain(m2Speed_web, current_MMAX2_reverso, current_MMAX_curva);
    
    if(m1Speed_web < 0) motor('d', 't', abs(m1Speed_web)); 
    else motor('d', 'f', abs(m1Speed_web));
    
    if(m2Speed_web < 0) motor('e', 't', abs(m2Speed_web));
    else motor('e', 'f', abs(m2Speed_web));
}



void handleIniciarWeb() {
    Serial.println("Comando Web 'Iniciar Exploração'!");
    if (estadoRoboAtual == PARADO_WEB || estadoRoboAtual == EM_NO_WEB || estadoRoboAtual == PAUSADO_WEB) {
        estadoRoboAtual = INICIANDO_EXPLORACAO_WEB;
        exploracaoWebIniciada = true;
        error = 0; lastError = 0; second_lastError = 0; I_pid = 0;
        httpServer.send(200, "text/plain", "Robô iniciando exploração!");
    } else { httpServer.send(200, "text/plain", "Robô ocupado ou em estado incompatível."); }
}

void handleRetornarWeb() {
    Serial.println("Comando Web 'Retornar ao Início'!");
    pararMotoresWebService(); estadoRoboAtual = PARADO_WEB; 
    httpServer.send(200, "text/plain", "Robô parando. Retorno não implementado.");
}

void handlePausarWeb() {
    if (estadoRoboAtual == SEGUINDO_LINHA_WEB || estadoRoboAtual == INICIANDO_EXPLORACAO_WEB) {
        pararMotoresWebService(); estadoRoboAtual = PAUSADO_WEB;
        Serial.println("Robô PAUSADO (Web).");
        httpServer.send(200, "text/plain", "Robô pausado.");
    } else { httpServer.send(200, "text/plain", "Não é possível pausar neste estado."); }
}

void handleContinuarWeb() {
    if (estadoRoboAtual == PAUSADO_WEB) {
        estadoRoboAtual = SEGUINDO_LINHA_WEB; 
        Serial.println("Robô CONTINUANDO (Web).");
        httpServer.send(200, "text/plain", "Robô continuando.");
    } else { httpServer.send(200, "text/plain", "Robô não estava pausado."); }
}

void handleRecalibrarLinhaWeb() {
    Serial.println("Comando Web 'Recalibrar Linha'!");
    if (estadoRoboAtual != CALIBRANDO_LINHA_WEB) {
        estadoRoboAtual = CALIBRANDO_LINHA_WEB; // Sinaliza para o loop ou executa direto
        // Para simplificar e dar resposta ao usuário, chamamos direto.
        // A função de calibração é bloqueante.
        executarCalibracaoLinhaWebService(); 
        // A função executarCalibracaoLinhaWebService já define estadoRoboAtual = PARADO_WEB no final.
        httpServer.send(200, "text/plain", "Calibração de linha concluída!");
    } else { httpServer.send(200, "text/plain", "Calibração já em progresso."); }
}

void handleNotFound() { httpServer.send(404, "text/plain", "Nao Encontrado"); }

// --- Função para Broadcast de Mensagens (Serial + WebSocket) ---
// void broadcastSerial(String message) { 
//     Serial.print(message); 
//     webSocketServer.broadcastTXT(message); 
// }

// void broadcastSerialLn(String message) { // Alterado para passar por valor
//     Serial.println(message);
//     String msgWithNewline = message + "\n";
//     webSocketServer.broadcastTXT(msgWithNewline); 
// }


// --- Eventos do WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            // Serial.printf("[%u] Desconectado!\n", num);
            broadcastSerialLn("[" + String(num) + "] Cliente WebSocket Desconectado!");
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocketServer.remoteIP(num);
            // Serial.printf("[%u] Conectado de %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            broadcastSerialLn("[" + String(num) + "] Cliente WebSocket Conectado: " + ip.toString());
            // Enviar uma mensagem de boas-vindas ou estado inicial se necessário
            webSocketServer.sendTXT(num, "Bem-vindo ao Log do Robô Bartinha!");
            }
            break;
        case WStype_TEXT:
            // Serial.printf("[%u] recebeu texto: %s\n", num, payload);
            // broadcastSerialLn("[" + String(num) + "] rx: " + String((char*)payload));
            // Ecoar de volta para o cliente (opcional)
            // webSocketServer.sendTXT(num, payload); 
            // Aqui você poderia processar comandos recebidos do cliente web via WebSocket, se necessário
            break;
        case WStype_BIN:
        case WStype_ERROR:			
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}
// Função auxiliar para determinar o Content Type (MIME type)
String getContentType(String filename) {
    if (httpServer.hasArg("download")) return "application/octet-stream";
    else if (filename.endsWith(".htm")) return "text/html";
    else if (filename.endsWith(".html")) return "text/html";
    else if (filename.endsWith(".css")) return "text/css";
    else if (filename.endsWith(".js")) return "application/javascript";
    else if (filename.endsWith(".png")) return "image/png";
    else if (filename.endsWith(".gif")) return "image/gif";
    else if (filename.endsWith(".jpg")) return "image/jpeg";
    else if (filename.endsWith(".ico")) return "image/x-icon";
    else if (filename.endsWith(".xml")) return "text/xml";
    else if (filename.endsWith(".pdf")) return "application/x-pdf";
    else if (filename.endsWith(".zip")) return "application/x-zip";
    else if (filename.endsWith(".gz")) return "application/x-gzip";
    return "text/plain";
}

// Função para servir arquivos do LittleFS

bool handleFileRead(String path) {
    // Usar broadcastSerialLn para que você veja isso no drawer da web também, quando funcionar
    broadcastSerialLn("[HTTP FS] handleFileRead: Req inicial: '" + path + "'");

    if (path.endsWith("/")) {
        path = "/index.html"; 
        broadcastSerialLn("[HTTP FS] Path é raiz, definido para: '" + path + "'");
    } else {
        // Garante que caminhos relativos (ex: "style.css") tenham a barra inicial
        if (!path.startsWith("/")) {
            path = "/" + path;
            broadcastSerialLn("[HTTP FS] Adicionada barra inicial, path agora: '" + path + "'");
        }
    }
    
    String contentType = getContentType(path);
    broadcastSerialLn("[HTTP FS] Tentando servir: '" + path + "' (Tipo: " + contentType + ")");

    bool fileExists = LittleFS.exists(path);
    broadcastSerialLn("[HTTP FS] LittleFS.exists(\"" + path + "\") -> " + (fileExists ? "ENCONTRADO" : "NAO ENCONTRADO"));

    if (fileExists) {
        File file = LittleFS.open(path, "r");
        if (file && !file.isDirectory()) {
            // Se o arquivo abriu corretamente:
            broadcastSerialLn("[HTTP FS] Arquivo '" + path + "' aberto. Tamanho: " + String(file.size()) + ". Enviando...");
            size_t sent = httpServer.streamFile(file, contentType);
            file.close();
            broadcastSerialLn("[HTTP FS] Enviados " + String(sent) + " bytes de '" + path + "'");
            return true; // Sucesso!
        } else {
            // Se LittleFS.open() falhou ou é um diretório
            broadcastSerialLn("[HTTP FS] ERRO: Falha ao abrir '" + path + "' ou é um diretório.");
            if (file) { // Se file é um objeto válido (ex: é um diretório), feche-o.
                file.close();
            }
            return false; // Falha
        }
    }
    
    // Se LittleFS.exists(path) retornou false
    broadcastSerialLn("[HTTP FS] ERRO FINAL: Arquivo '" + path + "' nao existe no FS.");
    return false; 
}


void executarCalibracaoLinhaWebService() {
    // O handler HTTP apenas muda o estado. A execução ocorre aqui no loop.
    // No entanto, como esta função é bloqueante, ela pode ser chamada diretamente pelo handler
    // se uma resposta imediata não for crítica ou se for rápida o suficiente.
    // Para esta versão, o handler vai chamar esta função diretamente.
    
    pararMotoresWebService(); 

    broadcastSerialLn("Iniciando CALIBRAÇÃO DE LINHA (Controle Web)...");
    setColor('a', 150, 0, 150); FastLED.show();

    motor('e', 'f', 30); 
    motor('d', 't', 30); 

    for (int i = 0; i < tempoCalibracaoLinha; i++) { 
        qtr.calibrate();
        if (i % (tempoCalibracaoLinha / 20) == 0) Serial.print(".");
        delay(20); 
    }
    pararMotoresWebService(); 
    broadcastSerialLn("Calibração de Linha (Web) CONCLUÍDA!");
    setColor('a', 0, 0, 0); FastLED.show();
    broadcastSerialLn("Valores minimos (Web Calib):");

    for (uint8_t i = 0; i < SensorCount; i++) 
    { 
        broadcastSerial(String(qtr.calibrationOn.minimum[i]) + " ");
    }
    broadcastSerialLn("Valores maximos (Web Calib):");
    for (uint8_t i = 0; i < SensorCount; i++) 
    { 
        broadcastSerial(String(qtr.calibrationOn.maximum[i]) + " "); 
    }
    broadcastSerialLn(" ");
    estadoRoboAtual = PARADO_WEB; // Define o estado após a calibração
}


// --- SETUP ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    EEPROM.begin(512);

    FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
    LEDS.setBrightness(100); setColor('a', 0,0,0); FastLED.show();

    // --- Inicializar LittleFS ---
    if(!LittleFS.begin(true)){ // true formata se não conseguir montar
        Serial.println("Falha ao montar LittleFS!");
        // Não podemos usar broadcastSerialLn aqui ainda se o WebSocket não estiver pronto
        // Mas o LED pode indicar o erro
        setColor('a', 0, 255, 255); // Cor de erro crítico (ex: ciano)
        FastLED.show();
        while(1) delay(1000); // Trava aqui
    }
    Serial.println("LittleFS montado com sucesso.");

    //  // --- TESTE DE EXISTÊNCIA DE ARQUIVOS ---
    // Serial.println("Verificando existência de arquivos específicos:");
    // const char* files_to_check[] = {"/index.html", "/script.js", "/style.css", "index.html", "script.js", "style.css"};
    // for (const char* f_name : files_to_check) {
    //     if (LittleFS.exists(f_name)) {
    //         File tempFile = LittleFS.open(f_name, "r");
    //         Serial.print("  Arquivo ENCONTRADO: "); Serial.print(f_name);
    //         Serial.print(" (Tamanho: "); Serial.print(tempFile.size()); Serial.println(" bytes)");
    //         tempFile.close();
    //     } else {
    //         Serial.print("  Arquivo NAO ENCONTRADO: "); Serial.println(f_name);
    //     }
    // }
    // Serial.println("------------------------------------");
    // delay(1000);

    // // Opcional: Listar arquivos para debug
    // File root = LittleFS.open("/");
    // File file = root.openNextFile();
    // Serial.println("Arquivos no LittleFS:");
    // while(file){
    //     Serial.print("  "); Serial.print(file.name()); Serial.print(" ("); Serial.print(file.size()); Serial.println(" bytes)");
    //     file = file.openNextFile();
    // }
    // root.close();

    Serial.print("Conectando a "); Serial.println(ssid);
    WiFi.begin(ssid, password);
    int wifi_try_count = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_try_count < 20) {
        delay(500); Serial.print("."); 
        setColor('a', 60, 255, 50); // Amarelo piscando para indicar tentativa de conexão
        FastLED.show();
        delay(100);
        setColor('a', 0, 0, 0);
        FastLED.show();
        wifi_try_count++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi conectado!"); Serial.print("IP: "); Serial.println(WiFi.localIP());
        setColor('a', 120, 255, 100); FastLED.show();
    } else {
        Serial.println("\nFalha ao conectar WiFi."); setColor('a', 0, 255, 100); FastLED.show();
    }

    //httpServer.on("/", HTTP_GET, handleRoot);
    httpServer.on("/iniciar", HTTP_GET, handleIniciarWeb); 
    httpServer.on("/retornar", HTTP_GET, handleRetornarWeb);
    httpServer.on("/pausar", HTTP_GET, handlePausarWeb);
    httpServer.on("/continuar", HTTP_GET, handleContinuarWeb);
    httpServer.on("/recalibrar_linha", HTTP_GET, handleRecalibrarLinhaWeb);
    httpServer.onNotFound([]() {
        if (!handleFileRead(httpServer.uri())) {
            // Se handleFileRead retornar false, o arquivo realmente não foi encontrado ou não pôde ser aberto.
            httpServer.send(404, "text/plain", "Recurso Nao Encontrado pelo Servidor ESP32");
            broadcastSerialLn("[HTTP] Resposta 404 enviada para: " + httpServer.uri());
        }
    });
    httpServer.begin(); Serial.println("Servidor HTTP iniciado!");

    // --- WebSocket Server Setup ---
    webSocketServer.begin();
    webSocketServer.onEvent(webSocketEvent);
    Serial.println("Servidor WebSocket iniciado na porta 81!");

    ledcSetup(LEDC_CANAL_MOTOR_ESQUERDO, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(pwmM1, LEDC_CANAL_MOTOR_ESQUERDO); 
    ledcSetup(LEDC_CANAL_MOTOR_DIREITO, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(pwmM2, LEDC_CANAL_MOTOR_DIREITO);  

    gpio_set_direction(in1, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(in2, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(in3, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(in4, GPIO_MODE_INPUT_OUTPUT); 

    pararMotoresWebService();

    uint8_t pins_qtr_config[] = {s1, s2, s3, s4, s5, s6, s7}; // Array para QTR
    qtr.setTypeAnalog();
    qtr.setSensorPins(pins_qtr_config, SensorCount);
    
    Serial.println("Configurando MPU6050...");
    broadcastSerialLn("Configurando MPU6050...");
    
    Wire.begin(GPIO_NUM_8, GPIO_NUM_9); // Pino I2C SDA, SCL - **VERIFIQUE OS SEUS PINOS!**
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
    delay(100);
    configurarEscalaGiroscopio();
    if (!carregarCalibracaoEEPROM()) {
        calibrarGiroscopio(); 
        salvarCalibracaoEEPROM();
    }
    previousTime_mpu = micros();

    Serial.println("Calibração inicial dos sensores de linha no setup...");
    broadcastSerialLn("Calibração inicial dos sensores de linha no setup...");
    executarCalibracaoLinhaWebService(); 

    Serial.println("Fim setup. Robô pronto em estado PARADO_WEB.");
    broadcastSerialLn("Fim setup. Robô pronto em estado PARADO_WEB.");
    estadoRoboAtual = PARADO_WEB;
    setColor('a', 200, 255, 100); FastLED.show(); delay(500); setColor('a', 0,0,0); FastLED.show();
    broadcastSerialLn("Robô Bartinha Finalizou o Setup!"); // Exemplo de uso
}

// --- LOOP PRINCIPAL ---
void loop() {
    httpServer.handleClient(); 
    webSocketServer.loop(); // ESSENCIAL: Processa eventos WebSocket

    switch (estadoRoboAtual) {
        case PARADO_WEB: break; 
        case INICIANDO_EXPLORACAO_WEB:
            broadcastSerialLn("WEB: Iniciando Exploração -> Seguindo Linha");
            estadoRoboAtual = SEGUINDO_LINHA_WEB;
            break;
        case SEGUINDO_LINHA_WEB:
            //pid_controlado_web();
            pid_seguelinha_original(); 
            break;
        case PAUSADO_WEB: break;
        case EM_NO_WEB:
            broadcastSerialLn("WEB: Robô em Nó. Aguardando...");
            break;
        case CALIBRANDO_LINHA_WEB:
            // A calibração é chamada diretamente pelo handler HTTP e é bloqueante.
            // Se o handler apenas mudasse o estado, a chamada seria aqui:
            // executarCalibracaoLinhaWebService(); 
            break;
        default:
            pararMotoresWebService(); estadoRoboAtual = PARADO_WEB;
            break;
    }
    delay(10); 
}

// --- Implementação das Suas Funções ---
void setColor(char sensor, int h, int s, int v) {
  uint8_t ledIndex = 0; bool setAll = false;
  if (sensor >= '0' && sensor <= '3') { ledIndex = sensor - '0'; }
  else if (sensor == 'a' || sensor == 'A') { setAll = true; }
  else if (sensor >= 0 && sensor <= 3) { ledIndex = sensor; }
  else { setAll = true; }
  if (setAll) { for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(h, s, v); }
  else { if (ledIndex < NUM_LEDS) leds[ledIndex] = CHSV(h, s, v); }
  FastLED.show();
}

void motorE_PWM(int vel)
{
  if(vel > 0)
  {
    gpio_set_pull_mode(in1, GPIO_PULLDOWN_ONLY);
    gpio_set_level(in2, 1);
  }
  else
  {
    gpio_set_pull_mode(in1, GPIO_PULLUP_ONLY);
    gpio_set_level(in2, 0);

    vel *= -1;
  }

  ledcWrite(0, vel);
}

void motorD_PWM(int vel)
{
  if(vel > 0)
  {
    gpio_set_pull_mode(in4, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(in3, GPIO_PULLDOWN_ONLY);

    ledcWrite(1, vel);
  }
  else
  {
    gpio_set_pull_mode(in3, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(in4, GPIO_PULLDOWN_ONLY);

    ledcWrite(1, -vel);
  }

  
}

void motor(char lado, char dir , int pwm) {
  if (lado == 'e' || lado == 'a') { 
    if (dir == 't') motorE_PWM(-pwm); else motorE_PWM(pwm);
  }
  if (lado == 'd' || lado == 'a') { 
    if (dir == 't') motorD_PWM(-pwm); else motorD_PWM(pwm);
  }
}

void lerSens() {
  posicao = qtr.readLineBlack(sensorValues);
}

// Sua função PID original, caso queira usá-la por outros meios (não chamada pela web diretamente)
void pid_seguelinha_original() {
    lerSens();
    int m1Speed;
    int m2Speed;
    
    second_lastError = lastError;
    lastError=error;
    error = posicao - setPoint;
    

    // Serial.println("Setpoint: " + String(setPoint) + " Erro: " + String(error) + " m1Speed: " + String(m1Speed) + " m2Speed: " + String(m2mSpeed) );
    
    if(abs(error) == 3000)
    {
        
        if(abs(lastError) < 1100 && abs(second_lastError) < 1100)
        {
            error = 0;  
            Serial.println("REEEEEEEEE");
        }  
        else Serial.println("talvez");
    } 

    I_pid = I_pid + error;
    
    int motorSpeed = current_KP * error + current_KD * (error - lastError) + current_KI*I_pid;
    
    


    if(abs(error) == 3000)
    {
        Serial.print("  talvez Le: "+ String(abs(lastError)) + " LLE: "+String(abs(second_lastError)) + "   ");
        //delay(1300);
        if(abs(lastError) < 1000 && abs(second_lastError) < 1000)
        {
            error = 0;   
            Serial.println("REEEEEEEEE");
            for(int i = 0; i < 50; i ++)
            {
              Serial.println("REEEEEEEEE");
              motor('e','f',200);
              motor('d','f',200);
            }
            
            // delay(500);
            // motor('a','f',0);
        }  
    }  
 
    if(abs(error) <= 200)
    {
         m1Speed = current_Mm1_reta - motorSpeed;
    
         m2Speed = current_Mm2_reta + motorSpeed;  
    }
    else
    {
         m1Speed = current_M1_base - motorSpeed;
    
         m2Speed = current_M2_base + motorSpeed;  
    }
    if (m1Speed < current_MMAX2_reverso) m1Speed =  current_MMAX2_reverso;           // Determina o limite inferior

    //Serial.println("Setpoint: " + String(setPoint) + " Erro: " + String(error) + " m1Speed: " + String(m1Speed) + " m2Speed: " + String(m2Speed)  + " motorSpeed: " + String(motorSpeed));

    if (m2Speed < current_MMAX2_reverso) m2Speed = current_MMAX2_reverso;           // Determina o limite inferior
    
    
    if (m1Speed > current_MMAX_curva) m1Speed = current_MMAX_curva;     // Determina o limite superior
    
    if (m2Speed > current_MMAX_curva)  m2Speed = current_MMAX_curva;     // Determina o limite superior
    
    if(m1Speed < 0)  motor('d', 't', abs(m1Speed)); 
    else motor('d', 'f', abs(m1Speed));
    
   
    if(m2Speed < 0)
    {
        motor('e', 't', abs(m2Speed)); 
    }
    else
    {
        motor('e', 'f', abs(m2Speed));
    }
}


// --- Funções MPU6050 ---
void configurarEscalaGiroscopio() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true); uint8_t gyroConfig = Wire.read();
  uint8_t fs_sel = (gyroConfig >> 3) & 0x03;
  if (fs_sel < 4) gyroScaleFactor = gyroScaleFactors[fs_sel]; else gyroScaleFactor = 131.0;
  Serial.print("Giroscopio configurado com escala: +/-");
  if (fs_sel == 0) Serial.println("250 deg/s"); else if (fs_sel == 1) Serial.println("500 deg/s");
  else if (fs_sel == 2) Serial.println("1000 deg/s"); else if (fs_sel == 3) Serial.println("2000 deg/s");
  else Serial.println("Desconhecido");
}

void calibrarGiroscopio() {
  long gxT=0, gyT=0, gzT=0; Serial.println("Calibrando giroscopio MPU...");
  for (int i=0; i < calibracoes_mpu; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    gyroX_raw = Wire.read()<<8|Wire.read(); gyroY_raw = Wire.read()<<8|Wire.read(); gyroZ_raw = Wire.read()<<8|Wire.read();
    gxT+=gyroX_raw; gyT+=gyroY_raw; gzT+=gyroZ_raw;
    if(i%(calibracoes_mpu/10)==0) Serial.print("."); delay(3);
  }
  Serial.println();
  gyroXOffset=(float)gxT/calibracoes_mpu; gyroYOffset=(float)gyT/calibracoes_mpu; gyroZOffset=(float)gzT/calibracoes_mpu;
  Serial.print("Offsets Gyro: X="); Serial.print(gyroXOffset); Serial.print(", Y="); Serial.print(gyroYOffset); Serial.print(", Z="); Serial.println(gyroZOffset);
  Serial.println("Cal. giroscopio MPU concluida.");
}

void salvarCalibracaoEEPROM() {
  EEPROM.begin(512);
  EEPROM.put(EEPROM_OFFSET_ADDR, gyroXOffset);
  EEPROM.put(EEPROM_OFFSET_ADDR + sizeof(gyroXOffset), gyroYOffset);
  EEPROM.put(EEPROM_OFFSET_ADDR + 2*sizeof(gyroXOffset), gyroZOffset);
  if(EEPROM.commit()) Serial.println("Calibracao MPU salva na EEPROM."); else Serial.println("Erro ao salvar cal. MPU na EEPROM.");
  EEPROM.end();
}

bool carregarCalibracaoEEPROM() {
  EEPROM.begin(512);
  EEPROM.get(EEPROM_OFFSET_ADDR, gyroXOffset);
  EEPROM.get(EEPROM_OFFSET_ADDR + sizeof(gyroXOffset), gyroYOffset);
  EEPROM.get(EEPROM_OFFSET_ADDR + 2*sizeof(gyroXOffset), gyroZOffset);
  EEPROM.end();
  if (isnan(gyroXOffset) || isnan(gyroYOffset) || isnan(gyroZOffset) || (abs(gyroXOffset) < 0.01 && abs(gyroYOffset) < 0.01 && abs(gyroZOffset) < 0.01 && gyroXOffset !=0 )) { // Adicionada checagem para valores muito próximos de zero mas não zero, o que pode indicar uma calibração não salva ou zerada
    Serial.println("Calibracao MPU nao encontrada ou invalida na EEPROM. Realizar nova calibracao."); return false;
  }
  Serial.println("Calibracao MPU carregada da EEPROM."); return true;
}

float getAngleMPU(char eixo) {
  currentTime_mpu = micros();
  elapsedTime_mpu = (currentTime_mpu - previousTime_mpu) / 1000000.0; 
  previousTime_mpu = currentTime_mpu;

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  accX_raw = Wire.read()<<8|Wire.read(); accY_raw = Wire.read()<<8|Wire.read(); accZ_raw = Wire.read()<<8|Wire.read();

  accAngleX = atan2(accY_raw, accZ_raw) * 180 / PI;
  accAngleY = atan2(-accX_raw, sqrt(accY_raw * accY_raw + accZ_raw * accZ_raw)) * 180 / PI;

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  gyroX_raw = Wire.read()<<8|Wire.read(); gyroY_raw = Wire.read()<<8|Wire.read(); gyroZ_raw = Wire.read()<<8|Wire.read();

  gyroRateX = (gyroX_raw - gyroXOffset) / gyroScaleFactor;
  gyroRateY = (gyroY_raw - gyroYOffset) / gyroScaleFactor;
  gyroRateZ = (gyroZ_raw - gyroZOffset) / gyroScaleFactor;

  angleX += gyroRateX * elapsedTime_mpu;
  angleY += gyroRateY * elapsedTime_mpu;
  angleZ += gyroRateZ * elapsedTime_mpu;

  angleX = accAngleCorrectionFactor * angleX + (1.0 - accAngleCorrectionFactor) * accAngleX;
  angleY = accAngleCorrectionFactor * angleY + (1.0 - accAngleCorrectionFactor) * accAngleY;

  if (eixo == 'X') return angleX;
  else if (eixo == 'Y') return angleY;
  else if (eixo == 'Z') return angleZ;
  return 0;
}
