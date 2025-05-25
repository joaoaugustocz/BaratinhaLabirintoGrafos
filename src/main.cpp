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


// Padrão bruto observado pelos sensores
enum TipoDePadraoSensor {
    PADRAO_LINHA_RETA,        // Sensores centrais ativos, erro PID baixo
    PADRAO_LINHA_SUMIU_CENTRO, // Sensores centrais brancos
    PADRAO_LATERAL_ESQUERDA_FORTE, // Sensores externos esquerdos bem ativos
    PADRAO_LATERAL_DIREITA_FORTE,  // Sensores externos direitos bem ativos
    PADRAO_MUITOS_SENSORES_PRETOS, // Ex: 4+ sensores pretos
    PADRAO_QUASE_TUDO_BRANCO,      // Ex: 0 ou 1 sensor preto
    PADRAO_AMBIGUO             // Outros casos que precisam de confirmação
};

// Classificação final do nó, após qualquer etapa de confirmação
enum TipoDeNoFinal {
    NO_FINAL_NAO_E,
    NO_FINAL_BECO_SEM_SAIDA,
    NO_FINAL_CURVA_90_ESQ,
    NO_FINAL_CURVA_90_DIR,
    NO_FINAL_T_COM_FRENTE_ESQ,   // Frente e Esquerda
    NO_FINAL_T_COM_FRENTE_DIR,   // Frente e Direita
    NO_FINAL_T_SEM_FRENTE,       // Apenas Esquerda e Direita ("pé" do T)
    NO_FINAL_CRUZAMENTO,         // Frente, Esquerda e Direita
    NO_FINAL_RETA_SIMPLES      // Caso especial se quisermos logar
};

// Variável global para armazenar o tipo de nó após confirmação
TipoDeNoFinal ultimoNoClassificado = NO_FINAL_NAO_E;

// Variáveis para o processo de confirmação
bool precisaConfirmarNo = false;
unsigned long inicioMovimentoConfirmacao = 0;
const int DURACAO_MOV_CONFIRMACAO_MS = 400; // Avançar por 200ms (ajuste!)
const int VELOCIDADE_CONFIRMACAO = 30;     // Velocidade baixa para avançar (ajuste!)
TipoDePadraoSensor padraoInicialDetectado;




enum TipoDeNo {
  NAO_E_NO,
  BECO_SEM_SAIDA,
  CURVA_ESQUERDA_90,
  CURVA_DIREITA_90,
  INTERSECAO // Para T, Cruz (+), ou qualquer situação com múltiplas opções de caminho
};

// === ESTADOS DO ROBÔ PARA CONTROLE WEB ===
enum EstadoRobo {
  PARADO_WEB,
  INICIANDO_EXPLORACAO_WEB,
  SEGUINDO_LINHA_WEB,
  PAUSADO_WEB,
  EM_NO_WEB,
  CALIBRANDO_LINHA_WEB,
  CONFIRMANDO_NO_AVANCA, 
  REAVALIANDO_NO_POS_AVANCO
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


// --- Novas Constantes e Variáveis Globais para o PID de 3 Sensores ---
const uint8_t NUM_SENSORES_PID = 3;
// Índices dos sensores centrais no array 'sensorValues' (0-6 para 7 sensores)
// Se s1=idx0, s2=idx1, s3=idx2, s4=idx3, s5=idx4, s6=idx5, s7=idx6
// Centrais são s3, s4, s5 -> índices 2, 3, 4
const uint8_t SENSOR_PID_OFFSET = 2; // Índice do primeiro sensor central (sensorValues[2])

uint16_t posicaoPID_3sensores = 0; // Nova variável para a posição do PID (0-2000)
const uint16_t setPoint_PID_3sensores = 1000; // Novo setPoint ( (3-1)*1000 / 2 )
                                          // Ou simplesmente (NUM_SENSORES_PID - 1) * 1000 / 2  

#define S_ESQUERDO_EXTREMO 0 // s1
#define S_ESQUERDO_INTERNO 1 // s2
#define S_CENTRAL_ESQUERDO 2 // s3
#define S_CENTRAL_MEIO     3 // s4 (centro exato do array de 7 sensores)
#define S_CENTRAL_DIREITO  4 // s5
#define S_DIREITO_INTERNO  5 // s6
#define S_DIREITO_EXTREMO  6 // s7

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



// Esta função assume que sensorValues[] já foi populado por qtr.readCalibrated()
// e que os valores estão na faixa 0-1000 (onde ~1000 = linha preta para readLineBlack)
uint16_t calcularPosicaoPID_3Sensores(uint16_t* qtr_sensorValues) {
    bool onLine = false;
    uint32_t avg = 0; // Para o total ponderado
    uint16_t sum = 0; // Para o denominador
    uint16_t lastValue = 0; // Para desempate quando a linha é perdida

    for (uint8_t i = 0; i < NUM_SENSORES_PID; i++) {
        uint16_t currentValue = qtr_sensorValues[i + SENSOR_PID_OFFSET];

        // A biblioteca QTR usa >200 para 'onLine' e >50 para incluir na média,
        // nos valores normalizados de 0-1000 (onde 1000 é linha).
        if (currentValue > 200) { // Limiar para considerar "na linha"
            onLine = true;
        }
        if (currentValue > 50) { // Limiar para incluir na média ponderada
            avg += (uint32_t)currentValue * (i * 1000); // i aqui é o índice local (0, 1, 2)
            sum += currentValue;
        }
        if(i == (NUM_SENSORES_PID - 1) / 2) { // Guarda o valor do sensor central
             lastValue = currentValue;
        }
    }

    if (!onLine) {
        // Se não detectou linha nos 3 sensores centrais:
        // Retorna 0 se o erro anterior (ou leitura do sensor central) indicava esquerda
        // Retorna 2000 se indicava direita
        // Isso é um pouco diferente do QTR original, que usa _lastPosition de todos os sensores.
        // Para 3 sensores, se o sensor central (índice 1 local) estava mais para um lado.
        // Ou, mais simples, se a última leitura do sensor central foi baixa (branco),
        // podemos assumir que perdemos a linha.
        // A QTR original usa _lastPosition. Como não temos isso aqui de forma fácil,
        // podemos usar um proxy ou simplesmente retornar uma extremidade se o erro PID anterior era grande.
        // Por agora, vamos simplificar: se não há linha nos 3 centrais, decidimos com base no erro PID anterior.
        if (error < 0) return 0; // Estava à esquerda
        else return (NUM_SENSORES_PID - 1) * 1000; // Estava à direita ou no centro/perdido
    }

    return avg / sum;
}


// --- Funções Auxiliares para Detecção de Nós (ATUALIZADAS) ---

// Retorna true se o sensor indicado está provavelmente sobre uma linha preta
// Assume que sensorValues[sensorIndex] está na faixa 0-1000,
// onde ~1000 é linha preta e ~0 é fundo branco (após readLineBlack).
bool sensorVePreto(int sensorIndex) {
    // Se o valor normalizado é alto (ex: > 700 de 1000), consideramos preto.
    // Este limiar (700) é um ponto de partida e pode precisar de ajuste fino.
    return sensorValues[sensorIndex] > 700; 
}

// Retorna true se o sensor indicado está provavelmente sobre uma superfície branca
// Assume que sensorValues[sensorIndex] está na faixa 0-1000,
// onde ~1000 é linha preta e ~0 é fundo branco (após readLineBlack).
bool sensorVeBranco(int sensorIndex) {
    // Se o valor normalizado é baixo (ex: < 300 de 1000), consideramos branco.
    // Este limiar (300) é um ponto de partida e pode precisar de ajuste fino.
    return sensorValues[sensorIndex] < 300;
}


TipoDePadraoSensor detectarPadraoSensores() {
    bool s[SensorCount];
    int contSensoresPretos = 0;
    for (int i = 0; i < SensorCount; i++) {
        s[i] = sensorVePreto(i); // true se preto
        if (s[i]) contSensoresPretos++;
    }

    bool pid_frente_forte = s[S_CENTRAL_ESQUERDO] && s[S_CENTRAL_MEIO] && s[S_CENTRAL_DIREITO];
    bool pid_sem_linha_frente = sensorVeBranco(S_CENTRAL_ESQUERDO) && sensorVeBranco(S_CENTRAL_MEIO) && sensorVeBranco(S_CENTRAL_DIREITO);
    
    bool tem_saida_esquerda_forte = s[S_ESQUERDO_EXTREMO] && s[S_ESQUERDO_INTERNO];
    // bool tem_saida_esquerda_parcial = s[S_ESQUERDO_EXTREMO] || s[S_ESQUERDO_INTERNO]; // Pode usar esta se a "forte" for muito restritiva
    bool tem_saida_direita_forte = s[S_DIREITO_EXTREMO] && s[S_DIREITO_INTERNO];
    // bool tem_saida_direita_parcial = s[S_DIREITO_EXTREMO] || s[S_DIREITO_INTERNO];


    // 1. Quase tudo branco -> Provável Beco sem Saída ou perda total
    if (contSensoresPretos <= 1 && pid_sem_linha_frente) {
        return PADRAO_QUASE_TUDO_BRANCO;
    }

    // 2. Muitos sensores pretos -> Provável grande área de intersecção
    if (contSensoresPretos >= (SensorCount - 1)) { // 6 ou 7 sensores pretos
        return PADRAO_MUITOS_SENSORES_PRETOS;
    }
    
    // 3. Linha em frente detectada pelos sensores do PID
    if (pid_frente_forte) {
        if (tem_saida_esquerda_forte || tem_saida_direita_forte) {
             // Tem frente E tem pelo menos uma saída lateral forte
             return PADRAO_AMBIGUO; // Pode ser T c/ frente, Cruzamento ou até início de curva se o robô está desalinhado
        }
        // Se tem frente forte e nenhuma saída lateral forte, é uma linha reta (ou curva suave)
        if (abs(error) < 300) { // Erro do PID (3 sensores) baixo
            return PADRAO_LINHA_RETA;
        } else {
            // Erro do PID alto, mas ainda vendo linha em frente -> pode ser curva suave ou desalinhamento
            return PADRAO_LINHA_RETA; // Ou um PADRAO_CURVA_SUAVE se quiser distinguir
        }
    }

    // 4. Linha em frente NÃO detectada fortemente pelos 3 sensores centrais do PID
    if (pid_sem_linha_frente || !s[S_CENTRAL_MEIO]) { // Se os centrais estão brancos ou o do meio não vê preto
        if (tem_saida_esquerda_forte && tem_saida_direita_forte) {
            return PADRAO_AMBIGUO; // Pode ser "pé" de T, ou um cruzamento onde a frente foi perdida momentaneamente
        }
        if (tem_saida_esquerda_forte) {
            // Sem frente clara, mas esquerda forte. Provável curva de 90 esq ou T onde a frente foi perdida.
            return PADRAO_LATERAL_ESQUERDA_FORTE; 
        }
        if (tem_saida_direita_forte) {
            // Sem frente clara, mas direita forte. Provável curva de 90 dir ou T onde a frente foi perdida.
            return PADRAO_LATERAL_DIREITA_FORTE;
        }
    }
    
    // Se o erro do PID (3 sensores) for muito grande, indica que a linha sumiu para um lado
    if (error < -700) return PADRAO_LATERAL_ESQUERDA_FORTE; // Erro extremo p/ esquerda
    if (error > 700)  return PADRAO_LATERAL_DIREITA_FORTE;  // Erro extremo p/ direita

    return PADRAO_LINHA_RETA; // Default se nada mais se encaixar
}

// --- Função Auxiliar para Converter Enum TipoDeNoFinal para String ---
String nomeDoNo(TipoDeNoFinal tipo) {
    switch (tipo) {
        case NO_FINAL_NAO_E: return "NAO_E_NO_FINAL"; // Renomeado para clareza
        case NO_FINAL_BECO_SEM_SAIDA: return "BECO_SEM_SAIDA";
        case NO_FINAL_CURVA_90_ESQ: return "CURVA_90_ESQ";
        case NO_FINAL_CURVA_90_DIR: return "CURVA_90_DIR";
        case NO_FINAL_T_COM_FRENTE_ESQ: return "T_FRENTE_ESQ";
        case NO_FINAL_T_COM_FRENTE_DIR: return "T_FRENTE_DIR";
        case NO_FINAL_T_SEM_FRENTE: return "T_SEM_FRENTE";
        case NO_FINAL_CRUZAMENTO: return "CRUZAMENTO";
        case NO_FINAL_RETA_SIMPLES: return "RETA_SIMPLES_POS_CONFIRMACAO"; // Para o caso pós-confirmação
        default: return "NO_DESCONHECIDO (" + String(tipo) + ")";
    }
}

void pid_controlado_web() {
    if (estadoRoboAtual != SEGUINDO_LINHA_WEB) return;

    lerSens(); 
    second_lastError = lastError;
    lastError = error;
    error = posicaoPID_3sensores - setPoint_PID_3sensores; 

    TipoDePadraoSensor padraoAtual = detectarPadraoSensores();

    // --- LÓGICA DE TRANSIÇÃO DE ESTADO ---
    if (padraoAtual == PADRAO_QUASE_TUDO_BRANCO) {
        broadcastSerialLn("PADRAO_QUASE_TUDO_BRANCO detectado.");
        pararMotoresWebService();
        ultimoNoClassificado = NO_FINAL_BECO_SEM_SAIDA;
        broadcastSerialLn("Nó Classificado Direto: " + nomeDoNo(ultimoNoClassificado)); // MENSAGEM ÚNICA
        estadoRoboAtual = EM_NO_WEB;
        I_pid = 0;
        return;
    } else if (padraoAtual == PADRAO_LATERAL_ESQUERDA_FORTE && abs(error) > 700 && sensorVeBranco(S_DIREITO_EXTREMO)) { 
        // Se erro do PID é grande e apenas um lado claramente ativo
        // (condição mais forte para curva, evitando confusão com T)
        broadcastSerialLn("PADRAO_LATERAL_ESQUERDA_FORTE (provável curva) detectado.");
        pararMotoresWebService();
        ultimoNoClassificado = NO_FINAL_CURVA_90_ESQ;
        broadcastSerialLn("Nó Classificado Direto: " + nomeDoNo(ultimoNoClassificado)); // MENSAGEM ÚNICA
        estadoRoboAtual = EM_NO_WEB;
        I_pid = 0;
        return;
    } else if (padraoAtual == PADRAO_LATERAL_DIREITA_FORTE && abs(error) > 700 && sensorVeBranco(S_ESQUERDO_EXTREMO)) {
        broadcastSerialLn("PADRAO_LATERAL_DIREITA_FORTE (provável curva) detectado.");
        pararMotoresWebService();
        ultimoNoClassificado = NO_FINAL_CURVA_90_DIR;
        broadcastSerialLn("Nó Classificado Direto: " + nomeDoNo(ultimoNoClassificado)); // MENSAGEM ÚNICA
        estadoRoboAtual = EM_NO_WEB;
        I_pid = 0;
        return;
    } else if (padraoAtual == PADRAO_AMBIGUO || padraoAtual == PADRAO_MUITOS_SENSORES_PRETOS ||
               (padraoAtual == PADRAO_LATERAL_DIREITA_FORTE && abs(error) <= 700) || // Lateral forte mas erro PID não tão extremo
               (padraoAtual == PADRAO_LATERAL_ESQUERDA_FORTE && abs(error) <= 700) ) {
        pararMotoresWebService(); // Para antes de avançar
        broadcastSerialLn("Padrão AMBIGUO/INTERSECAO INICIAL detectado (" + String(padraoAtual) + "). Avançando para confirmar...");
        padraoInicialDetectado = padraoAtual; // Guarda o padrão inicial
        precisaConfirmarNo = true;
        estadoRoboAtual = CONFIRMANDO_NO_AVANCA;
        inicioMovimentoConfirmacao = millis();
        motor('a', 'f', VELOCIDADE_CONFIRMACAO); // Inicia o movimento de avanço
        I_pid = 0;
        return;
    }
    // Se for PADRAO_LINHA_RETA ou não se encaixar acima, continua PID
    
    // Lógica PID (como antes, mas sem a detecção de nó original)
    I_pid = I_pid + error;
    I_pid = constrain(I_pid, -5000, 5000); // Ajuste este limite se necessário
    
    int motorSpeedCorrection = current_KP * error + current_KD * (error - lastError) + current_KI * I_pid;
    
    int m1Speed_web, m2Speed_web;
    if (abs(error) <= 100) { // Limiar para "reta" no PID de 3 sensores
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



// Nova função para classificar após o avanço
void classificarNoAposAvanco(TipoDePadraoSensor padraoAntesDoAvanco) {
    // 'sensorValues' já foi atualizado no estado REAVALIANDO_NO_POS_AVANCO
    bool s_depois[SensorCount];
    int contPretos_depois = 0;
    for (int i = 0; i < SensorCount; i++) {
        s_depois[i] = sensorVePreto(i);
        if (s_depois[i]) contPretos_depois++;
    }

    bool frente_depois = s_depois[S_CENTRAL_ESQUERDO] && s_depois[S_CENTRAL_MEIO] && s_depois[S_CENTRAL_DIREITO];
    bool esquerda_depois = s_depois[S_ESQUERDO_EXTREMO] || s_depois[S_ESQUERDO_INTERNO];
    bool direita_depois = s_depois[S_DIREITO_EXTREMO] || s_depois[S_DIREITO_INTERNO];

    broadcastSerialLn("[ConfirmNode] Pós-Avanço: F=" + String(frente_depois) + " E=" + String(esquerda_depois) + " D=" + String(direita_depois) + " Pretos=" + String(contPretos_depois));

    // --- Lógica de Decisão Final ---
    // Exemplo para o caso que era PADRAO_LATERAL_DIREITA_FORTE ou AMBIGUO (tendendo a direita)
    if (padraoAntesDoAvanco == PADRAO_LATERAL_DIREITA_FORTE || 
        (padraoAntesDoAvanco == PADRAO_AMBIGUO && direita_depois) ) { // Se o padrão inicial sugeria direita

        if (!frente_depois && direita_depois && !esquerda_depois) {
            // Linha em frente sumiu, só tem direita forte
            ultimoNoClassificado = NO_FINAL_CURVA_90_DIR;
        } else if (frente_depois && direita_depois && !esquerda_depois) {
            ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_DIR;
        } else if (frente_depois && direita_depois && esquerda_depois) {
            ultimoNoClassificado = NO_FINAL_CRUZAMENTO;
        } else if (!frente_depois && direita_depois && esquerda_depois) {
            ultimoNoClassificado = NO_FINAL_T_SEM_FRENTE; // Estava no pé do T e confirmou
        } else if (!frente_depois && !direita_depois && !esquerda_depois && contPretos_depois <=1) {
            // Tudo sumiu após o avanço -> provavelmente um beco ou overshot
            ultimoNoClassificado = NO_FINAL_BECO_SEM_SAIDA; // Ou um tipo de erro
        }
         else { // Não conseguiu classificar claramente, talvez continuar reto se houver frente?
            if(frente_depois) {
                ultimoNoClassificado = NO_FINAL_RETA_SIMPLES; // Ou um T_COM_FRENTE_DIR se a direita ainda estiver lá
                 if (direita_depois) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_DIR;
                 else if (esquerda_depois) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_ESQ; // Improvável se o erro era para direita
            } else {
                broadcastSerialLn("[ConfirmNode] Classificação DIREITA incerta pós-avanço.");
                ultimoNoClassificado = NO_FINAL_NAO_E; // Ou um tipo de erro específico
            }
        }
    } 
    // Adicionar lógica similar para PADRAO_LATERAL_ESQUERDA_FORTE e outros PADRAO_AMBIGUO
    else if (padraoAntesDoAvanco == PADRAO_LATERAL_ESQUERDA_FORTE ||
             (padraoAntesDoAvanco == PADRAO_AMBIGUO && esquerda_depois)) {
        if (!frente_depois && esquerda_depois && !direita_depois) {
            ultimoNoClassificado = NO_FINAL_CURVA_90_ESQ;
        } else if (frente_depois && esquerda_depois && !direita_depois) {
            ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_ESQ;
        } else if (frente_depois && direita_depois && esquerda_depois) {
            ultimoNoClassificado = NO_FINAL_CRUZAMENTO;
        } else if (!frente_depois && direita_depois && esquerda_depois) {
            ultimoNoClassificado = NO_FINAL_T_SEM_FRENTE;
        } else if (!frente_depois && !direita_depois && !esquerda_depois && contPretos_depois <=1) {
            ultimoNoClassificado = NO_FINAL_BECO_SEM_SAIDA;
        } else {
             if(frente_depois) {
                ultimoNoClassificado = NO_FINAL_RETA_SIMPLES;
                if (esquerda_depois) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_ESQ;
                else if (direita_depois) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_DIR;
            } else {
                broadcastSerialLn("[ConfirmNode] Classificação ESQUERDA incerta pós-avanço.");
                ultimoNoClassificado = NO_FINAL_NAO_E;
            }
        }
    }
    else if (padraoAntesDoAvanco == PADRAO_MUITOS_SENSORES_PRETOS || 
             (padraoAntesDoAvanco == PADRAO_AMBIGUO && frente_depois) ) { // Estava numa área preta grande
        if (frente_depois && esquerda_depois && direita_depois) ultimoNoClassificado = NO_FINAL_CRUZAMENTO;
        else if (frente_depois && esquerda_depois) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_ESQ;
        else if (frente_depois && direita_depois) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_DIR;
        else if (!frente_depois && esquerda_depois && direita_depois) ultimoNoClassificado = NO_FINAL_T_SEM_FRENTE;
        else if (frente_depois) ultimoNoClassificado = NO_FINAL_RETA_SIMPLES; // Ou um T mal detectado
        else {
            broadcastSerialLn("[ConfirmNode] Classificação ÁREA PRETA incerta pós-avanço.");
            ultimoNoClassificado = NO_FINAL_NAO_E; // Ou um tipo de erro específico
        }
    } else {
        broadcastSerialLn("[ConfirmNode] Padrão inicial não tratado para reclassificação: " + String(padraoAntesDoAvanco));
        ultimoNoClassificado = NO_FINAL_NAO_E;
    }


    if (ultimoNoClassificado == NO_FINAL_NAO_E || ultimoNoClassificado == NO_FINAL_RETA_SIMPLES) {
        estadoRoboAtual = SEGUINDO_LINHA_WEB; // Volta a seguir linha se não for um nó conclusivo
        broadcastSerialLn("[ConfirmNode] Decisão: Não é um nó definitivo ou é reta. Retomando seguimento de linha.");
    } else {
        estadoRoboAtual = EM_NO_WEB; // Nó classificado, para para processamento do grafo
        broadcastSerialLn("[ConfirmNode] Decisão Final do Nó: " + nomeDoNo(ultimoNoClassificado));
    }
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

    //Serial.println("Calibração inicial dos sensores de linha no setup...");
    //broadcastSerialLn("Calibração inicial dos sensores de linha no setup...");
    //executarCalibracaoLinhaWebService(); 

    Serial.println("Fim setup. Robô pronto em estado PARADO_WEB.");
    broadcastSerialLn("Fim setup. Robô pronto em estado PARADO_WEB.");
    estadoRoboAtual = PARADO_WEB;
    setColor('a', 200, 255, 100); FastLED.show(); delay(500); setColor('a', 0,0,0); FastLED.show();
    broadcastSerialLn("Robô Bartinha Finalizou o Setup!"); // Exemplo de uso
}

// --- LOOP PRINCIPAL ---
void loop() {
    httpServer.handleClient(); 
    webSocketServer.loop();    

    switch (estadoRoboAtual) {
        case PARADO_WEB: break; 
        case INICIANDO_EXPLORACAO_WEB:
            broadcastSerialLn("WEB: Iniciando Exploração -> Seguindo Linha");
            estadoRoboAtual = SEGUINDO_LINHA_WEB;
            precisaConfirmarNo = false; // Reseta flag de confirmação
            break;

        case SEGUINDO_LINHA_WEB:
            if (precisaConfirmarNo) { // Se estava confirmando e voltou para seguir linha (decisão foi NAO_E_NO)
                precisaConfirmarNo = false;
            }
            pid_controlado_web(); // Esta função agora vai detectar e pode mudar o estado
            break;

        case CONFIRMANDO_NO_AVANCA: // Novo estado
            // Verifica se o tempo de avanço já passou
            if (millis() - inicioMovimentoConfirmacao >= DURACAO_MOV_CONFIRMACAO_MS) {
                pararMotoresWebService();
                broadcastSerialLn("[ConfirmNode] Avanço concluído. Reavaliando...");
                estadoRoboAtual = REAVALIANDO_NO_POS_AVANCO;
            } else {
                // Continua avançando devagar (já iniciado em pid_controlado_web)
                // motor('a', 'f', VELOCIDADE_CONFIRMACAO); // Certifique-se que o motor está andando
            }
            break;

        case REAVALIANDO_NO_POS_AVANCO: // Novo estado
            lerSens(); // Lê os sensores após o pequeno avanço
            classificarNoAposAvanco(padraoInicialDetectado); // Nova função para tomar a decisão final
                                                             // Esta função definirá ultimoNoClassificado e mudará estado para EM_NO_WEB
            precisaConfirmarNo = false;
            break;

        case PAUSADO_WEB: break;
        case EM_NO_WEB:
            //broadcastSerialLn("WEB: Robô em Nó (" + nomeDoNo(ultimoNoClassificado) + "). Aguardando...");
            // Aqui você precisaria de uma lógica para o robô decidir o que fazer
            // com base no 'ultimoNoClassificado' e no seu algoritmo de grafo.
            // Por enquanto, ele para. Para reiniciar a exploração, o usuário clica em "Iniciar".
            break;
        case CALIBRANDO_LINHA_WEB:
            // Como a calibração é bloqueante e chamada pelo handler, este case pode não ser muito usado no loop
            break;
        default:
            pararMotoresWebService(); estadoRoboAtual = PARADO_WEB;
            broadcastSerialLn("[AVISO] Estado desconhecido, robô parado.");
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
    qtr.readCalibrated(sensorValues); // Popula 'sensorValues' com todos os 7 sensores (0-1000, onde ~1000=preto)
    
    // Calcula a posição para o PID usando os 3 sensores centrais
    posicaoPID_3sensores = calcularPosicaoPID_3Sensores(sensorValues); 

    // Opcional: ainda calcular a posição com todos os sensores se 'detectarTipoDeNo' usar
    // Se não for usar a 'posicao' (0-6000) original, pode remover esta linha.
    // posicao = qtr.readLineBlack(sensorValues); // Isto re-chamaria readCalibrated, o que não é ideal.
                                               // Se precisar da posicao 0-6000, é melhor ter uma função
                                               // que a calcule a partir do 'sensorValues' já lido,
                                               // similar a calcularPosicaoPID_3Sensores mas para 7 sensores.
                                               // Por agora, vamos focar no PID com 3 sensores.
                                               // 'detectarTipoDeNo' usará o array 'sensorValues' diretamente.
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
