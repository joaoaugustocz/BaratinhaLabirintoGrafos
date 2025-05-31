// src/Baratinha.cpp
#include "Baratinha.h"
#include <stdarg.h> 

Baratinha::Baratinha(WebSocketsServer& ws) : _webSocketServer(ws) {
    // Construtor pode ser vazio se a inicialização principal for nos métodos setupXxx()
}

void Baratinha::setupMotores() {
    ledcSetup(LEDC_CANAL_MOTOR_ESQUERDO, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(PWM_M1_PIN, LEDC_CANAL_MOTOR_ESQUERDO);
    ledcSetup(LEDC_CANAL_MOTOR_DIREITO, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(PWM_M2_PIN, LEDC_CANAL_MOTOR_DIREITO);

    gpio_set_direction(IN1_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(IN2_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(IN3_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(IN4_PIN, GPIO_MODE_INPUT_OUTPUT);
    pararMotores(); // Garante que os motores começam parados
}

void Baratinha::setupSensoresLinha() {
    // Os pinos já estão definidos no .h ou poderiam ser passados aqui
    uint8_t pins_qtr_config[] = {S1_PIN, S2_PIN, S3_PIN, S4_PIN, S5_PIN, S6_PIN, S7_PIN};
    qtr.setTypeAnalog();
    qtr.setSensorPins(pins_qtr_config, QTR_SENSOR_COUNT);
}

void Baratinha::setupLEDs() {
    FastLED.addLeds<WS2812B, DATA_PIN_LEDS, RGB>(ledsInternos, NUM_LEDS_BARATINHA);
    FastLED.setBrightness(100); // Ou um brilho padrão
    setCorLEDs('a', 0, 0, 0); // Apaga todos os LEDs inicialmente
    FastLED.show();
}

void Baratinha::calibrarSensoresLinha(int duracaoGiroMs) {
    // Adapte sua lógica de calibração para cá
    // Exemplo: girar para um lado por metade do tempo, depois para o outro
    // broadcastSerialLn("Iniciando Calibração de Sensores na Classe Baratinha...");
    setCorLEDs('a', 150, 255, 150); // Cor de calibração (ex: roxo)

    mover('e', 'f', 30);
    mover('d', 't', 30);
    unsigned long inicioCalib = millis();
    while(millis() - inicioCalib < (unsigned long)duracaoGiroMs / 2) {
        qtr.calibrate();
        delay(20);
    }
    pararMotores();

    mover('e', 't', 30);
    mover('d', 'f', 30);
    inicioCalib = millis();
    while(millis() - inicioCalib < (unsigned long)duracaoGiroMs) {
        qtr.calibrate();
        delay(20);
    }
    pararMotores();
    mover('e', 'f', 30);
    mover('d', 't', 30);
    inicioCalib = millis();
    while(millis() - inicioCalib < (unsigned long)duracaoGiroMs/2) {
        qtr.calibrate();
        delay(20);
    }
    pararMotores();
    setCorLEDs('a', 0,0,0);

    bcSerialln("Valores minimos (Web Calib):");

    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) 
    { 
        bcSerial(String(qtr.calibrationOn.minimum[i]) + " ");
    }
    bcSerialln("Valores maximos (Web Calib):");
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) 
    { 
        bcSerial(String(qtr.calibrationOn.maximum[i]) + " "); 
    }
    bcSerialln(" ");
    // broadcastSerialLn("Calibração de Sensores Concluída.");
    // Você pode querer que este método retorne os valores min/max ou os imprima.
}


// --- Implementações dos Métodos de Movimentação ---
void Baratinha::_motorE_PWM(int vel) { // Método privado
    if(vel > 0) {
        gpio_set_pull_mode(IN1_PIN, GPIO_PULLDOWN_ONLY);
        gpio_set_level(IN2_PIN, 1);
    } else {
        gpio_set_pull_mode(IN1_PIN, GPIO_PULLUP_ONLY);
        gpio_set_level(IN2_PIN, 0);
        vel *= -1;
    }
    ledcWrite(LEDC_CANAL_MOTOR_ESQUERDO, vel);
}

void Baratinha::_motorD_PWM(int vel) { // Método privado
     if(vel > 0) {
        gpio_set_pull_mode(IN4_PIN, GPIO_PULLUP_ONLY); // Verifique sua lógica original aqui
        gpio_set_pull_mode(IN3_PIN, GPIO_PULLDOWN_ONLY);
     } else {
        gpio_set_pull_mode(IN3_PIN, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(IN4_PIN, GPIO_PULLDOWN_ONLY);
        vel *= -1;
     }
    ledcWrite(LEDC_CANAL_MOTOR_DIREITO, vel);
}

void Baratinha::mover(char motorLado, char direcao, int pwm) {
    int velE = 0;
    int velD = 0;

    if (direcao == 'f') {
        // pwm é positivo
    } else if (direcao == 't') {
        pwm = -pwm; // Inverte para velocidade negativa
    } else {
        pwm = 0; // Parar se a direção for inválida
    }

    if (motorLado == 'e' || motorLado == 'a') {
        _motorE_PWM(pwm);
    }
    if (motorLado == 'd' || motorLado == 'a') {
        _motorD_PWM(pwm); // Para o motor direito, a lógica pode ser invertida dependendo da montagem
                          // Se for para frente com pwm positivo, e para trás com pwm negativo
                          // A lógica de _motorD_PWM já deve tratar isso.
                          // Se o motor direito gira ao contrário, você pode passar -pwm para ele aqui
                          // Ex: _motorD_PWM( (direcao == 'f' ? pwm_direita : -pwm_direita) );
                          // Mas sua _motorD_PWM parece correta, só precisa garantir que
                          // pwm positivo = para frente e pwm negativo = para trás para AMBOS os motores
                          // na perspectiva do robô.
                          // Sua lógica original de motorD_PWM parece ok, onde vel positivo é frente.
    }
     if (motorLado == 'a' && pwm == 0) { // Parar ambos
        _motorE_PWM(0);
        _motorD_PWM(0);
    }
}


void Baratinha::pararMotores() {
    mover('a', 'f', 0); // 'a' para ambos, 'f' é irrelevante se pwm é 0
}

void Baratinha::girar90GrausEsquerda(bool preciso) {
    bcSerialln("[Baratinha] Girando 90 Esq...");
    pararMotores();
    delay(100);
    // Adapte sua lógica de virar_esquerda_90_preciso() para cá
    // usando os defines _VELOCIDADE_ROTACAO_PRECISA, etc., da classe
    _motorE_PWM(-_VELOCIDADE_ROTACAO_PRECISA); // Motor esquerdo para trás
    _motorD_PWM(_VELOCIDADE_ROTACAO_PRECISA);  // Motor direito para frente

    unsigned long inicioTimerVirada = millis();
    bool linhaAlvoAlinhada = false;
    uint16_t posPidLocal; // para ler e calcular a posição PID aqui dentro

    while (millis() - inicioTimerVirada < (unsigned long)_TIMEOUT_VIRADA_90_MS) {
        lerSensoresLinhaCalibrados(qtrValoresSensores); // Usa o buffer interno
        posPidLocal = calcularPosicaoPID(qtrValoresSensores); // Usa o método da classe
        #define S_CENTRAL_MEIO S4_PIN
        if (millis() - inicioTimerVirada > (unsigned long)_TEMPO_MINIMO_SAIR_LINHA_MS) {
            if (abs(posPidLocal - _SETPOINT_PID_3SENSORES) < _LIMIAR_ALINHAMENTO_VIRADA &&
                qtrValoresSensores[S_CENTRAL_MEIO] > 700) { // Usando sensorVePreto(S_CENTRAL_MEIO) implícito
                linhaAlvoAlinhada = true;
                break;
            }
        }
        delay(10);
    }
    pararMotores();
    if (linhaAlvoAlinhada) {
        mover('a', 'f', _VELOCIDADE_AJUSTE_POS_VIRADA);
        delay(_TEMPO_AJUSTE_POS_VIRADA_MS);
        pararMotores();
    } else {
        bcSerialln("[Baratinha] ERRO ao girar 90 Esq.");
    }
}
// Implemente girar90GrausDireita e girar180Graus de forma similar...

void Baratinha::avancarCurto(int velocidade, int tempoMs) {
    mover('a', 'f', velocidade);
    delay(tempoMs);
    pararMotores();
}

// --- Implementações dos Métodos de Leitura de Sensores ---
void Baratinha::lerSensoresLinhaCalibrados(uint16_t* valoresLidos) {
    qtr.readCalibrated(valoresLidos); // Preenche o array passado como parâmetro
}

uint16_t Baratinha::calcularPosicaoPID(uint16_t* valoresSensores) {
    // Mova sua lógica de calcularPosicaoPID_3Sensores para cá,
    // usando _NUM_SENSORES_PID_INTERNO e _SENSOR_PID_OFFSET_INTERNO
    // Exemplo adaptado:
    bool onLine = false;
    uint32_t avg = 0;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < _NUM_SENSORES_PID_INTERNO; i++) {
        uint16_t currentValue = valoresSensores[i + _SENSOR_PID_OFFSET_INTERNO];
        if (currentValue > 200) onLine = true;
        if (currentValue > 50) {
            avg += (uint32_t)currentValue * (i * 1000);
            sum += currentValue;
        }
    }
    if (!onLine) {
        // Precisa de uma forma de obter o 'error' anterior do PID se for usar essa lógica.
        // Ou, para encapsulamento, a classe Baratinha poderia manter seu próprio estado de erro PID.
        // Por simplicidade aqui, vamos retornar o ponto médio se a linha for perdida.
        return (_NUM_SENSORES_PID_INTERNO - 1) * 1000 / 2;
    }
    if (sum == 0) return (_NUM_SENSORES_PID_INTERNO - 1) * 1000 / 2; // Evita divisão por zero
    return avg / sum;
}

// --- Implementações dos Métodos de LEDs ---
void Baratinha::setCorLEDs(char qualLed, int h, int s, int v) {
    // Adapte sua função setColor para usar ledsInternos
    uint8_t ledIndex = 0; bool setAll = false;
    if (qualLed >= '0' && qualLed < ('0' + NUM_LEDS_BARATINHA)) { ledIndex = qualLed - '0'; }
    else if (qualLed == 'a' || qualLed == 'A') { setAll = true; }
    // ... (lógica completa do seu setColor) ...
    if (setAll) {
        for (int i = 0; i < NUM_LEDS_BARATINHA; i++) ledsInternos[i] = CHSV(h, s, v);
    } else {
        if (ledIndex < NUM_LEDS_BARATINHA) ledsInternos[ledIndex] = CHSV(h, s, v);
    }
    FastLED.show();
}

void Baratinha::bcSerial(const String &message) {
    Serial.print(message); // Mantém o log no Serial Monitor
    String nonConstMessage = message;
    _webSocketServer.broadcastTXT(nonConstMessage); // Envia via WebSocket
}

void Baratinha::bcSerialln(const String &message) {
    Serial.println(message); // Mantém o log no Serial Monitor
    String nonConstMessage = message + "\n";
    _webSocketServer.broadcastTXT(nonConstMessage); // Envia via WebSocket
}

void Baratinha::bcSerialF(const char *format, ...) {
    char buf[256]; // Buffer para a string formatada
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    Serial.print(buf); // Mantém o log no Serial Monitor
    String messageToSend = String(buf);
    _webSocketServer.broadcastTXT(messageToSend); // Envia via WebSocket
}
// ... Implemente outros métodos ...