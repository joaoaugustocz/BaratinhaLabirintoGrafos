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

    mover('e', 'f', 50);
    mover('d', 't', 50);
    unsigned long inicioCalib = millis();
    while(millis() - inicioCalib < (unsigned long)duracaoGiroMs / 2) {
        qtr.calibrate();
        delay(20);
    }
    pararMotores();

    mover('e', 't', 50);
    mover('d', 'f', 50);
    inicioCalib = millis();
    while(millis() - inicioCalib < (unsigned long)duracaoGiroMs) {
        qtr.calibrate();
        delay(20);
    }
    pararMotores();
    mover('e', 'f', 50);
    mover('d', 't', 50);
    inicioCalib = millis();
    while(millis() - inicioCalib < ((unsigned long)duracaoGiroMs/2  + 1)) {
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


String Baratinha::nomeDoNo(TipoDeNoFinal tipo) { //
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
        case NO_FINAL_FIM_DO_LABIRINTO: return "FIM_DO_LABIRINTO"; // <<< ADICIONAR
        default: return "NO_DESCONHECIDO (" + String(tipo) + ")";
    }
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
void Baratinha::girar90GrausDireita(bool preciso) { // Retornando bool como as outras funções de virada
    bcSerialln("[Baratinha] Girando 90 Dir..."); // Usando seu método de log da Baratinha
    pararMotores();
    delay(100); // Pequena pausa

    // Lógica invertida em relação a girar para a esquerda
    _motorE_PWM(_VELOCIDADE_ROTACAO_PRECISA);  // Motor esquerdo para FRENTE
    _motorD_PWM(-_VELOCIDADE_ROTACAO_PRECISA); // Motor direito para TRÁS

    unsigned long inicioTimerVirada = millis();
    bool linhaAlvoAlinhada = false;
    uint16_t posPidLocal;

    // Índice para S_CENTRAL_MEIO (sensor 4 do array 0-6, ou seja, índice 3)
    // Se você definiu os BARATINHA_IDX_S_... em Baratinha.h, use-os.
    // Assumindo que S_CENTRAL_MEIO (global do main.cpp) ainda é acessível ou você tem um equivalente.
    // Idealmente, use um define da classe ou um valor numérico direto (ex: 3).
    // Para este exemplo, vou usar o define global S_CENTRAL_MEIO que você tinha no main.cpp,
    // mas o ideal seria BARATINHA_IDX_S_CENTRAL_MEIO.
    // Certifique-se que Baratinha.cpp possa "ver" S_CENTRAL_MEIO.
    // Se S_CENTRAL_MEIO foi movido para Baratinha.h como BARATINHA_IDX_S_CENTRAL_MEIO, use-o.
    const int SENSOR_CENTRAL_PARA_ALINHAMENTO = S_CENTRAL_MEIO; // Ou BARATINHA_IDX_S_CENTRAL_MEIO

    while (millis() - inicioTimerVirada < (unsigned long)_TIMEOUT_VIRADA_90_MS) {
        // Se lerSensoresLinhaCalibrados e calcularPosicaoPID foram substituídos por
        // atualizarLeituraSensores e getPosicaoPID na classe Baratinha:
        this->atualizarLeituraSensores(); 
        posPidLocal = this->getPosicaoPID();

        if (millis() - inicioTimerVirada > (unsigned long)_TEMPO_MINIMO_SAIR_LINHA_MS) {
            // Usa o método sensorVePreto da classe
            if (abs(posPidLocal - _SETPOINT_PID_3SENSORES) < _LIMIAR_ALINHAMENTO_VIRADA &&
                this->sensorVePreto(SENSOR_CENTRAL_PARA_ALINHAMENTO)) { 
                linhaAlvoAlinhada = true;
                bcSerialln("[Baratinha] Virada Dir: Linha alvo encontrada.");
                break;
            }
        }
        delay(10);
    }
    pararMotores();

    if (linhaAlvoAlinhada) {
        bcSerialln("[Baratinha] Virada Dir: Concluída com sucesso.");
        mover('a', 'f', _VELOCIDADE_AJUSTE_POS_VIRADA);
        delay(_TEMPO_AJUSTE_POS_VIRADA_MS);
        pararMotores();
        //return true;
    } else {
        bcSerialln("[Baratinha] Virada Dir: ERRO - Timeout ou falha ao alinhar.");
        //return false;
    }
}
void Baratinha::girar180Graus(bool preciso) {
    bcSerialln("[Baratinha] Girando 180 graus...");
    pararMotores();
    delay(100);
    // Lógica de girar 180 graus
    _motorE_PWM(_VELOCIDADE_ROTACAO_PRECISA); // Motor esquerdo para trás
    _motorD_PWM(-_VELOCIDADE_ROTACAO_PRECISA); // Motor direito para trás

    unsigned long inicioTimerVirada = millis();
    bool linhaAlvoAlinhada = false;
    uint16_t posPidLocal;

    while (millis() - inicioTimerVirada < (unsigned long)_TIMEOUT_VIRADA_180_MS) {
        lerSensoresLinhaCalibrados(qtrValoresSensores);
        posPidLocal = calcularPosicaoPID(qtrValoresSensores);
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
        bcSerialln("[Baratinha] ERRO ao girar 180 graus.");
    }
}

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
    uint8_t ledIndex = 0; bool setAll = false;
    if (qualLed >= '0' && qualLed <= '3') { ledIndex = qualLed - '0'; }
    else if (qualLed == 'a' || qualLed == 'A') { setAll = true; }
    else if (qualLed >= 0 && qualLed <= 3) { ledIndex = qualLed; }
    else { setAll = true; }
    if (setAll) { for (int i = 0; i < NUM_LEDS_BARATINHA; i++) ledsInternos[i] = CHSV(h, s, v); }
    else { if (ledIndex < NUM_LEDS_BARATINHA) ledsInternos[ledIndex] = CHSV(h, s, v); }
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

bool Baratinha::sensorVePreto(int sensorIndex) {
    // Use os defines BARATINHA_IDX_S_... se o sensorIndex passado for um desses.
    // Ou, se sensorIndex já for o índice numérico 0-6, pode usar diretamente.
    // Adicionando verificação de limites para segurança:
    if (sensorIndex < 0 || sensorIndex >= QTR_SENSOR_COUNT_BARATINHA) return false;
    return this->qtrValoresSensores[sensorIndex] > 700; 
}

bool Baratinha::sensorVeBranco(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= QTR_SENSOR_COUNT_BARATINHA) return false;
    return this->qtrValoresSensores[sensorIndex] < 300; 
}


void Baratinha::atualizarLeituraSensores() {
    qtr.readCalibrated(this->qtrValoresSensores); // Lê para o buffer interno da classe
}

uint16_t Baratinha::getPosicaoPID() {
    bool onLine = false;
    uint32_t avg = 0;
    uint16_t sum = 0;

    for (uint8_t i = 0; i < _NUM_SENSORES_PID_INTERNO; i++) {
        uint16_t currentValue = this->qtrValoresSensores[i + _SENSOR_PID_OFFSET_INTERNO];
        if (currentValue > 200) {
            onLine = true;
        }
        if (currentValue > 50) {
            avg += (uint32_t)currentValue * (i * 1000);
            sum += currentValue;
        }
    }

    if (!onLine) {
        if (_ultimaPosicaoPIDConhecida < _SETPOINT_PID_3SENSORES) {
            return 0; 
        } else {
            return (_NUM_SENSORES_PID_INTERNO - 1) * 1000;
        }
    }

    if (sum == 0) {
         _ultimaPosicaoPIDConhecida = (_NUM_SENSORES_PID_INTERNO - 1) * 1000 / 2; // Centro
    } else {
        _ultimaPosicaoPIDConhecida = avg / sum;
    }
    return _ultimaPosicaoPIDConhecida;
}

void Baratinha::getValoresSensoresCalibrados(uint16_t* bufferExterno) {
    if (bufferExterno != nullptr) {
        for (int i = 0; i < QTR_SENSOR_COUNT_BARATINHA; i++) { // Use a constante da classe
            bufferExterno[i] = this->qtrValoresSensores[i];
        }
    }
}


ResultadoIdentificacaoBaratinha Baratinha::identificarTipoDeNo(TipoDePadraoSensor padraoInicialBruto) {
    this->pararMotores(); // Garante que está parado
    this->bcSerialln("[IdentificaNo] Iniciando identificação de nó...");
    ResultadoIdentificacaoBaratinha resultado;
    //delay(2000);//XXXXXXXXXXXXXXXXXXXXXX

    // Variáveis locais para armazenar o que foi detectado
    bool achouEsquerdaAgora = false;
    bool achouDireitaAgora = false;
    bool achouFrenteAgora = false;

    resultado.temSaidaEsquerda = achouEsquerdaAgora;
    resultado.temSaidaFrente = achouFrenteAgora;
    resultado.temSaidaDireita = achouDireitaAgora;

    // 1. VERIFICAÇÕES RÁPIDAS (BASEADAS NO PADRÃO INICIAL QUE FEZ O PID PARAR)
    if (padraoInicialBruto == PADRAO_QUASE_TUDO_BRANCO) {
        this->atualizarLeituraSensores(); // Confirma com uma nova leitura
        int pretosAtuais = 0;
        for(int i=0; i<QTR_SENSOR_COUNT; i++) if(this->sensorVePreto(i)) pretosAtuais++;
        
        if (pretosAtuais <= 1 && 
            this->sensorVeBranco(S_CENTRAL_ESQUERDO) && 
            this->sensorVeBranco(S_CENTRAL_MEIO) && 
            this->sensorVeBranco(S_CENTRAL_DIREITO)) {
            this->bcSerialln("[IdentificaNo] Classificado como BECO_SEM_SAIDA (detecção rápida).");
            resultado.tipo = NO_FINAL_BECO_SEM_SAIDA;
            return resultado;
        }
    }

    // if (padraoInicialBruto == PADRAO_MUITOS_SENSORES_PRETOS) {
    //     this->bcSerialln("[IdentificaNo] Suspeita de FIM_DO_LABIRINTO (padrão inicial). Confirmando...");
        //this->avancarCurto(_VELOCIDADE_AVANCO_CONFIRM_FIM, _TEMPO_AVANCO_CONFIRMA_FIM_MS);
        this->setCorLEDs('a', 10, 255, 255);
        this->mover('a', 'f', 25);
        int tempx = millis();
        while(millis() - tempx < 600)
        {
            this->atualizarLeituraSensores();

            if(this->sensorVePreto(S_ESQUERDO_EXTREMO)) 
            {
                this->setCorLEDs('0', 100, 255, 255);
                this->setCorLEDs('3', 100, 255, 255);
                achouEsquerdaAgora = true;
            }
            if(this->sensorVePreto(S_DIREITO_EXTREMO))
            {

                this->setCorLEDs('1', 100, 255, 255);
                this->setCorLEDs('2', 100, 255, 255);
                achouDireitaAgora = true;
            } 
            achouFrenteAgora = this->sensorVePreto(S_CENTRAL_MEIO) ||
                       this->sensorVePreto(S_CENTRAL_ESQUERDO) ||
                       this->sensorVePreto(S_CENTRAL_DIREITO);
        }
        this->pararMotores(); // Garante que está parado após a verificação rápida
        this->bcSerialln("SEE: " + String(achouEsquerdaAgora) + 
                         " | SDE: " + String(achouDireitaAgora) + 
                         " | SC: " + String(achouFrenteAgora));
        //delay(3000);
        this->setCorLEDs('a', 0, 0, 0);

        this->atualizarLeituraSensores();
        int pretosConfirmacao = 0;
        for (int i = 0; i < QTR_SENSOR_COUNT; i++) {
            if (this->sensorVePreto(i)) pretosConfirmacao++;
        }


        resultado.temSaidaEsquerda = achouEsquerdaAgora;
        resultado.temSaidaFrente = achouFrenteAgora;
        resultado.temSaidaDireita = achouDireitaAgora;
                    
        this->bcSerialln(String("[IdentificaNo] Confirmação FIM: Pretos Pós-Avanço=") + pretosConfirmacao);

        if (pretosConfirmacao >= QTR_SENSOR_COUNT - 1) {
            this->bcSerialln("[IdentificaNo] Confirmado: FIM_DO_LABIRINTO.");
            resultado.tipo = NO_FINAL_FIM_DO_LABIRINTO;
            return resultado;
        } 
        
        if(achouFrenteAgora && !achouEsquerdaAgora && achouDireitaAgora)
        {
            this->bcSerialln("[IdentificaNo] Confirmado: T_COM_FRENTE_DIR (detecção rápida).");
            // Se só vê frente, mas não vê laterais, é uma reta simples.
            resultado.tipo = NO_FINAL_T_COM_FRENTE_DIR;
            return resultado;
        }
        else if(achouFrenteAgora && achouEsquerdaAgora && !achouDireitaAgora)
        {
            this->bcSerialln("[IdentificaNo] Confirmado: T_COM_FRENTE_ESQ (detecção rápida).");
            // Se só vê frente, mas não vê laterais, é uma reta simples.
            resultado.tipo = NO_FINAL_T_COM_FRENTE_ESQ; 
            return resultado;
        }
        else if(achouFrenteAgora && achouEsquerdaAgora && achouDireitaAgora)
        {
            this->bcSerialln("[IdentificaNo] Confirmado: CRUZAMENTO (detecção rápida).");
            // Se vê frente e laterais, é um cruzamento.
            resultado.tipo = NO_FINAL_CRUZAMENTO;
            return resultado;
        }
        else if(!achouFrenteAgora && achouEsquerdaAgora && !achouDireitaAgora)
        {
            this->bcSerialln("[IdentificaNo] Confirmado: CURVA_90_ESQ (detecção rápida).");
            // Se só vê esquerda, é uma curva 90 graus para a esquerda.
            resultado.tipo = NO_FINAL_CURVA_90_ESQ;
            return resultado;
        }
        else if(!achouFrenteAgora && !achouEsquerdaAgora && achouDireitaAgora)
        {
            this->bcSerialln("[IdentificaNo] Confirmado: CURVA_90_DIR (detecção rápida).");
            // Se só vê direita, é uma curva 90 graus para a direita.
            resultado.tipo = NO_FINAL_CURVA_90_DIR;
            return resultado;
        }
        else if(!achouFrenteAgora && achouEsquerdaAgora && achouDireitaAgora)
        {
            this->bcSerialln("[IdentificaNo] Confirmado: T_SEM_FRENTE (detecção rápida).");
            // Se vê laterais, mas não vê frente, é um T sem saída frontal.
            resultado.tipo = NO_FINAL_T_SEM_FRENTE;
            return resultado;
        }
        else if(!achouFrenteAgora && !achouEsquerdaAgora && !achouDireitaAgora)
        {
            this->bcSerialln("[IdentificaNo] Confirmado: BECO_SEM_SAIDA.");
            // Se não vê nada, é um beco sem saída.
            resultado.tipo = NO_FINAL_BECO_SEM_SAIDA;
            return resultado;
        }
        else
        {
            this->bcSerialln("[IdentificaNo] Falso alarme de FIM. Prosseguindo com análise detalhada.");
            // Se não bateu com nenhum caso claro, prossegue com análise detalhada
            resultado.tipo = NO_FINAL_NAO_E; // Não é um nó final claro
            return resultado; // Retorna o resultado com tipo NO_FINAL_NAO_E
        }
}


void Baratinha::atualizarOrientacaoAposVirada(AcaoDFS acao) { // AcaoDFS pode precisar ser renomeado para AcaoManobra se for mais genérico
    if (acao == ACAO_DFS_VIRAR_ESQUERDA) { // Ou um enum AcaoManobra::VIRAR_ESQUERDA
        orientacaoAtualRobo = (DirecaoGlobal)((orientacaoAtualRobo - 1 + 4) % 4); // +4 para lidar com NORTE-1
    } else if (acao == ACAO_DFS_VIRAR_DIREITA) {
        orientacaoAtualRobo = (DirecaoGlobal)((orientacaoAtualRobo + 1) % 4);
    } else if (acao == ACAO_DFS_RETROCEDER_180) {
        orientacaoAtualRobo = (DirecaoGlobal)((orientacaoAtualRobo + 2) % 4);
    }
    // Nenhuma mudança para ACAO_DFS_SEGUIR_FRENTE
    this->bcSerialln(String("Nova Orientação Robô: ") + (int)orientacaoAtualRobo);
}

// Função para traduzir uma direção relativa do robô para uma direção global
DirecaoGlobal Baratinha::getDirecaoGlobalRelativa(char direcaoRelativa) { // 'E'squerda, 'F'rente, 'D'ireita
    if (direcaoRelativa == 'F') return orientacaoAtualRobo;
    if (direcaoRelativa == 'E') return (DirecaoGlobal)((orientacaoAtualRobo - 1 + 4) % 4);
    if (direcaoRelativa == 'D') return (DirecaoGlobal)((orientacaoAtualRobo + 1) % 4);
    return orientacaoAtualRobo; // Default ou erro
}

// Função para determinar qual manobra fazer para encarar uma direção global
AcaoDFS Baratinha::getManobraParaEncarar(DirecaoGlobal direcaoDesejada) {
    int delta = (direcaoDesejada - orientacaoAtualRobo + 4) % 4;
    if (delta == 0) return ACAO_DFS_SEGUIR_FRENTE; // Já está virado
    if (delta == 1) return ACAO_DFS_VIRAR_DIREITA; // 90 graus à direita
    if (delta == 2) return ACAO_DFS_RETROCEDER_180; // 180 graus
    if (delta == 3) return ACAO_DFS_VIRAR_ESQUERDA; // 90 graus à esquerda (ou 270 direita)
    return ACAO_DFS_ERRO; // Não deveria acontecer
}