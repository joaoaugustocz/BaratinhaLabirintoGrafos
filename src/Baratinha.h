// src/Baratinha.h
#ifndef BARATINHA_H
#define BARATINHA_H

#include <Arduino.h>
#include <WebSocketsServer.h>
#include <QTRSensors.h>
#include <FastLED.h>
#include "tipos.h"

// --- Constantes e Definições de Pinos (podem vir para cá ou serem passadas no construtor) ---
// Motores
#define IN1_PIN GPIO_NUM_39
#define IN2_PIN GPIO_NUM_45
#define PWM_M1_PIN GPIO_NUM_46
#define IN3_PIN GPIO_NUM_40
#define IN4_PIN GPIO_NUM_41
#define PWM_M2_PIN GPIO_NUM_42
#define LEDC_CANAL_MOTOR_ESQUERDO 0
#define LEDC_CANAL_MOTOR_DIREITO  1
#define LEDC_FREQ 1000
#define LEDC_RESOLUTION 8

// Sensores QTR
#define S1_PIN GPIO_NUM_1
#define S2_PIN GPIO_NUM_2
#define S3_PIN GPIO_NUM_3
#define S4_PIN GPIO_NUM_4
#define S5_PIN GPIO_NUM_5
#define S6_PIN GPIO_NUM_6
#define S7_PIN GPIO_NUM_7
const uint8_t QTR_SENSOR_COUNT = 7;
const uint8_t QTR_SENSOR_COUNT_BARATINHA = 7; // Usando _BARATINHA


#define S_ESQUERDO_EXTREMO 0 
#define S_ESQUERDO_INTERNO 1 
#define S_CENTRAL_ESQUERDO 2 
#define S_CENTRAL_MEIO     3 
#define S_CENTRAL_DIREITO  4 
#define S_DIREITO_INTERNO  5 
#define S_DIREITO_EXTREMO  6 

// LEDs (se aplicável)
#define NUM_LEDS_BARATINHA 4 // Diferente do NUM_LEDS global se necessário
#define DATA_PIN_LEDS GPIO_NUM_48




class Baratinha {
public:
    //Baratinha(); // Construtor
    Baratinha(WebSocketsServer& ws);

    // --- Métodos de Configuração (chamados no setup() do main.cpp) ---
    void setupMotores();
    void setupSensoresLinha();
    void setupLEDs();
    void calibrarSensoresLinha(int duracaoGiroMs = 750); // Exemplo com parâmetro

    // --- Métodos de Movimentação ---
    void mover(char motorLado, char direcao, int pwm); // 'e', 'd', 'a' (ambos); 'f', 't'; pwm
    void pararMotores();
    void avancarCurto(int velocidade, int tempoMs); // Para os avanços de análise de nó

    void girar90GrausEsquerda(bool preciso = true); // 'preciso' pode usar a lógica de virada precisa
    void girar90GrausDireita(bool preciso = true);
    void girar180Graus(bool preciso = true);
    
    String nomeDoNo(TipoDeNoFinal tipo);//Função Auxiliar para Converter Enum TipoDeNoFinal para String

    // --- MÉTODOS DE SENSORES ATUALIZADOS ---
    void atualizarLeituraSensores();
    uint16_t getPosicaoPID();
    void getValoresSensoresCalibrados(uint16_t* bufferExterno);
    bool sensorVePreto(int sensorIndex);
    bool sensorVeBranco(int sensorIndex);
    const uint16_t* getInternalSensorValues() const { return qtrValoresSensores; }

    // --- MÉTODOS DE LOG ---
    void bcSerial(const String &message);
    void bcSerialln(const String &message);
    void bcSerialF(const char *format, ...); // Para formatação estilo printf

    // --- Métodos de LEDs ---
    void setCorLEDs(char qualLed, int h, int s, int v); // 'a' para todos ou índice


   ResultadoIdentificacaoBaratinha identificarTipoDeNo(TipoDePadraoSensor padraoInicial); 

    

    
    

    // --- Métodos de Leitura de Sensores ---
    void lerSensoresLinhaCalibrados(uint16_t* valoresLidos); // Passa um array para preencher
    uint16_t calcularPosicaoPID(uint16_t* valoresSensores); // Pode receber os valores já lidos
    // Você pode ter uma função que lê e calcula a posição PID internamente também.

    DirecaoGlobal orientacaoAtualRobo = NORTE;
    void atualizarOrientacaoAposVirada(AcaoDFS acaoDaManobra); // Ação que CAUSOU a mudança de orientação
    DirecaoGlobal getDirecaoGlobalRelativa(char direcaoRelativaRobo); // 'E', 'F', 'D'
    AcaoDFS getManobraParaEncarar(DirecaoGlobal direcaoGlobalDesejada);


private:

    WebSocketsServer& _webSocketServer; // Referência ao servidor WebSocket

    QTRSensors qtr; // Objeto QTR fica encapsulado aqui
    uint16_t qtrValoresSensores[QTR_SENSOR_COUNT]; // Buffer interno para os sensores
    CRGB ledsInternos[NUM_LEDS_BARATINHA];        // Buffer interno para LEDs

    // Métodos privados para controle de motor de baixo nível (se você quiser abstrair motorE_PWM, motorD_PWM)
    void _motorE_PWM(int vel);
    void _motorD_PWM(int vel);

    uint16_t _ultimaPosicaoPIDConhecida;
    const uint8_t _NUM_SENSORES_PID_INTERNO = 3;
    const uint8_t _SENSOR_PID_OFFSET_INTERNO = 2;
    const uint16_t _SETPOINT_PID_3SENSORES = 1000; // Se o cálculo do PID for interno

    // Constantes para a análise de nó (movidas de main.cpp para cá)
    const int _TEMPO_POSICIONAMENTO_NO_MS = 80;
    const int _VELOCIDADE_POSICIONAMENTO = 40;
    const int _TEMPO_CHECA_FRENTE_NO_MS = 150;
    const int _VELOCIDADE_CHECA_FRENTE = 35;
    const int _TEMPO_LEITURA_ESTAVEL_MS = 50;
    const int _VELOCIDADE_AVANCO_CONFIRM_FIM = 30; // Velocidade mais lenta para confirmar o FIM
    const int _TEMPO_AVANCO_CONFIRMA_FIM_MS = 300; // Tempo curto para confirmar o FIM

    // Constantes específicas da classe que não precisam ser globais
    const int _VELOCIDADE_ROTACAO_PRECISA = 60;
    const int _TEMPO_MINIMO_SAIR_LINHA_MS = 100;
    const int _TIMEOUT_VIRADA_90_MS = 2800;
    const int _TIMEOUT_VIRADA_180_MS = 4000;
    const int _LIMIAR_ALINHAMENTO_VIRADA = 200;
    const int _VELOCIDADE_AJUSTE_POS_VIRADA = 30;
    const int _TEMPO_AJUSTE_POS_VIRADA_MS = 70;
    
    

    #define _TEMPO_GIRO_BRUTO_90_MS   220   // ajuste empiricamente
    #define _TEMPO_GIRO_BRUTO_180_MS  480   // ~2× o de 90 °
    #define _DELAY_LOOP_VIRADA_MS       8   // intervalo de leitura PID
    

    

};

#endif // BARATINHA_H