// src/Baratinha.h
#ifndef BARATINHA_H
#define BARATINHA_H

#include <Arduino.h>
#include <QTRSensors.h> // Se a leitura dos sensores QTR ficar aqui
#include <FastLED.h>    // Se o controle dos LEDs ficar aqui
#include "tipos.h"      // Para enums como TipoDeNoFinal, se necessário para alguma lógica interna

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

// LEDs (se aplicável)
#define NUM_LEDS_BARATINHA 4 // Diferente do NUM_LEDS global se necessário
#define DATA_PIN_LEDS GPIO_NUM_48


class Baratinha {
public:
    Baratinha(); // Construtor

    // --- Métodos de Configuração (chamados no setup() do main.cpp) ---
    void setupMotores();
    void setupSensoresLinha();
    void setupLEDs();
    void calibrarSensoresLinha(int duracaoGiroMs = 2000); // Exemplo com parâmetro

    // --- Métodos de Movimentação ---
    void mover(char motorLado, char direcao, int pwm); // 'e', 'd', 'a' (ambos); 'f', 't'; pwm
    void pararMotores();
    void girar90GrausEsquerda(bool preciso = true); // 'preciso' pode usar a lógica de virada precisa
    void girar90GrausDireita(bool preciso = true);
    void girar180Graus(bool preciso = true);
    void avancarCurto(int velocidade, int tempoMs); // Para os avanços de análise de nó

    // --- Métodos de Leitura de Sensores ---
    void lerSensoresLinhaCalibrados(uint16_t* valoresLidos); // Passa um array para preencher
    uint16_t calcularPosicaoPID(uint16_t* valoresSensores); // Pode receber os valores já lidos
    // Você pode ter uma função que lê e calcula a posição PID internamente também.

    // --- Métodos de LEDs ---
    void setCorLEDs(char qualLed, int h, int s, int v); // 'a' para todos ou índice

    // --- Outras Funções Auxiliares do Robô ---
    // ... (qualquer outra função que seja específica do hardware/comportamento do robô) ...

private:
    QTRSensors qtr; // Objeto QTR fica encapsulado aqui
    uint16_t qtrValoresSensores[QTR_SENSOR_COUNT]; // Buffer interno para os sensores
    CRGB ledsInternos[NUM_LEDS_BARATINHA];        // Buffer interno para LEDs

    // Métodos privados para controle de motor de baixo nível (se você quiser abstrair motorE_PWM, motorD_PWM)
    void _motorE_PWM(int vel);
    void _motorD_PWM(int vel);

    // Constantes específicas da classe que não precisam ser globais
    const int _VELOCIDADE_ROTACAO_PRECISA = 60;
    const int _TEMPO_MINIMO_SAIR_LINHA_MS = 100;
    const int _TIMEOUT_VIRADA_90_MS = 2500;
    const int _TIMEOUT_VIRADA_180_MS = 4000;
    const int _LIMIAR_ALINHAMENTO_VIRADA = 200;
    const int _VELOCIDADE_AJUSTE_POS_VIRADA = 30;
    const int _TEMPO_AJUSTE_POS_VIRADA_MS = 70;
    const uint16_t _SETPOINT_PID_3SENSORES = 1000; // Se o cálculo do PID for interno
    const uint8_t _NUM_SENSORES_PID_INTERNO = 3;
    const uint8_t _SENSOR_PID_OFFSET_INTERNO = 2;

};

#endif // BARATINHA_H