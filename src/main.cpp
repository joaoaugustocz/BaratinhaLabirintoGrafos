#include <Arduino.h>
#include "Baratinha.h" // Incluindo a classe Baratinha
#include "DFSManager.h" // Incluindo o gerenciador de DFS
#include <FastLED.h>
#include <QTRSensors.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // Para WebSockets
#include <FS.h>               // Necessário para o sistema de arquivos
#include <LittleFS.h>         // Usando LittleFS
#include "tipos.h"


Baratinha bra;

//------ GRAFOS ------------//

// Variáveis globais para o mapeamento simples
int idProximoNo = 0;        // Contador para gerar IDs de nós únicos
int idNoAtual = -1;         // ID do nó onde o robô está ou acabou de classificar
int idNoAnterior = -1;      // ID do nó de onde o robô veio para chegar ao nó atual
bool primeiroNo = true;     // Flag para tratar o nó inicial



//------ GRAFOS ------------//


// --- Instância do DFSManager ---
DFSManager dfsManager;


// --- Configurações de Wi-Fi ---
const char* ssid = "Joao_2G";
const char* password = "Naotemsenha";
// const char* ssid = "CriarCe Coordenacao";
// const char* password = "inovareempreender";

// --- Servidor Web HTTP ---
WebServer httpServer(80);

// --- WebSocket Server ---
WebSocketsServer webSocketServer = WebSocketsServer(81); // Porta 81 para WebSockets



// Variável global para armazenar o tipo de nó após confirmação
TipoDeNoFinal ultimoNoClassificado = NO_FINAL_NAO_E;


// Variáveis Globais para a análise do nó
bool achouEsquerdaNoPonto1 = false;
bool achouDireitaNoPonto1 = false;
bool achouFrenteNoPonto2 = false; 


TipoDePadraoSensor padraoInicialDetectado;

EstadoRobo estadoRoboAtual = PARADO_WEB;
bool exploracaoWebIniciada = false;

bool lateralEsquerdaEncontradaBusca = false;
bool lateralDireitaEncontradaBusca = false;
unsigned long inicioAvanco = 0; // Timer genérico para os avanços

// unsigned long inicioAvanco; // Já existe
// const int VELOCIDADE_AVANCO_CONFIRM = 40; // Já existe, podemos renomear ou usar esta

const int TEMPO_POSICIONAMENTO_NO_MS = 80;   // Ajuste este valor (3pi usa 50ms a vel 50)
const int VELOCIDADE_POSICIONAMENTO = 40;   // Ajuste este valor

const int TEMPO_CHECA_FRENTE_NO_MS = 150; // Ajuste este valor (3pi usa 200ms a vel 40)
const int VELOCIDADE_CHECA_FRENTE = 35;    // Ajuste este valor

const int TEMPO_LEITURA_ESTAVEL_MS = 50; // Pequeno delay para estabilizar leitura após parar


// Constantes de tempo/velocidade para os avanços de confirmação (AJUSTE ESTES VALORES!)
const int DURACAO_BUSCA_LATERAL_MS = 600;     // Max tempo para achar uma lateral
const int DURACAO_AVANCO_FRENTE_MS = 200;     // Tempo para avançar e checar a frente
const int VELOCIDADE_AVANCO_CONFIRM = 40;   // Velocidade para os avanços

const int TEMPO_AVANCO_CONFIRMA_FIM_MS = 100; // Tempo para avançar e confirmar área toda preta (ajuste!)
// A VELOCIDADE_AVANCO_CONFIRM (40) pode ser usada aqui também, ou uma específica.

TipoDeNoFinal classificacaoPreliminarDoNo = NO_FINAL_NAO_E; 


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
#define KI_3 0.001//0.0003
#define KD_3 0.1//1.75
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


//--------------------------- Sensores Linha (Suas Definições)
QTRSensors qtr;
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];
#define tempoCalibracaoLinha 51 

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


// Constantes para Viradas Precisas (AJUSTE ESTES VALORES EXPERIMENTALMENTE!)
const int VELOCIDADE_ROTACAO_PRECISA = 60;    // Velocidade dos motores durante a rotação (0-255)
const int TEMPO_MINIMO_PARA_SAIR_DA_LINHA_MS = 100; // Tempo mínimo girando antes de procurar nova linha (evita re-detectar a linha original)
const int TIMEOUT_VIRADA_90_MS = 2500;       // Timeout máximo para virada de 90 graus (2.5 segundos)
const int TIMEOUT_VIRADA_180_MS = 4000;      // Timeout máximo para virada de 180 graus (4 segundos)
const int LIMIAR_ALINHAMENTO_VIRADA = 200;   // Quão perto de setPoint_PID_3sensores (1000) para considerar alinhado (faixa +/- 200)

const int VELOCIDADE_AJUSTE_FINAL = 30;     // Velocidade para pequeno avanço/recuo após virada
const int TEMPO_AJUSTE_FINAL_MS = 70;       // Duração do pequeno avanço/recuo

// --- Protótipos ---
void setColor(char sensor, int h, int s, int v);



void lerSens();
void motorE_PWM(int vel);
void motorD_PWM(int vel);
void motor(char lado, char dir, int pwm);
void pid_controlado_web();
void executarCalibracaoLinhaWebService();
void pararMotoresWebService();


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
        s[i] = sensorVePreto(i); 
        if (s[i]) contSensoresPretos++;
    }

    // Condições para os 3 sensores centrais do PID (S_CENTRAL_ESQUERDO, S_CENTRAL_MEIO, S_CENTRAL_DIREITO)
    bool pid_linha_nos_3centrais_forte = s[S_CENTRAL_ESQUERDO] || s[S_CENTRAL_MEIO] || s[S_CENTRAL_DIREITO];
    bool pid_linha_pelo_menos_no_meio = s[S_CENTRAL_MEIO];
    bool pid_sem_linha_nos_3centrais = sensorVeBranco(S_CENTRAL_ESQUERDO) && 
                                     sensorVeBranco(S_CENTRAL_MEIO) && 
                                     sensorVeBranco(S_CENTRAL_DIREITO);

    // Sensores laterais extremos
    bool lateral_E_extremo_preto = s[S_ESQUERDO_EXTREMO];
    bool lateral_D_extremo_preto = s[S_DIREITO_EXTREMO];
    bool alguma_lateral_extrema_ativa = lateral_E_extremo_preto || lateral_D_extremo_preto;

    // --- ORDEM DE DETECÇÃO ---

    // 1. FIM DO LABIRINTO / ÁREA TODA PRETA (MAIOR PRIORIDADE)
    if (contSensoresPretos >= (SensorCount - 1)) { // Ex: 6 ou 7 dos 7 sensores estão pretos
        return PADRAO_MUITOS_SENSORES_PRETOS;
    }

    // 2. BECO SEM SAÍDA CLARO
    if (pid_sem_linha_nos_3centrais && contSensoresPretos <= 1) { 
        if (sensorVeBranco(S_ESQUERDO_INTERNO) && sensorVeBranco(S_DIREITO_INTERNO)) { // Confirma com vizinhos
            return PADRAO_QUASE_TUDO_BRANCO;
        }
    }
    
    // 3. LINHA RETA (CONDIÇÃO PARA CONTINUAR O PID SEM PARAR)
    // Se os 3 centrais estão na linha, erro PID baixo, e NENHUM extremo lateral ativo.
    // if (pid_linha_nos_3centrais_forte && abs(error) < 550 && !alguma_lateral_extrema_ativa) {
    //     return PADRAO_LINHA_RETA;
    // }
    // // Uma condição um pouco mais relaxada para linha reta se o erro estiver pequeno
    // if (pid_linha_pelo_menos_no_meio && abs(error) < 200 && !alguma_lateral_extrema_ativa && contSensoresPretos >=1 && contSensoresPretos <=3) {
    //     return PADRAO_LINHA_RETA;
    // }

    if (pid_linha_nos_3centrais_forte && !alguma_lateral_extrema_ativa) {
        return PADRAO_LINHA_RETA;
    }
    // Uma condição um pouco mais relaxada para linha reta se o erro estiver pequeno
    if (pid_linha_pelo_menos_no_meio && !alguma_lateral_extrema_ativa && contSensoresPretos >=1 && contSensoresPretos <=3) {
        return PADRAO_LINHA_RETA;
    }


    // 4. SE NÃO FOR RETA NEM FIM NEM BECO, É UM PADRÃO QUE EXIGE ANÁLISE/CONFIRMAÇÃO
    // Isso inclui laterais ativas, erros grandes do PID, etc.
    // A função pid_controlado_web tratará qualquer retorno que não seja PADRAO_LINHA_RETA
    // como um gatilho para iniciar a análise de nó.
    // Podemos retornar PADRAO_AMBIGUO como um genérico, ou os padrões laterais específicos.

    if (alguma_lateral_extrema_ativa) {
        // Se tem lateral E linha em frente, é ambíguo (T, Cruz, início de curva)
        if (pid_linha_pelo_menos_no_meio) return PADRAO_AMBIGUO;
        // Se NÃO tem linha em frente clara, e uma lateral está ativa -> forte candidato a curva
        if (lateral_E_extremo_preto && !lateral_D_extremo_preto) return PADRAO_LATERAL_ESQUERDA_FORTE;
        if (lateral_D_extremo_preto && !lateral_E_extremo_preto) return PADRAO_LATERAL_DIREITA_FORTE;
        // Se ambas as laterais e sem frente -> Pé de T
        if (lateral_E_extremo_preto && lateral_D_extremo_preto) return PADRAO_AMBIGUO; // (Será classificado como T_SEM_FRENTE depois)
    }

    // Erro grande do PID sugere que a linha está escapando para um lado
    if (abs(error) > 700) { // Erro do PID de 3 sensores
        if (error < 0) return PADRAO_LATERAL_ESQUERDA_FORTE; // Tendência à esquerda
        else return PADRAO_LATERAL_DIREITA_FORTE; // Tendência à direita
    }
    
    // Se chegou até aqui, é uma situação não claramente reta, mas não um padrão lateral óbvio sozinho.
    // Pode ser uma curva suave que o PID está lidando, mas se o erro não for pequeno,
    // melhor classificar como ambíguo para forçar uma checagem.
    // A condição de PADRAO_LINHA_RETA acima já trata os casos de erro pequeno.
    return PADRAO_AMBIGUO; // Default para qualquer coisa que não seja claramente reta ou um evento já classificado
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
        case NO_FINAL_FIM_DO_LABIRINTO: return "FIM_DO_LABIRINTO"; // <<< ADICIONAR
        default: return "NO_DESCONHECIDO (" + String(tipo) + ")";
    }
}


void pid_controlado_web() {
    if (estadoRoboAtual != SEGUINDO_LINHA_WEB) return;

    lerSens(); 
    second_lastError = lastError;
    lastError = error;
    error = posicaoPID_3sensores - setPoint_PID_3sensores; 

    TipoDePadraoSensor padraoAtual = detectarPadraoSensores(); // Chama a função principal de detecção

    // Se for linha reta, continua o PID
    if (padraoAtual == PADRAO_LINHA_RETA) {
        I_pid = I_pid + error;
        I_pid = constrain(I_pid, -5000, 5000); 
        int motorSpeedCorrection = current_KP * error + current_KD * (error - lastError) + current_KI * I_pid;
        
        int m1Speed_web, m2Speed_web;
        if (abs(error) <= 100) {
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
        return; // Continua seguindo linha
    }

    // Se NÃO for PADRAO_LINHA_RETA, é um evento que requer parada e análise.
    pararMotoresWebService(); 
    I_pid = 0; 
    padraoInicialDetectado = padraoAtual; // GUARDA O PADRÃO QUE CAUSOU A PARADA

    // Log do padrão detectado
    // Para ter o nome do padrão, você precisaria de uma função similar a nomeDoNo() para TipoDePadraoSensor
    // Ex: String nomePadrao = obterNomeDoPadrao(padraoAtual);
    // broadcastSerialLn("[PID_CTRL] Padrão detectado: " + nomePadrao + ". Iniciando análise de nó...");
    broadcastSerialLn("[PID_CTRL] Padrão bruto " + String(padraoAtual) + " detectado. Iniciando análise...");


    if (padraoAtual == PADRAO_QUASE_TUDO_BRANCO) {
        ultimoNoClassificado = NO_FINAL_BECO_SEM_SAIDA;
        // A mensagem "Decisão Final do Nó" será impressa ao entrar em EM_NO_WEB
        estadoRoboAtual = EM_NO_WEB; 
    } 
    // Adicionado para tratar o FIM diretamente se o padrão for muito claro
    else if (padraoAtual == PADRAO_MUITOS_SENSORES_PRETOS) {
        //ultimoNoClassificado = NO_FINAL_FIM_DO_LABIRINTO; // Classifica direto
        // A mensagem "Decisão Final do Nó" será impressa ao entrar em EM_NO_WEB
        //estadoRoboAtual = EM_NO_WEB;
        estadoRoboAtual = PREPARANDO_ANALISE_NO; 
    }
    else { 
        // Para outros padrões (AMBIGUO, LATERAL_FORTE), inicia a sequência de confirmação com avanços
        estadoRoboAtual = PREPARANDO_ANALISE_NO; 
    }
}



// Nova função para classificar após o avanço
void classificarNoAposAvanco(TipoDePadraoSensor padraoDetectadoAntesDoAvanco) {
    lerSens(); // Sempre releia os sensores no momento da classificação final
    bool s_final[SensorCount];
    int contPretos_final = 0;
    for (int i = 0; i < SensorCount; i++) {
        s_final[i] = sensorVePreto(i);
        if (s_final[i]) contPretos_final++;
    }

    // Recalcula 'frente_final' com base na leitura atual
    bool frente_final = sensorVePreto(S_CENTRAL_MEIO) || 
                        (sensorVePreto(S_CENTRAL_ESQUERDO) || sensorVePreto(S_CENTRAL_DIREITO));

    // Logs para depuração (você já tem o de Pós-Avanço)
    // broadcastSerialLn("LeituraSensores em classificarNoAposAvanco: s2=" + String(sensorValues[S_CENTRAL_ESQUERDO]) + " s3=" + String(sensorValues[S_CENTRAL_MEIO]) + " s4=" + String(sensorValues[S_CENTRAL_DIREITO]));

    // --- Lógica de Decisão Final ATUALIZADA ---

    // 1. CHECAR SE É O FIM DO LABIRINTO (PRIORIDADE MÁXIMA)
    if (padraoDetectadoAntesDoAvanco == PADRAO_MUITOS_SENSORES_PRETOS && 
        contPretos_final >= (SensorCount - 1) ) { // Ex: 6 ou 7 dos 7 sensores ainda estão pretos
        ultimoNoClassificado = NO_FINAL_FIM_DO_LABIRINTO;
    } 
    // 2. LÓGICA EXISTENTE PARA OUTROS TIPOS DE NÓS (agora dentro de 'else if')
    //    Esta lógica usará 'frente_final' (calculado acima com base na leitura atual)
    //    e 'lateralEsquerdaEncontradaBusca' / 'lateralDireitaEncontradaBusca' (definidas na fase de busca lateral)
    else if (frente_final) { // Usa a 'frente_final' recalculada aqui
        if (lateralEsquerdaEncontradaBusca && lateralDireitaEncontradaBusca) ultimoNoClassificado = NO_FINAL_CRUZAMENTO;
        else if (lateralEsquerdaEncontradaBusca) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_ESQ;
        else if (lateralDireitaEncontradaBusca) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_DIR;
        else ultimoNoClassificado = NO_FINAL_RETA_SIMPLES; 
    } else { // Sem caminho em frente claro
        if (lateralEsquerdaEncontradaBusca && lateralDireitaEncontradaBusca) ultimoNoClassificado = NO_FINAL_T_SEM_FRENTE;
        else if (lateralEsquerdaEncontradaBusca) ultimoNoClassificado = NO_FINAL_CURVA_90_ESQ;
        else if (lateralDireitaEncontradaBusca) ultimoNoClassificado = NO_FINAL_CURVA_90_DIR;
        else { // Nenhuma lateral e nenhuma frente
            // Se o padrão inicial já indicava quase tudo branco, ou se agora está tudo branco
            if (padraoDetectadoAntesDoAvanco == PADRAO_QUASE_TUDO_BRANCO || contPretos_final <= 1) {
                 ultimoNoClassificado = NO_FINAL_BECO_SEM_SAIDA;
            } else {
                 broadcastSerialLn("[ClassificaFinal] Situação indefinida. Padrão inicial: " + String(padraoDetectadoAntesDoAvanco));
                 ultimoNoClassificado = NO_FINAL_NAO_E; // Ou um tipo de erro/perda
            }
        }
    }

    // Atualiza o log da decisão com o 'ultimoNoClassificado' definido acima
    broadcastSerialLn("[ClassificaFinal] Dados para classificar: EsqP1=" + String(lateralEsquerdaEncontradaBusca) + 
                                  " | DirP1=" + String(lateralDireitaEncontradaBusca) + 
                                  " | FreAgora=" + String(frente_final) + // Usando a frente recalculada
                                  " | PadrãoInicial=" + String(padraoDetectadoAntesDoAvanco) );
    // ... (Seu log de LeituraSensores dos valores brutos dos sensores centrais) ...
    broadcastSerialLn("LeituraSensores: s3: " + String(sensorValues[2]) + " s4: " + String(sensorValues[3]) + "s5: " + String(sensorValues[4]));


    // --- Transição de Estado Final --- (Esta parte do código parece boa)
    if (ultimoNoClassificado == NO_FINAL_NAO_E || ultimoNoClassificado == NO_FINAL_RETA_SIMPLES) {
        estadoRoboAtual = SEGUINDO_LINHA_WEB; 
        broadcastSerialLn("[ClassificaFinal] Decisão: Não é nó para parar / Reta. Retomando seguimento.");
    } else {
        estadoRoboAtual = EM_NO_WEB; 
        // A mensagem "Decisão Final do Nó: ..." será impressa ao entrar no case EM_NO_WEB
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

    for (int i = 0; i < tempoCalibracaoLinha/3; i++) { 
        qtr.calibrate();
        if (i % (tempoCalibracaoLinha / 20) == 0) Serial.print(".");
        delay(20); 
    }
    pararMotoresWebService(); 
    motor('e', 't', 30); 
    motor('d', 'f', 30); 

    for (int i = 0; i < tempoCalibracaoLinha - (tempoCalibracaoLinha/3); i++) { 
        qtr.calibrate();
        if (i % (tempoCalibracaoLinha / 20) == 0) Serial.print(".");
        delay(20); 
    }
    pararMotoresWebService(); 
    motor('e', 'f', 30); 
    motor('d', 't', 30); 

    for (int i = 0; i < tempoCalibracaoLinha/3; i++) { 
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

bool virar_direita_90_preciso() {
    broadcastSerialLn("[Virada] Iniciando: 90 graus DIREITA (precisão)...");
    pararMotoresWebService();
    delay(100); 

    motor('e', 'f', VELOCIDADE_ROTACAO_PRECISA); // Motor esquerdo para frente
    motor('d', 't', VELOCIDADE_ROTACAO_PRECISA); // Motor direito para trás

    unsigned long inicioTimerVirada = millis();
    bool linhaAlvoAlinhada = false;

    while (millis() - inicioTimerVirada < TIMEOUT_VIRADA_90_MS) {
        lerSens(); // Atualiza sensorValues e posicaoPID_3sensores

        // Condição para parar:
        // 1. Já passou um tempo mínimo para garantir que saiu da linha original.
        // 2. A 'posicaoPID_3sensores' está próxima do centro (setPoint_PID_3sensores = 1000).
        // 3. O sensor central do meio (S_CENTRAL_MEIO) está de fato vendo preto.
        if (millis() - inicioTimerVirada > TEMPO_MINIMO_PARA_SAIR_DA_LINHA_MS) {
            if (abs(posicaoPID_3sensores - setPoint_PID_3sensores) < LIMIAR_ALINHAMENTO_VIRADA &&
                sensorVePreto(S_CENTRAL_MEIO)) {
                linhaAlvoAlinhada = true;
                broadcastSerialLn("[Virada] Linha alvo encontrada e alinhada (DIR).");
                break; 
            }
        }
        delay(10); // Pequeno delay para não sobrecarregar e permitir leituras
    }

    pararMotoresWebService();

    if (linhaAlvoAlinhada) {
        broadcastSerialLn("[Virada] DIREITA 90 graus concluída com sucesso.");
        // Opcional: pequeno avanço para "travar" na linha
        motor('a', 'f', VELOCIDADE_AJUSTE_FINAL);
        delay(TEMPO_AJUSTE_FINAL_MS);
        pararMotoresWebService();
        return true;
    } else {
        broadcastSerialLn("[Virada] ERRO: Timeout ou falha ao alinhar na virada DIREITA.");
        // Tentar parar sobre qualquer coisa que pareça linha, se possível, ou apenas parar.
        // Se contSensoresPretos > 0 após o timeout, pode ser um pequeno ajuste.
        // Por enquanto, apenas reporta falha.
        return false;
    }
}
bool virar_esquerda_90_preciso() {
    broadcastSerialLn("[Virada] Iniciando: 90 graus ESQUERDA (precisão)...");
    pararMotoresWebService();
    delay(100);

    motor('e', 't', VELOCIDADE_ROTACAO_PRECISA); // Motor esquerdo para trás
    motor('d', 'f', VELOCIDADE_ROTACAO_PRECISA); // Motor direito para frente

    unsigned long inicioTimerVirada = millis();
    bool linhaAlvoAlinhada = false;

    while (millis() - inicioTimerVirada < TIMEOUT_VIRADA_90_MS) {
        lerSens();

        if (millis() - inicioTimerVirada > TEMPO_MINIMO_PARA_SAIR_DA_LINHA_MS) {
            if (abs(posicaoPID_3sensores - setPoint_PID_3sensores) < LIMIAR_ALINHAMENTO_VIRADA &&
                sensorVePreto(S_CENTRAL_MEIO)) {
                linhaAlvoAlinhada = true;
                broadcastSerialLn("[Virada] Linha alvo encontrada e alinhada (ESQ).");
                break; 
            }
        }
        delay(10); 
    }

    pararMotoresWebService();

    if (linhaAlvoAlinhada) {
        broadcastSerialLn("[Virada] ESQUERDA 90 graus concluída com sucesso.");
        motor('a', 'f', VELOCIDADE_AJUSTE_FINAL);
        delay(TEMPO_AJUSTE_FINAL_MS);
        pararMotoresWebService();
        return true;
    } else {
        broadcastSerialLn("[Virada] ERRO: Timeout ou falha ao alinhar na virada ESQUERDA.");
        return false;
    }
}
bool virar_180_preciso() {
    broadcastSerialLn("[Virada] Iniciando: 180 graus (Meia Volta)...");
    pararMotoresWebService();
    delay(100);

    // Escolha uma direção de rotação (ex: virar como se fosse para a direita por mais tempo)
    motor('e', 'f', VELOCIDADE_ROTACAO_PRECISA); 
    motor('d', 't', VELOCIDADE_ROTACAO_PRECISA); 

    unsigned long inicioTimerVirada = millis();
    bool linhaAlvoAlinhada = false;
    
    // Para 180, o tempo mínimo para sair da linha precisa ser maior, 
    // para passar dos ~90 graus antes de começar a procurar ativamente.
    // Vamos usar TEMPO_MINIMO_VIRADA_90_MS * 1.8 (aproximadamente)
    // ou uma constante própria como TEMPO_MINIMO_VIRADA_180_MS.
    // Por simplicidade, vamos aumentar o tempo total e manter o TEMPO_MINIMO_PARA_SAIR_DA_LINHA_MS.
    // A ideia é que ele vai girar, perder a linha, e depois encontrá-la novamente.

    while (millis() - inicioTimerVirada < TIMEOUT_VIRADA_180_MS) {
        lerSens();

        // Para 180 graus, esperamos que ele passe da linha, perca-a completamente
        // e depois a encontre novamente. A condição de "tempo mínimo" aqui é mais
        // para garantir que ele não pare prematuramente na linha original se o giro for lento.
        // Um tempo mínimo maior antes de aceitar o alinhamento pode ser útil.
        // Vamos usar TEMPO_MINIMO_PARA_SAIR_DA_LINHA_MS * 2 como estimativa para passar do ponto original.
        if (millis() - inicioTimerVirada > (TEMPO_MINIMO_PARA_SAIR_DA_LINHA_MS * 3) ) { // Aumentado para 180
            if (abs(posicaoPID_3sensores - setPoint_PID_3sensores) < LIMIAR_ALINHAMENTO_VIRADA &&
                sensorVePreto(S_CENTRAL_MEIO)) {
                linhaAlvoAlinhada = true;
                broadcastSerialLn("[Virada] Linha alvo encontrada e alinhada (180).");
                break; 
            }
        }
        delay(10); 
    }

    pararMotoresWebService();

    if (linhaAlvoAlinhada) {
        broadcastSerialLn("[Virada] 180 graus (Meia Volta) concluída com sucesso.");
        motor('a', 'f', VELOCIDADE_AJUSTE_FINAL);
        delay(TEMPO_AJUSTE_FINAL_MS);
        pararMotoresWebService();
        return true;
    } else {
        broadcastSerialLn("[Virada] ERRO: Timeout ou falha ao alinhar na virada 180.");
        return false;
    }
}


void callbackParaCriarNoWeb(int id, TipoDeNoFinal tipo, int idPai) {
    String label = nomeDoNo(tipo) + "_ID" + String(id); // Usa sua função nomeDoNo
    String msgNode = "newNode:" + String(id) + ":" + label;
    webSocketServer.broadcastTXT(msgNode);
    broadcastSerialLn("[Grafo Web] Nó Criado: ID=" + String(id) + " Label=" + label + " Pai=" + String(idPai));
}

void callbackParaCriarArestaWeb(int idOrigem, int idDestino, const char* edgeLabel) {
    String msgEdge = "newEdge:" + String(idOrigem) + ":" + String(idDestino) + ":" + String(edgeLabel);
    webSocketServer.broadcastTXT(msgEdge);
    broadcastSerialLn("[Grafo Web] Aresta Criada: De=" + String(idOrigem) + " Para=" + String(idDestino) + " Label=" + String(edgeLabel));
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    delay(1000);

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

    bra.setupLEDs();
    bra.setupMotores();
    bra.setupSensoresLinha();

   
    dfsManager.resetarDFS(); // Importante para limpar o estado do DFS
    broadcastSerialLn("DFS Manager resetado no setup.");

    Serial.println("Fim setup. Robô pronto em estado PARADO_WEB.");
    broadcastSerialLn("Fim setup. Robô pronto em estado PARADO_WEB.");
    estadoRoboAtual = PARADO_WEB;
    setColor('a', 200, 255, 100); FastLED.show(); delay(500); setColor('a', 0,0,0); FastLED.show();
    broadcastSerialLn("Robô Bartinha Finalizou o Setup!"); // Exemplo de uso
}



// --- LOOP PRINCIPAL ---
void loop() 
{
    httpServer.handleClient(); 
    webSocketServer.loop();    

    switch (estadoRoboAtual) {
        case PARADO_WEB:
            pararMotoresWebService(); 
            // Nenhuma variável local inicializada aqui que cause problema
            break; 

        case INICIANDO_EXPLORACAO_WEB:
            if (primeiroNo) 
            { // 'primeiroNo' é a sua flag global
                idNoAnterior = -1;       // Correto para o nó INICIO (não tem pai na lógica do main)
                idNoAtual = idProximoNo++; // idNoAtual recebe o ID do INICIO (ex: 0), idProximoNo avança (ex: para 1)

                String nomeNoInicio = "INICIO_ID" + String(idNoAtual);
                broadcastSerialLn("[Grafo] Criando Nó Inicial! ID: " + String(idNoAtual) + ", Label: " + nomeNoInicio);
                String msgNode = "newNode:" + String(idNoAtual) + ":" + nomeNoInicio;
                webSocketServer.broadcastTXT(msgNode);

                // Informa o DFSManager. NO_FINAL_RETA_SIMPLES é um tipo placeholder para o INICIO.
                // 'true' para temFrenteInicial indica que a partir do INICIO, o robô vai "em frente".
                dfsManager.iniciarNovaExploracao(idNoAtual, NO_FINAL_RETA_SIMPLES, true);
                
                primeiroNo = false;
            }


            // Nenhuma variável local inicializada aqui que cause problema
            broadcastSerialLn("WEB: Iniciando Exploração -> Seguindo Linha");
            idNoAnterior = idNoAtual; 
            estadoRoboAtual = SEGUINDO_LINHA_WEB;
            lateralEsquerdaEncontradaBusca = false;
            lateralDireitaEncontradaBusca = false;
            break;

        case SEGUINDO_LINHA_WEB:
            // Nenhuma variável local inicializada aqui que cause problema
            pid_controlado_web(); 
            break;

       

          case PREPARANDO_ANALISE_NO:
            pararMotoresWebService(); // Garante que parou completamente
            achouEsquerdaNoPonto1 = false; // Reseta flags
            achouDireitaNoPonto1 = false;
            achouFrenteNoPonto2 = false;
            
            broadcastSerialLn("[AnalisaNó] Fase 1: Iniciando avanço curto para posicionamento...");
            inicioAvanco = millis();
            motor('a', 'f', VELOCIDADE_POSICIONAMENTO); // Começa a avançar
            estadoRoboAtual = AVANCO_POSICIONAMENTO_NO;
            break;

        case AVANCO_POSICIONAMENTO_NO:
            if (millis() - inicioAvanco >= TEMPO_POSICIONAMENTO_NO_MS) {
                pararMotoresWebService();
                broadcastSerialLn("[AnalisaNó] Avanço de posicionamento concluído. Lendo laterais...");
                delay(TEMPO_LEITURA_ESTAVEL_MS); // Espera um pouco para sensores estabilizarem
                lerSens(); // Atualiza sensorValues

                // Checa saídas laterais (sensores extremos 0 e 6)
                if (sensorVePreto(S_ESQUERDO_EXTREMO)) {
                    achouEsquerdaNoPonto1 = true;
                }
                if (sensorVePreto(S_DIREITO_EXTREMO)) {
                    achouDireitaNoPonto1 = true;
                }
                broadcastSerialLn("[AnalisaNó] Ponto 1 - Esquerda: " + String(achouEsquerdaNoPonto1) + " | Direita: " + String(achouDireitaNoPonto1));

                // Prepara para o próximo avanço (checar frente)
                broadcastSerialLn("[AnalisaNó] Fase 2: Iniciando avanço para checar frente...");
                inicioAvanco = millis();
                motor('a', 'f', VELOCIDADE_CHECA_FRENTE);
                estadoRoboAtual = AVANCO_CHECA_FRENTE_NO;
            } else {
                 motor('a', 'f', VELOCIDADE_POSICIONAMENTO); // Continua avançando se o tempo não acabou
            }
            break;

        case AVANCO_CHECA_FRENTE_NO:
            if (millis() - inicioAvanco >= TEMPO_CHECA_FRENTE_NO_MS) {
                pararMotoresWebService();
                broadcastSerialLn("[AnalisaNó] Avanço para checar frente concluído. Lendo frente...");
                delay(TEMPO_LEITURA_ESTAVEL_MS);
                lerSens();

                // --- DEBUG DETALHADO AQUI ---
                bool sCE = sensorVePreto(S_CENTRAL_ESQUERDO); // sensorValues[2]
                bool sCM = sensorVePreto(S_CENTRAL_MEIO);     // sensorValues[3]
                bool sCD = sensorVePreto(S_CENTRAL_DIREITO);  // sensorValues[4]

                broadcastSerialLn("[DEBUG ChecaFrente] Valores Brutos Centrais: S2=" + String(sensorValues[S_CENTRAL_ESQUERDO]) + 
                                                                " S3=" + String(sensorValues[S_CENTRAL_MEIO]) + 
                                                                " S4=" + String(sensorValues[S_CENTRAL_DIREITO]));
                broadcastSerialLn("[DEBUG ChecaFrente] sensorVePreto Resultados: sCE=" + String(sCE) + 
                                                                        " sCM=" + String(sCM) + 
                                                                        " sCD=" + String(sCD));
                // --- FIM DO DEBUG DETALHADO ---

                // Checa caminho à frente (sensores centrais do PID: 2, 3, 4)
                // Uma condição mais robusta: pelo menos o central E um dos vizinhos, ou todos os 3
                if (sensorVePreto(S_CENTRAL_MEIO) || 
                    (sensorVePreto(S_CENTRAL_ESQUERDO) || sensorVePreto(S_CENTRAL_DIREITO)) ) {
                    achouFrenteNoPonto2 = true;
                }
                // Ou, se os 3 estiverem pretos:
                // if (sensorVePreto(S_CENTRAL_ESQUERDO) && sensorVePreto(S_CENTRAL_MEIO) && sensorVePreto(S_CENTRAL_DIREITO)) {
                //    achouFrenteNoPonto2 = true;
                // }
                broadcastSerialLn("[AnalisaNó] Ponto 2 - Frente: " + String(achouFrenteNoPonto2));
                
                estadoRoboAtual = CLASSIFICACAO_DETALHADA_NO;
            } else {
                motor('a', 'f', VELOCIDADE_CHECA_FRENTE); // Continua avançando
            }
            break;

        case CLASSIFICACAO_DETALHADA_NO:
        {
            lerSens(); 
            
            bool s_final[SensorCount];
            int contPretos_final_neste_ponto = 0; // Renomeado para evitar confusão com uma possível global
            for (int i = 0; i < SensorCount; i++) {
                s_final[i] = sensorVePreto(i); 
                if (s_final[i]) contPretos_final_neste_ponto++;
            }

            bool frente_classificada_final = s_final[S_CENTRAL_MEIO] || 
                                        (s_final[S_CENTRAL_ESQUERDO] || s_final[S_CENTRAL_DIREITO]);

            // Logs de depuração (como você já tem)
            broadcastSerialLn("[ClassificaDetalhada] Dados: EsqP1=" + String(achouEsquerdaNoPonto1) + 
                                                        " | DirP1=" + String(achouDireitaNoPonto1) + 
                                                        " | FreP2_global=" + String(achouFrenteNoPonto2) + // O que foi detectado no avanço anterior
                                                        " | FreAgora=" + String(frente_classificada_final) + // O que está vendo agora
                                                        " | PretosAgora=" + String(contPretos_final_neste_ponto) );
             // Seu log dos valores dos sensores centrais
            broadcastSerialLn("LeituraSensores em CLASSIFICACAO_DETALHADA_NO: s2=" + String(sensorValues[S_CENTRAL_ESQUERDO]) + 
                                                                            " s3=" + String(sensorValues[S_CENTRAL_MEIO]) + 
                                                                            " s4=" + String(sensorValues[S_CENTRAL_DIREITO]));



            // --- Lógica de Classificação Preliminar ---
            TipoDeNoFinal tipoNoPreliminar = NO_FINAL_NAO_E; // Variável local para a classificação desta etapa

            if (frente_classificada_final) { // Usa a 'frente_classificada_final' baseada na leitura atual
                if (achouEsquerdaNoPonto1 && achouDireitaNoPonto1) tipoNoPreliminar = NO_FINAL_CRUZAMENTO;
                else if (achouEsquerdaNoPonto1) tipoNoPreliminar = NO_FINAL_T_COM_FRENTE_ESQ;
                else if (achouDireitaNoPonto1) tipoNoPreliminar = NO_FINAL_T_COM_FRENTE_DIR;
                else tipoNoPreliminar = NO_FINAL_RETA_SIMPLES; 
            } else { 
                if (achouEsquerdaNoPonto1 && achouDireitaNoPonto1) tipoNoPreliminar = NO_FINAL_T_SEM_FRENTE; 
                else if (achouEsquerdaNoPonto1) tipoNoPreliminar = NO_FINAL_CURVA_90_ESQ;
                else if (achouDireitaNoPonto1) tipoNoPreliminar = NO_FINAL_CURVA_90_DIR;
                else {
                    if (contPretos_final_neste_ponto <= 1) { // Se realmente quase nada preto
                        tipoNoPreliminar = NO_FINAL_BECO_SEM_SAIDA;
                    } else { // Poucos sensores pretos, mas não claramente um beco (pode ser perda de linha)
                        broadcastSerialLn("[ClassificaDetalhada] Padrão incerto, poucos pretos, não é beco claro.");
                        tipoNoPreliminar = NO_FINAL_NAO_E; // Ou um tipo de erro
                    }
                }
            }
            broadcastSerialLn("[ClassificaDetalhada] Classificação preliminar: " + nomeDoNo(tipoNoPreliminar));

            // --- Checagem Adicional para FIM DO LABIRINTO ---
            // Se a classificação preliminar é um tipo de intersecção ou reta (onde o robô poderia avançar)
            // E atualmente está tudo preto, então vamos confirmar se é o FIM.
            bool condicaoParaChecarFim = (tipoNoPreliminar == NO_FINAL_CRUZAMENTO ||
                                        tipoNoPreliminar == NO_FINAL_T_COM_FRENTE_ESQ ||
                                        tipoNoPreliminar == NO_FINAL_T_COM_FRENTE_DIR ||
                                        tipoNoPreliminar == NO_FINAL_RETA_SIMPLES);

            if (condicaoParaChecarFim && contPretos_final_neste_ponto >= (SensorCount - 1)) { // Ex: 6 ou 7 sensores pretos
                broadcastSerialLn("[ClassificaDetalhada] Suspeita de FIM DO LABIRINTO (área preta). Avançando para confirmar...");
                classificacaoPreliminarDoNo = tipoNoPreliminar; // Guarda a classificação caso não seja o FIM
                estadoRoboAtual = CONFIRMANDO_FIM_DO_LABIRINTO;
                inicioAvanco = millis(); // Prepara timer para o avanço de confirmação do FIM
                motor('a', 'f', VELOCIDADE_AVANCO_CONFIRM); // Avança devagar
            } 
            // Se não for suspeita de FIM, ou se for um nó que não precisa de mais checagem (curva, beco)
            else if (tipoNoPreliminar == NO_FINAL_NAO_E || tipoNoPreliminar == NO_FINAL_RETA_SIMPLES) {
                ultimoNoClassificado = tipoNoPreliminar; // Aceita a classificação
                estadoRoboAtual = SEGUINDO_LINHA_WEB; 
                broadcastSerialLn("[ClassificaDetalhada] Decisão: Não é nó para parar / Reta. Retomando seguimento.");
            } else { // Para Curvas, Becos, T sem Frente (que não são "tudo preto")
                ultimoNoClassificado = tipoNoPreliminar; // Aceita a classificação
                estadoRoboAtual = EM_NO_WEB; 
                // A mensagem "Decisão Final do Nó" será impressa ao entrar em EM_NO_WEB
            }
        } 
            break;

        case CONFIRMANDO_FIM_DO_LABIRINTO:
            motor('a', 'f', VELOCIDADE_AVANCO_CONFIRM); // Continua avançando devagar
            
            if (millis() - inicioAvanco > TEMPO_AVANCO_CONFIRMA_FIM_MS) {
                pararMotoresWebService();
                lerSens(); // Leitura final após o avanço extra

                int contPretos_confirmacao_fim = 0;
                for (int i = 0; i < SensorCount; i++) {
                    if (sensorVePreto(i)) contPretos_confirmacao_fim++;
                }
                broadcastSerialLn("[ConfirmaFIM] Pós avanço extra, Pretos=" + String(contPretos_confirmacao_fim));

                if (contPretos_confirmacao_fim >= (SensorCount - 1)) { // Continua tudo preto?
                    ultimoNoClassificado = NO_FINAL_FIM_DO_LABIRINTO;
                    broadcastSerialLn("[ConfirmaFIM] Confirmado: É o FIM DO LABIRINTO!");
                } else {
                    // Não era o FIM, era apenas uma área preta que parecia uma intersecção.
                    // Volta para a classificação que tinha antes de suspeitar do FIM.
                    ultimoNoClassificado = classificacaoPreliminarDoNo; 
                    broadcastSerialLn("[ConfirmaFIM] NÃO era o FIM. Revertendo para classificação preliminar: " + nomeDoNo(ultimoNoClassificado));
                }
                estadoRoboAtual = EM_NO_WEB; // Vai para EM_NO_WEB para logar o nó e decidir a manobra
            }
            break;

        case PAUSADO_WEB: 
            // Nenhuma variável local inicializada aqui que cause problema
            break;
        case EM_NO_WEB:
        {
            // idNoAnterior (global) é o ID do nó de onde viemos (ex: 0 para o INICIO).
            // ultimoNoClassificado é o tipo do local físico ATUAL.
            // achouEsquerdaNoPonto1, etc., são as saídas do local físico ATUAL.

            // Para um novo local físico, usamos o próximo ID da interface como candidato.
            // Se estivermos voltando para um nó conhecido (após backtrack), idNoAtual já
            // terá sido ajustado pelo DFSManager na chamada anterior para refletir o nó pai.
            // A primeira vez que chegamos aqui após INICIO, idNoAtual ainda é o ID do INICIO (ex:0).
            // Isso precisa ser tratado: se não estamos explicitamente voltando (backtracking),
            // a parada atual é um *novo local potencial*.

            int idParaProcessarNoDFS;
            int idPaiParaDFS = idNoAnterior; // De onde viemos é o pai do nó atual, se for novo

            // Lógica para determinar se estamos descobrindo um novo nó ou revisitando um nó via backtrack:
            // Por agora, vamos assumir que, a menos que uma ação de backtrack tenha explicitamente
            // definido idNoAtual para um nó pai, qualquer parada em EM_NO_WEB após SEGUINDO_LINHA_WEB
            // é uma potencial nova descoberta.
            // Se idNoAtual ainda é o mesmo que idNoAnterior, é porque o DFS está re-processando o mesmo nó
            // (como no seu log), ou é a primeira parada após o INICIO onde idNoAtual era o INICIO e idNoAnterior também se tornou INICIO.

            // Se idNoAtual ainda é o ID do nó de ONDE SAÍMOS (idNoAnterior),
            // então este local atual é um NOVO local físico e precisa de um novo ID candidato.
            if (idNoAtual == idNoAnterior && idNoAnterior != -1) { // Checagem para o caso de primeira parada após um nó
                idParaProcessarNoDFS = idProximoNo; // Usa o próximo ID disponível da interface
            } else {
                // Se idNoAtual foi modificado por um backtrack para ser o ID do pai, ou é o INICIO pela primeira vez.
                idParaProcessarNoDFS = idNoAtual;
            }


            broadcastSerialLn("EM_NO_WEB - Candidato ID p/ DFS: " + String(idParaProcessarNoDFS) +
                            ", Pai Potencial (Vindo de): " + String(idPaiParaDFS) +
                            ", Tipo Detectado: " + nomeDoNo(ultimoNoClassificado));

            AcaoDFS acaoDecidida = dfsManager.processarNoAtual(
                idParaProcessarNoDFS,  // Passado por referência. Será atualizado pelo DFSManager com o ID real do nó processado.
                idPaiParaDFS,          // Passado por referência. Será atualizado pelo DFSManager se for uma ação de avanço.
                ultimoNoClassificado,
                achouEsquerdaNoPonto1,
                achouFrenteNoPonto2,
                achouDireitaNoPonto1,
                callbackParaCriarNoWeb,
                callbackParaCriarArestaWeb
            );

            // Após a chamada, idParaProcessarNoDFS contém o ID DFS real do nó que acabamos de analisar/registrar.
            idNoAtual = idParaProcessarNoDFS; // Atualiza o idNoAtual global do main.cpp

            // Se um novo nó foi de fato criado e utilizou o idProximoNo da interface, avançamos o contador.
            // O DFSManager agora gerencia seus próprios IDs com idProximoNoUnico e os retorna.
            // Se o idNoAtual (que foi atualizado pelo DFSManager) for >= ao idProximoNo do main,
            // significa que o DFSManager usou ou atribuiu um ID que o main ainda não usou para a interface.
            if (idNoAtual >= idProximoNo) {
                idProximoNo = idNoAtual + 1; // Garante que o próximo ID da interface seja novo
            }

            int idParaSerOAnteriorNoProximoPasso = idNoAtual;

            bool manobraFisicaOK = false;
            switch (acaoDecidida) {
                case ACAO_DFS_VIRAR_ESQUERDA:
                case ACAO_DFS_SEGUIR_FRENTE:
                case ACAO_DFS_VIRAR_DIREITA:
                    broadcastSerialLn("[EM_NO_WEB] DFS decidiu: AVANÇAR/VIRAR");
                    if (acaoDecidida == ACAO_DFS_VIRAR_ESQUERDA) manobraFisicaOK = virar_esquerda_90_preciso();
                    else if (acaoDecidida == ACAO_DFS_SEGUIR_FRENTE) manobraFisicaOK = true; // Assume que seguir em frente é só continuar
                    else if (acaoDecidida == ACAO_DFS_VIRAR_DIREITA) manobraFisicaOK = virar_direita_90_preciso();
                    
                    if (manobraFisicaOK) {
                        idNoAnterior = idParaSerOAnteriorNoProximoPasso; // O nó atual se torna o anterior para o próximo movimento
                    }
                    break;
                case ACAO_DFS_RETROCEDER_180:
                    broadcastSerialLn("[EM_NO_WEB] DFS decidiu: RETROCEDER_180");
                    manobraFisicaOK = virar_180_preciso();
                    // Para retrocesso, dfsManager.processarNoAtual já deve ter atualizado
                    // idNoAtual (para ser o ID do pai) e idNoAnterior (para ser o ID do nó de onde voltamos).
                    // A variável 'idParaSerOAnteriorNoProximoPasso' não é usada para setar idNoAnterior neste caso.
                    break;
                // ... outros cases (FIM_LABIRINTO, EXPLORACAO_CONCLUIDA, ERRO) ...
                // Copie do exemplo anterior.
                case ACAO_DFS_FIM_LABIRINTO:
                    broadcastSerialLn("FIM DO LABIRINTO ENCONTRADO PELO DFS!");
                    estadoRoboAtual = PARADO_WEB;
                    manobraFisicaOK = true;
                    break;
                case ACAO_DFS_EXPLORACAO_CONCLUIDA:
                    broadcastSerialLn("EXPLORAÇÃO DFS CONCLUÍDA!");
                    estadoRoboAtual = PARADO_WEB;
                    manobraFisicaOK = true;
                    break;
                case ACAO_DFS_ERRO:
                default:
                    broadcastSerialLn("ERRO NO DFS OU AÇÃO DESCONHECIDA!");
                    estadoRoboAtual = PARADO_WEB;
                    manobraFisicaOK = true; 
                    break;
            }

            if (manobraFisicaOK && estadoRoboAtual != PARADO_WEB) {
                estadoRoboAtual = SEGUINDO_LINHA_WEB;
            } else if (!manobraFisicaOK) {
                broadcastSerialLn("[EM_NO_WEB] ERRO na execução da manobra física. Robô PARADO.");
                estadoRoboAtual = PARADO_WEB;
            }
        }
        break;
        default:
            // Nenhuma variável local inicializada aqui que cause problema
            pararMotoresWebService(); estadoRoboAtual = PARADO_WEB;
            broadcastSerialLn("[AVISO] Estado desconhecido, robô parado.");
            break;
    }
    delay(20); 
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
