#include <Arduino.h>
#include <FastLED.h>
#include <QTRSensors.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // Para WebSockets
#include <FS.h>               // Necessário para o sistema de arquivos
#include <LittleFS.h>         // Usando LittleFS


//------ GRAFOS ------------//

// Variáveis globais para o mapeamento simples
int idProximoNo = 0;        // Contador para gerar IDs de nós únicos
int idNoAtual = -1;         // ID do nó onde o robô está ou acabou de classificar
int idNoAnterior = -1;      // ID do nó de onde o robô veio para chegar ao nó atual
bool primeiroNo = true;     // Flag para tratar o nó inicial



//------ GRAFOS ------------//





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
    NO_FINAL_RETA_SIMPLES,      // Caso especial se quisermos logar
    NO_FINAL_FIM_DO_LABIRINTO  // <<< NOVO TIPO AQUI
};

// Variável global para armazenar o tipo de nó após confirmação
TipoDeNoFinal ultimoNoClassificado = NO_FINAL_NAO_E;



// === ESTADOS DO ROBÔ PARA CONTROLE WEB ===
enum EstadoRobo {
  PARADO_WEB,
  INICIANDO_EXPLORACAO_WEB,
  SEGUINDO_LINHA_WEB,
  
  PREPARANDO_ANALISE_NO,      // Robô parou, vai iniciar análise
  AVANCO_POSICIONAMENTO_NO,   // Primeiro avanço curto
  AVANCO_CHECA_FRENTE_NO,     // Segundo avanço para checar frente
  CLASSIFICACAO_DETALHADA_NO, // Estado para classificar após os avanços
  
  PAUSADO_WEB,
  EM_NO_WEB, 
  CALIBRANDO_LINHA_WEB
  // Remova NODE_DETECTADO_PARANDO, AVANCANDO_BUSCA_LATERAL, AVANCANDO_CHECA_FRENTE, CLASSIFICACAO_FINAL_NO se eram os antigos
};

// Variáveis Globais para a análise do nó
bool achouEsquerdaNoPonto1 = false;
bool achouDireitaNoPonto1 = false;
bool achouFrenteNoPonto2 = false; 


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


//--------------------------- Sensores Linha (Suas Definições)
QTRSensors qtr;
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];
#define tempoCalibracaoLinha 120 

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

    bool pid_frente_forte = s[S_CENTRAL_ESQUERDO] && s[S_CENTRAL_MEIO] && s[S_CENTRAL_DIREITO];
    bool pid_sem_linha_frente = sensorVeBranco(S_CENTRAL_ESQUERDO) && sensorVeBranco(S_CENTRAL_MEIO) && sensorVeBranco(S_CENTRAL_DIREITO);
    bool alguma_lateral_ativa = s[S_ESQUERDO_EXTREMO] || s[S_ESQUERDO_INTERNO] || s[S_DIREITO_EXTREMO] || s[S_DIREITO_INTERNO];

    if (pid_sem_linha_frente && contSensoresPretos <= 1) {
        return PADRAO_QUASE_TUDO_BRANCO; // Provável Beco sem Saída
    }

    if (pid_frente_forte && !alguma_lateral_ativa && abs(error) < 400) { // Erro do PID 3 sensores
        return PADRAO_LINHA_RETA; // Linha em frente clara, sem laterais, PID centrado
    }

    // Qualquer outra situação (muitos sensores pretos, laterais ativas, erro PID grande com laterais)
    // será tratada como um potencial nó que precisa de confirmação.
    if (contSensoresPretos >= (SensorCount -1) ) return PADRAO_MUITOS_SENSORES_PRETOS; // Ex: 6 ou 7
    if (alguma_lateral_ativa && pid_frente_forte) return PADRAO_AMBIGUO; // Frente + Lateral
    if (alguma_lateral_ativa && !pid_frente_forte) { // Sem frente clara, mas com lateral
        if (s[S_ESQUERDO_EXTREMO] || s[S_ESQUERDO_INTERNO]) return PADRAO_LATERAL_ESQUERDA_FORTE;
        if (s[S_DIREITO_EXTREMO] || s[S_DIREITO_INTERNO]) return PADRAO_LATERAL_DIREITA_FORTE;
    }
    if (abs(error) > 700) { // Erro grande do PID de 3 sensores, linha perdida para um lado
         if (error < 0 && (s[S_ESQUERDO_EXTREMO] || s[S_ESQUERDO_INTERNO])) return PADRAO_LATERAL_ESQUERDA_FORTE;
         if (error > 0 && (s[S_DIREITO_EXTREMO] || s[S_DIREITO_INTERNO])) return PADRAO_LATERAL_DIREITA_FORTE;
    }
    
    // Se chegou aqui, mas não é uma linha reta clara, trata como ambíguo para o processo de confirmação
    // ou pode ser que o PID ainda esteja corrigindo uma curva suave.
    // Para forçar a checagem se não for uma linha reta perfeita:
    if (! (pid_frente_forte && !alguma_lateral_ativa && abs(error) < 400) ) {
        return PADRAO_AMBIGUO;
    }

    return PADRAO_LINHA_RETA; // Default
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


bool checarEventoDeParada() {
    // 'sensorValues' é o array global atualizado por lerSens() -> qtr.readCalibrated()
    // S_CENTRAL_... e S_EXTREMO... são seus #defines para os índices dos sensores (0 a 6)

    // Condição 1: Beco Sem Saída (inspirado no 3pi: sensors[1,2,3] < 100)
    // Vamos checar se os 3 sensores centrais do PID estão vendo "branco".
    bool frenteTotalmenteBrancaPID = sensorVeBranco(S_CENTRAL_ESQUERDO) &&
                                   sensorVeBranco(S_CENTRAL_MEIO) &&
                                   sensorVeBranco(S_CENTRAL_DIREITO);
    
    // Para ser mais robusto, podemos verificar se QUASE NENHUM sensor vê preto
    int contSensoresPretos = 0;
    for (int i = 0; i < SensorCount; i++) {
        if (sensorVePreto(i)) {
            contSensoresPretos++;
        }
    }

    if (frenteTotalmenteBrancaPID && contSensoresPretos <= 1) { // Se os 3 centrais do PID estão brancos E no máximo 1 sensor no total vê preto
        broadcastSerialLn("[EventoParada] Detectado: Provável BECO SEM SAÍDA (centrais PID brancos, <=1 preto total)");
        return true; // Deve parar
    }

    // Condição 2: Intersecção/Ramificação Lateral (inspirado no 3pi: sensors[0] > 200 || sensors[4] > 200)
    // Se o sensor MAIS EXTERNO da esquerda OU o MAIS EXTERNO da direita veem preto.
    // Usamos S_ESQUERDO_EXTREMO (índice 0) e S_DIREITO_EXTREMO (índice 6)
    if (sensorVePreto(S_ESQUERDO_EXTREMO) || sensorVePreto(S_DIREITO_EXTREMO)) {
        // Para evitar paradas em curvas muito suaves onde um extremo pode tocar a linha brevemente
        // enquanto o PID ainda está corrigindo, podemos adicionar uma condição de que o erro do PID
        // não seja extremo ou que os sensores centrais ainda vejam um pouco da linha.
        // Por enquanto, vamos manter simples como o 3pi: qualquer extremo ativo para.
        // Depois podemos refinar se ele parar demais em curvas normais.
        broadcastSerialLn("[EventoParada] Detectado: Provável INTERSECÇÃO/RAMIFICAÇÃO (sensor extremo ativo)");
        return true; // Deve parar
    }

    return false; // Continuar seguindo a linha
}

void pid_controlado_web() {
    if (estadoRoboAtual != SEGUINDO_LINHA_WEB) return;

    lerSens(); // Atualiza sensorValues e posicaoPID_3sensores

    // Calcula o erro para o PID de 3 sensores
    second_lastError = lastError;
    lastError = error;
    error = posicaoPID_3sensores - setPoint_PID_3sensores; 

    // --- VERIFICA EVENTO DE PARADA ---
    if (checarEventoDeParada()) {
        pararMotoresWebService();
        I_pid = 0; // Reseta o integral do PID
        estadoRoboAtual = PREPARANDO_ANALISE_NO; // Muda para o novo estado
        // Não classificamos o nó final aqui ainda, isso virá depois da análise
        return; 
    }
    // --- FIM DA VERIFICAÇÃO DE EVENTO DE PARADA ---
    
    // Se não houve evento de parada, continua com o PID normal para seguir linha:
    I_pid = I_pid + error;
    I_pid = constrain(I_pid, -5000, 5000); 
    
    int motorSpeedCorrection = current_KP * error + current_KD * (error - lastError) + current_KI * I_pid;
    
    int m1Speed_web, m2Speed_web; // Direito, Esquerdo
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
            // Nenhuma variável local inicializada aqui que cause problema
            broadcastSerialLn("WEB: Iniciando Exploração -> Seguindo Linha");
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
            broadcastSerialLn("[Classifica] Dados para classificar: EsqP1=" + String(achouEsquerdaNoPonto1) + 
                                                                " | DirP1=" + String(achouDireitaNoPonto1) + 
                                                                " | FreP2=" + String(achouFrenteNoPonto2) );


            if (achouFrenteNoPonto2) {
                if (achouEsquerdaNoPonto1 && achouDireitaNoPonto1) ultimoNoClassificado = NO_FINAL_CRUZAMENTO;
                else if (achouEsquerdaNoPonto1) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_ESQ;
                else if (achouDireitaNoPonto1) ultimoNoClassificado = NO_FINAL_T_COM_FRENTE_DIR;
                else ultimoNoClassificado = NO_FINAL_RETA_SIMPLES; // Linha reta (inesperado aqui, mas é uma saída)
            } else { // Sem caminho em frente claro após o segundo avanço
                if (achouEsquerdaNoPonto1 && achouDireitaNoPonto1) ultimoNoClassificado = NO_FINAL_T_SEM_FRENTE; // "Pé" do T
                else if (achouEsquerdaNoPonto1) ultimoNoClassificado = NO_FINAL_CURVA_90_ESQ;
                else if (achouDireitaNoPonto1) ultimoNoClassificado = NO_FINAL_CURVA_90_DIR;
                else ultimoNoClassificado = NO_FINAL_BECO_SEM_SAIDA; 
            }
            broadcastSerialLn("LeituraSensores: \n s3: " + String(sensorValues[2]) + " s4: " + sensorValues[3] + " s5: " + sensorValues[4]);
            broadcastSerialLn("Decisão Final do Nó: " + nomeDoNo(ultimoNoClassificado));
            estadoRoboAtual = EM_NO_WEB;
            break;

        case PAUSADO_WEB: 
            // Nenhuma variável local inicializada aqui que cause problema
            break;
         case EM_NO_WEB:
            {
            // A mensagem "Decisão Final do Nó: ..." já foi impressa.
            // Este é o ponto onde o robô PAROU e CLASSIFICOU um nó.

            // 1. ATUALIZAR INFORMAÇÕES DO NÓ ATUAL E ANTERIOR
            // Só registra se o nó for "real" e não um erro ou simples reta que não deveria ter parado aqui.
            // E também não registra se for o FIM, pois a exploração para.
            if (ultimoNoClassificado != NO_FINAL_NAO_E && 
                ultimoNoClassificado != NO_FINAL_RETA_SIMPLES &&
                ultimoNoClassificado != NO_FINAL_FIM_DO_LABIRINTO) { // Não registra o "FIM" como um nó do qual se parte
                
                idNoAnterior = idNoAtual; 
                idNoAtual = idProximoNo++;  

                broadcastSerialLn("[Grafo] Nó Descoberto! ID Atual: " + String(idNoAtual) + 
                                  ", Tipo: " + nomeDoNo(ultimoNoClassificado) + 
                                  ". Veio do Nó ID: " + String(idNoAnterior));

                String msgNode = "newNode:" + String(idNoAtual) + ":" + nomeDoNo(ultimoNoClassificado) + "_ID" + String(idNoAtual);
                webSocketServer.broadcastTXT(msgNode);

                if (!primeiroNo && idNoAnterior != -1) { 
                    String msgEdge = "newEdge:" + String(idNoAnterior) + ":" + String(idNoAtual) + ":Segue";
                    webSocketServer.broadcastTXT(msgEdge);
                }
                primeiroNo = false; 
            } else if (ultimoNoClassificado == NO_FINAL_FIM_DO_LABIRINTO && primeiroNo) {
                // Caso especial: Se o PRIMEIRO nó encontrado já é o fim.
                idNoAnterior = -1; // Não há nó anterior válido para a aresta inicial
                idNoAtual = idProximoNo++;
                broadcastSerialLn("[Grafo] Nó Descoberto! ID Atual: " + String(idNoAtual) + 
                                  ", Tipo: " + nomeDoNo(ultimoNoClassificado) + 
                                  ". (É o FIM e o primeiro nó)");
                String msgNode = "newNode:" + String(idNoAtual) + ":" + nomeDoNo(ultimoNoClassificado) + "_ID" + String(idNoAtual);
                webSocketServer.broadcastTXT(msgNode);
                primeiroNo = false; 
            } else if (ultimoNoClassificado == NO_FINAL_FIM_DO_LABIRINTO && idNoAnterior != -1) {
                 // Se o FIM é encontrado e não é o primeiro nó (ou seja, veio de algum lugar)
                 // Registra o nó do FIM e a aresta até ele.
                idNoAnterior = idNoAtual; // O nó que ele estava antes de chegar ao FIM
                idNoAtual = idProximoNo++;  // ID para o nó FIM
                broadcastSerialLn("[Grafo] Nó Descoberto! ID Atual: " + String(idNoAtual) + 
                                  ", Tipo: " + nomeDoNo(ultimoNoClassificado) + 
                                  ". Veio do Nó ID: " + String(idNoAnterior));
                String msgNode = "newNode:" + String(idNoAtual) + ":" + nomeDoNo(ultimoNoClassificado) + "_ID" + String(idNoAtual);
                webSocketServer.broadcastTXT(msgNode);
                String msgEdge = "newEdge:" + String(idNoAnterior) + ":" + String(idNoAtual) + ":ChegouAoFim";
                webSocketServer.broadcastTXT(msgEdge);
                primeiroNo = false;
            }

            // 3. LÓGICA DE DECISÃO DE EXPLORAÇÃO (DFS MUITO SIMPLES - REGRA DA MÃO ESQUERDA)
            // Prioridade: Esquerda, Frente, Direita, Ré.
            // Isso é uma simplificação. Um DFS real precisaria marcar caminhos já explorados A PARTIR deste nó.
            bool manobraExecutada = false;
            
             // --- ADICIONAR CHECAGEM PRIORITÁRIA PARA FIM_DO_LABIRINTO ---
            if (ultimoNoClassificado == NO_FINAL_FIM_DO_LABIRINTO) {
                broadcastSerialLn("[AcaoNo] FIM DO LABIRINTO ALCANÇADO! Exploração interrompida.");
                pararMotoresWebService(); 
                // Não executa manobra de virada, não volta a seguir linha.
                // O robô pode mudar para PARADO_WEB ou um estado específico de "FIM_ENCONTRADO".
                // Para o Dijkstra depois, este 'idNoAtual' será o nó de destino.
                estadoRoboAtual = PARADO_WEB; // Para a exploração.
                manobraExecutada = true; // Consideramos que a "ação" (parar) foi concluída.
            }
            // --- FIM DA CHECAGEM DE FIM_DO_LABIRINTO ---
            else if (ultimoNoClassificado == NO_FINAL_BECO_SEM_SAIDA) {
                broadcastSerialLn("[DFS] Beco sem saída. Dando meia volta...");
                manobraExecutada = virar_180_preciso();
            } 
            else if (ultimoNoClassificado == NO_FINAL_CURVA_90_ESQ ||
                       ultimoNoClassificado == NO_FINAL_T_COM_FRENTE_ESQ ||
                       ultimoNoClassificado == NO_FINAL_T_SEM_FRENTE || 
                       ultimoNoClassificado == NO_FINAL_CRUZAMENTO) {
                // Lógica de priorizar esquerda para DFS simplificado
                broadcastSerialLn("[DFS] Opção à Esquerda disponível/priorizada. Virando à ESQUERDA.");
                manobraExecutada = virar_esquerda_90_preciso();
            } 
            else if (ultimoNoClassificado == NO_FINAL_T_COM_FRENTE_DIR) { 
                // Se não caiu na prioridade da esquerda, e é um T com frente e direita, testa direita
                broadcastSerialLn("[DFS] Opção Frente e Direita. TESTE: Virando à DIREITA.");
                manobraExecutada = virar_direita_90_preciso();
            } 
            else if (ultimoNoClassificado == NO_FINAL_RETA_SIMPLES) {  
                broadcastSerialLn("[DFS] Reta simples detectada pós-confirmação. Seguindo em FRENTE.");
                motor('a', 'f', VELOCIDADE_AJUSTE_FINAL); delay(TEMPO_AJUSTE_FINAL_MS); pararMotoresWebService();
                manobraExecutada = true; 
            } 
            else if (ultimoNoClassificado == NO_FINAL_CURVA_90_DIR) {
                broadcastSerialLn("[DFS] Curva de 90 Direita. Virando à DIREITA.");
                manobraExecutada = virar_direita_90_preciso();
            }
            else { // NO_FINAL_NAO_E ou default não tratado explicitamente acima
                broadcastSerialLn("[DFS] Nenhuma regra de exploração clara para " + nomeDoNo(ultimoNoClassificado) + ". Parando ou voltando.");
                // Poderia ser uma meia volta como fallback se for um tipo de nó inesperado
                // ou se for NO_FINAL_NAO_E (que não deveria acontecer se classificarNoAposAvanco for robusto)
                if (ultimoNoClassificado == NO_FINAL_NAO_E) {
                    manobraExecutada = false; // Indica falha em classificar, leva à parada
                } else {
                    manobraExecutada = virar_180_preciso(); // Fallback para outros tipos de nós não listados
                }
            }

            // 4. DECIDIR PRÓXIMO ESTADO GERAL
            if (manobraExecutada) {
                // Se o estado já foi definido para PARADO_WEB (como no caso do FIM_DO_LABIRINTO), não muda.
                if (estadoRoboAtual != PARADO_WEB) { 
                    broadcastSerialLn("[DFS] Manobra de exploração ("+ nomeDoNo(ultimoNoClassificado) +") bem-sucedida. Retomando seguimento de linha...");
                    estadoRoboAtual = INICIANDO_EXPLORACAO_WEB; 
                }
            } else {
                broadcastSerialLn("[DFS] ERRO na manobra ("+ nomeDoNo(ultimoNoClassificado) +") ou manobra não definida. Robô PARADO.");
                estadoRoboAtual = PARADO_WEB; 
            }
        } // Fim do escopo do case EM_NO_WEB
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
