#include <Arduino.h>
#include <WiFi.h>
#include "Baratinha.h" 
#include <WebServer.h>
#include <FS.h>               
#include <LittleFS.h>        
#include "tipos.h"
#include <headers.h>
#include <set>                 // Para std::set
#include "DFS.h"       // Incluindo a classe DFS

#define wifi1
//#define wifi2

// --- WebSocket Server ---
WebSocketsServer webSocketServer = WebSocketsServer(81); // Porta 81 para WebSockets

Baratinha bra(webSocketServer);

//------ GRAFOS ------------//

// ---------- Instâncias globais ----------
DFS dfs(bra);                 // nova DFS

ResultadoAcaoDFS ultimaDecisaoDFS;   // NOVA variável global

// --- Variáveis de Controle do Mapeamento DFS ---
int idNoAtualWeb = 0;
int idNoAnteriorWeb = -1;
int idProximoNoWeb = 1;
bool primeiroNoDaExploracao = true;
std::set<int> nosCriadosVisualmente; // Para evitar criar nós duplicados na interface
TipoDePadraoSensor padraoInicialDetectadoGlobal; // Padrão que causou a parada do PID
DirecaoGlobal direcaoArestaExecutada; // Direção global da última aresta seguida

// --- Configurações de Wi-Fi ---
#ifdef wifi1
    const char* ssid = "Joao_2G";
    const char* password = "Naotemsenha";
#endif

#ifdef wifi2
    const char* ssid = "CriarCe Coordenacao";
    const char* password = "inovareempreender";
#endif

#ifdef wifi3
    const char* ssid = "CriarCe Coordenacao";
    const char* password = "inovareempreender";
#endif
// --- Servidor Web HTTP ---
WebServer httpServer(80);

// Variável global para armazenar o tipo de nó após confirmação
TipoDeNoFinal ultimoNoClassificado = NO_FINAL_NAO_E;


// Variáveis Globais para a análise do nó
bool achouEsquerdaNoPonto1 = false;
bool achouDireitaNoPonto1 = false;
bool achouFrenteNoPonto2 = false; 


TipoDePadraoSensor padraoInicialDetectado;

EstadoRobo estadoRobo = PRINC_PARADO;
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


//--------------------------- PID (Suas Constantes e Variáveis Globais)
int posicao = 0; 

#define KP_3 0.1
#define KI_3 0.000//0.001//0.0003
#define KD_3 0.0//1.75
#define M1_BASE_3 40 
#define M2_BASE_3 40  
#define Mm1_RETA_3 40 
#define Mm2_RETA_3 40 
#define MMAX_CURVA_3 70
#define MMAX2_REVERSO_3 -30


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
const int TEMPO_MINIMO_PARA_SAIR_DA_LINHA_MS_180 = 600;
const int TIMEOUT_VIRADA_90_MS = 2500;       // Timeout máximo para virada de 90 graus (2.5 segundos)
const int TIMEOUT_VIRADA_180_MS = 4000;      // Timeout máximo para virada de 180 graus (4 segundos)
const int LIMIAR_ALINHAMENTO_VIRADA = 200;   // Quão perto de setPoint_PID_3sensores (1000) para considerar alinhado (faixa +/- 200)

const int VELOCIDADE_AJUSTE_FINAL = 30;     // Velocidade para pequeno avanço/recuo após virada
const int TEMPO_AJUSTE_FINAL_MS = 70;       // Duração do pequeno avanço/recuo

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
    LEDS.setBrightness(100); bra.setCorLEDs('a', 0,0,0); FastLED.show();

    // --- Inicializar LittleFS ---
    if(!LittleFS.begin(true)){ // true formata se não conseguir montar
        Serial.println("Falha ao montar LittleFS!");
        // Não podemos usar broadcastSerialLn aqui ainda se o WebSocket não estiver pronto
        // Mas o LED pode indicar o erro
        bra.setCorLEDs('a', 0, 255, 255); // Cor de erro crítico (ex: ciano)
        FastLED.show();
        while(1) delay(1000); // Trava aqui
    }
    Serial.println("LittleFS montado com sucesso.");

    Serial.print("Conectando a "); Serial.println(ssid);
    WiFi.begin(ssid, password);
    int wifi_try_count = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_try_count < 20) {
        delay(500); Serial.print("."); 
        bra.setCorLEDs('a', 60, 255, 50); // Amarelo piscando para indicar tentativa de conexão
        FastLED.show();
        delay(100);
        bra.setCorLEDs('a', 0, 0, 0);
        FastLED.show();
        wifi_try_count++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi conectado!"); Serial.print("IP: "); Serial.println(WiFi.localIP());
        bra.setCorLEDs('a', 120, 255, 100); FastLED.show();
    } else {
        Serial.println("\nFalha ao conectar WiFi."); bra.setCorLEDs('a', 0, 255, 100); FastLED.show();
    }

    //httpServer.on("/", HTTP_GET, handleRoot);
    
    // httpServer.on("/iniciar", HTTP_GET, handleIniciarWeb); 
    httpServer.on("/retornar", HTTP_GET, handleRetornarWeb);
    httpServer.on("/pausar", HTTP_GET, handlePausarWeb);
    httpServer.on("/continuar", HTTP_GET, handleContinuarWeb);
    httpServer.on("/recalibrar_linha", HTTP_GET, handleRecalibrarLinhaWeb);
    httpServer.on("/iniciar", HTTP_GET, handleIniciarMapeamentoDFS); 
    //httpServer.on("/iniciar_dfs", HTTP_GET, handleIniciarMapeamentoDFS);
    httpServer.onNotFound([]() {
        if (!handleFileRead(httpServer.uri())) {
            // Se handleFileRead retornar false, o arquivo realmente não foi encontrado ou não pôde ser aberto.
            httpServer.send(404, "text/plain", "Recurso Nao Encontrado pelo Servidor ESP32");
            bra.bcSerialln("[HTTP] Resposta 404 enviada para: " + httpServer.uri());
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


    Serial.println("Fim setup. Robô pronto em estado PARADO_WEB.");
    bra.bcSerialln("Fim setup. Robô pronto em estado PARADO_WEB.");
    estadoRobo = PRINC_PARADO;
    bra.setCorLEDs('a', 200, 255, 100); FastLED.show(); delay(500); bra.setCorLEDs('a', 0,0,0); FastLED.show();
    bra.bcSerialln("Robô Bartinha Finalizou o Setup!"); // Exemplo de uso
}

// --- LOOP PRINCIPAL ---
void loop()
{
    /*— serviços de rede sempre rodando —*/
    httpServer.handleClient();
    webSocketServer.loop();

    switch (estadoRobo)
    {
        /*----------------------------------------------------------*/
        case PRINC_PARADO:
            bra.pararMotores();             // quieto até receber /iniciar
            break;

        /*----------------------------------------------------------*/
        case PRINC_SEGUINDO_LINHA_DFS:
            /*  PID continua guiando sobre a linha.
                Quando ele encontrar um padrão ≠ LINHA_RETA,
                a própria pid_controlado_web() deve trocar
                estadoRobo → PRINC_PROCESSANDO_NO_DFS
                e gravar padraoInicialDetectadoGlobal           */
            pid_controlado_web();
            break;

        /*----------------------------------------------------------*/
        case PRINC_PROCESSANDO_NO_DFS:
        {
            bra.pararMotores();   // garante que o robô esteja parado
            bra.bcSerialln("[DFS] Processando nó " + String(idNoAtualWeb));

            /* 1 ─ Identificar o nó físico onde o robô parou */
            ResultadoIdentificacaoBaratinha resNo =
                bra.identificarTipoDeNo(padraoInicialDetectadoGlobal);
            TipoDeNoFinal tipoNoFisico = resNo.tipo;

            /* 2 ─ Se for o primeiro nó descoberto, dispare a DFS */
            if (primeiroNoDaExploracao)
            {
                dfs.iniciar(
                    idNoAtualWeb,
                    resNo.temSaidaEsquerda,
                    resNo.temSaidaFrente,
                    resNo.temSaidaDireita
                    /* ainda não passamos orientação aqui— 
                    ela será sempre “NORTE” no início */
                );
                primeiroNoDaExploracao = false;
            }

            /* 3 ─ Perguntar à DFS qual é a próxima ação */
            ultimaDecisaoDFS = dfs.proximaAcao(
                idNoAtualWeb,
                resNo.temSaidaEsquerda,
                resNo.temSaidaFrente,
                resNo.temSaidaDireita,
                bra.orientacaoAtualRobo
            );

            /* 4 ─ Guarde o ID anterior (o pai visual) antes de mudar o atual */
            idNoAnteriorWeb = idNoAtualWeb;

            /* 5 ─ Atualize o idNoAtualWeb para o próximo nó:
                • Se DFS devolveu idNoDestino == -1 → é nó novo → crie um ID único
                • Senão, está voltando a um nó já existente      */

            idNoAtualWeb = ultimaDecisaoDFS.idNoDestino;
            // if (ultimaDecisaoDFS.idNoDestino == -1) {
            //     idNoAtualWeb = idProximoNoWeb++;
            // }
            // else {
            //     idNoAtualWeb = ultimaDecisaoDFS.idNoDestino;
            // }

            /* 6 ─ Agora que idNoAtualWeb está correto, CRIE O NÓ na interface se for novo */
            if (nosCriadosVisualmente.insert(idNoAtualWeb).second) {
                // Se insere com sucesso no set, significa que ainda não existia
                callbackParaCriarNoWeb(
                    idNoAtualWeb,     // ID do nó recém‐descoberto ou retornado
                    tipoNoFisico,     // Tipo físico (curva, beco, T, etc.)
                    idNoAnteriorWeb   // ID do pai visual
                );
            }

            /* 7 ─ Crie a aresta somente se estamos indo a um nó DIFERENTE do pai
                (evita “0 → 0” ou “1 → 1”) e se idNoAnteriorWeb for válido */
            if (idNoAnteriorWeb != -1 && idNoAnteriorWeb != idNoAtualWeb) {
                callbackParaCriarArestaWeb(
                    idNoAnteriorWeb,
                    idNoAtualWeb,
                    String((int)ultimaDecisaoDFS.direcaoGlobalParaSeguir).c_str()
                );
            }

            /* 8 ─ Finalmente, mude o estado para a manobra ou para concluído/erro */
            switch (ultimaDecisaoDFS.acao)
            {
                case ACAO_DFS_SEGUIR_FRENTE:
                case ACAO_DFS_VIRAR_ESQUERDA:
                case ACAO_DFS_VIRAR_DIREITA:
                case ACAO_DFS_RETROCEDER_180:
                    estadoRobo = PRINC_EXECUTANDO_MANOBRA_DFS;
                    break;

                case ACAO_DFS_EXPLORACAO_CONCLUIDA:
                    estadoRobo = PRINC_MAPEAMENTO_CONCLUIDO_DFS;
                    break;

                case ACAO_DFS_ERRO:
                    estadoRobo = PRINC_ERRO_DFS;
                    break;
            }
            break;
        }

        /*----------------------------------------------------------*/
        case PRINC_EXECUTANDO_MANOBRA_DFS:
        {
            /* Executa a manobra decidida pela DFS */
            switch (ultimaDecisaoDFS.acao)
            {
                case ACAO_DFS_SEGUIR_FRENTE:
                    // nada a virar, apenas alinhar se desejar
                    break;

                case ACAO_DFS_VIRAR_ESQUERDA:
                    bra.girar90GrausEsquerda(true);
                    break;

                case ACAO_DFS_VIRAR_DIREITA:
                    bra.girar90GrausDireita(true);
                    break;

                case ACAO_DFS_RETROCEDER_180:
                    bra.girar180Graus(true);
                    break;

                default:
                    /* FIM/ERRO não devem chegar aqui */
                    break;
            }
            /* Ajusta bússola interna */
            bra.atualizarOrientacaoAposVirada(ultimaDecisaoDFS.acao);

            /* Volta ao modo de seguir linha até o próximo nó */
            estadoRobo = PRINC_SEGUINDO_LINHA_DFS;
            break;
        }

        /*----------------------------------------------------------*/
        case PRINC_MAPEAMENTO_CONCLUIDO_DFS:
            bra.pararMotores();
            // Aqui você pode piscar LEDs ou aguardar comandos
            break;

        case PRINC_ERRO_DFS:
            bra.pararMotores();
            // Tratamento de erro: LED vermelho, log, etc.
            break;
    }
}



void lerSens() 
{
    bra.lerSensoresLinhaCalibrados(sensorValues); // Popula 'sensorValues' com todos os 7 sensores (0-1000, onde ~1000=preto)
    
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
void pararMotoresWebService() {
    bra.mover('a', 'f', 0); 
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

void pid_controlado_web()
{
    if (estadoRobo != PRINC_SEGUINDO_LINHA_DFS) return;

    lerSens();

    /* --- cálculo do erro PID --- */
    second_lastError = lastError;
    lastError = error;
    error = posicaoPID_3sensores - setPoint_PID_3sensores;

    /* pequenos atalhos para zerar erro quando está bem centralizado */
    if (bra.sensorVePreto(S_CENTRAL_MEIO) &&
        bra.sensorVePreto(S_CENTRAL_DIREITO) &&
        bra.sensorVePreto(S_CENTRAL_ESQUERDO))
        error = 0;

    if (bra.sensorVePreto(S_CENTRAL_MEIO) &&
        (bra.sensorVePreto(S_DIREITO_INTERNO) || bra.sensorVePreto(S_ESQUERDO_INTERNO)))
        error = 0;

    TipoDePadraoSensor padraoAtual = detectarPadraoSensores();

    /* ---------------- LINHA RETA ---------------- */
    if (padraoAtual == PADRAO_LINHA_RETA)
    {
        I_pid += error;
        I_pid = constrain(I_pid, -5000, 5000);

        int corr = current_KP * error
                 + current_KD * (error - lastError)
                 + current_KI * I_pid;

        int m1 = (abs(error) <= 100 ? current_Mm1_reta : current_M1_base) - corr;
        int m2 = (abs(error) <= 100 ? current_Mm2_reta : current_M2_base) + corr;

        m1 = constrain(m1, current_MMAX2_reverso, current_MMAX_curva);
        m2 = constrain(m2, current_MMAX2_reverso, current_MMAX_curva);

        bra.mover('d', m1 < 0 ? 't' : 'f', abs(m1));
        bra.mover('e', m2 < 0 ? 't' : 'f', abs(m2));
        return;                       // continua seguindo a linha
    }

    /* ------------- NÃO É LINHA RETA ------------- */
    bra.pararMotores();
    padraoInicialDetectadoGlobal = padraoAtual;
    estadoRobo = PRINC_PROCESSANDO_NO_DFS;

    I_pid = 0;
    error = lastError = second_lastError = 0;
}
void handleIniciarWeb() {
    bra.bcSerialln("Comando Web 'Iniciar Exploração'!");
    if (estadoRobo == PARADO_WEB || estadoRobo == EM_NO_WEB || estadoRobo == PAUSADO_WEB) {
        estadoRobo = PRINC_SEGUINDO_LINHA_DFS;
        exploracaoWebIniciada = true;
        error = 0; lastError = 0; second_lastError = 0; I_pid = 0;
        httpServer.send(200, "text/plain", "Robô iniciando exploração!");
    } else { httpServer.send(200, "text/plain", "Robô ocupado ou em estado incompatível."); }
}

void handleIniciarMapeamentoDFS()
{
    bra.bcSerialln("Comando 'Iniciar Mapeamento DFS'!");

    /* se quiser, teste se o robô estava realmente parado:
       if (estadoRobo != PRINC_PARADO &&
           estadoRobo != PRINC_MAPEAMENTO_CONCLUIDO_DFS &&
           estadoRobo != PRINC_ERRO_DFS)
    {
        httpServer.send(200, "text/plain",
                        "Robô ocupado ou em estado incompatível.");
        return;
    } */

    bra.pararMotores();                       // garantia

    /*── reset de variáveis do grafo/DFS ──*/
    idNoAtualWeb        = 0;
    idNoAnteriorWeb     = -1;
    idProximoNoWeb      = 1;
    primeiroNoDaExploracao = true;
    nosCriadosVisualmente.clear();
    dfs.reset();                              // <<< novo

    webSocketServer.broadcastTXT("clearGraph");

    callbackParaCriarNoWeb(0, NO_INICIO, -1);
    nosCriadosVisualmente.insert(0);

    bra.orientacaoAtualRobo = NORTE;          // convenção

    /* zera PID */
    I_pid = 0;
    error = lastError = second_lastError = 0;

    /* NÃO forçamos padraoInicialDetectadoGlobal aqui.
       Ele será preenchido pela pid_controlado_web()
       quando encontrar o primeiro nó. */

    estadoRobo = PRINC_SEGUINDO_LINHA_DFS;    // começa rodando PID

    webSocketServer.broadcastTXT(
        "estadoRobo: PRINC_SEGUINDO_LINHA_DFS");

    httpServer.send(200, "text/plain",
                    "Robô iniciando mapeamento DFS!");
}

void handleRetornarWeb() {
    //Serial.println("Comando Web 'Retornar ao Início'!");
    //pararMotoresWebService(); estadoRobo = PARADO_WEB; 
    bra.bcSerialln("S1: " + String(sensorValues[S_ESQUERDO_EXTREMO]) + 
                   " S2: " + String(sensorValues[S_ESQUERDO_INTERNO]) + 
                   " S3: " + String(sensorValues[S_CENTRAL_ESQUERDO]) + 
                   " S4: " + String(sensorValues[S_CENTRAL_MEIO]) + 
                   " S5: " + String(sensorValues[S_CENTRAL_DIREITO]) + 
                   " S6: " + String(sensorValues[S_DIREITO_INTERNO]) + 
                   " S7: " + String(sensorValues[S_DIREITO_EXTREMO]));
    //httpServer.send(200, "text/plain", "Robô parando. Retorno não implementado.");
}

void handlePausarWeb() {
    if (estadoRobo == SEGUINDO_LINHA_WEB || estadoRobo == INICIANDO_EXPLORACAO_WEB) {
        pararMotoresWebService(); estadoRobo = PRINC_PARADO;
        Serial.println("Robô PAUSADO (Web).");
        httpServer.send(200, "text/plain", "Robô pausado.");
    } else { httpServer.send(200, "text/plain", "Não é possível pausar neste estado."); }
}

void handleContinuarWeb() {
    if (estadoRobo == PAUSADO_WEB) {
        estadoRobo = SEGUINDO_LINHA_WEB; 
        Serial.println("Robô CONTINUANDO (Web).");
        httpServer.send(200, "text/plain", "Robô continuando.");
    } else { httpServer.send(200, "text/plain", "Robô não estava pausado."); }
}

void handleRecalibrarLinhaWeb() {
    Serial.println("Comando Web 'Recalibrar Linha'!");
    if (estadoRobo != CALIBRANDO_LINHA_WEB) {
        estadoRobo = CALIBRANDO_LINHA_WEB; // Sinaliza para o loop ou executa direto
        // Para simplificar e dar resposta ao usuário, chamamos direto.
        // A função de calibração é bloqueante.
        executarCalibracaoLinhaWebService(); 
        // A função executarCalibracaoLinhaWebService já define estadoRobo = PARADO_WEB no final.
        httpServer.send(200, "text/plain", "Calibração de linha concluída!");
    } else { httpServer.send(200, "text/plain", "Calibração já em progresso."); }
}

void handleNotFound() { httpServer.send(404, "text/plain", "Nao Encontrado"); }

// --- Eventos do WebSocket ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            // Serial.printf("[%u] Desconectado!\n", num);
            bra.bcSerialln("[" + String(num) + "] Cliente WebSocket Desconectado!");
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocketServer.remoteIP(num);
            // Serial.printf("[%u] Conectado de %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            bra.bcSerialln("[" + String(num) + "] Cliente WebSocket Conectado: " + ip.toString());
            // Enviar uma mensagem de boas-vindas ou estado inicial se necessário
            webSocketServer.sendTXT(num, "Bem-vindo ao Log do Robô Bartinha!");
            }
            break;
        case WStype_TEXT:
            // Serial.printf("[%u] recebeu texto: %s\n", num, payload);
            // bra.bcSerialln("[" + String(num) + "] rx: " + String((char*)payload));
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
    bra.bcSerialln("[HTTP FS] handleFileRead: Req inicial: '" + path + "'");

    if (path.endsWith("/")) {
        path = "/index.html"; 
        bra.bcSerialln("[HTTP FS] Path é raiz, definido para: '" + path + "'");
    } else {
        // Garante que caminhos relativos (ex: "style.css") tenham a barra inicial
        if (!path.startsWith("/")) {
            path = "/" + path;
            bra.bcSerialln("[HTTP FS] Adicionada barra inicial, path agora: '" + path + "'");
        }
    }
    
    String contentType = getContentType(path);
    bra.bcSerialln("[HTTP FS] Tentando servir: '" + path + "' (Tipo: " + contentType + ")");

    bool fileExists = LittleFS.exists(path);
    bra.bcSerialln("[HTTP FS] LittleFS.exists(\"" + path + "\") -> " + (fileExists ? "ENCONTRADO" : "NAO ENCONTRADO"));

    if (fileExists) {
        File file = LittleFS.open(path, "r");
        if (file && !file.isDirectory()) {
            // Se o arquivo abriu corretamente:
            bra.bcSerialln("[HTTP FS] Arquivo '" + path + "' aberto. Tamanho: " + String(file.size()) + ". Enviando...");
            size_t sent = httpServer.streamFile(file, contentType);
            file.close();
            bra.bcSerialln("[HTTP FS] Enviados " + String(sent) + " bytes de '" + path + "'");
            return true; // Sucesso!
        } else {
            // Se LittleFS.open() falhou ou é um diretório
            bra.bcSerialln("[HTTP FS] ERRO: Falha ao abrir '" + path + "' ou é um diretório.");
            if (file) { // Se file é um objeto válido (ex: é um diretório), feche-o.
                file.close();
            }
            return false; // Falha
        }
    }
    
    // Se LittleFS.exists(path) retornou false
    bra.bcSerialln("[HTTP FS] ERRO FINAL: Arquivo '" + path + "' nao existe no FS.");
    return false; 
}

void executarCalibracaoLinhaWebService() 
{
    // O handler HTTP apenas muda o estado. A execução ocorre aqui no loop.
    // No entanto, como esta função é bloqueante, ela pode ser chamada diretamente pelo handler
    // se uma resposta imediata não for crítica ou se for rápida o suficiente.
    // Para esta versão, o handler vai chamar esta função diretamente.
    
    pararMotoresWebService(); 

    bra.bcSerialln("Iniciando CALIBRAÇÃO DE LINHA (Controle Web)...");
    bra.calibrarSensoresLinha();
    bra.bcSerialln("Calibração de Linha (Web) CONCLUÍDA!");

    estadoRobo = PARADO_WEB; // Define o estado após a calibração
}

void callbackParaCriarNoWeb(int id, TipoDeNoFinal tipo, int idPai) {
    String label = bra.nomeDoNo(tipo) + "_ID" + id;

    
    String msgNode = "newNode:" + String(id) + ":" + label;
    webSocketServer.broadcastTXT(msgNode);
    bra.bcSerialln("[Grafo Web] Nó Criado: ID=" + String(id) + " Label=" + label + " Pai=" + String(idPai));
}

void callbackParaCriarArestaWeb(int idOrigem, int idDestino, const char* edgeLabel) {
    String msgEdge = "newEdge:" + String(idOrigem) + ":" + String(idDestino) + ":" + String(edgeLabel);
    webSocketServer.broadcastTXT(msgEdge);
    bra.bcSerialln("[Grafo Web] Aresta Criada: De=" + String(idOrigem) + " Para=" + String(idDestino) + " Label=" + String(edgeLabel));
}
