#include <Arduino.h>
#include <tipos.h>
#include "Baratinha.h" // Incluindo a classe Baratinha

void handleIniciarMapeamentoDFS();
void executar_pid_seguindo_linha_dfs();
void lerSens();
void pid_controlado_web();
void executarCalibracaoLinhaWebService();
void pararMotoresWebService();
uint16_t calcularPosicaoPID_3Sensores(uint16_t* qtr_sensorValues);
bool sensorVePreto(int sensorIndex);
bool sensorVeBranco(int sensorIndex);
TipoDePadraoSensor detectarPadraoSensores();
String nomeDoNo(TipoDeNoFinal tipo);
void classificarNoAposAvanco(TipoDePadraoSensor padraoDetectadoAntesDoAvanco);
void handleIniciarWeb();
void handleRetornarWeb();
void handlePausarWeb();
void handleContinuarWeb();
void handleRecalibrarLinhaWeb();
void handleNotFound();
bool handleFileRead(String path);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
String getContentType(String filename);
void executarCalibracaoLinhaWebService();
bool virar_direita_90_preciso();
bool virar_esquerda_90_preciso();
bool virar_180_preciso();
void callbackParaCriarNoWeb(int id, TipoDeNoFinal tipo, int idPai);
void callbackParaCriarArestaWeb(int idOrigem, int idDestino, const char* edgeLabel);
