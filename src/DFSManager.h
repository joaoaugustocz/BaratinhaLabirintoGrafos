// src/DFSManager.h
#ifndef DFSMANAGER_H
#define DFSMANAGER_H

#include <Arduino.h> 
#include "tipos.h" 
#include "Baratinha.h" 

#define MAX_NODES_DFS 50 // Defina sua constante AQUI, antes de ser usada

class DFSManager {
public:
    DFSManager();
    DFSManager(Baratinha& braRef);
    void iniciarNovaExploracao(int idNoInicial, TipoDeNoFinal tipoNoInicial, bool temFrenteInicial);
    void resetarDFS();

    AcaoDFS processarNoAtual(
        int &idNoProcessado, // ID do nó físico atual que paramos
        int &idPaiParaProximoNo, // ID do nó que será o pai do *próximo* nó a ser descoberto
        TipoDeNoFinal tipoNoDetectado,
        bool temSaidaEsquerda,
        bool temSaidaFrente,
        bool temSaidaDireita,
        // Callbacks para notificar o main.cpp sobre a necessidade de criar elementos visuais
        void (*callbackNovoNoWeb)(int id, TipoDeNoFinal tipo, int idPai),
        void (*callbackArestaWeb)(int idOrigem, int idDestino, const char* label)
    );

    int obterProximoIDParaNovoNo(); // Para que o main.cpp saiba qual ID usar

private:
    Baratinha& _bra;
    NodeInfo nosDoLabirinto[MAX_NODES_DFS];
    int contadorNosRegistrados; // Quantos nós estão em nosDoLabirinto
    int idProximoNoUnico;     // Contador interno para gerar IDs únicos para nós DFS
    // ... outros membros privados e métodos auxiliares ...
    NodeInfo* encontrarNodeInfoPeloID(int id);
    NodeInfo* registrarNovoNoInterno(int idNovo, int idPai, TipoDeNoFinal tipo, bool esq, bool fren, bool dir);
};

#endif // DFSMANAGER_H