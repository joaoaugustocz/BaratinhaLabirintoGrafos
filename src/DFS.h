// src/DFS.h
#ifndef DFS_H
#define DFS_H

#include "tipos.h"     // Para TipoDeNoFinal, AcaoDFS, NodeInfo (ajustado), DirecaoGlobal
#include "Baratinha.h" // Para interagir com o robô (obter orientação, etc.)

#define MAX_DFS_NODES 50 // Número máximo de nós que o DFS pode memorizar

// Ajuste na struct NodeInfo para o DFS (pode ficar aqui ou em tipos.h se preferir)
// Esta struct é específica para o que o DFS precisa saber sobre cada nó.
struct NoDFSInfo {
    int id = -1;
    TipoDeNoFinal tipo;
    int idPai = -1;             // ID do nó do qual este foi descoberto
    DirecaoGlobal dirVinda;     // Direção GLOBAL pela qual chegamos a ESTE nó (vindo do pai)

    // Saídas GLOBAIS disponíveis e exploradas
    bool saidaDisponivelNorte = false, exploradoNorte = false;
    bool saidaDisponivelLeste = false, exploradoLeste = false;
    bool saidaDisponivelSul = false,   exploradoSul = false;
    bool saidaDisponivelOeste = false, exploradoOeste = false;

    bool totalmenteExplorado = false;
};

class DFS {
public:
    DFS(Baratinha& robo); // Construtor recebe referência da Baratinha

    void iniciar(int idNoInicial, TipoDeNoFinal tipoInicial, 
                 bool temSaidaFisicaEsquerda, bool temSaidaFisicaFrente, bool temSaidaFisicaDireita);
    
    ResultadoAcaoDFS proximaAcao(int idNoAtualMain, TipoDeNoFinal tipoNoAtualFisico,
                                 bool temSaidaFisicaEsquerda, bool temSaidaFisicaFrente, bool temSaidaFisicaDireita);

    bool exploracaoCompleta();
    void resetar();

private:
    Baratinha& _rob; // Referência ao objeto Baratinha
    NoDFSInfo _mapa[MAX_DFS_NODES];
    int _contadorNosMapa;
    int _idProximoNoDFS; // Contador interno para novos IDs de nós no DFS

    // Pilha para retrocesso (armazena IDs dos nós no caminho atual)
    int _pilhaCaminho[MAX_DFS_NODES];
    int _topoPilha;

    NoDFSInfo* encontrarOuCriarNo(int idNoAtualMain, int idNoPaiMain, TipoDeNoFinal tipoAtual,
                                  bool temSaidaFisicaEsquerda, bool temSaidaFisicaFrente, bool temSaidaFisicaDireita);
    NoDFSInfo* getNoInfo(int id);
    void registrarSaidasGlobais(NoDFSInfo* noInfo, DirecaoGlobal orientacaoRoboNaDescoberta,
                                bool temSaidaFisicaEsquerda, bool temSaidaFisicaFrente, bool temSaidaFisicaDireita);
    DirecaoGlobal getDirecaoOposta(DirecaoGlobal dir);                             
};

#endif // DFS_H