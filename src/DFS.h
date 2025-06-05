// DFS.h  (proposta)
#ifndef DFS_H
#define DFS_H

#include "tipos.h"
#include "Baratinha.h"

#define MAX_DFS_NODES 30

enum ColorDFS : uint8_t { WHITE, GRAY, BLACK };

struct NoDFSInfo {
    int id = -1;
    int pi   = -1;           // pai (π)
    uint16_t d = 0, f = 0;   // discovery / finish
    ColorDFS color = WHITE;

    // vizinhos nas 4 direções globais (-1 se não existe)
    int viz[4] = {-1, -1, -1, -1};

    // flags de existência física descobertas (para não tentar passar na parede)
    bool saidaFisica[4] = {false,false,false,false};
};

class DFS {
public:
    explicit DFS(Baratinha& robo);

    void iniciar(int idInicial,
                 bool temE, bool temF, bool temD);

    /// Chamada cada vez que o robô PARAR num nó físico.
    ResultadoAcaoDFS proximaAcao(int idNoAtual,
                             bool temE, bool temF, bool temD,
                             DirecaoGlobal orientacaoRobo);   // + parâmetro

    bool exploracaoConcluida() const { return _pilhaVazia; }

    void reset();          // limpa pilha e vetor de nós

private:
    // ---------- dados ----------
    Baratinha& br;
    NoDFSInfo  nos[MAX_DFS_NODES];
    uint8_t    nNos   = 0;
    uint16_t   tempo  = 0;

    // pilha de vértices (índices no vetor `nos`)
    int  stackV[MAX_DFS_NODES];
    int  topoV   = -1;
    // pilha de próximas direções a tentar em cada vértice
    uint8_t stackDir[MAX_DFS_NODES];

    bool _pilhaVazia = true;

    // ---------- helpers ----------
    NoDFSInfo* getNo(int id);
    NoDFSInfo* criaNo(int id);
    void push(int idx, uint8_t proxDir);
    void pop();
    uint8_t proxDirecaoDisponivel(NoDFSInfo* no, DirecaoGlobal orient);
};

#endif
