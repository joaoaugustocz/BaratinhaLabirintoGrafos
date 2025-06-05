// DFS.cpp  (proposta resumida)
#include "DFS.h"

DFS::DFS(Baratinha& robo) : br(robo) {}

NoDFSInfo* DFS::getNo(int id) {
    for(uint8_t i=0;i<nNos;++i) if(nos[i].id==id) return &nos[i];
    return nullptr;
}
NoDFSInfo* DFS::criaNo(int id){
    if(nNos>=MAX_DFS_NODES) return nullptr;
    nos[nNos].id = id;
    return &nos[nNos++];
}

void DFS::push(int idx, uint8_t proxDir)
{ 
    stackV[++topoV] = idx; 
    stackDir[topoV]   = proxDir;
    _pilhaVazia = false; 
}
void DFS::pop()          { if (topoV >= 0) --topoV; _pilhaVazia = (topoV < 0); }

uint8_t DFS::proxDirecaoDisponivel(NoDFSInfo* no, DirecaoGlobal orientacaoRobo) {
    // 1) calculemos os índices absolutos para [frente, direita, trás, esquerda]
    uint8_t frenteAbs = (static_cast<uint8_t>(orientacaoRobo) + 0) & 0x03;
    uint8_t dirAbs    = (static_cast<uint8_t>(orientacaoRobo) + 1) & 0x03;
    uint8_t trasAbs   = (static_cast<uint8_t>(orientacaoRobo) + 2) & 0x03;
    uint8_t esqAbs    = (static_cast<uint8_t>(orientacaoRobo) + 3) & 0x03;

    // 2) Primeiro, tente “Frente” se existir saída física e não estiver mapeado
    if (no->saidaFisica[frenteAbs] && no->viz[frenteAbs] == -1) {
        return frenteAbs;
    }
    // 3) Se não há “Frente”, tente “Esquerda”
    if (no->saidaFisica[esqAbs] && no->viz[esqAbs] == -1) {
        return esqAbs;
    }
    // 4) Se nem “Frente” nem “Esquerda” estiverem livres, tente “Direita”
    if (no->saidaFisica[dirAbs] && no->viz[dirAbs] == -1) {
        return dirAbs;
    }
    // 5) Por último, tente “Trás”
    if (no->saidaFisica[trasAbs] && no->viz[trasAbs] == -1) {
        return trasAbs;
    }

    // 6) Se todos já têm “viz[abs] != -1”, mas ainda existirem vizinhos WHITE,
    //    também exploramos nessa mesma ordem relativa:
    NoDFSInfo* w;
    if (no->viz[frenteAbs] != -1) {
        w = getNo(no->viz[frenteAbs]);
        if (w && w->color == WHITE) return frenteAbs;
    }
    if (no->viz[esqAbs] != -1) {
        w = getNo(no->viz[esqAbs]);
        if (w && w->color == WHITE) return esqAbs;
    }
    if (no->viz[dirAbs] != -1) {
        w = getNo(no->viz[dirAbs]);
        if (w && w->color == WHITE) return dirAbs;
    }
    if (no->viz[trasAbs] != -1) {
        w = getNo(no->viz[trasAbs]);
        if (w && w->color == WHITE) return trasAbs;
    }

    // 7) Se nada disso estiver disponível, retorna 255
    return 255;
}

// ---------- API PRINCIPAL ----------
void DFS::iniciar(int idInicial,bool temE,bool temF,bool temD){
    nNos=0; tempo=0; topoV=-1; _pilhaVazia=true;

    NoDFSInfo* s = criaNo(idInicial);
    s->color = GRAY;                     // descoberto
    s->d     = ++tempo;
    s->saidaFisica[OESTE]=temE;
    s->saidaFisica[NORTE]=temF;          // “frente” será NORTE, pois começamos virados p/ NORTE
    s->saidaFisica[LESTE]=temD;
    push(0, 0);
}

ResultadoAcaoDFS DFS::proximaAcao(int idAtual,
                                 bool temE,
                                 bool temF,
                                 bool temD,
                                 DirecaoGlobal orientacaoRobo)
{
    // Inicializa com ERRO (caso algo dê errado) e idDestino = -1
    ResultadoAcaoDFS r{ACAO_DFS_ERRO, -1, NORTE};

    // Se a pilha já estiver vazia, significa que o DFS acabou
    if (_pilhaVazia) {
        r.acao = ACAO_DFS_EXPLORACAO_CONCLUIDA;
        return r;
    }

    //────────────────────────────────────────────────────────────────────────
    // 1) SINCRONIZAR o nó onde o robô parou (O U “atual”):
    //────────────────────────────────────────────────────────────────────────
    NoDFSInfo* u = getNo(idAtual);
    if (!u) {
        // Se ainda não existia esse nó no nosso “nos[]”, criamos agora:
        u = criaNo(idAtual);
    }

    // 1.1) Atualiza as saídas físicas “absolutas” (0=N,1=E,2=S,3=O),
    //      combinando (temF, temD, temE) com a orientação atual do robô:
    bool temRel[4] = { temF, temD, false, temE };   // 0=Frente,1=Direita,2=Trás*,3=Esquerda
    bool temAbs[4] = { false, false, false, false };

    for (uint8_t rel = 0; rel < 4; ++rel) {
        if (!temRel[rel]) continue;
        // converte “relativo → absoluto” via (orientação + deslocamento) mod 4
        uint8_t absDir = (static_cast<uint8_t>(orientacaoRobo)
                          + (rel == 0 ? 0  // Frente
                             : rel == 1 ? 1  // Direita
                             : rel == 3 ? 3  // Esquerda
                             : 2))            // Trás
                         & 0x03;
        temAbs[absDir] = true;
    }
    // “OR” nos vetores existentes (pode ser que já tivéssemos marcado alguma direção)
    for (uint8_t d = 0; d < 4; ++d) {
        u->saidaFisica[d] |= temAbs[d];
    }

    //────────────────────────────────────────────────────────────────────────
    // 2) Se viemos de outro nó, criamos a aresta “pai → u” no grafo interno
    //────────────────────────────────────────────────────────────────────────
    if (topoV >= 0) {
        // “pai” é o vértice cujo índice está no topo da pilha
        NoDFSInfo* pai = &nos[stackV[topoV]];

        // A direção da aresta pai→u (em coordenadas ABSOLUTAS) é simplesmente
        // a orientação atual do robô:
        uint8_t d = static_cast<uint8_t>(orientacaoRobo); // 0=N,1=E,2=S,3=O

        pai->viz[d]           = u->id;         // pai → u
        u->viz[(d + 2) & 0x03] = pai->id;      // u → pai (inversa)
    }

    //────────────────────────────────────────────────────────────────────────
    // 3) TENTA UMA SAÍDA NOVA: “Chamada Recursiva” do DFS
    //────────────────────────────────────────────────────────────────────────
    uint8_t dir = proxDirecaoDisponivel(u, orientacaoRobo);
    if (dir != 255) {
        // *** Ainda existe um “vizinho branco/desconhecido” nessa direção ***

        // 3.1) Preenchemos a ação do DFS de acordo com a direção relativa:
        //      (0=frente, 1=L, 2=S (180°), 3=O).
        //      Para descobrir se devemos VIRAR_ESQUERDA / VIRAR_DIREITA, etc,
        //      usamos a lógica “(dir - orientacaoRobo) mod 4”:
        uint8_t rel = (dir + 4 - static_cast<uint8_t>(orientacaoRobo)) & 0x03;
        switch (rel) {
            case 0:  r.acao = ACAO_DFS_SEGUIR_FRENTE;   break;
            case 1:  r.acao = ACAO_DFS_VIRAR_DIREITA;   break;
            case 2:  r.acao = ACAO_DFS_RETROCEDER_180;  break;
            case 3:  r.acao = ACAO_DFS_VIRAR_ESQUERDA;  break;
        }

        // 3.2) Cria um “idFilho” único (você pode manter a heurística antiga,
        //      por exemplo “idAtual * 10 + dir”, ou usar um contador próprio).
        //      O importante é que seja um valor que nunca conflite com os já usados.
        int idFilho = idAtual * 10 + dir;

        // 3.3) CRIAR efetivamente o nó-filho na estrutura interna “nos[]”:
        NoDFSInfo* v = criaNo(idFilho);
        v->pi    = u->id;
        v->color = GRAY;
        v->d     = ++tempo;

        // 3.4) Empilhar o índice DESTE FILHO ( NÃO de “u”! ), para que, no retorno,
        //      o pop() puxe exatamente ESTE v. Assim o próximo pai será (u).
        int idxFilho = (v - nos);
        push(idxFilho, dir);

        // 3.5) Devolve esse novo idFilho ao main para que ele saiba “para onde mover o robô”:
        r.idNoDestino = idFilho;
        return r;
    }

    //────────────────────────────────────────────────────────────────────────
    // 4) “Não há saída não explorada em u” ⇒ pinto u de BLACK e volto
    //────────────────────────────────────────────────────────────────────────
    u->color = BLACK;
    u->f     = ++tempo;

    // 4.1) ANTES de remover u da pilha, capturamos o índice que está no topo:
    int idxPai = stackV[topoV];
    pop();  // tira u (o nó cujo idx estava em topoV)

    if (_pilhaVazia) {
        // Não há mais nós para voltar: exploração concluída
        r.acao = ACAO_DFS_EXPLORACAO_CONCLUIDA;
    } else {
        // Ainda existe pai na pilha, então presto a “voltar” a ele:
        r.acao = ACAO_DFS_RETROCEDER_180;
        r.idNoDestino = nos[idxPai].id;

        // A direção absoluta para ir “de volta” é sempre 180°:
        r.direcaoGlobalParaSeguir =
            static_cast<DirecaoGlobal>((static_cast<uint8_t>(orientacaoRobo) + 2) & 0x03);
    }
    return r;
}


void DFS::reset()
{
    nNos = 0;
    topoV = -1;
    _pilhaVazia = true;
    tempo = 0;
}