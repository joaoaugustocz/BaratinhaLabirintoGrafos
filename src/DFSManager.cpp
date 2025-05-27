// src/DFSManager.cpp
#include "DFSManager.h"

// Supondo que broadcastSerialLn está em main.cpp e você quer usá-la para debug aqui
// Para fazer isso corretamente, você precisaria de uma declaração extern ou passar uma função de log.
// Por enquanto, vamos usar broadcastSerialLn para simplicidade nos stubs.
extern void broadcastSerialLn(const String &message);

DFSManager::DFSManager() {
    idProximoNoUnico = 0;
    resetarDFS();
}

void DFSManager::resetarDFS() {
    contadorNosRegistrados = 0;
    idProximoNoUnico = 0;
    for (int i = 0; i < MAX_NODES_DFS; ++i) {
        nosDoLabirinto[i].id = -1;
        nosDoLabirinto[i].tipoNo = NO_FINAL_NAO_E; // Supondo que NO_FINAL_NAO_E é o default
        nosDoLabirinto[i].idPaiDFS = -1;
        nosDoLabirinto[i].saidaDisponivelEsquerda = false;
        nosDoLabirinto[i].saidaDisponivelFrente = false;
        nosDoLabirinto[i].saidaDisponivelDireita = false;
        nosDoLabirinto[i].exploradoEsquerda = false;
        nosDoLabirinto[i].exploradoFrente = false;
        nosDoLabirinto[i].exploradoDireita = false;
        nosDoLabirinto[i].totalmenteExplorado = false;
    }
    broadcastSerialLn("[DFSManager] DFS Resetado.");
}

void DFSManager::iniciarNovaExploracao(int idNoInicial, TipoDeNoFinal tipoNoInicial, bool temFrenteInicial) {
    broadcastSerialLn(String("[DFSManager] Iniciando nova exploração. Nó inicial ID: ") + idNoInicial);
    resetarDFS(); // Garante que tudo está limpo

    // Registra o nó inicial no DFS
    // Assumimos que o ID 0 é o INICIO e ele já foi criado na interface pelo main.cpp
    idProximoNoUnico = idNoInicial; // Sincroniza o contador interno com o ID do nó inicial

    NodeInfo* infoNodeInicial = registrarNovoNoInterno(
        idNoInicial,
        -1, // Nó inicial não tem pai
        tipoNoInicial,
        false, // Sem saída esquerda no início (suposição)
        temFrenteInicial, // Saída frente conforme parâmetro
        false  // Sem saída direita no início (suposição)
    );
    
    if (infoNodeInicial) {
        // O nó INICIO não precisa explorar para trás, então marcamos "entradas" como exploradas.
        // A lógica de exploração se dará a partir da "frente".
    }
    idProximoNoUnico++; // Prepara para o próximo ID real do labirinto
}

NodeInfo* DFSManager::encontrarNodeInfoPeloID(int id) {
    for (int i = 0; i < contadorNosRegistrados; ++i) {
        if (nosDoLabirinto[i].id == id) {
            return &nosDoLabirinto[i];
        }
    }
    return nullptr;
}

NodeInfo* DFSManager::registrarNovoNoInterno(int idNovo, int idPai, TipoDeNoFinal tipo, bool esq, bool fren, bool dir) {
    if (contadorNosRegistrados >= MAX_NODES_DFS) {
        broadcastSerialLn("[DFSManager] ERRO: Sem slots para novos nós!");
        return nullptr;
    }
    NodeInfo* novo = &nosDoLabirinto[contadorNosRegistrados++]; // Usa e depois incrementa
    novo->id = idNovo;
    novo->tipoNo = tipo;
    novo->idPaiDFS = idPai;
    novo->saidaDisponivelEsquerda = esq;
    novo->saidaDisponivelFrente = fren;
    novo->saidaDisponivelDireita = dir;
    // Reseta flags de exploração para este novo nó
    novo->exploradoEsquerda = false;
    novo->exploradoFrente = false;
    novo->exploradoDireita = false;
    novo->totalmenteExplorado = false;

    broadcastSerialLn(String("[DFSManager] Nó Registrado/Atualizado ID: ") + novo->id + ", Tipo: " + (int)novo->tipoNo + ", Pai: " + novo->idPaiDFS);
    return novo;
}


AcaoDFS DFSManager::processarNoAtual(
    int &idNoProcessado, // ID do nó físico ATUAL que o robô parou (vindo do main)
    int &idPaiParaProximoNo, // ID do nó que será o pai do PRÓXIMO nó a ser descoberto (este é o idNoProcessado)
    TipoDeNoFinal tipoNoDetectado,
    bool temSaidaEsquerda,
    bool temSaidaFrente,
    bool temSaidaDireita,
    void (*callbackNovoNoWeb)(int id, TipoDeNoFinal tipo, int idPai),
    void (*callbackArestaWeb)(int idOrigem, int idDestino, const char* label)
) {
    broadcastSerialLn(String("[DFSManager] Processando nó atual ID (entrada): ") + idNoProcessado + ", Tipo: " + (int)tipoNoDetectado);
    NodeInfo* infoNodeAtual = encontrarNodeInfoPeloID(idNoProcessado);

    // Se o nó é desconhecido para o DFS (ou seja, estamos descobrindo um novo nó)
    if (!infoNodeAtual) {
        int novoNodeID = idProximoNoUnico; // Pega o próximo ID disponível gerenciado pelo DFSManager
        infoNodeAtual = registrarNovoNoInterno(novoNodeID, idPaiParaProximoNo, tipoNoDetectado, temSaidaEsquerda, temSaidaFrente, temSaidaDireita);
        if (!infoNodeAtual) return ACAO_DFS_ERRO;

        idNoProcessado = novoNodeID; // Atualiza o ID do nó atual para o novo ID gerado pelo DFS
        idProximoNoUnico++;          // Incrementa para o próximo

        broadcastSerialLn(String("[DFSManager] Novo nó descoberto ID: ") + idNoProcessado + ", Pai: " + idPaiParaProximoNo);

        // Notifica main.cpp para criar o nó na interface web
        // O label será construído pela função nomeDoNo em main.cpp quando o callback for chamado.
        if (callbackNovoNoWeb) {
            callbackNovoNoWeb(idNoProcessado, tipoNoDetectado, idPaiParaProximoNo);
        }
        // A aresta já deve ter sido criada pelo main.cpp antes de chamar o processarNoAtual,
        // ou o callbackArestaWeb pode ser usado aqui se o idPaiParaProximoNo for válido.
        // No modelo anterior, a aresta era criada depois que o nó era classificado e um novo ID era gerado.
        // Se idPaiParaProximoNo é o nó de onde viemos, e idNoProcessado é o novo nó:
        if (callbackArestaWeb && idPaiParaProximoNo != -1) {
             callbackArestaWeb(idPaiParaProximoNo, idNoProcessado, "SegueDFS");
        }


    } else {
        broadcastSerialLn(String("[DFSManager] Revisitando nó ID: ") + idNoProcessado);
        // Aqui, estamos retornando a um nó já conhecido (ex: após um backtrack).
        // As saídas disponíveis (temSaidaEsquerda, etc.) são as detectadas originalmente.
        // Não precisa registrar novamente, apenas usar o infoNodeAtual existente.
    }

    // --- Lógica de Decisão DFS (simplificada por enquanto) ---
    if (infoNodeAtual->tipoNo == NO_FINAL_FIM_DO_LABIRINTO) {
        broadcastSerialLn("[DFSManager] Decisão: FIM DO LABIRINTO.");
        return ACAO_DFS_FIM_LABIRINTO;
    }
    if (infoNodeAtual->tipoNo == NO_FINAL_BECO_SEM_SAIDA) {
        infoNodeAtual->totalmenteExplorado = true;
        broadcastSerialLn("[DFSManager] Decisão: BECO, retroceder.");
        // (A lógica de retrocesso atualizará idNoProcessado e idPaiParaProximoNo)
    }

    // Tentar explorar saídas não visitadas
    if (infoNodeAtual->saidaDisponivelEsquerda && !infoNodeAtual->exploradoEsquerda) {
        infoNodeAtual->exploradoEsquerda = true;
        idPaiParaProximoNo = infoNodeAtual->id; // O nó atual será o pai do próximo
        broadcastSerialLn("[DFSManager] Decisão: ESQUERDA.");
        return ACAO_DFS_VIRAR_ESQUERDA;
    }
    if (infoNodeAtual->saidaDisponivelFrente && !infoNodeAtual->exploradoFrente) {
        infoNodeAtual->exploradoFrente = true;
        idPaiParaProximoNo = infoNodeAtual->id;
        broadcastSerialLn("[DFSManager] Decisão: FRENTE.");
        return ACAO_DFS_SEGUIR_FRENTE;
    }
    if (infoNodeAtual->saidaDisponivelDireita && !infoNodeAtual->exploradoDireita) {
        infoNodeAtual->exploradoDireita = true;
        idPaiParaProximoNo = infoNodeAtual->id;
        broadcastSerialLn("[DFSManager] Decisão: DIREITA.");
        return ACAO_DFS_VIRAR_DIREITA;
    }

    // Se chegou aqui, todas as saídas foram exploradas ou é um beco
    infoNodeAtual->totalmenteExplorado = true;
    if (infoNodeAtual->idPaiDFS != -1) { // Se tem um pai para onde retroceder
        broadcastSerialLn(String("[DFSManager] Decisão: NÓ TOTALMENTE EXPLORADO, retroceder para pai ID: ") + infoNodeAtual->idPaiDFS);
        // Prepara para o retrocesso: o nó atual se torna o "anterior" para o pai.
        // O nó que estamos processando (infoNodeAtual->id) será o "de onde viemos".
        // O destino é o pai.
        idPaiParaProximoNo = infoNodeAtual->id; // Ao chegar no pai, este nó é o "anterior"
        idNoProcessado = infoNodeAtual->idPaiDFS; // O "nó atual" para a próxima iteração do EM_NO_WEB será o pai
        return ACAO_DFS_RETROCEDER_180;
    } else {
        // Nó raiz (INICIO) e totalmente explorado
        broadcastSerialLn("[DFSManager] Decisão: NÓ INICIAL TOTALMENTE EXPLORADO.");
        return ACAO_DFS_EXPLORACAO_CONCLUIDA;
    }
}

int DFSManager::obterProximoIDParaNovoNo() {
    broadcastSerialLn("[DFSManager] obterProximoIDParaNovoNo chamado - ESTA FUNÇÃO PODE SER REMOVIDA SE O DFS GERENCIA INTERNAMENTE.");
    // Esta função pode não ser necessária se o DFSManager gerenciar os IDs internamente
    // e os comunicar via callbacks ou atualizando idNoProcessado.
    // Por enquanto, vamos retornar o contador interno, mas a lógica de ID precisa ser consistente.
    return idProximoNoUnico;
}