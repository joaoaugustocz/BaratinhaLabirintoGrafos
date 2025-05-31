// src/DFSManager.cpp
#include "DFSManager.h"
#include "Baratinha.h"

// Supondo que broadcastSerialLn está em main.cpp e você quer usá-la para debug aqui
// Para fazer isso corretamente, você precisaria de uma declaração extern ou passar uma função de log.
// Por enquanto, vamos usar broadcastSerialLn para simplicidade nos stubs.
DFSManager::DFSManager(Baratinha& braRef) : _bra(braRef) {
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
    //_bra.bcSerialln("[DFSManager] DFS Resetado.");
}

void DFSManager::iniciarNovaExploracao(int idNoInicial, TipoDeNoFinal tipoNoInicial, bool temFrenteInicial) {
    _bra.bcSerialln(String("[DFSManager] Iniciando nova exploração. Nó inicial ID: ") + idNoInicial);
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
        _bra.bcSerialln("[DFSManager] ERRO: Sem slots para novos nós!");
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

    _bra.bcSerialln(String("[DFSManager] Nó Registrado/Atualizado ID: ") + novo->id + ", Tipo: " + (int)novo->tipoNo + ", Pai: " + novo->idPaiDFS);
    return novo;
}


// src/DFSManager.cpp
#include "DFSManager.h" // Assegure que todas as definições necessárias estão aqui ou em tipos.h

// Supondo que você tem uma função para converter TipoDeNoFinal para String para os logs,
// similar à nomeDoNo() do main.cpp, ou use Serial.print((int)tipoNo) para debug.
// Para os logs do DFSManager, vou usar (int)tipoNo para simplificar.
// Se broadcastSerialLn está em main.cpp, use _bra.bcSerialln para logs internos do DFSManager,
// ou configure uma forma de chamar broadcastSerialLn (ex: extern ou callback de log).

// ... (outros métodos do DFSManager como construtor, resetarDFS, iniciarNovaExploracao,
//      encontrarNodeInfoPeloID, registrarNovoNoInterno devem estar implementados) ...

AcaoDFS DFSManager::processarNoAtual(
    int &idNoProcessado, // IN/OUT: ID do nó que o main.cpp acha que está. Será o ID DFS real.
    int &idPaiDoProximoNo, // IN: Pai se idNoProcessado for novo. OUT: Nó de origem se retroceder.
    TipoDeNoFinal tipoNoDetectado_param, // Tipo físico detectado AGORA
    bool temSaidaEsquerda_param,      // Saída física detectada AGORA
    bool temSaidaFrente_param,        // Saída física detectada AGORA
    bool temSaidaDireita_param,       // Saída física detectada AGORA
    // Callbacks para notificar o main.cpp sobre a necessidade de criar elementos visuais
    void (*callbackNovoNoWeb)(int id, TipoDeNoFinal tipo, int idPai),
    void (*callbackArestaWeb)(int idOrigem, int idDestino, const char* label)
) {
    _bra.bcSerialln(String("[DFSManager::processarNoAtual] INICIO - ID Candidato/Atual Recebido: ") + idNoProcessado +
                   ", ID Pai Recebido (de onde vim): " + idPaiDoProximoNo +
                   ", Tipo Detectado Fisicamente: " + (int)tipoNoDetectado_param);

    NodeInfo* infoNodeParaTrabalhar = encontrarNodeInfoPeloID(idNoProcessado);

    bool foiNovoNoDescoberto = false;

    if (!infoNodeParaTrabalhar) {
        // É um novo local físico para o DFS, ou o ID passado pelo main não corresponde a um nó DFS existente.
        // Vamos registrar um novo nó DFS. O idPaiDoProximoNo (que é o main::idNoAnterior) é o pai deste novo nó.
        int novoNodeIDGeradoPeloDFS = idProximoNoUnico; // Usa o contador interno do DFSManager

        infoNodeParaTrabalhar = registrarNovoNoInterno(
            novoNodeIDGeradoPeloDFS,
            idPaiDoProximoNo, // O nó de onde viemos é o pai deste novo nó
            tipoNoDetectado_param,  // Usa o tipo recém-detectado para o novo nó
            temSaidaEsquerda_param, // Usa as saídas recém-detectadas
            temSaidaFrente_param,
            temSaidaDireita_param
        );

        if (!infoNodeParaTrabalhar) {
            _bra.bcSerialln("[DFSManager::processarNoAtual] ERRO: Falha ao registrar novo nó!");
            return ACAO_DFS_ERRO;
        }

        idNoProcessado = novoNodeIDGeradoPeloDFS; // ATUALIZA o ID no main.cpp para o ID real do DFS
        idProximoNoUnico++;                      // Incrementa o contador interno do DFSManager
        foiNovoNoDescoberto = true;

        _bra.bcSerialln(String("[DFSManager::processarNoAtual] Novo nó DFS registrado. ID DFS: ") + infoNodeParaTrabalhar->id +
                       ", Pai DFS: " + infoNodeParaTrabalhar->idPaiDFS +
                       ", Tipo: " + (int)infoNodeParaTrabalhar->tipoNo);

        // Notifica main.cpp para criar o nó e a aresta na interface web
        if (callbackNovoNoWeb) {
            callbackNovoNoWeb(infoNodeParaTrabalhar->id, infoNodeParaTrabalhar->tipoNo, infoNodeParaTrabalhar->idPaiDFS);
        }
        // A aresta é do pai (idPaiDoProximoNo, que era main::idNoAnterior) para este novo nó
        if (callbackArestaWeb && infoNodeParaTrabalhar->idPaiDFS != -1) {
            callbackArestaWeb(infoNodeParaTrabalhar->idPaiDFS, infoNodeParaTrabalhar->id, "SegueDFS");
        }

    } else {
        // Estamos revisitando um nó DFS existente (idNoProcessado já era um ID DFS válido).
        // Isso geralmente acontece após um backtrack, onde idNoProcessado foi definido para o ID do pai.
        _bra.bcSerialln(String("[DFSManager::processarNoAtual] Revisitando nó DFS existente. ID: ") + infoNodeParaTrabalhar->id +
                       ", Tipo Armazenado: " + (int)infoNodeParaTrabalhar->tipoNo +
                       ", Pai DFS: " + infoNodeParaTrabalhar->idPaiDFS);
        // Para um nó revisitado, usamos suas informações ARMAZENADAS de tipo e saídas,
        // não as que foram detectadas fisicamente agora (temSaidaEsquerda_param, etc.),
        // pois a detecção ao retornar pode ser menos precisa.
    }

    // --- Lógica de Decisão DFS baseada em infoNodeParaTrabalhar (seja novo ou existente) ---


    // <<< ADICIONE ESTES LOGS DE DEPURAÇÃO AQUI >>>
    if (infoNodeParaTrabalhar) { // Garante que infoNodeParaTrabalhar não é nulo
        String logMsg = "[DFSManager DEBUG] Processando Nó ID: " + String(infoNodeParaTrabalhar->id);
        logMsg += " | Tipo Armazenado: " + String((int)infoNodeParaTrabalhar->tipoNo);
        logMsg += " | Pai: " + String(infoNodeParaTrabalhar->idPaiDFS);
        logMsg += " | Saidas (D/E/F): ";
        logMsg += String(infoNodeParaTrabalhar->saidaDisponivelEsquerda) + "/";
        logMsg += String(infoNodeParaTrabalhar->saidaDisponivelFrente) + "/";
        logMsg += String(infoNodeParaTrabalhar->saidaDisponivelDireita);
        logMsg += " | Explorado (E/F/D): ";
        logMsg += String(infoNodeParaTrabalhar->exploradoEsquerda) + "/";
        logMsg += String(infoNodeParaTrabalhar->exploradoFrente) + "/";
        logMsg += String(infoNodeParaTrabalhar->exploradoDireita);
        logMsg += " | Totalmente Explorado: " + String(infoNodeParaTrabalhar->totalmenteExplorado);
        _bra.bcSerialln(logMsg); // Ou _bra.bcSerialln(logMsg);
    } else {
        _bra.bcSerialln("[DFSManager DEBUG] ERRO: infoNodeParaTrabalhar é NULO antes da decisão!");
        return ACAO_DFS_ERRO;
    }
    // <<< FIM DOS LOGS DE DEPURAÇÃO >>>



    // Se o tipo ARMAZENADO do nó é um ponto terminal para exploração A PARTIR DELE:
    if (infoNodeParaTrabalhar->tipoNo == NO_FINAL_BECO_SEM_SAIDA ||
        infoNodeParaTrabalhar->tipoNo == NO_FINAL_FIM_DO_LABIRINTO) { 

        infoNodeParaTrabalhar->totalmenteExplorado = true;
        _bra.bcSerialln(String("[DFSManager::processarNoAtual] Nó ID: ") + infoNodeParaTrabalhar->id +
                       " é BECO ou FIM. Marcado para retroceder (totalmente explorado).");
    } else {
        // Se NÃO FOR BECO NEM FIM, tenta explorar saídas
        // Adicione este log extra para depuração IMEDIATAMENTE ANTES DOS IFs de decisão de caminho
        _bra.bcSerialln(String("[DFS CondCheck] Nó:") + infoNodeParaTrabalhar->id +
                       " sE:" + infoNodeParaTrabalhar->saidaDisponivelEsquerda + ",!eE:" + !infoNodeParaTrabalhar->exploradoEsquerda +
                       " sF:" + infoNodeParaTrabalhar->saidaDisponivelFrente + ",!eF:" + !infoNodeParaTrabalhar->exploradoFrente +
                       " sD:" + infoNodeParaTrabalhar->saidaDisponivelDireita + ",!eD:" + !infoNodeParaTrabalhar->exploradoDireita);

        if (infoNodeParaTrabalhar->saidaDisponivelEsquerda && !infoNodeParaTrabalhar->exploradoEsquerda) {
            infoNodeParaTrabalhar->exploradoEsquerda = true;
            // idPaiDoProximoNo = infoNodeParaTrabalhar->id; // Removido conforme sua última alteração
            _bra.bcSerialln(String("[DFSManager::processarNoAtual] Nó ID: ") + infoNodeParaTrabalhar->id + " Decisão: ESQUERDA.");
            return ACAO_DFS_VIRAR_ESQUERDA; // <<< ESSENCIAL
        }
        if (infoNodeParaTrabalhar->saidaDisponivelFrente && !infoNodeParaTrabalhar->exploradoFrente) {
            infoNodeParaTrabalhar->exploradoFrente = true;
            // idPaiDoProximoNo = infoNodeParaTrabalhar->id; // Removido
            _bra.bcSerialln(String("[DFSManager::processarNoAtual] Nó ID: ") + infoNodeParaTrabalhar->id + " Decisão: FRENTE.");
            return ACAO_DFS_SEGUIR_FRENTE; // <<< ESSENCIAL
        }
        if (infoNodeParaTrabalhar->saidaDisponivelDireita && !infoNodeParaTrabalhar->exploradoDireita) {
            infoNodeParaTrabalhar->exploradoDireita = true;
            // idPaiDoProximoNo = infoNodeParaTrabalhar->id; // Removido
            _bra.bcSerialln(String("[DFSManager::processarNoAtual] Nó ID: ") + infoNodeParaTrabalhar->id + " Decisão: DIREITA.");
            return ACAO_DFS_VIRAR_DIREITA; // <<< ESSENCIAL
        }
    }

    // Se chegou aqui: ou era BECO/FIM, ou todas as saídas foram exploradas.
    infoNodeParaTrabalhar->totalmenteExplorado = true; 
    _bra.bcSerialln(String("[DFSManager::processarNoAtual] Nó ID: ") + infoNodeParaTrabalhar->id + " agora é considerado totalmente explorado (caiu para backtrack).");

    if (infoNodeParaTrabalhar->idPaiDFS != -1) { // Se tem um pai para onde retroceder
        _bra.bcSerialln(String("[DFSManager::processarNoAtual] Retrocedendo do Nó DFS ID: ") + infoNodeParaTrabalhar->id +
                       " para o Pai DFS ID: " + infoNodeParaTrabalhar->idPaiDFS);
        idNoProcessado = infoNodeParaTrabalhar->idPaiDFS;     // Atualiza ref para main::idNoAtual
        idPaiDoProximoNo = infoNodeParaTrabalhar->id;      // Atualiza ref para main::idNoAnterior
        return ACAO_DFS_RETROCEDER_180;
    } else {
        // Nó raiz (INICIO) e totalmente explorado.
        _bra.bcSerialln(String("[DFSManager::processarNoAtual] Nó inicial/raiz DFS ID: ") + infoNodeParaTrabalhar->id +
                       " totalmente explorado. Fim da exploração DFS.");
        return ACAO_DFS_EXPLORACAO_CONCLUIDA;
    }
}

int DFSManager::obterProximoIDParaNovoNo() {
    _bra.bcSerialln("[DFSManager] obterProximoIDParaNovoNo chamado - ESTA FUNÇÃO PODE SER REMOVIDA SE O DFS GERENCIA INTERNAMENTE.");
    // Esta função pode não ser necessária se o DFSManager gerenciar os IDs internamente
    // e os comunicar via callbacks ou atualizando idNoProcessado.
    // Por enquanto, vamos retornar o contador interno, mas a lógica de ID precisa ser consistente.
    return idProximoNoUnico;
}