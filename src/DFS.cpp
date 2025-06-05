// src/DFS.cpp
#include "DFS.h"

DFS::DFS(Baratinha& robo) : _rob(robo) {
    resetar();
}

void DFS::resetar() {
    _contadorNosMapa = 0;
    _idProximoNoDFS = 0; // IDs do DFS podem começar de 0
    _topoPilha = -1;
    for (int i = 0; i < MAX_DFS_NODES; ++i) {
        _mapa[i].id = -1; // Marcar como não utilizado
        // Resetar outras flags se necessário, mas registrarNovoNoInterno fará isso
    }
    _rob.orientacaoAtualRobo = NORTE; // Robô sempre começa virado para NORTE no DFS
    //_rob.bcSerialln("[DFS] Resetado.");
}

void DFS::iniciar(int idNoInicialMain, TipoDeNoFinal tipoInicial, 
                  bool temSaidaFisicaEsq, bool temSaidaFisicaFre, bool temSaidaFisicaDir) {
    resetar(); // Garante que está começando do zero
    _rob.bcSerialln(String("[DFS] Iniciando exploração. Nó inicial ID Main: ") + idNoInicialMain + 
                    ", Tipo: " + (int)tipoInicial + ", Orientação Robô: " + (int)_rob.orientacaoAtualRobo);

    // O primeiro nó é especial, seu ID DFS pode ser o mesmo que o ID do main
    _idProximoNoDFS = idNoInicialMain; 
    NoDFSInfo* noInicial = encontrarOuCriarNo(idNoInicialMain, -1, tipoInicial, 
                                             temSaidaFisicaEsq, temSaidaFisicaFre, temSaidaFisicaDir);
    
    if (noInicial) {
        // Coloca o nó inicial na pilha
        _pilhaCaminho[++_topoPilha] = noInicial->id;
        _rob.bcSerialln(String("[DFS] Nó inicial ID DFS: ") + noInicial->id + " na pilha.");
    } else {
        _rob.bcSerialln("[DFS] ERRO: Falha ao criar nó inicial no DFS!");
    }
}

NoDFSInfo* DFS::getNoInfo(int id) {
    for (int i = 0; i < _contadorNosMapa; ++i) {
        if (_mapa[i].id == id) {
            return &_mapa[i];
        }
    }
    return nullptr;
}

void DFS::registrarSaidasGlobais(NoDFSInfo* noInfo, DirecaoGlobal orientacaoRoboNaDescoberta,
                                 bool temSaidaFisicaEsquerda, bool temSaidaFisicaFrente, bool temSaidaFisicaDireita) {
    // Resetar saídas antes de definir
    noInfo->saidaDisponivelNorte = false; noInfo->saidaDisponivelLeste = false;
    noInfo->saidaDisponivelSul = false;   noInfo->saidaDisponivelOeste = false;

    if (temSaidaFisicaEsquerda) {
        DirecaoGlobal dg = (DirecaoGlobal)((orientacaoRoboNaDescoberta - 1 + 4) % 4);
        if (dg == NORTE) noInfo->saidaDisponivelNorte = true;
        else if (dg == LESTE) noInfo->saidaDisponivelLeste = true;
        else if (dg == SUL) noInfo->saidaDisponivelSul = true;
        else if (dg == OESTE) noInfo->saidaDisponivelOeste = true;
    }
    if (temSaidaFisicaFrente) {
        DirecaoGlobal dg = orientacaoRoboNaDescoberta;
        if (dg == NORTE) noInfo->saidaDisponivelNorte = true;
        else if (dg == LESTE) noInfo->saidaDisponivelLeste = true;
        else if (dg == SUL) noInfo->saidaDisponivelSul = true;
        else if (dg == OESTE) noInfo->saidaDisponivelOeste = true;
    }
    if (temSaidaFisicaDireita) {
        DirecaoGlobal dg = (DirecaoGlobal)((orientacaoRoboNaDescoberta + 1) % 4);
        if (dg == NORTE) noInfo->saidaDisponivelNorte = true;
        else if (dg == LESTE) noInfo->saidaDisponivelLeste = true;
        else if (dg == SUL) noInfo->saidaDisponivelSul = true;
        else if (dg == OESTE) noInfo->saidaDisponivelOeste = true;
    }
    // Se for um BECO ou FIM, não deveria ter saídas exploráveis
     if (noInfo->tipo == NO_FINAL_BECO_SEM_SAIDA || noInfo->tipo == NO_FINAL_FIM_DO_LABIRINTO) {
        noInfo->saidaDisponivelNorte = false; noInfo->saidaDisponivelLeste = false;
        noInfo->saidaDisponivelSul = false;   noInfo->saidaDisponivelOeste = false;
    }
}


NoDFSInfo* DFS::encontrarOuCriarNo(int idNoAtualMain, int idNoPaiMain, TipoDeNoFinal tipoAtual,
                                   bool temSaidaFisicaEsq, bool temSaidaFisicaFre, bool temSaidaFisicaDir) {
    // No nosso sistema simplificado de IDs, o idNoAtualMain pode já ser um ID DFS
    // se estivermos revisitando. Se for uma nova descoberta, o main.cpp pode passar
    // o próximo ID da interface, e o DFS pode usar esse ID se ainda não existir.

    NoDFSInfo* noExistente = getNoInfo(idNoAtualMain);
    if (noExistente) {
        _rob.bcSerialln(String("[DFS] Encontrado Nó DFS existente ID: ") + idNoAtualMain);
        return noExistente;
    }

    // Se não encontrou, é um novo nó para o DFS
    if (_contadorNosMapa >= MAX_DFS_NODES) {
        _rob.bcSerialln("[DFS] ERRO: Limite de nós do mapa DFS atingido!");
        return nullptr;
    }

    NoDFSInfo* novoNo = &_mapa[_contadorNosMapa++];
    novoNo->id = idNoAtualMain; // Usa o ID fornecido pelo main (que o main gerencia para a interface)
                                // Poderia ser _idProximoNoDFS++ se quiséssemos IDs DFS totalmente independentes.
                                // Por simplicidade, vamos usar o ID do main.
    if (idNoAtualMain >= _idProximoNoDFS) { // Garante que o contador interno do DFS acompanhe
        _idProximoNoDFS = idNoAtualMain + 1;
    }

    novoNo->tipo = tipoAtual;
    novoNo->idPai = idNoPaiMain;
    novoNo->dirVinda = _rob.orientacaoAtualRobo; // A direção que o robô estava ao chegar NESTE nó.
    
    registrarSaidasGlobais(novoNo, _rob.orientacaoAtualRobo, 
                           temSaidaFisicaEsq, temSaidaFisicaFre, temSaidaFisicaDir);

    novoNo->exploradoNorte = false; novoNo->exploradoLeste = false;
    novoNo->exploradoSul = false;   novoNo->exploradoOeste = false;
    novoNo->totalmenteExplorado = false;

    _rob.bcSerialln(String("[DFS] Novo Nó DFS Registrado ID: ") + novoNo->id + " Tipo: " + (int)novoNo->tipo +
                   " Pai: " + novoNo->idPai + " DirVinda: " + (int)novoNo->dirVinda);
    _rob.bcSerialln(String("    Saídas N/L/S/O: ") + novoNo->saidaDisponivelNorte + "/" + novoNo->saidaDisponivelLeste + "/" +
                   novoNo->saidaDisponivelSul + "/" + novoNo->saidaDisponivelOeste);
    return novoNo;
}

DirecaoGlobal DFS::getDirecaoOposta(DirecaoGlobal dir) {
    return (DirecaoGlobal)((dir + 2) % 4);
}

ResultadoAcaoDFS DFS::proximaAcao(int idNoAtualMain, TipoDeNoFinal tipoNoAtualFisico,
                                   bool temSaidaFisicaEsq, bool temSaidaFisicaFre, bool temSaidaFisicaDir) {
    ResultadoAcaoDFS resultadoDFS;
    resultadoDFS.acao = ACAO_DFS_ERRO; // Default
    resultadoDFS.idNoDestino = -1;    // Default
    resultadoDFS.direcaoGlobalParaSeguir = NORTE; // Default (ou uma direção inválida)


    
    _rob.bcSerialln(String("[DFS::proximaAcao] Processando Nó Main ID: ") + idNoAtualMain + 
                   ", Tipo Físico: " + (int)tipoNoAtualFisico + 
                   ", Orientação Robô: " + (int)_rob.orientacaoAtualRobo);

    if (_topoPilha < 0) {
        _rob.bcSerialln("[DFS] ERRO: Pilha vazia, exploração não iniciada ou concluída indevidamente.");
        resultadoDFS.acao = ACAO_DFS_EXPLORACAO_CONCLUIDA; // Ou ERRO se isso for inesperado
        return resultadoDFS;
    }
    int idNoAtualDFS = _pilhaCaminho[_topoPilha];
    NoDFSInfo* noSendoProcessado = getNoInfo(idNoAtualDFS);

    if (!noSendoProcessado) {
        _rob.bcSerialln(String("[DFS] ERRO: Nó ID ") + idNoAtualDFS + " da pilha não encontrado no mapa!");
        resultadoDFS.acao = ACAO_DFS_ERRO;
        return resultadoDFS; // Retorna erro se não encontrar o nó
    }
    
    _rob.bcSerialln(String("[DFS DEBUG] Processando Nó ID DFS: ") + noSendoProcessado->id +
                   " | Tipo: " + (int)noSendoProcessado->tipo +
                   " | Pai: " + noSendoProcessado->idPai +
                   " | DirVinda: " + (int)noSendoProcessado->dirVinda +
                   " | Saidas N/L/S/O: " + noSendoProcessado->saidaDisponivelNorte + noSendoProcessado->saidaDisponivelLeste + noSendoProcessado->saidaDisponivelSul + noSendoProcessado->saidaDisponivelOeste +
                   " | Expl N/L/S/O: " + noSendoProcessado->exploradoNorte + noSendoProcessado->exploradoLeste + noSendoProcessado->exploradoSul + noSendoProcessado->exploradoOeste +
                   " | TotalExpl: " + noSendoProcessado->totalmenteExplorado);


    if (noSendoProcessado->tipo == NO_FINAL_BECO_SEM_SAIDA || noSendoProcessado->tipo == NO_FINAL_FIM_DO_LABIRINTO) {
        noSendoProcessado->totalmenteExplorado = true;
        _rob.bcSerialln(String("[DFS] Nó ID: ") + noSendoProcessado->id + " é BECO/FIM, marcado para retroceder.");
    } else {
        DirecaoGlobal direcoesParaTentar[] = {NORTE, LESTE, SUL, OESTE}; // Ordem de preferência
        bool* pSaidas[] = { // Ponteiros para os membros corretos
            &noSendoProcessado->saidaDisponivelNorte, 
            &noSendoProcessado->saidaDisponivelLeste, 
            &noSendoProcessado->saidaDisponivelSul, 
            &noSendoProcessado->saidaDisponivelOeste
        };
        bool* pExplorado[] = { // Ponteiros para os membros corretos
            &noSendoProcessado->exploradoNorte, 
            &noSendoProcessado->exploradoLeste, 
            &noSendoProcessado->exploradoSul, 
            &noSendoProcessado->exploradoOeste
        };

        DirecaoGlobal direcaoOpostaAoPai = (noSendoProcessado->idPai != -1) ? 
                                           getDirecaoOposta(noSendoProcessado->dirVinda) : 
                                           (DirecaoGlobal) -1; // Valor inválido se não tem pai

        // Tenta explorar caminhos NOVOS primeiro
        for (int i = 0; i < 4; ++i) {
            DirecaoGlobal dirGlobalAlvo = direcoesParaTentar[i];
            if (dirGlobalAlvo == direcaoOpostaAoPai) { // Não volta para o pai ainda
                continue; 
            }

            if (*(pSaidas[i]) && !(*(pExplorado[i]))) {
                *(pExplorado[i]) = true;
                _rob.bcSerialln(String("[DFS] Nó ID: ") + noSendoProcessado->id + " decidiu explorar (NOVO CAMINHO) Global: " + (int)dirGlobalAlvo);
                // Ao avançar para um novo nó, o nó atual se torna o pai na pilha DFS
                // A chamada a encontrarOuCriarNo no main.cpp vai usar o idNoAtualMain (que é noSendoProcessado->id) como pai
                // e a orientação atual do robô para definir a dirVinda do novo nó.
                // O main.cpp atualizará sua variável idNoAnterior para noSendoProcessado->id.
                resultadoDFS.acao = _rob.getManobraParaEncarar(dirGlobalAlvo); // getManobraParaEncarar retorna AcaoDFS
                resultadoDFS.direcaoGlobalParaSeguir = dirGlobalAlvo;
                
                return resultadoDFS;
            }
        }

        // Se não encontrou caminho novo, e tem um pai, e o caminho para o pai está disponível e não explorado
        // (embora "explorado" para o caminho do pai tenha um significado diferente - é a última rota a tomar)
        // Esta lógica de "última opção de volta ao pai" precisa ser cuidadosa.
        // A forma mais simples é: se todos os outros caminhos falharam, então o nó está totalmente explorado
        // e a lógica de backtrack abaixo será acionada.
    }

    // Se chegou aqui, o nó atual é BECO/FIM ou todas as suas saídas "novas" foram exploradas.
    noSendoProcessado->totalmenteExplorado = true; // Marca como totalmente explorado
    _rob.bcSerialln(String("[DFS] Nó ID: ") + noSendoProcessado->id + " totalmente explorado. Retrocedendo da pilha.");
    
    _topoPilha--; // Remove o nó atual da pilha

    if (_topoPilha < 0) { 
        _rob.bcSerialln("[DFS] Pilha vazia. Exploração DFS concluída.");
        resultadoDFS.acao = ACAO_DFS_EXPLORACAO_CONCLUIDA;
        return resultadoDFS; // Retorna que a exploração está concluída
    } else {
        int idNoPaiNaPilha = _pilhaCaminho[_topoPilha]; // Este é o nó para o qual estamos efetivamente voltando
        NoDFSInfo* infoNoPaiNaPilha = getNoInfo(idNoPaiNaPilha);

        if(!infoNoPaiNaPilha) 
        { 
            resultadoDFS.acao = ACAO_DFS_ERRO;
            return resultadoDFS;
        }

        // O robô está fisicamente no local de 'noSendoProcessado'.
        // Ele precisa se virar para encarar 'infoNoPaiNaPilha'.
        // A direção que ele tomou para chegar em 'noSendoProcessado' a partir de 'infoNoPaiNaPilha'
        // está em 'noSendoProcessado->dirVinda'.
        // Portanto, para voltar, ele precisa encarar a direção oposta a 'noSendoProcessado->dirVinda'.
        DirecaoGlobal dirParaVoltarFisicamente = getDirecaoOposta(noSendoProcessado->dirVinda);
        
        _rob.bcSerialln(String("[DFS] Retrocedendo FISICAMENTE de ") + noSendoProcessado->id + 
                       " para o local do Nó Pai na Pilha ID: " + infoNoPaiNaPilha->id + 
                       ". Robô precisa encarar global: " + (int)dirParaVoltarFisicamente);

        // O main.cpp executará a virada de 180 (ou a manobra para encarar dirParaVoltarFisicamente)
        // e DEPOIS chamará proximaAcao() novamente. Nessa próxima chamada, o
        // idNoAtualMain será idNoPaiNaPilha, e o DFS irá processar esse nó pai.
        resultadoDFS.acao = _rob.getManobraParaEncarar(dirParaVoltarFisicamente);
        resultadoDFS.idNoDestino = infoNoPaiNaPilha->id; // O ID do nó para o qual estamos voltando
        resultadoDFS.direcaoGlobalParaSeguir = dirParaVoltarFisicamente;
        return resultadoDFS;
    }
}
// Função obterProximoIDParaNovoNo pode ser removida se o main não precisar dela diretamente.
// ...