// src/tipos_robo.h

#ifndef TIPOS_ROBO_H // Isso é um "include guard" para evitar inclusões múltiplas do mesmo arquivo
#define TIPOS_ROBO_H

// Padrão bruto observado pelos sensores
enum TipoDePadraoSensor {
    PADRAO_LINHA_RETA,
    PADRAO_LINHA_SUMIU_CENTRO,
    PADRAO_LATERAL_ESQUERDA_FORTE,
    PADRAO_LATERAL_DIREITA_FORTE,
    PADRAO_MUITOS_SENSORES_PRETOS,
    PADRAO_QUASE_TUDO_BRANCO,
    PADRAO_AMBIGUO
};

// Classificação final do nó, após qualquer etapa de confirmação
enum TipoDeNoFinal {
    NO_FINAL_NAO_E,
    NO_FINAL_BECO_SEM_SAIDA,
    NO_FINAL_CURVA_90_ESQ,
    NO_FINAL_CURVA_90_DIR,
    NO_FINAL_T_COM_FRENTE_ESQ,
    NO_FINAL_T_COM_FRENTE_DIR,
    NO_FINAL_T_SEM_FRENTE,
    NO_FINAL_CRUZAMENTO,
    NO_FINAL_RETA_SIMPLES,
    NO_FINAL_FIM_DO_LABIRINTO
};

enum DirecaoGlobal {
    NORTE,  // 0
    LESTE,  // 1
    SUL,    // 2
    OESTE   // 3
};

// Estados do Robô para controle geral e da máquina de estados principal
enum EstadoRobo {
  PARADO_WEB,
  INICIANDO_EXPLORACAO_WEB,
  SEGUINDO_LINHA_WEB,
  PREPARANDO_ANALISE_NO,
  AVANCO_POSICIONAMENTO_NO,
  AVANCO_CHECA_FRENTE_NO,
  CLASSIFICACAO_DETALHADA_NO,
  CONFIRMANDO_FIM_DO_LABIRINTO,
  PAUSADO_WEB,
  EM_NO_WEB,
  PRINC_PARADO,
    PRINC_INICIANDO_MAPEAMENTO_DFS,
    PRINC_SEGUINDO_LINHA_DFS,
    PRINC_PROCESSANDO_NO_DFS,
    PRINC_EXECUTANDO_MANOBRA_DFS,
    PRINC_MAPEAMENTO_CONCLUIDO_DFS,
    PRINC_ERRO_DFS,
  CALIBRANDO_LINHA_WEB
};

// Se a classe DFSManager precisar de um enum para as ações,
// e o main.cpp também precisar saber sobre ele, ele pode vir aqui.
// Caso contrário, pode ficar dentro de DFSManager.h se for de uso mais restrito.
enum AcaoDFS {
    ACAO_DFS_VIRAR_ESQUERDA,
    ACAO_DFS_SEGUIR_FRENTE,
    ACAO_DFS_VIRAR_DIREITA,
    ACAO_DFS_RETROCEDER_180,
    ACAO_DFS_FIM_LABIRINTO, // Quando o nó final do labirinto é alcançado
    ACAO_DFS_EXPLORACAO_CONCLUIDA, // Quando o DFS retorna à raiz e tudo foi explorado
    ACAO_DFS_ERRO
};

struct ResultadoAcaoDFS {
    AcaoDFS acao;
    int idNoDestino; // ID do nó de destino se a ação for retroceder ou ir para um nó já conhecido. -1 caso contrário.
    DirecaoGlobal direcaoGlobalParaSeguir; // Direção GLOBAL que o robô deve encarar e seguir.
};

struct ResultadoIdentificacaoBaratinha {
    TipoDeNoFinal tipo;
    bool temSaidaEsquerda;
    bool temSaidaFrente;
    bool temSaidaDireita;
    // Adicione outros campos se necessário, como contagem de sensores pretos na confirmação
    // int pretosConfirmacao;
};

struct NodeInfo {
    int id = -1;
    TipoDeNoFinal tipoNo;
    int idPaiDFS = -1;
    
    // Saídas disponíveis nas direções GLOBAIS
    bool saidaDisponivelNorte = false;
    bool saidaDisponivelLeste = false;
    bool saidaDisponivelSul = false;
    bool saidaDisponivelOeste = false;

    // Controle de quais saídas GLOBAIS já foram exploradas
    bool exploradoNorte = false;
    bool exploradoLeste = false;
    bool exploradoSul = false;
    bool exploradoOeste = false;

    bool totalmenteExplorado = false;
    DirecaoGlobal direcaoDeChegadaAoPai = NORTE; // Em qual direção o robô se moveu PARA CHEGAR A ESTE NÓ a partir do seu pai
};

// Você também pode colocar structs aqui se forem usadas por múltiplos arquivos.
// Por exemplo, se a struct NodeInfo for usada fora da classe DFSManager.
// Se for usada apenas internamente pela classe DFSManager, pode ficar em DFSManager.h.

#endif // TIPOS_ROBO_H