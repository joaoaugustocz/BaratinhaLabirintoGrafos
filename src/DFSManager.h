// src/DFSManager.h
#ifndef DFSMANAGER_H
#define DFSMANAGER_H

#include <Arduino.h> // Para String, uint16_t, etc.
// Seus enums como TipoDeNoFinal devem estar acessíveis.
// Se estiverem em main.cpp, você pode precisar movê-los para um .h separado
// ou passar os valores como int e lidar com a conversão.
// Para simplificar, vamos supor que TipoDeNoFinal está em um .h que ambos incluem,
// ou você pode redefinir o enum AcaoDFS sem depender de TipoDeNoFinal diretamente aqui
// se a classe só retornar a ação.

// Supondo que TipoDeNoFinal está definido em outro lugar e incluído
// ou você pode passar os tipos como parâmetros int e a classe não precisa saber o enum.
// Para o exemplo, vamos assumir que TipoDeNoFinal é conhecido.
#include "tipos.h" // Se TipoDeNoFinal estiver lá (não ideal, melhor um .h dedicado para tipos)
                 // Ou, melhor, crie um "tipos.h" e inclua em ambos.

// ... (definição da struct NodeInfo e enum AcaoDFS) ...
// ... (declaração da classe DFSManager) ...

#define MAX_NODES_DFS 50 // Defina sua constante AQUI, antes de ser usada

class DFSManager {
public:
    DFSManager();
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
    NodeInfo nosDoLabirinto[MAX_NODES_DFS];
    int contadorNosRegistrados; // Quantos nós estão em nosDoLabirinto
    int idProximoNoUnico;     // Contador interno para gerar IDs únicos para nós DFS
    // ... outros membros privados e métodos auxiliares ...
    NodeInfo* encontrarNodeInfoPeloID(int id);
    NodeInfo* registrarNovoNoInterno(int idNovo, int idPai, TipoDeNoFinal tipo, bool esq, bool fren, bool dir);
};

#endif // DFSMANAGER_H