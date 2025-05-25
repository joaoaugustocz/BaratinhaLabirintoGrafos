// Variáveis globais para o estado da UI
var roboPausadoWeb = false;
var serialMonitorVisible = false;
var webSocket;

// Configuração do grafo Vis.js
var nodes = new vis.DataSet([

]);
var edges = new vis.DataSet([

]);
var data = { nodes: nodes, edges: edges };
var options = { 
    layout: { hierarchical: false }, 
    edges: { arrows: 'to', smooth: { type: 'cubicBezier' }}, 
    interaction: { dragNodes: true, dragView: true, zoomView: true }
};
var container = document.getElementById('mynetwork');
var network = new vis.Network(container, data, options);

// Funções de Alerta e Controle do Robô (HTTP)
function showAlert(message, type = 'info') { 
    // Poderia ser um modal mais elegante no futuro
    alert(message); 
}

function iniciarRobo() {
    console.log("Solicitando /iniciar");
    fetch('/iniciar').then(r => r.text()).then(d => {
        showAlert(d); 
        roboPausadoWeb = false; 
        let btn = document.getElementById('btnPausarContinuar');
        btn.textContent = 'Pausar Robô';
        btn.className = 'warning'; // Garante a classe correta
    }).catch(e => { console.error("Erro /iniciar:", e); showAlert("Erro ao iniciar robô."); });
}

function retornarInicio() {
    console.log("Solicitando /retornar");
    fetch('/retornar').then(r => r.text()).then(d => showAlert(d))
    .catch(e => { console.error("Erro /retornar:", e); showAlert("Erro ao retornar ao início."); });
}

function togglePausaRobo() {
    const path = roboPausadoWeb ? '/continuar' : '/pausar';
    console.log(`Solicitando ${path}`);
    fetch(path).then(r => r.text()).then(d => {
        showAlert(d); 
        roboPausadoWeb = !roboPausadoWeb;
        let btn = document.getElementById('btnPausarContinuar');
        btn.textContent = roboPausadoWeb ? 'Continuar Robô' : 'Pausar Robô';
        btn.className = roboPausadoWeb ? 'button success' : 'button warning'; // Adiciona 'button' para estilos base
    }).catch(e => { console.error(`Erro ${path}:`, e); showAlert("Erro ao pausar/continuar."); });
}

function recalibrarSensores() {
    if (confirm("Recalibrar sensores de linha? O robô irá parar e se mover.")) {
        console.log("Solicitando /recalibrar_linha");
        fetch('/recalibrar_linha').then(r => r.text()).then(d => showAlert(d))
        .catch(e => { console.error("Erro /recalibrar_linha:", e); showAlert("Erro ao recalibrar."); });
    }
}

// Funções do Monitor Serial (Drawer e WebSocket)
function initWebSocket() {
    var wsHost = window.location.hostname || "192.168.1.47"; // Fallback para IP AP se hostname vazio
    console.log("Tentando conectar WebSocket a: ws://" + wsHost + ':81/');
    webSocket = new WebSocket('ws://' + wsHost + ':81/');

    webSocket.onopen = function(event) {
        console.log('WebSocket Conectado');
        webSocket.send('Cliente Web Conectado via JS!');
        document.getElementById('serialOutput').innerHTML += '<span style="color: #61afef;">[WebSocket Conectado]</span>\n';
        // Não abre automaticamente, deixa o botão controlar
    };

    webSocket.onmessage = function(event) {
        var serialOutput = document.getElementById('serialOutput');
        var messageData = event.data; // A mensagem pura
        
        // Tenta processar como comando de grafo primeiro
        var parts = messageData.split(':');
        var command = parts[0];

        if (command === "newNode" && parts.length >= 3) {
            var nodeId = parseInt(parts[1]);
            var nodeLabel = parts.slice(2).join(':'); // Reconstrói o label caso ele contenha ':'
            adicionarNo(nodeId, nodeLabel);
            // Adiciona ao log também
            serialOutput.innerHTML += '<span style="color: #98c379;">[MAPA] Novo Nó: ID=' + nodeId + ', Label=' + nodeLabel + '</span>\n';
        } else if (command === "newEdge" && parts.length >= 3) { // Precisa de pelo menos from:to
            var fromNode = parseInt(parts[1]);
            var toNode = parseInt(parts[2]);
            var edgeLabel = (parts.length >= 4) ? parts.slice(3).join(':') : ""; // Label opcional
            adicionarAresta(fromNode, toNode, edgeLabel);
            serialOutput.innerHTML += '<span style="color: #61afef;">[MAPA] Nova Aresta: ' + fromNode + ' -> ' + toNode + (edgeLabel ? ' ('+edgeLabel+')' : '') + '</span>\n';
        } else {
            // Se não for um comando de grafo, trata como mensagem de log normal
            var now = new Date();
            var timestamp = now.getHours().toString().padStart(2, '0') + ':' + 
                            now.getMinutes().toString().padStart(2, '0') + ':' + 
                            now.getSeconds().toString().padStart(2, '0');
            var sanitizedMessage = messageData.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
            serialOutput.innerHTML += '<span class="timestamp">[' + timestamp + ']</span> ' + sanitizedMessage; // Removido \n extra, pois broadcastSerialLn já adiciona
        }
        serialOutput.scrollTop = serialOutput.scrollHeight; 
    };

    webSocket.onclose = function(event) {
        console.log('WebSocket Desconectado');
        var serialOutput = document.getElementById('serialOutput');
        serialOutput.innerHTML += '<span style="color: #e06c75;">[WebSocket Desconectado] - Tentando reconectar em 3s...</span>\n';
        serialOutput.scrollTop = serialOutput.scrollHeight;
        setTimeout(initWebSocket, 3000); 
    };

    webSocket.onerror = function(error) {
        console.error('Erro no WebSocket: ', error);
        var serialOutput = document.getElementById('serialOutput');
        serialOutput.innerHTML += '<span style="color: #e06c75;">[Erro no WebSocket Detectado]</span>\n';
        serialOutput.scrollTop = serialOutput.scrollHeight;
    };
}

function toggleSerialMonitor() {
    var drawer = document.getElementById('serialMonitorDrawer');
    var btn = document.getElementById('btnToggleSerial');
    serialMonitorVisible = !serialMonitorVisible;
    if (serialMonitorVisible) {
        drawer.classList.add('open');
        btn.textContent = 'Fechar Log';
    } else {
        drawer.classList.remove('open');
        btn.textContent = 'Abrir Log';
    }
}

function limparSerialMonitor() {
    document.getElementById('serialOutput').innerHTML = "";
    // Adiciona uma mensagem de log limpo (opcional)
    if (webSocket && webSocket.readyState === WebSocket.OPEN) {
         document.getElementById('serialOutput').innerHTML = '<span style="color: #61afef;">[Log Limpo pelo Usuário]</span>\n';
    }
}

// Funções para atualizar o grafo (você já as tinha)
function adicionarNo(nodeId, nodeLabel) { 
    try {
        if (nodes.get(nodeId) == null) { // Só adiciona se o nó não existir
            nodes.add({id: nodeId, label: nodeLabel || ('Nó ' + nodeId)});
            console.log("Nó Adicionado:", nodeId, nodeLabel);
        } else {
            console.log("Nó já existe:", nodeId);
            // Opcional: atualizar o label se necessário
            // nodes.update({id: nodeId, label: nodeLabel || ('Nó ' + nodeId)});
        }
    } catch(err) { 
        console.error("Erro ao adicionar nó:", err, "ID:", nodeId, "Label:", nodeLabel); 
    }
}

function adicionarAresta(fromNodeId, toNodeId, edgeLabel) { 
    try {
        // Evitar arestas duplicadas (Vis.js pode lidar com isso, mas uma checagem pode ser útil)
        // Um ID de aresta pode ser fromNodeId + "-" + toNodeId
        var edgeId = String(fromNodeId) + "-" + String(toNodeId);
        var reverseEdgeId = String(toNodeId) + "-" + String(fromNodeId); // Para grafos não direcionados no display

        if (edges.get(edgeId) == null && edges.get(reverseEdgeId) == null) {
            edges.add({id: edgeId, from: fromNodeId, to: toNodeId, label: edgeLabel || ''});
            console.log("Aresta Adicionada:", fromNodeId, "->", toNodeId, edgeLabel);
        } else {
            console.log("Aresta já existe:", edgeId, "ou", reverseEdgeId);
        }
    } catch(err) { 
        console.error("Erro ao adicionar aresta:", err, "De:", fromNodeId, "Para:", toNodeId, "Label:", edgeLabel); 
    }
}

// Inicializar WebSocket quando a página carregar
window.addEventListener('load', initWebSocket);