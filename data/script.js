// Variáveis globais para o estado da UI
var roboPausadoWeb = false;
var serialMonitorVisible = false;
var webSocket;

// Configuração do grafo Vis.js
var nodes = new vis.DataSet([
    {id: 1, label: 'Início'}, {id: 2, label: 'Nó 2'},
    {id: 3, label: 'Nó 3'}, {id: 4, label: 'Fim'}
]);
var edges = new vis.DataSet([
    {from: 1, to: 2, label: 'Caminho A'}, {from: 1, to: 3, label: 'Caminho B'},
    {from: 2, to: 4, label: 'Caminho C'}, {from: 3, to: 4, label: 'Caminho D'}
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
        var message = event.data.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
        
        // Adiciona timestamp (opcional)
        var now = new Date();
        var timestamp = now.getHours().toString().padStart(2, '0') + ':' + 
                        now.getMinutes().toString().padStart(2, '0') + ':' + 
                        now.getSeconds().toString().padStart(2, '0') + '.ms' +
                        now.getMilliseconds().toString().padStart(3, '0');

        serialOutput.innerHTML += '<span class="timestamp">[' + timestamp + ']</span> ' + message + '\n';
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
function adicionarNo(id, label) { 
    try { nodes.add({id: id, label: label}); } 
    catch(err) { console.error("Erro ao adicionar nó:", err, "ID:", id, "Label:", label); }
}
function adicionarAresta(from, to, label) { 
    try { edges.add({from: from, to: to, label: label || ''}); }
    catch(err) { console.error("Erro ao adicionar aresta:", err, "De:", from, "Para:", to, "Label:", label); }
}

// Inicializar WebSocket quando a página carregar
window.addEventListener('load', initWebSocket);