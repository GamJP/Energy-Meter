let socket;
let chart;
let chartData = { labels: [], voltages: [], currents: [] };

const statusEl = document.getElementById("connection-status");
const voltageEl = document.getElementById("voltage");
const currentEl = document.getElementById("current");
const requestBtn = document.getElementById("requestBtn");
const clearBtn = document.getElementById("clearBtn");

// === Inicializar la gráfica ===
const ctx = document.getElementById("dataChart").getContext("2d");
chart = new Chart(ctx, {
  type: "line",
  data: {
    labels: chartData.labels,
    datasets: [
      {
        label: "Tensión (V)",
        data: chartData.voltages,
        borderWidth: 2,
        fill: false,
        borderColor: "#0099ff",
        tension: 0.2,
      },
      {
        label: "Corriente (A)",
        data: chartData.currents,
        borderWidth: 2,
        fill: false,
        borderColor: "#ff4444",
        tension: 0.2,
      },
    ],
  },
  options: {
    responsive: true,
    scales: {
      x: { title: { display: true, text: "Muestras" } },
      y: { title: { display: true, text: "Valor" } },
    },
  },
});

// === Intento de conexión automática ===
function connectToESP32() {
  const ip = "192.168.4.1"; // 🔹 IP fija del ESP32 en modo AP
  statusEl.textContent = "Conectando al medidor...";
  statusEl.style.color = "#555";

  socket = new WebSocket(`ws://${ip}:81/`);

  socket.onopen = () => {
    statusEl.textContent = "Estado: Conectado";
    statusEl.style.color = "#6ab744";
    requestBtn.disabled = false;
    console.log("Conectado a ESP32 correctamente.");
  };

  socket.onerror = () => {
    statusEl.textContent = "Error de conexión. Reintentando...";
    statusEl.style.color = "red";
    setTimeout(connectToESP32, 3000); // 🔁 Reintenta cada 3 segundos
  };

  socket.onclose = () => {
    statusEl.textContent = "Desconectado. Reintentando...";
    statusEl.style.color = "red";
    requestBtn.disabled = true;
    setTimeout(connectToESP32, 3000); // 🔁 Reintenta conexión automáticamente
  };

  socket.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (Array.isArray(data)) {
      updateChart(data);
    } else {
      console.log("Dato recibido:", data);
    }
  };
}

// === Llamar conexión al cargar la página ===
window.addEventListener("load", connectToESP32);

// === Solicitar datos al ESP32 ===
requestBtn.addEventListener("click", () => {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send("REQUEST_DATA");
  }
});

// === Limpiar gráfica ===
clearBtn.addEventListener("click", () => {
  chartData = { labels: [], voltages: [], currents: [] };
  chart.data.labels = [];
  chart.data.datasets[0].data = [];
  chart.data.datasets[1].data = [];
  chart.update();
});

// === Actualizar gráfica y valores ===
function updateChart(values) {
  chartData.labels = values.map((_, i) => i + 1);
  chartData.voltages = values.map(v => v.V);
  chartData.currents = values.map(v => v.I);

  chart.data.labels = chartData.labels;
  chart.data.datasets[0].data = chartData.voltages;
  chart.data.datasets[1].data = chartData.currents;
  chart.update();

  const lastV = chartData.voltages.at(-1) || 0;
  const lastI = chartData.currents.at(-1) || 0;

  voltageEl.textContent = `${lastV.toFixed(3)} V`;
  currentEl.textContent = `${lastI.toFixed(3)} A`;
}
