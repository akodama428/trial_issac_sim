const statusChip = document.getElementById("status-chip");
const stage = document.getElementById("stage");
const viewport = document.getElementById("viewport");
const camera = document.getElementById("camera");
const resultMessage = document.getElementById("result-message");
const failureReason = document.getElementById("failure-reason");
const attemptCount = document.getElementById("attempt-count");
const logList = document.getElementById("log-list");
const helpText = document.getElementById("help-text");
const harvestButton = document.getElementById("harvest-button");
const resetButton = document.getElementById("reset-button");

async function fetchState() {
  const response = await fetch("/api/state");
  return response.json();
}

function renderState(state) {
  statusChip.textContent = state.status;
  statusChip.dataset.status = state.status.toLowerCase();
  stage.innerHTML = state.stageHtml;
  viewport.innerHTML = state.viewportSvg;
  camera.innerHTML = state.cameraSvg;
  resultMessage.textContent = state.resultMessage;
  failureReason.textContent = state.failureReason ?? "";
  attemptCount.textContent = `Attempts completed: ${state.attemptsCompleted}`;
  helpText.textContent = state.helpText;
  logList.innerHTML = state.logs
    .map((entry) => `<li><strong>${entry.step}</strong> ${entry.message}</li>`)
    .join("");

  const busy = ["Approaching", "Grasping", "Pulling"].includes(state.status);
  harvestButton.disabled = busy || state.status === "Loading";
  resetButton.disabled = busy || state.status === "Loading";
}

async function post(path) {
  const response = await fetch(path, { method: "POST" });
  const state = await response.json();
  renderState(state);
}

harvestButton.addEventListener("click", () => post("/api/harvest"));
resetButton.addEventListener("click", () => post("/api/reset"));

async function poll() {
  try {
    const state = await fetchState();
    renderState(state);
  } catch (error) {
    resultMessage.textContent = "Failed to reach PoC server";
    failureReason.textContent = String(error);
  }
}

poll();
setInterval(poll, 400);
