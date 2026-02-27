function buildMainStreamUrl() {
  const motionEnabled = document.getElementById("motionEnabled").checked ? "1" : "0";
  const alpha = document.getElementById("alpha").value;
  const threshold = document.getElementById("threshold").value;
  return `/stream/main?motion=${motionEnabled}&alpha=${alpha}&threshold=${threshold}`;
}

function applyControls() {
  const main = document.getElementById("mainStream");
  main.src = buildMainStreamUrl();
}

document.getElementById("applyBtn").addEventListener("click", applyControls);
applyControls();

let latestMode = "rotate_world_client";
let latestYawRad = 0.0;

function updateMainRotation() {
  const enabled = document.getElementById("bodyHeadingView").checked;
  const main = document.getElementById("mainStream");
  let deg = 0.0;
  if (enabled && latestMode === "rotate_world_client") {
    deg = (latestYawRad * 180.0) / Math.PI;
  }
  main.style.transform = `rotate(${deg.toFixed(3)}deg)`;
}

document.getElementById("bodyHeadingView").addEventListener("change", updateMainRotation);

async function pollMainMeta() {
  try {
    const res = await fetch("/meta/main", { cache: "no-store" });
    if (!res.ok) {
      return;
    }
    const meta = await res.json();
    if (typeof meta.mode === "string") {
      latestMode = meta.mode;
    }
    if (typeof meta.yaw_rad === "number") {
      latestYawRad = meta.yaw_rad;
    }
    updateMainRotation();
  } catch (_) {
  }
}

setInterval(pollMainMeta, 100);
