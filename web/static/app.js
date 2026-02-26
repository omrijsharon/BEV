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
