document.getElementById("year").textContent = new Date().getFullYear();

function setActiveNav() {
  const path = location.pathname.split("/").pop() || "index.html";
  document.querySelectorAll("[data-nav]").forEach(a => {
    if (a.getAttribute("href") === path) a.style.textDecoration = "underline";
  });
}
setActiveNav();
