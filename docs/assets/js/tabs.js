document.addEventListener("DOMContentLoaded", function () {
    const tabContainers = document.querySelectorAll(".tab-container");

    tabContainers.forEach(container => {
      const tabs = container.querySelectorAll(".tab");
      const contents = container.querySelectorAll(".tab-content");

      tabs.forEach(tab => {
        tab.addEventListener("click", function () {
          const target = this.getAttribute("data-tab");

          tabs.forEach(tab => tab.classList.remove("active"));
          this.classList.add("active");

          contents.forEach(content => content.classList.remove("active"));
          container.querySelector(`#${target}`).classList.add("active");
        });
      });
    });
  });