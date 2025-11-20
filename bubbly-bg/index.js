document.addEventListener("DOMContentLoaded", function () { // 这个表示在html加载完成后执行

    bubbly({
    compose: "source-over",
    bubbles: {
        count: 200,
        radius: () => 1 + Math.random() * 4,
        fill: () => `hsla(${150 + Math.random() * 100}, 100%, 50%, .3)`,
        angle: () => Math.random() > 0.5 ? Math.PI : 2 * Math.PI,
        velocity: () => 4 + Math.random() * 4,
    },
    background: () => "#f9f9f9",
	});
})

