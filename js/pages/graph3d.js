(function () {
  if (!window.BambooPageRuntime) return;

  window.BambooPageRuntime.register('graph3d', async function (root) {
    // Keep ForceGraph on its bundled Three.js runtime. A separate ES module is used
    // only for custom planet meshes so it cannot pollute Globe.gl through window.THREE.
    try { delete window.THREE; } catch (error) { window.THREE = undefined; }
    await window.BambooPageRuntime.loadScript('/js/vendor/3d-force-graph.min.js', 'ForceGraph3D');
    var THREE = await import('https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.min.js');

    var container = root.querySelector('#cosmos-graph');
    var payloadNode = root.querySelector('#graph3d-data');
    if (!container || !payloadNode) return;

    var payload = JSON.parse(payloadNode.textContent);
    var search = root.querySelector('#cosmos-search');
    var reset = root.querySelector('#cosmos-reset');
    var rotate = root.querySelector('#cosmos-rotate');
    var detailTitle = root.querySelector('#cosmos-detail-title');
    var detailCopy = root.querySelector('#cosmos-detail-copy');
    var openButton = root.querySelector('#cosmos-open');
    var selectedNode = null;
    var planetObjects = {};

    function planetTexture(node) {
      var canvas = document.createElement('canvas');
      canvas.width = canvas.height = 128;
      var context = canvas.getContext('2d');
      var gradient = context.createRadialGradient(34, 28, 2, 64, 64, 78);
      gradient.addColorStop(0, '#fff9d4');
      gradient.addColorStop(.12, '#d6f7ff');
      gradient.addColorStop(.28, node.type === 'tag' ? '#4ea5ce' : '#8a82d8');
      gradient.addColorStop(.66, node.type === 'tag' ? '#0f4268' : '#39285f');
      gradient.addColorStop(1, '#030914');
      context.fillStyle = gradient;
      context.fillRect(0, 0, 128, 128);
      for (var index = 0; index < 62; index += 1) {
        context.globalAlpha = .08 + (index % 5) * .045;
        context.fillStyle = index % 3 === 0 ? '#c8f5ff' : (index % 2 ? '#0a2549' : '#ffd987');
        context.beginPath();
        context.arc((index * 31) % 128, (index * 53) % 128, 2 + (index % 8), 0, Math.PI * 2);
        context.fill();
      }
      var texture = new THREE.CanvasTexture(canvas);
      texture.needsUpdate = true;
      return texture;
    }

    function makeLabel(text) {
      var canvas = document.createElement('canvas');
      var context = canvas.getContext('2d');
      context.font = '600 24px sans-serif';
      var width = Math.ceil(context.measureText(text).width) + 34;
      canvas.width = Math.max(150, width);
      canvas.height = 46;
      context = canvas.getContext('2d');
      context.font = '600 24px sans-serif';
      context.fillStyle = 'rgba(4, 10, 26, .72)';
      context.roundRect(0, 0, canvas.width, canvas.height, 20);
      context.fill();
      context.fillStyle = '#f7fbff';
      context.textAlign = 'center';
      context.textBaseline = 'middle';
      context.fillText(text, canvas.width / 2, canvas.height / 2 + 1);
      var material = new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), transparent: true, opacity: .8, depthWrite: false });
      var label = new THREE.Sprite(material);
      label.scale.set(canvas.width / 22, canvas.height / 22, 1);
      return label;
    }

    function makePlanet(node) {
      if (!THREE) return undefined;
      var group = new THREE.Group();
      var surface = new THREE.Group();
      var radius = node.type === 'tag' ? 4.8 + Math.min(node.count || 0, 8) * .22 : 2.2;
      var globe = new THREE.Mesh(new THREE.SphereGeometry(radius, 32, 32), new THREE.MeshStandardMaterial({ map: planetTexture(node), roughness: .48, metalness: .3, emissive: node.type === 'tag' ? '#2d7dff' : '#7f4ee8', emissiveIntensity: node.type === 'tag' ? .42 : .3 }));
      surface.add(globe);
      var atmosphere = new THREE.Mesh(new THREE.SphereGeometry(radius * 1.08, 28, 28), new THREE.MeshBasicMaterial({ color: node.type === 'tag' ? '#6bd8ff' : '#c886ff', transparent: true, opacity: .14, side: THREE.BackSide, blending: THREE.AdditiveBlending, depthWrite: false }));
      surface.add(atmosphere);
      var halo = new THREE.Mesh(new THREE.RingGeometry(radius * 1.24, radius * 1.3, 64), new THREE.MeshBasicMaterial({ color: node.type === 'tag' ? '#81e5ff' : '#d49bff', transparent: true, opacity: .55, side: THREE.DoubleSide, blending: THREE.AdditiveBlending, depthWrite: false }));
      halo.rotation.x = Math.PI / 2.65;
      halo.rotation.y = .35;
      surface.add(halo);
      group.add(surface);
      var label = makeLabel(node.name);
      label.position.set(0, radius + 3.4, 0);
      group.add(label);
      group.userData.label = label;
      group.userData.surface = surface;
      planetObjects[node.id] = group;
      return group;
    }

    var graph = window.ForceGraph3D({ controlType: 'orbit', rendererConfig: { antialias: true, alpha: true } })(container)
      .width(container.clientWidth)
      .height(container.clientHeight)
      .backgroundColor('rgba(0,0,0,0)')
      .showNavInfo(false)
      .nodeLabel(function (node) {
        return '<div class="cosmos-tooltip"><strong>' + node.name + '</strong><span>' + (node.type === 'tag' ? node.count + ' 篇文章' : node.date || '博客文章') + '</span></div>';
      })
      .nodeVal('val')
      .nodeColor('color')
      .nodeOpacity(0.95)
      .nodeResolution(20)
      .nodeThreeObject(makePlanet)
      .nodeThreeObjectExtend(false)
      .linkColor(function () { return 'rgba(127,178,255,0.42)'; })
      .linkOpacity(0.55)
      .linkWidth(function (link) { return link.__highlight ? 1.8 : 0.55; })
      .linkDirectionalParticles(function (link) { return link.__highlight ? 3 : 1; })
      .linkDirectionalParticleWidth(function (link) { return link.__highlight ? 2.4 : 0.8; })
      .linkDirectionalParticleSpeed(0.004)
      .warmupTicks(80)
      .cooldownTicks(180)
      .d3VelocityDecay(0.32)
      .onNodeHover(function (node) {
        container.style.cursor = node ? 'pointer' : 'grab';
      })
      .onNodeClick(function (node) {
        selectNode(node, true);
      })
      .onEngineTick(function () {
        if (!THREE) return;
        var camera = graph.camera();
        payload.nodes.forEach(function (node) {
          var object = planetObjects[node.id];
          if (!object || !object.userData.label || !isFinite(node.x)) return;
          var label = object.userData.label;
          object.userData.surface.rotation.y += node.type === 'tag' ? .004 : .007;
          var distance = Math.hypot(camera.position.x - node.x, camera.position.y - node.y, camera.position.z - node.z);
          // Zooming in makes labels quieter; zooming out restores their opacity.
          label.material.opacity = Math.max(.16, Math.min(.94, (distance - 42) / 240));
        });
      })
      .onBackgroundClick(function () {
        selectedNode = null;
        payload.links.forEach(function (link) { link.__highlight = false; });
        graph.linkWidth(graph.linkWidth()).linkDirectionalParticles(graph.linkDirectionalParticles());
        detailTitle.textContent = '选择一颗星球';
        detailCopy.textContent = '点击标签查看它连接的文章，点击文章即可前往阅读。';
        openButton.hidden = true;
      })
      .graphData(payload);

    graph.lights([
      new THREE.AmbientLight('#7bb4ff', 1.4),
      new THREE.DirectionalLight('#ffffff', 2.3),
      new THREE.PointLight('#4bb6ff', 2.6, 600),
      new THREE.PointLight('#b572ff', 1.8, 520)
    ]);
    graph.lights()[1].position.set(120, 170, 220);
    graph.lights()[2].position.set(-150, 80, 180);
    graph.lights()[3].position.set(100, -130, -140);

    graph.d3Force('charge').strength(function (node) { return node.type === 'tag' ? -210 : -55; });
    graph.d3Force('link').distance(96);

    var controls = graph.controls();
    controls.autoRotate = true;
    controls.autoRotateSpeed = 0.32;
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.zoomSpeed = 1.25;
    controls.minDistance = 24;
    controls.maxDistance = 1600;
    graph.camera().near = 0.5;
    graph.camera().far = 5000;
    graph.camera().updateProjectionMatrix();

    function selectNode(node, moveCamera) {
      selectedNode = node;
      var connected = {};
      connected[node.id] = true;
      payload.links.forEach(function (link) {
        var source = typeof link.source === 'object' ? link.source.id : link.source;
        var target = typeof link.target === 'object' ? link.target.id : link.target;
        link.__highlight = source === node.id || target === node.id;
        if (link.__highlight) {
          connected[source] = true;
          connected[target] = true;
        }
      });
      graph.linkWidth(graph.linkWidth()).linkDirectionalParticles(graph.linkDirectionalParticles());
      graph.nodeOpacity(function (item) { return connected[item.id] ? 1 : 0.22; });
      window.setTimeout(function () { graph.nodeOpacity(0.95); }, 2600);

      detailTitle.textContent = node.name;
      detailCopy.textContent = node.type === 'tag' ? '这颗星球连接了 ' + node.count + ' 篇文章。' : (node.date ? '发布于 ' + node.date + '，点击打开文章。' : '点击打开文章。');
      openButton.hidden = !node.url;
      openButton.textContent = node.type === 'post' ? '阅读文章' : '查看标签';

      if (moveCamera && isFinite(node.x)) {
        var distance = 58;
        var ratio = 1 + distance / Math.hypot(node.x, node.y, node.z);
        graph.cameraPosition({ x: node.x * ratio, y: node.y * ratio, z: node.z * ratio }, node, 900);
      }
    }

    function applySearch() {
      var query = search.value.trim().toLowerCase();
      if (!query) {
        graph.nodeVisibility(true).linkVisibility(true);
        return;
      }
      var visible = {};
      payload.nodes.forEach(function (node) {
        if (node.name.toLowerCase().indexOf(query) !== -1) visible[node.id] = true;
      });
      payload.links.forEach(function (link) {
        var source = typeof link.source === 'object' ? link.source.id : link.source;
        var target = typeof link.target === 'object' ? link.target.id : link.target;
        if (visible[source]) visible[target] = true;
        if (visible[target]) visible[source] = true;
      });
      graph.nodeVisibility(function (node) { return !!visible[node.id]; });
      graph.linkVisibility(function (link) {
        var source = typeof link.source === 'object' ? link.source.id : link.source;
        var target = typeof link.target === 'object' ? link.target.id : link.target;
        return !!visible[source] && !!visible[target];
      });
    }

    function resetView() {
      graph.zoomToFit(900, 56);
      search.value = '';
      graph.nodeVisibility(true).linkVisibility(true).nodeOpacity(0.95);
    }

    function toggleRotate() {
      controls.autoRotate = !controls.autoRotate;
      rotate.classList.toggle('is-active', controls.autoRotate);
      rotate.setAttribute('aria-pressed', String(controls.autoRotate));
    }

    function openSelected() {
      if (selectedNode && selectedNode.url) window.BambooPageRuntime.navigate(selectedNode.url);
    }

    search.addEventListener('input', applySearch);
    reset.addEventListener('click', resetView);
    rotate.addEventListener('click', toggleRotate);
    openButton.addEventListener('click', openSelected);

    var resizeObserver = new ResizeObserver(function () {
      graph.width(container.clientWidth).height(container.clientHeight);
    });
    resizeObserver.observe(container);
    window.setTimeout(function () { graph.zoomToFit(1000, 48); }, 700);

    return function () {
      search.removeEventListener('input', applySearch);
      reset.removeEventListener('click', resetView);
      rotate.removeEventListener('click', toggleRotate);
      openButton.removeEventListener('click', openSelected);
      resizeObserver.disconnect();
      controls.autoRotate = false;
      if (typeof graph.pauseAnimation === 'function') graph.pauseAnimation();
      if (typeof graph._destructor === 'function') graph._destructor();
      container.innerHTML = '';
    };
  });
})();
