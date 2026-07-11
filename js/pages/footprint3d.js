(function () {
  if (!window.BambooPageRuntime) return;

  window.BambooPageRuntime.register('footprint3d', async function (root) {
    // Use an isolated Three.js module. The previous Globe.gl bundle could block the
    // main thread while creating the earth and high-resolution country polygons.
    try { delete window.THREE; } catch (error) { window.THREE = undefined; }
    var THREE = await import('https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.min.js');

    var container = root.querySelector('#travel-globe');
    var payloadNode = root.querySelector('#footprint3d-data');
    if (!container || !payloadNode) return;

    var payload = JSON.parse(payloadNode.textContent);
    var storedTrips = [];
    try { storedTrips = JSON.parse(localStorage.getItem('bambooFootprintTrips') || '[]'); } catch (error) {}
    payload.trips = payload.trips.concat(storedTrips);

    var rotateButton = root.querySelector('#globe-rotate');
    var resetButton = root.querySelector('#globe-reset');
    var zoomInButton = root.querySelector('#globe-zoom-in');
    var zoomOutButton = root.querySelector('#globe-zoom-out');
    var focusButton = root.querySelector('#footprint-focus');
    var addButton = root.querySelector('#footprint-add');
    var viewAllButton = root.querySelector('#footprint-view-all');
    var storyList = root.querySelector('#footprint-story-list');
    var detail = root.querySelector('#footprint-detail');
    var detailClose = root.querySelector('#footprint-detail-close');
    var dialog = root.querySelector('#footprint-dialog');
    var form = root.querySelector('#footprint-form');
    var dialogClose = root.querySelector('[data-dialog-close]');
    var gpsStatus = root.querySelector('#footprint-gps-status');
    var photoInput = form.querySelector('input[name="photo"]');
    var currentView = { lat: 28, lng: 108, altitude: 2.18 };

    // The page can be entered through PJAX while its stylesheet is still settling.
    // Wait one frame and never initialise the WebGL canvas with a zero-size viewport.
    await new Promise(function (resolve) { requestAnimationFrame(resolve); });

    function createTravelGlobe(element, trips, onTripClick) {
      var width = Math.max(320, element.clientWidth);
      var height = Math.max(420, element.clientHeight);
      var scene = new THREE.Scene();
      var camera = new THREE.PerspectiveCamera(42, width / height, .1, 1600);
      camera.position.z = 318;
      var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
      renderer.setPixelRatio(Math.min(1.6, window.devicePixelRatio || 1));
      renderer.setSize(width, height);
      renderer.setClearColor(0x000000, 0);
      element.innerHTML = '';
      element.appendChild(renderer.domElement);

      var globeGroup = new THREE.Group();
      var markerGroup = new THREE.Group();
      var countryGroup = new THREE.Group();
      globeGroup.add(markerGroup, countryGroup);
      scene.add(globeGroup);

      var textureLoader = new THREE.TextureLoader();
      var earthMaterial = new THREE.MeshStandardMaterial({ roughness: .7, metalness: .06 });
      globeGroup.add(new THREE.Mesh(new THREE.SphereGeometry(100, 56, 56), earthMaterial));
      var atmosphereMaterial = new THREE.MeshBasicMaterial({ color: '#6bbdff', transparent: true, opacity: .17, side: THREE.BackSide, blending: THREE.AdditiveBlending, depthWrite: false });
      globeGroup.add(new THREE.Mesh(new THREE.SphereGeometry(105, 40, 40), atmosphereMaterial));
      scene.add(new THREE.AmbientLight('#b8d8ff', 1.8));
      var sunlight = new THREE.DirectionalLight('#ffffff', 2.7);
      sunlight.position.set(-150, 130, 240);
      scene.add(sunlight);

      var controls = { autoRotate: true, autoRotateSpeed: .42, enableDamping: true, dampingFactor: .07, minDistance: 170, maxDistance: 520 };
      var markerMeshes = [];
      var animationId = 0;
      var pointerStart = null;
      var dragged = false;
      var raycaster = new THREE.Raycaster();
      var pointer = new THREE.Vector2();

      function toVector(lat, lng, radius) {
        var phi = (90 - Number(lat)) * Math.PI / 180;
        var theta = (Number(lng) + 180) * Math.PI / 180;
        return new THREE.Vector3(-radius * Math.sin(phi) * Math.cos(theta), radius * Math.cos(phi), radius * Math.sin(phi) * Math.sin(theta));
      }

      function createLabel(text, winter) {
        var canvas = document.createElement('canvas');
        canvas.width = 192; canvas.height = 48;
        var context = canvas.getContext('2d');
        var labelText = (winter ? '❄ ' : '') + text;
        context.clearRect(0, 0, canvas.width, canvas.height);
        context.font = '600 18px sans-serif'; context.textAlign = 'center'; context.textBaseline = 'middle';
        context.lineWidth = 4; context.strokeStyle = 'rgba(3,12,24,.82)'; context.strokeText(labelText, 96, 25);
        context.fillStyle = winter ? '#bfeaff' : '#fff'; context.fillText(labelText, 96, 25);
        var sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), transparent: true, depthWrite: false }));
        sprite.scale.set(18, 4.5, 1);
        return sprite;
      }

      function clearGroup(group) {
        group.children.slice().forEach(function (child) {
          group.remove(child);
          if (child.geometry) child.geometry.dispose();
          if (child.material) child.material.dispose();
        });
      }

      function setTrips(nextTrips) {
        clearGroup(markerGroup);
        markerMeshes = [];
        (nextTrips || []).forEach(function (trip) {
          var position = toVector(trip.lat, trip.lng, 103);
          var marker = new THREE.Mesh(new THREE.SphereGeometry(trip.photo ? 2.5 : 1.9, 16, 16), new THREE.MeshStandardMaterial({ color: trip.winter ? '#bdefff' : (trip.photo ? '#ff7287' : '#ffd35c'), emissive: trip.winter ? '#69cfff' : '#ff8a65', emissiveIntensity: .75 }));
          marker.position.copy(position); marker.userData.trip = trip;
          markerGroup.add(marker); markerMeshes.push(marker);
          var label = createLabel(trip.city, trip.winter);
          label.position.copy(position.clone().multiplyScalar(1.08)); markerGroup.add(label);
          if (trip.photo) textureLoader.load(trip.photo, function (texture) {
            var photo = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthWrite: false }));
            photo.position.copy(position.clone().multiplyScalar(1.14)); photo.scale.set(14, 10, 1); markerGroup.add(photo);
          });
        });
      }

      function setCountries(features) {
        clearGroup(countryGroup);
        var vertices = [];
        function add(value) {
          if (!Array.isArray(value) || !value.length) return;
          if (Array.isArray(value[0]) && typeof value[0][0] === 'number') {
            for (var index = 1; index < value.length; index += 1) {
              vertices.push(toVector(value[index - 1][1], value[index - 1][0], 100.7), toVector(value[index][1], value[index][0], 100.7));
            }
          } else value.forEach(add);
        }
        (features || []).forEach(function (feature) { if (feature.geometry) add(feature.geometry.coordinates); });
        if (vertices.length) countryGroup.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(vertices), new THREE.LineBasicMaterial({ color: '#d9f5ff', transparent: true, opacity: .5 })));
      }

      function resize(nextWidth, nextHeight) {
        width = Math.max(320, nextWidth); height = Math.max(420, nextHeight);
        camera.aspect = width / height; camera.updateProjectionMatrix(); renderer.setSize(width, height);
      }

      function pointOfView(view) {
        if (!view) return { lat: 0, lng: 0, altitude: (camera.position.z - 140) / 82 };
        globeGroup.rotation.x = Number(view.lat || 0) * Math.PI / 180;
        globeGroup.rotation.y = -Number(view.lng || 0) * Math.PI / 180;
        camera.position.z = Math.max(170, Math.min(520, 140 + Number(view.altitude || 2.18) * 82));
        return api;
      }

      function onPointerDown(event) { pointerStart = { x: event.clientX, y: event.clientY, rx: globeGroup.rotation.x, ry: globeGroup.rotation.y }; dragged = false; controls.autoRotate = false; }
      function onPointerMove(event) {
        if (!pointerStart) return;
        var dx = event.clientX - pointerStart.x, dy = event.clientY - pointerStart.y;
        if (Math.abs(dx) + Math.abs(dy) > 3) dragged = true;
        globeGroup.rotation.y = pointerStart.ry + dx * .006;
        globeGroup.rotation.x = Math.max(-1.25, Math.min(1.25, pointerStart.rx + dy * .004));
      }
      function onPointerUp(event) {
        if (!dragged) {
          var rect = renderer.domElement.getBoundingClientRect();
          pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
          raycaster.setFromCamera(pointer, camera);
          var hit = raycaster.intersectObjects(markerMeshes, false)[0];
          if (hit) onTripClick(hit.object.userData.trip);
        }
        pointerStart = null;
      }
      function onWheel(event) { event.preventDefault(); camera.position.z = Math.max(controls.minDistance, Math.min(controls.maxDistance, camera.position.z + event.deltaY * .18)); }
      renderer.domElement.addEventListener('pointerdown', onPointerDown);
      window.addEventListener('pointermove', onPointerMove);
      window.addEventListener('pointerup', onPointerUp);
      renderer.domElement.addEventListener('wheel', onWheel, { passive: false });

      function animate() {
        animationId = requestAnimationFrame(animate);
        if (controls.autoRotate) globeGroup.rotation.y += .0018 * controls.autoRotateSpeed;
        renderer.render(scene, camera);
      }

      var api = {
        width: function (value) { if (value == null) return width; resize(value, height); return api; },
        height: function (value) { if (value == null) return height; resize(width, value); return api; },
        controls: function () { return controls; },
        pointOfView: pointOfView,
        pointsData: function (value) { setTrips(value); return api; },
        setCountries: function (value) { setCountries(value); return api; },
        globeImageUrl: function (url) { textureLoader.load(url, function (texture) { texture.colorSpace = THREE.SRGBColorSpace; earthMaterial.map = texture; earthMaterial.needsUpdate = true; }); return api; },
        atmosphereColor: function (color) { atmosphereMaterial.color.set(color); return api; },
        _destructor: function () {
          cancelAnimationFrame(animationId);
          renderer.domElement.removeEventListener('pointerdown', onPointerDown);
          window.removeEventListener('pointermove', onPointerMove);
          window.removeEventListener('pointerup', onPointerUp);
          renderer.domElement.removeEventListener('wheel', onWheel);
          renderer.dispose(); element.innerHTML = '';
        }
      };
      setTrips(trips); animate();
      return api;
    }

    var globe = createTravelGlobe(container, payload.trips, openTrip);

    function simplifyRing(ring) {
      if (!Array.isArray(ring) || ring.length <= 72) return ring;
      var step = Math.ceil(ring.length / 72);
      var reduced = ring.filter(function (point, index) { return index % step === 0; });
      var last = ring[ring.length - 1];
      if (reduced[reduced.length - 1] !== last) reduced.push(last);
      return reduced;
    }

    function simplifyCoordinates(value) {
      if (!Array.isArray(value) || !value.length) return value;
      if (Array.isArray(value[0]) && typeof value[0][0] === 'number') return simplifyRing(value);
      return value.map(simplifyCoordinates);
    }

    // Country polygons are simplified and added after the first interactive frame.
    // The original high-resolution GeoJSON otherwise blocks low-power/mobile GPUs.
    window.setTimeout(function () {
      fetch('/footprint/data/world.geojson').then(function (response) { return response.json(); }).then(function (world) {
        var countries = (world.features || []).map(function (feature) {
          return {
            type: 'Feature',
            properties: feature.properties || {},
            geometry: feature.geometry ? { type: feature.geometry.type, coordinates: simplifyCoordinates(feature.geometry.coordinates) } : null
          };
        }).filter(function (feature) { return feature.geometry; });
        globe.setCountries(countries);
      }).catch(function () {});
    }, 450);

    var controls = globe.controls();
    controls.autoRotate = true;
    controls.autoRotateSpeed = 0.42;
    controls.enableDamping = true;
    controls.dampingFactor = 0.07;
    controls.minDistance = 170;
    controls.maxDistance = 520;
    globe.pointOfView(currentView, 0);

    function setView(view, duration) {
      currentView = { lat: Number(view.lat), lng: Number(view.lng), altitude: Number(view.altitude) };
      globe.pointOfView(currentView, duration || 0);
    }

    function updateTheme() {
      var dark = document.body.classList.contains('darkModel');
      globe.globeImageUrl(dark ? '/medias/earth/earth-night.jpg' : '/medias/earth/earth-blue-marble.jpg');
      globe.atmosphereColor(dark ? '#72bfff' : '#5bb5ef');
      root.classList.toggle('is-dark-earth', dark);
    }

    function tripById(id) {
      return payload.trips.find(function (trip) { return String(trip.id) === String(id); });
    }

    function openTrip(trip) {
      if (!trip) return;
      root.querySelector('#footprint-detail-date').textContent = trip.date || '旅行记录';
      root.querySelector('#footprint-detail-title').textContent = trip.city + ' · ' + (trip.title || '旅行记忆');
      root.querySelector('#footprint-detail-story').textContent = trip.story || '照片和旅行故事等待你在这里补充。';
      root.querySelector('#footprint-detail-location').textContent = (trip.country || '') + '  ' + Number(trip.lat).toFixed(3) + ', ' + Number(trip.lng).toFixed(3);
      var image = root.querySelector('#footprint-detail-image');
      image.hidden = !trip.photo;
      image.src = trip.photo || '';
      image.alt = trip.city + '旅行照片';
      detail.classList.add('is-open');
      detail.setAttribute('aria-hidden', 'false');
      setView({ lat: Number(trip.lat), lng: Number(trip.lng), altitude: 1.55 }, 900);
      controls.autoRotate = false;
      rotateButton.classList.remove('is-active');
      rotateButton.setAttribute('aria-pressed', 'false');
    }

    function closeDetail() {
      detail.classList.remove('is-open');
      detail.setAttribute('aria-hidden', 'true');
    }

    function toggleRotate() {
      controls.autoRotate = !controls.autoRotate;
      rotateButton.classList.toggle('is-active', controls.autoRotate);
      rotateButton.setAttribute('aria-pressed', String(controls.autoRotate));
    }

    function resetView() {
      setView({ lat: 28, lng: 108, altitude: 2.18 }, 850);
      closeDetail();
    }

    function changeZoom(delta) {
      setView({ lat: currentView.lat, lng: currentView.lng, altitude: Math.max(0.65, Math.min(4, currentView.altitude + delta)) }, 350);
    }

    function showAllStories() {
      var items = payload.trips.filter(function (trip) { return trip.photo; });
      if (!items.length) {
        storyList.innerHTML = '<div class="footprint-photo-placeholder"><i class="fas fa-image"></i><strong>照片位置已预留</strong><span>上传旅行照片后，会自动显示在对应城市</span></div>';
        return;
      }
      storyList.innerHTML = items.map(function (trip) {
        return '<button type="button" data-trip-id="' + trip.id + '"><img src="' + trip.photo + '" alt="' + trip.city + '旅行照片"><span><strong>' + trip.city + ' · ' + trip.title + '</strong><small>' + trip.date + ' · ' + trip.story + '</small></span></button>';
      }).join('');
    }

    function handleStoryClick(event) {
      var button = event.target.closest('[data-trip-id]');
      if (button) openTrip(tripById(button.dataset.tripId));
    }

    function openDialog() {
      form.reset();
      gpsStatus.textContent = '';
      if (typeof dialog.showModal === 'function') dialog.showModal();
      else dialog.setAttribute('open', '');
    }

    function closeDialog() {
      if (typeof dialog.close === 'function') dialog.close();
      else dialog.removeAttribute('open');
    }

    async function readGps() {
      var file = photoInput.files && photoInput.files[0];
      if (!file) return;
      gpsStatus.textContent = '正在读取照片位置信息…';
      try {
        await window.BambooPageRuntime.loadScript('/js/vendor/exifr.umd.js', 'exifr');
        var position = await window.exifr.gps(file);
        if (position && isFinite(position.latitude) && isFinite(position.longitude)) {
          form.elements.lat.value = position.latitude.toFixed(6);
          form.elements.lng.value = position.longitude.toFixed(6);
          gpsStatus.textContent = '已从照片读取 GPS 位置。';
        } else {
          gpsStatus.textContent = '照片没有 GPS 信息，请手动填写经纬度。';
        }
      } catch (error) {
        gpsStatus.textContent = '未能读取 GPS，请手动填写经纬度。';
      }
    }

    function fileToDataUrl(file) {
      return new Promise(function (resolve, reject) {
        var reader = new FileReader();
        reader.onload = function () { resolve(reader.result); };
        reader.onerror = reject;
        reader.readAsDataURL(file);
      });
    }

    async function saveTrip(event) {
      event.preventDefault();
      var file = photoInput.files && photoInput.files[0];
      if (!file) return;
      var trip = {
        id: 'local-' + Date.now(),
        city: form.elements.city.value.trim(),
        country: '我的旅行',
        date: form.elements.date.value,
        lat: Number(form.elements.lat.value),
        lng: Number(form.elements.lng.value),
        title: '新的旅行记忆',
        story: form.elements.story.value.trim() || '一段刚刚收藏的旅行故事。',
        photo: await fileToDataUrl(file)
      };
      storedTrips.unshift(trip);
      payload.trips.push(trip);
      try { localStorage.setItem('bambooFootprintTrips', JSON.stringify(storedTrips.slice(0, 8))); } catch (error) {}
      globe.pointsData(payload.trips);
      root.querySelector('#footprint-count').textContent = payload.trips.length + ' 个';
      var photoCount = payload.trips.filter(function (item) { return item.photo; }).length;
      root.querySelector('#footprint-photo-count').textContent = photoCount + ' 张';
      closeDialog();
      showAllStories();
      openTrip(trip);
    }

    rotateButton.addEventListener('click', toggleRotate);
    resetButton.addEventListener('click', resetView);
    zoomInButton.addEventListener('click', function () { changeZoom(-0.3); });
    zoomOutButton.addEventListener('click', function () { changeZoom(0.3); });
    focusButton.addEventListener('click', resetView);
    addButton.addEventListener('click', openDialog);
    viewAllButton.addEventListener('click', showAllStories);
    storyList.addEventListener('click', handleStoryClick);
    detailClose.addEventListener('click', closeDetail);
    dialogClose.addEventListener('click', closeDialog);
    photoInput.addEventListener('change', readGps);
    form.addEventListener('submit', saveTrip);

    var resizeObserver = new ResizeObserver(function () {
      globe.width(Math.max(320, container.clientWidth)).height(Math.max(420, container.clientHeight));
    });
    resizeObserver.observe(container);

    var themeObserver = new MutationObserver(updateTheme);
    themeObserver.observe(document.body, { attributes: true, attributeFilter: ['class'] });
    updateTheme();

    return function () {
      resizeObserver.disconnect();
      themeObserver.disconnect();
      controls.autoRotate = false;
      rotateButton.removeEventListener('click', toggleRotate);
      resetButton.removeEventListener('click', resetView);
      focusButton.removeEventListener('click', resetView);
      addButton.removeEventListener('click', openDialog);
      viewAllButton.removeEventListener('click', showAllStories);
      storyList.removeEventListener('click', handleStoryClick);
      detailClose.removeEventListener('click', closeDetail);
      dialogClose.removeEventListener('click', closeDialog);
      photoInput.removeEventListener('change', readGps);
      form.removeEventListener('submit', saveTrip);
      if (typeof globe._destructor === 'function') globe._destructor();
      container.innerHTML = '';
    };
  });
})();
