(function () {
  if (!window.BambooPageRuntime) return;
  window.BambooPageRuntime.register('footprint3d', async function (root) {
    var THREE = await import('/js/vendor/three/three.module.min.js');
    if (!root.isConnected) return;
    var container = root.querySelector('#travel-globe');
    var payload = JSON.parse(root.querySelector('#footprint3d-data').textContent);
    var message = root.querySelector('#footprint-selection');
    var errorNode = root.querySelector('[data-page-error]');
    var disposed = false, frame = 0, autoRotate = false, selected = null;
    var view = {lat: 28, lng: 108, distance: 300};
    var scene = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(42, 1, .1, 1200);
    var renderer;
    try { renderer = new THREE.WebGLRenderer({antialias:true, alpha:true}); }
    catch (error) { errorNode.textContent = '当前浏览器无法显示三维地球，仍可查看城市和地区统计。'; return; }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0,0);
    container.appendChild(renderer.domElement);
    var resources = [], textures = [], markers = [], pointerStart, pinchDistance = 0;
    var pointers = new Map(), raycaster = new THREE.Raycaster();
    var earthMaterial = new THREE.MeshPhongMaterial({color:'#ffffff', shininess:5});
    var earth = new THREE.Mesh(new THREE.SphereGeometry(100,80,64), earthMaterial);
    scene.add(earth);
    resources.push(earth);
    scene.add(new THREE.AmbientLight('#ffffff',2));
    var sun = new THREE.DirectionalLight('#ffffff',2.2);
    scene.add(sun);
    var atmosphere = new THREE.Mesh(new THREE.SphereGeometry(102,64,48), new THREE.MeshBasicMaterial({color:'#a6e2ff', transparent:true, opacity:.2, side:THREE.BackSide}));
    scene.add(atmosphere); resources.push(atmosphere);
    var loader = new THREE.TextureLoader();
    function vector(lat,lng,r) {
      var p = (90-lat)*Math.PI/180, t = (lng+180)*Math.PI/180;
      return new THREE.Vector3(-r*Math.sin(p)*Math.cos(t), r*Math.cos(p), r*Math.sin(p)*Math.sin(t));
    }
    var day = loader.load('/medias/earth/earth-blue-marble.jpg', function(t) {
      if (disposed) { t.dispose(); return; }
      t.colorSpace=THREE.SRGBColorSpace; earthMaterial.map=t; earthMaterial.needsUpdate=true;
    }, undefined, function() {errorNode.textContent='地球纹理加载失败，请刷新页面重试。';});
    textures.push(day);
    await document.fonts.load('900 48px "Font Awesome 5 Free"');
    if (!root.isConnected) { renderer.dispose(); textures.forEach(function(t){t.dispose();}); return; }
    function pinTexture(color) {
      var canvas=document.createElement('canvas'); canvas.width=64; canvas.height=80;
      var ctx=canvas.getContext('2d');
      ctx.font='900 60px "Font Awesome 5 Free"'; ctx.textAlign='center'; ctx.textBaseline='top';
      ctx.lineWidth=5; ctx.strokeStyle='#ffffff'; ctx.strokeText('\uf3c5',32,6); ctx.fillStyle=color; ctx.fillText('\uf3c5',32,6);
      var texture=new THREE.CanvasTexture(canvas); textures.push(texture); return texture;
    }
    var pinMaps={};
    payload.regions.forEach(function(r){pinMaps[r.name]=pinTexture(r.color);});
    var valid=payload.trips.filter(function(t){return t.lat!=null && t.lng!=null && String(t.lat).trim()!=='' && String(t.lng).trim()!=='' && isFinite(t.lat) && isFinite(t.lng);});
    valid.forEach(function(t){
      var sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:pinMaps[t.continent] || pinMaps['其他地区'], depthTest:true}));
      sprite.position.copy(vector(Number(t.lat),Number(t.lng),102));
      sprite.scale.set(4.8,6,1); sprite.center.set(.5,.08); sprite.userData.trip=t;
      scene.add(sprite); markers.push(sprite); resources.push(sprite);
    });
    if(valid.length!==payload.trips.length) errorNode.textContent=(payload.trips.length-valid.length)+' 个城市暂缺坐标，已计入统计。';
    fetch('/footprint/data/world.geojson').then(function(r){if(!r.ok)throw Error();return r.json();}).then(function(world){
      if(disposed)return;
      var vertices=[];
      function ring(coords) {
        if(!Array.isArray(coords) || !coords.length)return;
        if(typeof coords[0][0]==='number') {
          var step=Math.max(1,Math.floor(coords.length/160));
          for(var i=0;i<coords.length-1;i+=step) {
            var a=coords[i],b=coords[Math.min(i+step,coords.length-1)];
            vertices.push(vector(a[1],a[0],100.3),vector(b[1],b[0],100.3));
          }
        } else coords.forEach(ring);
      }
      world.features.forEach(function(f){if(f.geometry)ring(f.geometry.coordinates);});
      var lines=new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(vertices), new THREE.LineBasicMaterial({color:'#dbf4fa',transparent:true,opacity:.42}));
      scene.add(lines);resources.push(lines);
    }).catch(function(){});
    function updateCamera() {
      camera.position.copy(vector(view.lat,view.lng,view.distance)); camera.lookAt(0,0,0);
      sun.position.copy(camera.position).add(new THREE.Vector3(-80,120,50));
    }
    function resize() {
      var w=Math.max(1,container.clientWidth), h=Math.max(1,container.clientHeight);
      camera.aspect=w/h; camera.updateProjectionMatrix(); renderer.setSize(w,h);
      // Fit a complete globe even in the narrow desktop center column.
      var fit=112/Math.sin(Math.atan(Math.tan(21*Math.PI/180)*Math.min(1,w/h)));
      view.distance=Math.max(245,fit); updateCamera();
    }
    function setRotation(value) {
      autoRotate=value;
      var button=root.querySelector('#globe-rotate'); button.classList.toggle('is-active',value); button.setAttribute('aria-pressed',String(value));
    }
    function selectTrip(trip) {
      if(!trip)return;
      selected=trip.id;setRotation(false);
      root.querySelectorAll('[data-trip-id]').forEach(function(b){b.setAttribute('aria-pressed',String(b.dataset.tripId===String(trip.id)));});
      if(valid.indexOf(trip)===-1){message.textContent=trip.city+' · 暂缺定位坐标';return;}
      view.lat=Number(trip.lat);view.lng=Number(trip.lng);updateCamera();
      message.textContent=trip.city+' · '+trip.country+' · '+Number(trip.lat).toFixed(3)+', '+Number(trip.lng).toFixed(3);
    }
    function filterRegion(name) {
      var list=name?payload.trips.filter(function(t){return t.continent===name;}):payload.trips;
      root.querySelector('#footprint-region-name').textContent=name||'全部';
      root.querySelector('#footprint-region-count').textContent=list.length+' 个城市';
      root.querySelectorAll('[data-trip-id]').forEach(function(b){b.hidden=!!name && b.dataset.region!==name;});
      root.querySelectorAll('[data-filter-region]').forEach(function(b){b.setAttribute('aria-pressed',String(b.dataset.filterRegion===name));});
      markers.forEach(function(m){m.visible=!name||m.userData.trip.continent===name;});
      message.textContent=list.length?'点击城市或地图标记查看位置':name+' · 还没有足迹';
      if(list.length && name)selectTrip(list[0]);
    }
    function reset() {selected=null;view.lat=28;view.lng=108;resize();filterRegion('');root.querySelectorAll('[data-trip-id]').forEach(function(b){b.setAttribute('aria-pressed','false');});}
    function click(event) {
      var b=event.target.closest('button');if(!b)return;
      if(b.dataset.tripId)selectTrip(payload.trips.find(function(t){return String(t.id)===b.dataset.tripId;}));
      if(b.dataset.filterRegion)filterRegion(b.dataset.filterRegion);
      if(b.id==='footprint-view-all'||b.id==='globe-reset')reset();
      if(b.id==='globe-rotate')setRotation(!autoRotate);
      if(b.id==='globe-zoom-in')view.distance=Math.max(160,view.distance-30);
      if(b.id==='globe-zoom-out')view.distance=Math.min(650,view.distance+30);
      updateCamera();
    }
    function down(e) {pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});renderer.domElement.setPointerCapture(e.pointerId);pointerStart={x:e.clientX,y:e.clientY,lat:view.lat,lng:view.lng,moved:false};setRotation(false);if(pointers.size===2){var p=Array.from(pointers.values());pinchDistance=Math.hypot(p[0].x-p[1].x,p[0].y-p[1].y);}}
    function move(e) {
      if(!pointers.has(e.pointerId))return;
      pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});
      if(pointers.size===2){var p=Array.from(pointers.values()), d=Math.hypot(p[0].x-p[1].x,p[0].y-p[1].y);view.distance=Math.max(160,Math.min(650,view.distance+(pinchDistance-d)*.6));pinchDistance=d;pointerStart.moved=true;}
      else if(pointerStart){var dx=e.clientX-pointerStart.x,dy=e.clientY-pointerStart.y;pointerStart.moved=pointerStart.moved||Math.abs(dx)+Math.abs(dy)>4;view.lng=pointerStart.lng-dx*.3;view.lat=Math.max(-80,Math.min(80,pointerStart.lat+dy*.3));}
      updateCamera();
    }
    function up(e) {
      if(pointerStart&&!pointerStart.moved&&pointers.size===1){
        var box=renderer.domElement.getBoundingClientRect();
        raycaster.setFromCamera(new THREE.Vector2((e.clientX-box.left)/box.width*2-1,-(e.clientY-box.top)/box.height*2+1),camera);
        var hits=raycaster.intersectObjects([earth].concat(markers)).filter(function(h){return h.object.visible;});
        if(hits.length&&hits[0].object.userData.trip)selectTrip(hits[0].object.userData.trip);
      }
      pointers.delete(e.pointerId);pointerStart=null;
    }
    function wheel(e){e.preventDefault();setRotation(false);view.distance=Math.max(160,Math.min(650,view.distance+e.deltaY*.15));updateCamera();}
    var canvas=renderer.domElement;
    canvas.addEventListener('pointerdown',down);canvas.addEventListener('pointermove',move);canvas.addEventListener('pointerup',up);canvas.addEventListener('pointercancel',up);canvas.addEventListener('wheel',wheel,{passive:false});
    root.addEventListener('click',click);
    var observer=new ResizeObserver(resize);observer.observe(container);resize();
    function animate(){if(disposed)return;frame=requestAnimationFrame(animate);if(autoRotate){view.lng+=.035;updateCamera();}markers.forEach(function(m){var active=m.userData.trip.id===selected;m.scale.set(active?6.4:4.8,active?8:6,1);});renderer.render(scene,camera);}
    animate();
    return function(){
      disposed=true;cancelAnimationFrame(frame);observer.disconnect();root.removeEventListener('click',click);
      canvas.removeEventListener('pointerdown',down);canvas.removeEventListener('pointermove',move);canvas.removeEventListener('pointerup',up);canvas.removeEventListener('pointercancel',up);canvas.removeEventListener('wheel',wheel);
      resources.forEach(function(o){if(o.geometry)o.geometry.dispose();if(o.material)o.material.dispose();});
      textures.forEach(function(t){t.dispose();});renderer.dispose();container.innerHTML='';
    };
  });
})();
