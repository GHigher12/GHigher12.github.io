(function () {
  if (!window.BambooPageRuntime) return;

  window.BambooPageRuntime.register('love-space', async function (root) {
    var STORAGE_KEY = 'bambooLoveSpace:v1';
    var SESSION_KEY = 'bambooLoveUnlocked';
    var currentSection = 'home';
    var currentAddType = 'tasks';
    var maps = [];
    var elapsedTimer = null;
    var geocodeRequestId = 0;
    var tripCoordinatePromise = null;
    var knownCityCoordinates = {
      '北京': [39.9042, 116.4074], '天津': [39.1333, 117.2000], '上海': [31.2304, 121.4737],
      '淄博': [36.8131, 118.0548], '临沂': [35.1047, 118.3564], '香港': [22.3193, 114.1694],
      '广州': [23.1291, 113.2644], '深圳': [22.5431, 114.0579], '杭州': [30.2741, 120.1551],
      '日照': [35.4164, 119.5269], '威海': [37.5131, 122.1204], '烟台': [37.4638, 121.4479],
      '济南': [36.6512, 117.1201], '廊坊': [39.5380, 116.6838], '德州': [37.4367, 116.3595],
      '青岛': [36.0671, 120.3826], '沈阳': [41.8057, 123.4315], '菏泽': [35.2338, 115.4807]
    };

    var lockView = root.querySelector('#love-lock');
    var app = root.querySelector('#love-app');
    var lockForm = root.querySelector('#love-lock-form');
    var passwordInput = root.querySelector('#love-password');
    var passwordToggle = root.querySelector('#love-password-toggle');
    var lockError = root.querySelector('#love-lock-error');
    var lockNow = root.querySelector('#love-lock-now');
    var nav = root.querySelector('#love-nav');
    var dashboard = root.querySelector('#love-dashboard');
    var sectionView = root.querySelector('#love-section-view');
    var sectionContent = root.querySelector('#love-section-content');
    var sectionSearch = root.querySelector('#love-section-search');
    var sectionAdd = root.querySelector('#love-section-add');
    var pageTitle = root.querySelector('#love-page-title');
    var quickAdd = root.querySelector('#love-quick-add');
    var dialog = root.querySelector('#love-dialog');
    var recordForm = root.querySelector('#love-record-form');
    var dialogFields = root.querySelector('#love-dialog-fields');
    var dialogTitle = root.querySelector('#love-dialog-title');
    var dialogClose = root.querySelector('[data-love-dialog-close]');
    var formStatus = root.querySelector('#love-form-status');
    var lightbox = root.querySelector('#love-lightbox');
    var lightboxImage = root.querySelector('#love-lightbox-image');
    var lightboxCaption = root.querySelector('#love-lightbox-caption');
    var lightboxPrev = root.querySelector('[data-lightbox-prev]');
    var lightboxNext = root.querySelector('[data-lightbox-next]');
    var lightboxItems = [];
    var lightboxIndex = 0;
    var cropDialog = root.querySelector('#love-crop-dialog');
    var cropCanvas = root.querySelector('#love-crop-canvas');
    var cropContext = cropCanvas.getContext('2d');
    var cropAspect = root.querySelector('#love-crop-aspect');
    var cropZoom = root.querySelector('#love-crop-zoom');
    var cropConfirm = root.querySelector('#love-crop-confirm');
    var cropCancel = root.querySelector('#love-crop-cancel');
    var cropClose = root.querySelector('#love-crop-close');
    var cropState = { input: null, image: null, zoom: 1, offsetX: 0, offsetY: 0, dragging: false, x: 0, y: 0 };

    var defaultState = {
      version: 7,
      sourceRevision: '',
      profile: { startDate: '2026-04-11', metDate: '2026-04-11', nameA: '我', nameB: '你' },
      tasks: [],
      trips: [],
      media: [],
      timeline: []
    };

    var markdownDataNode = root.querySelector('#love-markdown-data');
    if (markdownDataNode) {
      try {
        var markdownData = JSON.parse(markdownDataNode.textContent || '{}');
        defaultState.sourceRevision = markdownData.__revision || '';
        if (markdownData.profile) defaultState.profile = Object.assign({}, defaultState.profile, markdownData.profile);
        if (Array.isArray(markdownData.tasks)) {
          defaultState.tasks = markdownData.tasks.map(function (task, index) {
            return Object.assign({
              id: 'task-' + index,
              title: '',
              category: index < 20 ? '日常' : index < 50 ? '旅行' : '未来',
              done: false,
              date: '',
              location: '',
              note: '',
              photo: '',
              emoji: ''
            }, task);
          });
        }
        if (Array.isArray(markdownData.trips)) defaultState.trips = markdownData.trips;
        if (Array.isArray(markdownData.timeline)) defaultState.timeline = markdownData.timeline;
        if (Array.isArray(markdownData.media)) {
          defaultState.media = markdownData.media.map(function (item, index) {
            if (typeof item === 'string') {
              return { id: 'media-' + index, url: item, caption: '', date: '', place: '' };
            }
            return Object.assign({ id: 'media-' + index, url: '', caption: '', date: '', place: '' }, item);
          }).filter(function (item) { return !!item.url; });
        }
      } catch (error) {
        console.error('小窝 Markdown 数据读取失败', error);
      }
    }

    function cloneDefaults() { return JSON.parse(JSON.stringify(defaultState)); }

    function loadState() {
      try {
        var saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || 'null');
        if (!saved) return cloneDefaults();
        if ((saved.sourceRevision || '') !== (defaultState.sourceRevision || '')) {
          var refreshedState = cloneDefaults();
          localStorage.setItem(STORAGE_KEY, JSON.stringify(refreshedState));
          return refreshedState;
        }
        var nextState = Object.assign(cloneDefaults(), saved, { profile: Object.assign({}, defaultState.profile, saved.profile || {}) });
        if (!saved.version || saved.version < defaultState.version) {
          nextState = cloneDefaults();
          localStorage.setItem(STORAGE_KEY, JSON.stringify(nextState));
        }
        nextState.tasks = (nextState.tasks || []).map(function (task) {
          return Object.assign({ location: '', note: '', photo: '', emoji: '' }, task);
        });
        nextState.version = defaultState.version;
        return nextState;
      } catch (error) {
        return cloneDefaults();
      }
    }

    var state = loadState();

    function saveState() {
      try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
        formStatus.textContent = '';
        return true;
      } catch (error) {
        formStatus.textContent = '照片较大，浏览器空间不足。请尝试选择更小的照片。';
        return false;
      }
    }

    function escapeHtml(value) {
      return String(value == null ? '' : value).replace(/[&<>'"]/g, function (char) {
        return { '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[char];
      });
    }

    async function sha256(value) {
      var bytes = new TextEncoder().encode(value);
      var digest = await crypto.subtle.digest('SHA-256', bytes);
      return Array.from(new Uint8Array(digest)).map(function (byte) { return byte.toString(16).padStart(2, '0'); }).join('');
    }

    function formatDateCN(value) {
      if (!value) return '尚未设置';
      var date = new Date(value + 'T00:00:00');
      return date.getFullYear() + '年' + (date.getMonth() + 1) + '月' + date.getDate() + '日';
    }

    function tripLandmarkPhoto(trip) {
      if (trip.photo) return trip.photo;
      if (trip.city === '\u5317\u4eac') return '/medias/love/beijing-forbidden-city.jpg';
      if (trip.city === '\u5929\u6d25') return '/medias/love/tianjin-eye.jpg';
      return '/medias/love/alpine-lake.png';
    }

    function updateDashboard() {
      var now = new Date();
      var completed = state.tasks.filter(function (task) { return task.done; }).length;

      root.querySelector('#love-start-date').textContent = formatDateCN(state.profile.startDate);
      root.querySelector('#love-met-date').textContent = formatDateCN(state.profile.metDate);
      root.querySelector('#love-stat-trips').textContent = state.trips.length + ' 次';
      root.querySelector('#love-stat-photos').textContent = state.media.length + ' 张';
      root.querySelector('#love-stat-tasks').textContent = completed + ' / 100';
      root.querySelector('#dashboard-task-progress').textContent = '已完成 ' + completed + '/100';
      root.querySelector('#dashboard-task-bar').style.width = Math.min(100, completed) + '%';

      root.querySelector('#dashboard-task-list').innerHTML = state.tasks.slice(0, 4).map(taskCard).join('');
      root.querySelector('#dashboard-trip-strip').innerHTML = state.trips.slice(0, 4).map(function (trip) {
        var photo = tripLandmarkPhoto(trip);
        return '<button type="button" data-photo="' + escapeHtml(photo) + '" data-caption="' + escapeHtml(trip.city + ' · ' + trip.date) + '"><img src="' + escapeHtml(photo) + '" alt="' + escapeHtml(trip.city) + '标志性景点"><span><strong>' + escapeHtml(trip.city) + '</strong><small>' + escapeHtml(trip.date) + '</small></span></button>';
      }).join('');
      root.querySelector('#dashboard-photo-grid').innerHTML = state.media.slice(0, 5).map(photoCard).join('');
      root.querySelector('#dashboard-timeline-list').innerHTML = state.timeline.slice().reverse().map(function (item) {
        return '<article><i></i><div><small>' + escapeHtml(item.date) + '</small><strong>' + escapeHtml(item.title) + '</strong><p>' + escapeHtml(item.text) + '</p></div><img src="' + escapeHtml(item.photo || '/medias/love/couple-sunset.png') + '" alt="' + escapeHtml(item.title) + '"></article>';
      }).join('');
    }

    function updateElapsedClock() {
      var start = new Date(state.profile.startDate + 'T00:00:00');
      var now = new Date();
      var years = now.getFullYear() - start.getFullYear();
      var anniversary = new Date(start);
      anniversary.setFullYear(start.getFullYear() + years);
      if (anniversary > now) { years -= 1; anniversary.setFullYear(anniversary.getFullYear() - 1); }
      var seconds = Math.max(0, Math.floor((now - anniversary) / 1000));
      var days = Math.floor(seconds / 86400); seconds %= 86400;
      var hours = Math.floor(seconds / 3600); seconds %= 3600;
      var minutes = Math.floor(seconds / 60); seconds %= 60;
      root.querySelector('#love-elapsed-clock').textContent = years + '年 ' + days + '天 ' + hours + '时 ' + minutes + '分 ' + seconds + '秒';
    }

    function startElapsedClock() {
      if (elapsedTimer) window.clearInterval(elapsedTimer);
      updateElapsedClock();
      elapsedTimer = window.setInterval(updateElapsedClock, 1000);
    }

    function taskCard(task) {
      var taskMeta = task.done ? ((task.date || '已完成') + (task.location ? ' · ' + task.location : '')) : task.category;
      var taskPhoto = task.photo ? '<img class="love-task-photo" src="' + escapeHtml(task.photo) + '" alt="' + escapeHtml(task.title) + '打卡照片">' : '';
      var taskEmoji = task.emoji ? '<b class="love-task-emoji">' + escapeHtml(task.emoji) + '</b>' : '';
      return '<button type="button" class="love-task-item ' + (task.done ? 'is-done' : '') + '" data-task-open="' + escapeHtml(task.id) + '"><span class="task-check"><i class="fas fa-check"></i></span><span><strong>' + escapeHtml(task.title) + '</strong><small>' + escapeHtml(taskMeta) + '</small></span>' + taskEmoji + taskPhoto + '</button>';
    }

    function taskManageCard(task) {
      return '<article class="love-task-manage">' + taskCard(task) + '</article>';
    }

    function photoCard(item) {
      return '<button type="button" data-photo="' + escapeHtml(item.url) + '" data-caption="' + escapeHtml((item.caption || '我们的回忆') + ' · ' + (item.place || item.date || '')) + '"><img src="' + escapeHtml(item.url) + '" alt="' + escapeHtml(item.caption || '回忆照片') + '"><span>' + escapeHtml(item.caption || '我们的回忆') + '</span></button>';
    }

    function albumManageCard(item) {
      return '<article class="love-album-manage">' + photoCard(item) + '</article>';
    }

    function collectionForType(type) {
      return type === 'album' ? state.media : state[type];
    }

    var sectionMeta = {
      tasks: ['100件小事', '新增一件小事'], trips: ['旅行记录', '新增旅行'], timeline: ['回忆时间轴', '新增节点'], album: ['照片相册', '上传照片'], settings: ['空间设置', '编辑资料']
    };

    function renderSection(section, query) {
      currentSection = section;
      query = (query || '').trim().toLowerCase();
      pageTitle.textContent = sectionMeta[section] ? sectionMeta[section][0] : '我们的时光';
      sectionAdd.innerHTML = '<i class="fas fa-plus"></i> ' + (sectionMeta[section] ? sectionMeta[section][1] : '新增记录');
      sectionAdd.hidden = section === 'settings';
      dashboard.hidden = true;
      sectionView.hidden = false;

      var html = '';
      if (section === 'tasks') {
        var tasks = state.tasks.filter(function (item) { return !query || item.title.toLowerCase().indexOf(query) !== -1; });
        var done = state.tasks.filter(function (item) { return item.done; }).length;
        html = '<div class="love-section-summary"><div><span>共同进度</span><strong>' + done + ' / 100</strong></div><div class="love-progress"><span style="width:' + done + '%"></span></div></div><div class="love-task-grid">' + tasks.map(taskManageCard).join('') + '</div>';
      } else if (section === 'trips') {
        var trips = state.trips.filter(function (item) { return !query || item.city.toLowerCase().indexOf(query) !== -1 || item.story.toLowerCase().indexOf(query) !== -1; });
        html = '<div class="love-travel-layout"><div class="love-map love-map-large" id="love-trips-map"><span><i class="fas fa-map-marked-alt"></i> 正在加载旅行地图</span></div><div class="love-trip-cards">' + trips.map(function (trip) {
          var coordinateCopy = hasValidCoordinates(trip)
            ? Number(trip.lat).toFixed(3) + ', ' + Number(trip.lng).toFixed(3)
            : (trip._geocodeFailed ? '高德暂未定位到该地点' : '正在通过高德地图定位…');
          return '<article><img src="' + escapeHtml(tripLandmarkPhoto(trip)) + '" alt="' + escapeHtml(trip.city) + '标志性景点"><div><small>' + escapeHtml(trip.date) + '</small><h3>' + escapeHtml(trip.city) + '</h3><p>' + escapeHtml(trip.story) + '</p><span><i class="fas fa-map-marker-alt"></i> ' + coordinateCopy + '</span></div></article>';
        }).join('') + '</div></div>';
      } else if (section === 'album') {
        var media = state.media.filter(function (item) { return !query || (item.caption || '').toLowerCase().indexOf(query) !== -1 || (item.place || '').toLowerCase().indexOf(query) !== -1; });
        html = '<div class="love-album-grid">' + media.map(albumManageCard).join('') + '</div>';
      } else if (section === 'timeline') {
        html = '<div class="love-timeline-full">' + state.timeline.slice().reverse().filter(function (item) { return !query || item.title.toLowerCase().indexOf(query) !== -1; }).map(function (item) {
          return '<article><div class="timeline-date">' + escapeHtml(item.date) + '</div><i></i><div class="timeline-card">' + (item.photo ? '<img src="' + escapeHtml(item.photo) + '" alt="' + escapeHtml(item.title) + '">' : '') + '<small>' + escapeHtml(item.mood || '回忆') + '</small><h3>' + escapeHtml(item.title) + '</h3><p>' + escapeHtml(item.text) + '</p></div></article>';
        }).join('') + '</div>';
      } else if (section === 'settings') {
        html = '<form class="love-settings-form" id="love-settings-form"><div class="love-settings-card"><h2>重要日期</h2><label>恋爱开始日期<input type="date" name="startDate" value="' + escapeHtml(state.profile.startDate) + '"></label><label>相识日期<input type="date" name="metDate" value="' + escapeHtml(state.profile.metDate) + '"></label><button type="submit" class="love-primary-button">保存设置</button></div><div class="love-settings-card"><h2>隐私与数据</h2><p>当前预览数据保存在本机浏览器中，不会上传到博客静态文件。</p><button type="button" data-export-love><i class="fas fa-download"></i> 导出备份</button><button type="button" data-reset-love class="danger"><i class="fas fa-trash-alt"></i> 恢复示例数据</button></div></form>';
      }
      sectionContent.innerHTML = html || '<div class="love-empty">还没有记录，点击右上角添加第一条。</div>';
      if (section === 'trips') requestAnimationFrame(function () { initMap('love-trips-map', state.trips); });
    }

    function showHome() {
      currentSection = 'home';
      pageTitle.textContent = '我们的时光';
      dashboard.hidden = false;
      sectionView.hidden = true;
      updateDashboard();
      startElapsedClock();
      requestAnimationFrame(function () { initMap('love-dashboard-map', state.trips); });
    }

    function activateSection(section) {
      nav.querySelectorAll('[data-love-section]').forEach(function (button) { button.classList.toggle('is-active', button.dataset.loveSection === section); });
      sectionSearch.value = '';
      if (section === 'home') showHome();
      else {
        renderSection(section);
        if (section === 'trips') refreshTripCoordinates();
      }
    }

    function field(name, label, type, extra) {
      type = type || 'text';
      return '<label>' + label + '<input name="' + name + '" type="' + type + '" ' + (extra || '') + '></label>';
    }

    function selectOptions(values, selected) {
      return values.map(function (value) { return '<option' + (value === selected ? ' selected' : '') + '>' + escapeHtml(value) + '</option>'; }).join('');
    }

    function taskEmojiPicker(selected) {
      var emojis = ['💗', '🥰', '✨', '🎉', '🌷', '🫶', '📸', '🍰', '🧳', '🌈', '💫', '🥂'];
      return '<label>打卡表情<input type="hidden" name="emoji" value="' + escapeHtml(selected || '💗') + '"><span class="love-emoji-picker">' + emojis.map(function (emoji) {
        return '<button type="button" class="' + (emoji === (selected || '💗') ? 'is-selected' : '') + '" data-task-emoji="' + emoji + '" aria-label="选择表情 ' + emoji + '">' + emoji + '</button>';
      }).join('') + '</span></label>';
    }

    function openDialog(type) {
      currentAddType = type === 'home' ? 'tasks' : type;
      formStatus.textContent = '';
      recordForm.reset();
      dialogTitle.textContent = sectionMeta[currentAddType] ? sectionMeta[currentAddType][1] : '记录此刻';
      var today = new Date().toISOString().slice(0, 10);
      var fields = '';
      if (currentAddType === 'tasks') fields = field('title', '想一起完成的事情', 'text', 'required placeholder="例如：一起去看海 🌊"') + '<label>分类<select name="category">' + selectOptions(['日常', '旅行', '成长', '未来'], '日常') + '</select></label>' + taskEmojiPicker('💗');
      else if (currentAddType === 'trips') fields = field('city', '旅行地点', 'text', 'required placeholder="输入城市或景点后自动定位"') + field('date', '旅行日期', 'date', 'required value="' + today + '"') + '<input name="lat" type="hidden"><input name="lng" type="hidden"><label>照片<input name="photo" type="file" accept="image/*"><small>可上传旅行照片</small></label><label>旅行故事<textarea name="story" rows="4" required></textarea></label><p class="love-gps-status" data-love-gps></p>';
      else if (currentAddType === 'album') fields = '<label>照片<input name="photo" type="file" accept="image/*" required><small>选择后将自动打开裁剪器</small></label>';
      else if (currentAddType === 'timeline') fields = field('title', '回忆标题', 'text', 'required') + field('date', '发生日期', 'date', 'required value="' + today + '"') + field('mood', '心情标签', 'text', 'placeholder="开心、浪漫、感动"') + '<label>回忆照片<input name="photo" type="file" accept="image/*"><small>可选</small></label><label>回忆故事<textarea name="text" rows="4" required></textarea></label>';
      dialogFields.innerHTML = fields;
      var photo = recordForm.elements.photo;
      if (photo && currentAddType === 'trips') photo.addEventListener('change', readDialogGps, { once: true });
      var cityInput = recordForm.elements.city;
      if (cityInput && currentAddType === 'trips') {
        cityInput.addEventListener('change', geocodeDialogCity);
        cityInput.addEventListener('blur', geocodeDialogCity);
      }
      if (typeof dialog.showModal === 'function') dialog.showModal(); else dialog.setAttribute('open', '');
    }

    function closeDialog() {
      if (typeof dialog.close === 'function') dialog.close(); else dialog.removeAttribute('open');
    }

    async function readDialogGps() {
      var file = recordForm.elements.photo.files && recordForm.elements.photo.files[0];
      var status = dialogFields.querySelector('[data-love-gps]');
      if (!file || !status) return;
      status.textContent = '正在读取照片 GPS…';
      try {
        await window.BambooPageRuntime.loadScript('/js/vendor/exifr.umd.js', 'exifr');
        var gps = await window.exifr.gps(file);
        if (gps) {
          recordForm.elements.lat.value = gps.latitude.toFixed(6);
          recordForm.elements.lng.value = gps.longitude.toFixed(6);
          status.textContent = '已读取照片位置。';
        } else status.textContent = '照片没有 GPS，请手动填写。';
      } catch (error) { status.textContent = '未能读取 GPS，请手动填写。'; }
    }

    async function loadAMapApi() {
      if (window.AMap) return window.AMap;
      var key = root.dataset.amapKey;
      if (!key) throw new Error('未配置高德地图 Key');
      window._AMapSecurityConfig = { securityJsCode: root.dataset.amapSecurity || '' };
      await window.BambooPageRuntime.loadScript('https://webapi.amap.com/maps?v=2.0&key=' + encodeURIComponent(key), 'AMap');
      return window.AMap;
    }

    function hasValidCoordinates(trip) {
      if (!trip || trip.lat === null || trip.lng === null || trip.lat === undefined || trip.lng === undefined) return false;
      if (String(trip.lat).trim() === '' || String(trip.lng).trim() === '') return false;
      var lat = Number(trip.lat);
      var lng = Number(trip.lng);
      return isFinite(lat) && isFinite(lng) && lat >= -90 && lat <= 90 && lng >= -180 && lng <= 180;
    }

    function loadAMapGeocoder(AMap) {
      return new Promise(function (resolve, reject) {
        var settled = false;
        var timer = window.setTimeout(function () {
          if (!settled) { settled = true; reject(new Error('高德地理编码组件加载超时')); }
        }, 8000);
        try {
          AMap.plugin('AMap.Geocoder', function () {
            if (settled) return;
            settled = true;
            window.clearTimeout(timer);
            resolve(new AMap.Geocoder({ city: '全国' }));
          });
        } catch (error) {
          window.clearTimeout(timer);
          reject(error);
        }
      });
    }

    function geocodeAddress(geocoder, address) {
      return new Promise(function (resolve) {
        var settled = false;
        var timer = window.setTimeout(function () {
          if (!settled) { settled = true; resolve(null); }
        }, 2500);
        try {
          geocoder.getLocation(address, function (resultStatus, result) {
            if (settled) return;
            settled = true;
            window.clearTimeout(timer);
            var geocode = resultStatus === 'complete' && result && result.geocodes && result.geocodes[0];
            resolve(geocode && geocode.location ? geocode : null);
          });
        } catch (error) {
          window.clearTimeout(timer);
          resolve(null);
        }
      });
    }

    async function geocodeDialogCity() {
      if (currentAddType !== 'trips') return;
      var cityInput = recordForm.elements.city;
      var status = dialogFields.querySelector('[data-love-gps]');
      var address = cityInput && cityInput.value.trim();
      if (!address || !status) return;
      var requestId = ++geocodeRequestId;
      status.textContent = '正在根据地点获取经纬度…';
      try {
        var AMap = await loadAMapApi();
        AMap.plugin('AMap.Geocoder', function () {
          var geocoder = new AMap.Geocoder({ city: '全国' });
          geocoder.getLocation(address, function (resultStatus, result) {
            if (requestId !== geocodeRequestId) return;
            var geocode = resultStatus === 'complete' && result.geocodes && result.geocodes[0];
            if (!geocode || !geocode.location) {
              status.textContent = '没有找到该地点，请输入更完整的城市或景点名称。';
              return;
            }
            recordForm.elements.lng.value = Number(geocode.location.lng).toFixed(6);
            recordForm.elements.lat.value = Number(geocode.location.lat).toFixed(6);
            status.textContent = '已定位：' + (geocode.formattedAddress || address) + '，坐标仍可手动修改。';
          });
        });
      } catch (error) {
        if (requestId === geocodeRequestId) status.textContent = '自动定位失败，请检查网络或手动填写经纬度。';
      }
    }

    async function resolveTripCoordinates(trips) {
      var missing = (trips || []).filter(function (trip) {
        return !hasValidCoordinates(trip);
      });
      if (!missing.length) return 0;
      try {
        var AMap = await loadAMapApi();
        var geocoder = await loadAMapGeocoder(AMap);
        var results = await Promise.all(missing.map(function (trip) { return geocodeAddress(geocoder, trip.city); }));
        var resolvedCount = 0;
        results.forEach(function (geocode, index) {
          var trip = missing[index];
          var normalizedCity = String(trip.city || '').trim().replace(/(市|地区|特别行政区)$/u, '');
          var fallback = knownCityCoordinates[normalizedCity];
          trip._geocodeFailed = !geocode && !fallback;
          if (geocode) {
            trip.lng = Number(geocode.location.lng);
            trip.lat = Number(geocode.location.lat);
          } else if (fallback) {
            trip.lat = fallback[0];
            trip.lng = fallback[1];
          } else return;
          delete trip._geocodeFailed;
          resolvedCount += 1;
        });
        return resolvedCount;
      } catch (error) {
        console.warn('小窝旅行地点自动定位失败', error);
        return 0;
      }
    }

    function refreshTripCoordinates() {
      if (!tripCoordinatePromise) {
        tripCoordinatePromise = resolveTripCoordinates(state.trips).then(function (resolvedCount) {
          if (resolvedCount > 0) saveState();
          return resolvedCount;
        }).finally(function () { tripCoordinatePromise = null; });
      }
      return tripCoordinatePromise.then(function (resolvedCount) {
        destroyMaps();
        updateDashboard();
        if (currentSection === 'trips') renderSection('trips', sectionSearch.value);
        else if (currentSection === 'home') requestAnimationFrame(function () { initMap('love-dashboard-map', state.trips); });
        return resolvedCount;
      });
    }

    function imageToDataUrl(file) {
      return new Promise(function (resolve, reject) {
        var reader = new FileReader();
        reader.onerror = reject;
        reader.onload = function () {
          var image = new Image();
          image.onload = function () {
            var max = 1280;
            var ratio = Math.min(1, max / Math.max(image.width, image.height));
            var canvas = document.createElement('canvas');
            canvas.width = Math.round(image.width * ratio);
            canvas.height = Math.round(image.height * ratio);
            canvas.getContext('2d').drawImage(image, 0, 0, canvas.width, canvas.height);
            resolve(canvas.toDataURL('image/jpeg', .82));
          };
          image.onerror = reject;
          image.src = reader.result;
        };
        reader.readAsDataURL(file);
      });
    }

    function cropCanvasSize() {
      var ratio = Number(cropAspect.value) || 1;
      if (ratio >= 1) {
        cropCanvas.width = 960;
        cropCanvas.height = Math.round(960 / ratio);
      } else {
        cropCanvas.height = 960;
        cropCanvas.width = Math.round(960 * ratio);
      }
    }

    function drawCropImage() {
      if (!cropState.image) return;
      var image = cropState.image;
      var baseScale = Math.max(cropCanvas.width / image.naturalWidth, cropCanvas.height / image.naturalHeight);
      var scale = baseScale * cropState.zoom;
      var width = image.naturalWidth * scale;
      var height = image.naturalHeight * scale;
      var maxX = Math.max(0, (width - cropCanvas.width) / 2);
      var maxY = Math.max(0, (height - cropCanvas.height) / 2);
      cropState.offsetX = Math.max(-maxX, Math.min(maxX, cropState.offsetX));
      cropState.offsetY = Math.max(-maxY, Math.min(maxY, cropState.offsetY));
      cropContext.clearRect(0, 0, cropCanvas.width, cropCanvas.height);
      cropContext.drawImage(image, (cropCanvas.width - width) / 2 + cropState.offsetX, (cropCanvas.height - height) / 2 + cropState.offsetY, width, height);
    }

    function openImageCrop(input, file) {
      if (!file || !file.type || file.type.indexOf('image/') !== 0) return;
      var reader = new FileReader();
      reader.onload = function () {
        var image = new Image();
        image.onload = function () {
          cropState.input = input;
          cropState.image = image;
          cropState.zoom = 1;
          cropState.offsetX = 0;
          cropState.offsetY = 0;
          cropZoom.value = '1';
          cropAspect.value = currentAddType === 'tasks' ? '1' : '1.333333';
          cropCanvasSize();
          drawCropImage();
          if (typeof cropDialog.showModal === 'function') cropDialog.showModal(); else cropDialog.setAttribute('open', '');
        };
        image.src = reader.result;
      };
      reader.readAsDataURL(file);
    }

    function closeImageCrop(clearInput) {
      if (clearInput && cropState.input) {
        cropState.input.value = '';
        delete cropState.input.dataset.croppedImage;
      }
      if (typeof cropDialog.close === 'function') cropDialog.close(); else cropDialog.removeAttribute('open');
      cropState.input = null;
      cropState.image = null;
      cropState.dragging = false;
    }

    function confirmImageCrop() {
      if (!cropState.input || !cropState.image) return closeImageCrop(false);
      cropState.input.dataset.croppedImage = cropCanvas.toDataURL('image/jpeg', .88);
      var label = cropState.input.closest('label');
      if (label) {
        var ready = label.querySelector('.love-crop-ready');
        if (!ready) {
          ready = document.createElement('span');
          ready.className = 'love-crop-ready';
          label.appendChild(ready);
        }
        ready.innerHTML = '<i class="fas fa-check-circle"></i> 图片已裁剪，保存记录后生效';
      }
      closeImageCrop(false);
    }

    function handleImageSelection(event) {
      var input = event.target.closest('input[type="file"][accept*="image"]');
      if (!input) return;
      delete input.dataset.croppedImage;
      var file = input.files && input.files[0];
      if (file) openImageCrop(input, file);
    }

    function handleCropPointerDown(event) {
      if (!cropState.image) return;
      cropState.dragging = true;
      cropState.x = event.clientX;
      cropState.y = event.clientY;
    }

    function handleCropPointerMove(event) {
      if (!cropState.dragging || !cropState.image) return;
      var rect = cropCanvas.getBoundingClientRect();
      cropState.offsetX += (event.clientX - cropState.x) * (cropCanvas.width / rect.width);
      cropState.offsetY += (event.clientY - cropState.y) * (cropCanvas.height / rect.height);
      cropState.x = event.clientX;
      cropState.y = event.clientY;
      drawCropImage();
    }

    function handleCropPointerUp() { cropState.dragging = false; }

    function handleCropAspect() {
      cropState.offsetX = 0;
      cropState.offsetY = 0;
      cropCanvasSize();
      drawCropImage();
    }

    function handleCropZoom() {
      cropState.zoom = Number(cropZoom.value) || 1;
      drawCropImage();
    }

    function cancelImageCrop(event) {
      if (event) event.preventDefault();
      closeImageCrop(true);
    }

    function croppedImageFromInput(input) {
      if (!input) return Promise.resolve('');
      if (input.dataset.croppedImage) return Promise.resolve(input.dataset.croppedImage);
      var file = input.files && input.files[0];
      return file ? imageToDataUrl(file) : Promise.resolve('');
    }

    async function saveRecord(event) {
      event.preventDefault();
      var data = new FormData(recordForm);
      if (currentAddType === 'trips' && (!isFinite(Number(data.get('lat'))) || !isFinite(Number(data.get('lng'))) || !String(data.get('lat')).trim() || !String(data.get('lng')).trim())) {
        var pendingTripLocation = { city: String(data.get('city') || '').trim() };
        await resolveTripCoordinates([pendingTripLocation]);
        if (!isFinite(Number(pendingTripLocation.lat)) || !isFinite(Number(pendingTripLocation.lng))) {
          throw new Error('无法定位该地点，请输入更完整的城市或景点名称。');
        }
        data.set('lat', pendingTripLocation.lat);
        data.set('lng', pendingTripLocation.lng);
      }
      var id = currentAddType + '-' + Date.now();
      var item;
      if (currentAddType === 'tasks') item = { id: id, title: data.get('title'), category: data.get('category'), done: false, date: '', location: '', note: '', photo: '', emoji: data.get('emoji') || '💗' };
      else if (currentAddType === 'trips') {
        var tripPhoto = await croppedImageFromInput(recordForm.elements.photo);
        item = { id: id, city: data.get('city'), date: data.get('date'), lat: Number(data.get('lat')), lng: Number(data.get('lng')), story: data.get('story'), photo: tripPhoto };
        if (tripPhoto) state.media.unshift({ id: 'media-' + Date.now(), url: tripPhoto, caption: data.get('city') + '旅行', date: data.get('date'), place: data.get('city') });
      } else if (currentAddType === 'album') {
        item = { id: id, url: await croppedImageFromInput(recordForm.elements.photo), caption: '', date: '', place: '' };
      } else if (currentAddType === 'timeline') {
        item = { id: id, title: data.get('title'), date: data.get('date'), mood: data.get('mood'), text: data.get('text'), photo: await croppedImageFromInput(recordForm.elements.photo) };
      }
      if (!item) return;
      collectionForType(currentAddType).unshift(item);
      if (currentAddType === 'trips') destroyMaps();
      if (saveState()) {
        closeDialog();
        updateDashboard();
        if (currentSection !== 'home') renderSection(currentSection, sectionSearch.value);
      }
    }

    function toggleTask(id) {
      var task = state.tasks.find(function (item) { return item.id === id; });
      if (!task) return;
      task.done = !task.done;
      task.date = task.done ? new Date().toISOString().slice(0, 10) : '';
      saveState();
      updateDashboard();
      if (currentSection === 'tasks') renderSection('tasks', sectionSearch.value);
    }

    function showLightboxPhoto(index) {
      if (!lightboxItems.length) return;
      lightboxIndex = (index + lightboxItems.length) % lightboxItems.length;
      var item = lightboxItems[lightboxIndex];
      lightboxImage.src = item.url;
      lightboxCaption.textContent = item.caption || '我们的回忆';
      var hasMultiple = lightboxItems.length > 1;
      lightboxPrev.hidden = !hasMultiple;
      lightboxNext.hidden = !hasMultiple;
    }

    function openPhoto(url, caption) {
      if (!url) return;
      var seen = {};
      lightboxItems = Array.from(root.querySelectorAll('[data-photo]')).map(function (button) {
        return { url: button.dataset.photo, caption: button.dataset.caption || '我们的回忆' };
      }).filter(function (item) {
        if (!item.url || seen[item.url]) return false;
        seen[item.url] = true;
        return true;
      });
      lightboxIndex = lightboxItems.findIndex(function (item) { return item.url === url; });
      if (lightboxIndex < 0) {
        lightboxItems.push({ url: url, caption: caption || '我们的回忆' });
        lightboxIndex = lightboxItems.length - 1;
      }
      showLightboxPhoto(lightboxIndex);
      if (typeof lightbox.showModal === 'function') lightbox.showModal(); else lightbox.setAttribute('open', '');
    }

    function navigateLightbox(step) {
      if (lightboxItems.length > 1) showLightboxPhoto(lightboxIndex + step);
    }

    function closeLightbox() {
      if (typeof lightbox.close === 'function') lightbox.close(); else lightbox.removeAttribute('open');
    }

    function handleLightboxKeydown(event) {
      if (event.key === 'ArrowLeft') { event.preventDefault(); navigateLightbox(-1); }
      if (event.key === 'ArrowRight') { event.preventDefault(); navigateLightbox(1); }
    }

    function handleRootClick(event) {
      var navButton = event.target.closest('[data-love-section]');
      if (navButton) return activateSection(navButton.dataset.loveSection);
      var openSection = event.target.closest('[data-open-section]');
      if (openSection) return activateSection(openSection.dataset.openSection);
      var addType = event.target.closest('[data-add-type]');
      if (addType) return openDialog(addType.dataset.addType);
      var taskEmoji = event.target.closest('[data-task-emoji]');
      if (taskEmoji) {
        var emojiInput = dialogFields.querySelector('input[name="emoji"]');
        if (emojiInput) emojiInput.value = taskEmoji.dataset.taskEmoji;
        dialogFields.querySelectorAll('[data-task-emoji]').forEach(function (button) { button.classList.toggle('is-selected', button === taskEmoji); });
        return;
      }
      var task = event.target.closest('[data-task-open]');
      if (task) return toggleTask(task.dataset.taskOpen);
      var photo = event.target.closest('[data-photo]');
      if (photo) return openPhoto(photo.dataset.photo, photo.dataset.caption);
      var closeBox = event.target.closest('[data-lightbox-close]');
      if (closeBox) return closeLightbox();
      var previousPhoto = event.target.closest('[data-lightbox-prev]');
      if (previousPhoto) return navigateLightbox(-1);
      var nextPhoto = event.target.closest('[data-lightbox-next]');
      if (nextPhoto) return navigateLightbox(1);
      var exportButton = event.target.closest('[data-export-love]');
      if (exportButton) return exportData();
      var resetButton = event.target.closest('[data-reset-love]');
      if (resetButton && confirm('确定恢复示例数据吗？本机新增的记录会被清除。')) {
        state = cloneDefaults(); saveState(); renderSection('settings');
      }
    }

    function exportData() {
      var blob = new Blob([JSON.stringify(state, null, 2)], { type: 'application/json' });
      var url = URL.createObjectURL(blob);
      var link = document.createElement('a');
      link.href = url;
      link.download = 'our-memory-backup-' + new Date().toISOString().slice(0, 10) + '.json';
      link.click();
      URL.revokeObjectURL(url);
    }

    async function initMap(id, trips) {
      var element = root.querySelector('#' + id);
      if (!element || element.dataset.mapReady === 'true') return;
      var key = root.dataset.amapKey;
      if (!key) {
        element.innerHTML = '<span><i class="fas fa-map-marked-alt"></i> 地图密钥未配置</span>';
        return;
      }
      element.dataset.mapReady = 'true';
      window._AMapSecurityConfig = { securityJsCode: root.dataset.amapSecurity || '' };
      try {
        await window.BambooPageRuntime.loadScript('https://webapi.amap.com/maps?v=2.0&key=' + encodeURIComponent(key), 'AMap');
        element.innerHTML = '';
        var map = new window.AMap.Map(element, { zoom: id === 'love-dashboard-map' ? 3.7 : 4.3, center: [105, 35], resizeEnable: true, mapStyle: document.body.classList.contains('darkModel') ? 'amap://styles/darkblue' : 'amap://styles/whitesmoke' });
        maps.push(map);
        trips.filter(function (trip) {
          return hasValidCoordinates(trip);
        }).forEach(function (trip) {
          var marker = new window.AMap.Marker({ position: [trip.lng, trip.lat], title: trip.city, anchor: 'bottom-center' });
          marker.setMap(map);
        });
        if (trips.some(hasValidCoordinates)) map.setFitView();
      } catch (error) {
        element.dataset.mapReady = 'false';
        element.innerHTML = '<span><i class="fas fa-map-marked-alt"></i> 地图暂时无法加载</span>';
      }
    }

    function destroyMaps() {
      maps.forEach(function (map) { try { map.destroy(); } catch (error) {} });
      maps = [];
      root.querySelectorAll('.love-map').forEach(function (element) { delete element.dataset.mapReady; });
    }

    async function unlock(event) {
      if (event) event.preventDefault();
      var hash = await sha256(passwordInput.value);
      if (hash !== root.dataset.passwordHash) {
        lockError.textContent = '密码不正确，请再试一次。';
        lockForm.classList.remove('is-shaking');
        void lockForm.offsetWidth;
        lockForm.classList.add('is-shaking');
        passwordInput.select();
        return;
      }
      sessionStorage.setItem(SESSION_KEY, 'true');
      lockError.textContent = '';
      showApp();
    }

    function showApp() {
      lockView.hidden = true;
      app.hidden = false;
      var hour = new Date().getHours();
      root.querySelector('#love-greeting').textContent = hour < 11 ? '早上好' : hour < 18 ? '下午好' : '晚上好';
      updateDashboard();
      startElapsedClock();
      Promise.race([
        refreshTripCoordinates(),
        new Promise(function (resolve) { window.setTimeout(resolve, 3500); })
      ]).then(function () {
        updateDashboard();
        requestAnimationFrame(function () { initMap('love-dashboard-map', state.trips); });
      });
    }

    function relock() {
      sessionStorage.removeItem(SESSION_KEY);
      if (elapsedTimer) window.clearInterval(elapsedTimer);
      elapsedTimer = null;
      destroyMaps();
      app.hidden = true;
      lockView.hidden = false;
      passwordInput.value = '';
      passwordInput.focus();
    }

    function togglePassword() {
      passwordInput.type = passwordInput.type === 'password' ? 'text' : 'password';
      passwordToggle.querySelector('i').className = passwordInput.type === 'password' ? 'fas fa-eye' : 'fas fa-eye-slash';
    }

    function searchSection() { if (currentSection !== 'home') renderSection(currentSection, sectionSearch.value); }

    function handleRecordSubmit(event) {
      saveRecord(event).catch(function (error) {
        if (formStatus) formStatus.textContent = error && error.message ? error.message : '保存失败，请稍后重试。';
      });
    }

    function handleSettingsSubmit(event) {
      var form = event.target.closest('#love-settings-form');
      if (!form) return;
      event.preventDefault();
      state.profile.startDate = form.elements.startDate.value;
      state.profile.metDate = form.elements.metDate.value;
      saveState();
      updateDashboard();
      renderSection('settings');
    }

    lockForm.addEventListener('submit', unlock);
    passwordToggle.addEventListener('click', togglePassword);
    lockNow.addEventListener('click', relock);
    root.addEventListener('click', handleRootClick);
    root.addEventListener('submit', handleSettingsSubmit);
    root.addEventListener('change', handleImageSelection);
    recordForm.addEventListener('submit', handleRecordSubmit);
    dialogClose.addEventListener('click', closeDialog);
    sectionSearch.addEventListener('input', searchSection);
    sectionAdd.addEventListener('click', function () { openDialog(currentSection); });
    quickAdd.addEventListener('click', function () { openDialog(currentSection === 'home' ? 'tasks' : currentSection); });
    cropAspect.addEventListener('change', handleCropAspect);
    cropZoom.addEventListener('input', handleCropZoom);
    cropCanvas.addEventListener('pointerdown', handleCropPointerDown);
    window.addEventListener('pointermove', handleCropPointerMove);
    window.addEventListener('pointerup', handleCropPointerUp);
    cropConfirm.addEventListener('click', confirmImageCrop);
    cropCancel.addEventListener('click', cancelImageCrop);
    cropClose.addEventListener('click', cancelImageCrop);
    cropDialog.addEventListener('cancel', cancelImageCrop);
    lightbox.addEventListener('keydown', handleLightboxKeydown);

    if (sessionStorage.getItem(SESSION_KEY) === 'true') showApp();
    else { lockView.hidden = false; app.hidden = true; }

    return function () {
      destroyMaps();
      if (elapsedTimer) window.clearInterval(elapsedTimer);
      lockForm.removeEventListener('submit', unlock);
      passwordToggle.removeEventListener('click', togglePassword);
      lockNow.removeEventListener('click', relock);
      root.removeEventListener('click', handleRootClick);
      root.removeEventListener('submit', handleSettingsSubmit);
      root.removeEventListener('change', handleImageSelection);
      recordForm.removeEventListener('submit', handleRecordSubmit);
      dialogClose.removeEventListener('click', closeDialog);
      sectionSearch.removeEventListener('input', searchSection);
      cropAspect.removeEventListener('change', handleCropAspect);
      cropZoom.removeEventListener('input', handleCropZoom);
      cropCanvas.removeEventListener('pointerdown', handleCropPointerDown);
      window.removeEventListener('pointermove', handleCropPointerMove);
      window.removeEventListener('pointerup', handleCropPointerUp);
      cropConfirm.removeEventListener('click', confirmImageCrop);
      cropCancel.removeEventListener('click', cancelImageCrop);
      cropClose.removeEventListener('click', cancelImageCrop);
      cropDialog.removeEventListener('cancel', cancelImageCrop);
      lightbox.removeEventListener('keydown', handleLightboxKeydown);
    };
  });
})();
