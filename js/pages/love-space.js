(function () {
  if (!window.BambooPageRuntime) return;

  window.BambooPageRuntime.register('love-space', async function (root) {
    var STORAGE_KEY = 'bambooLoveSpace:v1';
    var SESSION_KEY = 'bambooLoveUnlocked';
    var currentSection = 'home';
    var currentAddType = 'tasks';
    var activeTaskId = null;
    var activeRecordId = null;
    var maps = [];
    var elapsedTimer = null;

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
    var cropDialog = root.querySelector('#love-crop-dialog');
    var cropCanvas = root.querySelector('#love-crop-canvas');
    var cropContext = cropCanvas.getContext('2d');
    var cropAspect = root.querySelector('#love-crop-aspect');
    var cropZoom = root.querySelector('#love-crop-zoom');
    var cropConfirm = root.querySelector('#love-crop-confirm');
    var cropCancel = root.querySelector('#love-crop-cancel');
    var cropClose = root.querySelector('#love-crop-close');
    var cropState = { input: null, image: null, zoom: 1, offsetX: 0, offsetY: 0, dragging: false, x: 0, y: 0 };

    var taskTitles = [
      '一起过生日 🎂', '一起去旅行 🧳', '一起过情人节 💛', '一起吃海底捞 🍲', '一起吊娃娃 🧸',
      '一起去海边 🐟', '一起回老家 🚗', '一起看电影 🎬', '一起大扫除 🧹', '一起穿情侣装 🧥',
      '一起吃火锅 🍲', '一起吃烤肉 🥓', '一起吃烤鱼 🐟', '一起吃宵夜 🦪', '一起打羽毛球 🏸',
      '一起逛菜市场 🥬', '一起做饭 🍳', '一起跑步 🏃', '一起搬家 🏠', '一起去游乐园 🎡',
      '一起制作专属相册 📔', '一起用情侣手机壳 📱', '一起用勺子吃西瓜 🍉', '一起发朋友圈合照 📸', '一起海边看日落 🌅',
      '一起拍日常 Vlog 📹', '一起庆祝纪念日 🥂', '一起晚饭后散步 🚶', '一起跟朋友吃饭 🥘', '一起换情侣头像 🖼️',
      '一起玩五子棋 ✋', '一起骑电动车 🛵', '一起去小吃街 🍢', '一起自驾游 🚙', '一起打电动 🎮',
      '一起玩游戏 🕹️', '一起看球赛 ⚽', '一起剪头发 ✂️', '一起刮彩票 🎟️', '一起逛超市 🛒',
      '一起买家具 🪑', '一起打扑克牌 ♠️', '一起跨年 🧧', '一起健身 💪', '一起划船 🚣',
      '一起赏花 🌸', '一起熬夜 🤭', '一起种植物 🪴', '一起介绍给彼此的朋友 🧑‍🤝‍🧑', '一起给对方准备礼物 🎁',
      '难过时的陪伴 😣', '一起给我吹头发 💇', '一起给我穿鞋 👟', '一起照顾对方 🙂', '一起陪我做美甲 💅',
      '一起接我下班 🚉', '一起拥有车子 🚗', '一起给我送花 🌹', '一起吵架后和好 🫂', '一起参加别人的婚礼 💒',
      '一起去听一场演唱会 🎵', '一起去农场摘水果 🍑', '一起逛盒马生鲜 🏪', '一起逛花鸟市场 🐦', '一起拍情侣写真 💑',
      '一起去南山祈福 🧧', '一起去水上乐园 🔫', '一起海边看日出 🌄', '一起坐绿皮火车 🚆', '一起去天涯海角 🏖️',
      '一起去迪士尼 👑', '一起放孔明灯 🙏', '一起户外烧烤 🍖', '一起郊游踏青 🌴', '一起坐直升机 🚁',
      '一起去植物园 🌳', '一起去动物园 🐒', '一起坐飞机 ✈️', '一起住民宿 🏘️', '一起去体检 🏥',
      '一起看烟花 🎆', '一起打雪仗 ❄️', '一起敷面膜 🥰', '一起坐缆车 🚠', '一起去 K 歌 🎤',
      '一起钓鱼 🎣', '一起潜水 🤿', '一起冲浪 🏄', '一起露营 ⛺', '一起爬山 🧗',
      '一起漂流 🏞️', '一起喝酒 🍻', '一起泡脚 🦶', '一起见对方父母 👪', '一起出国旅游 🌍',
      '一起领结婚证 💍', '一起买房子 🏠', '一起结婚 👰', '一起生娃 🍼', '一起白头偕老 👵'
    ];

    var defaultState = {
      version: 5,
      profile: { startDate: '2026-04-10', metDate: '2026-04-10', nameA: '我', nameB: '你' },
      tasks: taskTitles.map(function (title, index) { return { id: 'task-' + index, title: title, category: index < 20 ? '日常' : index < 50 ? '旅行' : '未来', done: index < 4, date: index < 4 ? '2026-06-' + String(12 + index).padStart(2, '0') : '', location: '', note: '', photo: '', emoji: '' }; }),
      trips: [
        { id: 'trip-qingdao', city: '青岛', date: '2026-06-18', lat: 36.067, lng: 120.382, story: '沿着海岸慢慢散步，把晚霞和海风一起留在记忆里。', photo: '/medias/love/couple-sunset.png' },
        { id: 'trip-dali', city: '大理', date: '2026-05-01', lat: 25.606, lng: 100.267, story: '清晨的湖水很安静，远处的山被第一束光点亮。', photo: '/medias/love/alpine-lake.png' },
        { id: 'trip-lijiang', city: '丽江', date: '2026-04-22', lat: 26.872, lng: 100.229, story: '雨后的石板路倒映着暖色灯光。', photo: '/medias/love/old-town-night.png' }
      ],
      media: [
        { id: 'media-1', url: '/medias/love/couple-sunset.png', caption: '海风与晚霞', date: '2026-06-18', place: '青岛' },
        { id: 'media-2', url: '/medias/love/alpine-lake.png', caption: '山湖之间', date: '2026-05-01', place: '大理' },
        { id: 'media-3', url: '/medias/love/old-town-night.png', caption: '雨后的古城', date: '2026-04-22', place: '丽江' }
      ],
      timeline: [
        { id: 'time-1', date: '2026-04-10', title: '故事从这一天开始', text: '我们决定认真收藏往后的每一天。', mood: '心动', photo: '/medias/love/couple-sunset.png' },
        { id: 'time-2', date: '2026-04-22', title: '第一次双人旅行', text: '雨后的古城，走了很久也不觉得累。', mood: '开心', photo: '/medias/love/old-town-night.png' },
        { id: 'time-3', date: '2026-05-01', title: '山湖之间的清晨', text: '一起等到了湖面上的第一束光。', mood: '宁静', photo: '/medias/love/alpine-lake.png' },
        { id: 'time-4', date: '2026-06-18', title: '海边的晚霞', text: '我们在海风里拍下了最喜欢的照片。', mood: '浪漫', photo: '/medias/love/couple-sunset.png' }
      ],
      diaries: [{ id: 'diary-1', date: '2026-07-10', title: '写给今天', content: '今天也想认真记录我们，平凡的小事以后都会变成珍贵的回忆。', mood: '甜蜜' }],
      anniversaries: [{ id: 'ann-1', date: '2027-04-10', title: '恋爱一周年' }, { id: 'ann-2', date: '2026-10-10', title: '在一起半年' }],
      gifts: [{ id: 'gift-1', date: '2026-05-20', title: '一束认真挑选的花', from: '我们', story: '收到礼物时的开心也被好好保存。' }],
      wishes: [{ id: 'wish-1', title: '一起去看极光', status: '想去' }, { id: 'wish-2', title: '拥有一本共同旅行相册', status: '进行中' }],
      plans: [{ id: 'plan-1', title: '秋天去看一次山海', date: '2026-10-01', status: '计划中' }],
      messages: [{ id: 'msg-1', date: '2026-07-10', from: '我', content: '愿以后每一段普通日子，都能在这里被温柔地记住。' }]
    };

    // 小窝旅行记录是独立数据，只保存在本页的 localStorage 中；不与足迹页同步。
    defaultState.trips = [
      { id: 'trip-beijing', city: '\u5317\u4eac', date: '', lat: 39.9042, lng: 116.4074, story: '\u7167\u7247\u4f4d\u7f6e\u5df2\u9884\u7559\uff0c\u7b49\u5f85\u4e0a\u4f20\u5171\u540c\u65c5\u884c\u56de\u5fc6\u3002', photo: '/medias/love/beijing-forbidden-city.jpg' },
      { id: 'trip-tianjin', city: '\u5929\u6d25', date: '', lat: 39.1333, lng: 117.2000, story: '\u7167\u7247\u4f4d\u7f6e\u5df2\u9884\u7559\uff0c\u7b49\u5f85\u4e0a\u4f20\u5171\u540c\u65c5\u884c\u56de\u5fc6\u3002', photo: '/medias/love/tianjin-eye.jpg' }
    ];

    function cloneDefaults() { return JSON.parse(JSON.stringify(defaultState)); }

    function loadState() {
      try {
        var saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || 'null');
        if (!saved) return cloneDefaults();
        var nextState = Object.assign(cloneDefaults(), saved, { profile: Object.assign({}, defaultState.profile, saved.profile || {}) });
        var approvedCities = ['\u5317\u4eac', '\u5929\u6d25'];
        if (!saved.version || saved.version < 5) {
          nextState.trips = cloneDefaults().trips;
          nextState.version = 5;
        } else {
          nextState.trips = (nextState.trips || []).filter(function (trip) { return approvedCities.indexOf(trip.city) !== -1; });
          if (!nextState.trips.length) nextState.trips = cloneDefaults().trips;
        }
        if (!saved.version || saved.version < 3) {
          var savedTasksByTitle = {};
          (saved.tasks || []).forEach(function (task) { savedTasksByTitle[task.title] = task; });
          nextState.tasks = defaultState.tasks.map(function (task) {
            return Object.assign({}, task, savedTasksByTitle[task.title] || {}, { title: task.title });
          });
          nextState.version = 3;
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

    function daysBetween(from, to) {
      return Math.max(0, Math.floor((to.getTime() - from.getTime()) / 86400000));
    }

    function nextAnniversaryDays(dateString) {
      var start = new Date(dateString + 'T00:00:00');
      var now = new Date();
      var next = new Date(now.getFullYear(), start.getMonth(), start.getDate());
      if (next < new Date(now.getFullYear(), now.getMonth(), now.getDate())) next.setFullYear(next.getFullYear() + 1);
      return { days: Math.ceil((next.getTime() - now.getTime()) / 86400000), date: next };
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

    function recordActions(type, id) {
      return '<div class="love-record-actions"><button type="button" data-record-edit="' + escapeHtml(id) + '" data-record-type="' + type + '"><i class="fas fa-pen"></i> 修改</button><button type="button" class="is-delete" data-record-delete="' + escapeHtml(id) + '" data-record-type="' + type + '"><i class="fas fa-trash-alt"></i> 删除</button></div>';
    }

    function taskManageCard(task) {
      return '<article class="love-task-manage">' + taskCard(task) + recordActions('tasks', task.id) + '</article>';
    }

    function photoCard(item) {
      return '<button type="button" data-photo="' + escapeHtml(item.url) + '" data-caption="' + escapeHtml((item.caption || '我们的回忆') + ' · ' + (item.place || item.date || '')) + '"><img src="' + escapeHtml(item.url) + '" alt="' + escapeHtml(item.caption || '回忆照片') + '"><span>' + escapeHtml(item.caption || '我们的回忆') + '</span></button>';
    }

    function albumManageCard(item) {
      return '<article class="love-album-manage">' + photoCard(item) + recordActions('album', item.id) + '</article>';
    }

    function collectionForType(type) {
      return type === 'album' ? state.media : state[type];
    }

    var sectionMeta = {
      tasks: ['100件小事', '新增一件小事'], trips: ['旅行记录', '新增旅行'], timeline: ['回忆时间轴', '新增节点'], album: ['照片相册', '上传照片'],
      diaries: ['情侣日记', '写日记'], anniversaries: ['纪念日', '新增纪念日'], gifts: ['礼物记录', '记录礼物'], wishes: ['愿望清单', '新增愿望'],
      plans: ['未来计划', '新增计划'], messages: ['留言板', '写留言'], settings: ['空间设置', '编辑资料']
    };
    // These retired modules stay out of both the navigation and the generic add-record flow.
    ['diaries', 'anniversaries', 'gifts', 'wishes', 'plans', 'messages'].forEach(function (section) { delete sectionMeta[section]; });

    function toggleTask(id) {
      var task = state.tasks.find(function (item) { return item.id === id; });
      if (!task) return;
      task.done = !task.done;
      if (task.done && !task.date) task.date = new Date().toISOString().slice(0, 10);
      if (saveState()) {
        updateDashboard();
        if (currentSection === 'tasks') renderSection('tasks', sectionSearch.value);
      }
    }

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
          return '<article><img src="' + escapeHtml(tripLandmarkPhoto(trip)) + '" alt="' + escapeHtml(trip.city) + '标志性景点"><div><small>' + escapeHtml(trip.date) + '</small><h3>' + escapeHtml(trip.city) + '</h3><p>' + escapeHtml(trip.story) + '</p><span><i class="fas fa-map-marker-alt"></i> ' + Number(trip.lat).toFixed(3) + ', ' + Number(trip.lng).toFixed(3) + '</span>' + recordActions('trips', trip.id) + '</div></article>';
        }).join('') + '</div></div>';
      } else if (section === 'album') {
        var media = state.media.filter(function (item) { return !query || (item.caption || '').toLowerCase().indexOf(query) !== -1 || (item.place || '').toLowerCase().indexOf(query) !== -1; });
        html = '<div class="love-album-grid">' + media.map(albumManageCard).join('') + '</div>';
      } else if (section === 'timeline') {
        html = '<div class="love-timeline-full">' + state.timeline.slice().reverse().filter(function (item) { return !query || item.title.toLowerCase().indexOf(query) !== -1; }).map(function (item) {
          return '<article><div class="timeline-date">' + escapeHtml(item.date) + '</div><i></i><div class="timeline-card">' + (item.photo ? '<img src="' + escapeHtml(item.photo) + '" alt="' + escapeHtml(item.title) + '">' : '') + '<small>' + escapeHtml(item.mood || '回忆') + '</small><h3>' + escapeHtml(item.title) + '</h3><p>' + escapeHtml(item.text) + '</p>' + recordActions('timeline', item.id) + '</div></article>';
        }).join('') + '</div>';
      } else if (section === 'settings') {
        html = '<form class="love-settings-form" id="love-settings-form"><div class="love-settings-card"><h2>重要日期</h2><label>恋爱开始日期<input type="date" name="startDate" value="' + escapeHtml(state.profile.startDate) + '"></label><label>相识日期<input type="date" name="metDate" value="' + escapeHtml(state.profile.metDate) + '"></label><button type="submit" class="love-primary-button">保存设置</button></div><div class="love-settings-card"><h2>隐私与数据</h2><p>当前预览数据保存在本机浏览器中，不会上传到博客静态文件。</p><button type="button" data-export-love><i class="fas fa-download"></i> 导出备份</button><button type="button" data-reset-love class="danger"><i class="fas fa-trash-alt"></i> 恢复示例数据</button></div></form>';
      } else {
        var collection = state[section] || [];
        html = '<div class="love-record-list">' + collection.filter(function (item) {
          var haystack = JSON.stringify(item).toLowerCase();
          return !query || haystack.indexOf(query) !== -1;
        }).slice().reverse().map(function (item) {
          var title = item.title || item.content || '一段记录';
          var copy = item.content || item.story || item.status || item.from || '';
          return '<article><span class="record-icon"><i class="' + sectionIcon(section) + '"></i></span><div><small>' + escapeHtml(item.date || item.status || '我们的记录') + '</small><h3>' + escapeHtml(title) + '</h3><p>' + escapeHtml(copy) + '</p></div></article>';
        }).join('') + '</div>';
      }
      sectionContent.innerHTML = html || '<div class="love-empty">还没有记录，点击右上角添加第一条。</div>';
      if (section === 'trips') requestAnimationFrame(function () { initMap('love-trips-map', state.trips); });
    }

    function sectionIcon(section) {
      return { diaries: 'fas fa-book-open', anniversaries: 'fas fa-calendar-alt', gifts: 'fas fa-gift', wishes: 'fas fa-star', plans: 'fas fa-calendar-check', messages: 'fas fa-comment-heart' }[section] || 'fas fa-heart';
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
      else renderSection(section);
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

    function openTaskCheckin(id) {
      var task = state.tasks.find(function (item) { return item.id === id; });
      if (!task) return;
      activeTaskId = task.id;
      currentAddType = 'task-checkin';
      formStatus.textContent = '';
      recordForm.reset();
      dialogTitle.textContent = (task.done ? '编辑打卡' : '完成打卡') + ' · ' + task.title;
      var today = new Date().toISOString().slice(0, 10);
      dialogFields.innerHTML = '<div class="love-task-checkin-title"><i class="fas fa-heart"></i><span>' + escapeHtml(task.title) + '</span></div>' + taskEmojiPicker(task.emoji) + '<div class="form-row">' + field('date', '完成日期', 'date', 'required value="' + (task.date || today) + '"') + field('location', '完成地点', 'text', 'placeholder="例如：青岛" value="' + escapeHtml(task.location || '') + '"') + '</div><label>打卡照片<input name="photo" type="file" accept="image/jpeg,image/png,image/webp"></label><label>此刻感想<textarea name="note" rows="4" placeholder="记录当时的快乐瞬间">' + escapeHtml(task.note || '') + '</textarea></label>';
      if (typeof dialog.showModal === 'function') dialog.showModal(); else dialog.setAttribute('open', '');
    }

    function openDialog(type, recordId) {
      currentAddType = type === 'home' ? 'diaries' : type;
      activeTaskId = null;
      activeRecordId = recordId || null;
      formStatus.textContent = '';
      recordForm.reset();
      var editingCollection = collectionForType(currentAddType);
      var editingItem = activeRecordId && editingCollection ? editingCollection.find(function (item) { return item.id === activeRecordId; }) : null;
      dialogTitle.textContent = editingItem ? '修改' + (sectionMeta[currentAddType] ? sectionMeta[currentAddType][0] : '记录') : (sectionMeta[currentAddType] ? sectionMeta[currentAddType][1] : '记录此刻');
      var today = new Date().toISOString().slice(0, 10);
      var fields = '';
      if (currentAddType === 'tasks') fields = field('title', '想一起完成的事情', 'text', 'required placeholder="例如：一起去看海 🌊" value="' + escapeHtml(editingItem ? editingItem.title : '') + '"') + '<label>分类<select name="category">' + selectOptions(['日常', '旅行', '成长', '未来'], editingItem ? editingItem.category : '日常') + '</select></label>' + taskEmojiPicker(editingItem ? editingItem.emoji : '💗');
      else if (currentAddType === 'trips') fields = field('city', '旅行地点', 'text', 'required value="' + escapeHtml(editingItem ? editingItem.city : '') + '"') + field('date', '旅行日期', 'date', 'required value="' + escapeHtml(editingItem ? editingItem.date : today) + '"') + '<div class="form-row">' + field('lat', '纬度', 'number', 'step="any" required value="' + escapeHtml(editingItem ? editingItem.lat : '') + '"') + field('lng', '经度', 'number', 'step="any" required value="' + escapeHtml(editingItem ? editingItem.lng : '') + '"') + '</div><label>照片<input name="photo" type="file" accept="image/*"><small>' + (editingItem && editingItem.photo ? '不重新选择将保留现有照片' : '可上传旅行照片') + '</small></label><label>旅行故事<textarea name="story" rows="4" required>' + escapeHtml(editingItem ? editingItem.story : '') + '</textarea></label><p class="love-gps-status" data-love-gps></p>';
      else if (currentAddType === 'album') fields = '<label>照片<input name="photo" type="file" accept="image/*"' + (editingItem ? '' : ' required') + '><small>' + (editingItem ? '重新选择图片可再次裁剪；不选择则保留原图' : '选择后将自动打开裁剪器') + '</small></label>' + field('caption', '照片名称', 'text', 'required value="' + escapeHtml(editingItem ? editingItem.caption : '') + '"') + field('date', '拍摄日期', 'date', 'value="' + escapeHtml(editingItem ? editingItem.date : today) + '"') + field('place', '拍摄地点', 'text', 'value="' + escapeHtml(editingItem ? editingItem.place : '') + '"');
      else if (currentAddType === 'timeline') fields = field('title', '回忆标题', 'text', 'required value="' + escapeHtml(editingItem ? editingItem.title : '') + '"') + field('date', '发生日期', 'date', 'required value="' + escapeHtml(editingItem ? editingItem.date : today) + '"') + field('mood', '心情标签', 'text', 'placeholder="开心、浪漫、感动" value="' + escapeHtml(editingItem ? editingItem.mood : '') + '"') + '<label>回忆照片<input name="photo" type="file" accept="image/*"><small>' + (editingItem && editingItem.photo ? '不重新选择将保留现有照片' : '可选') + '</small></label><label>回忆故事<textarea name="text" rows="4" required>' + escapeHtml(editingItem ? editingItem.text : '') + '</textarea></label>';
      else if (currentAddType === 'diaries') fields = field('title', '日记标题', 'text', 'required') + field('date', '日期', 'date', 'required value="' + today + '"') + field('mood', '今天的心情', 'text', 'placeholder="甜蜜"') + '<label>日记内容<textarea name="content" rows="6" required></textarea></label>';
      else if (currentAddType === 'anniversaries') fields = field('title', '纪念日名称', 'text', 'required') + field('date', '日期', 'date', 'required');
      else if (currentAddType === 'gifts') fields = field('title', '礼物名称', 'text', 'required') + field('date', '日期', 'date', 'value="' + today + '"') + field('from', '赠送人', 'text', 'placeholder="我 / 你 / 我们"') + '<label>礼物故事<textarea name="story" rows="4"></textarea></label>';
      else if (currentAddType === 'messages') fields = '<label>留言人<select name="from"><option>我</option><option>你</option></select></label><label>想说的话<textarea name="content" rows="5" required></textarea></label>';
      else fields = field('title', currentAddType === 'plans' ? '计划名称' : '愿望内容', 'text', 'required') + (currentAddType === 'plans' ? field('date', '目标日期', 'date') : '') + '<label>状态<select name="status"><option>想去</option><option>计划中</option><option>进行中</option><option>已完成</option></select></label>';
      dialogFields.innerHTML = fields;
      var photo = recordForm.elements.photo;
      if (photo && currentAddType === 'trips') photo.addEventListener('change', readDialogGps, { once: true });
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
          cropAspect.value = currentAddType === 'tasks' || currentAddType === 'task-checkin' ? '1' : '1.333333';
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
      var id = currentAddType + '-' + Date.now();
      var item;
      if (currentAddType === 'task-checkin') {
        var task = state.tasks.find(function (entry) { return entry.id === activeTaskId; });
        if (!task) return;
        var taskPhotoData = await croppedImageFromInput(recordForm.elements.photo);
        task.done = true;
        task.date = data.get('date');
        task.location = data.get('location') || '';
        task.note = data.get('note') || '';
        task.emoji = data.get('emoji') || '💗';
        if (taskPhotoData) task.photo = taskPhotoData;
        if (saveState()) {
          closeDialog();
          updateDashboard();
          if (currentSection === 'tasks') renderSection('tasks', sectionSearch.value);
        }
        return;
      }
      if (activeRecordId && ['tasks', 'trips', 'timeline', 'album'].indexOf(currentAddType) !== -1) {
        var activeCollection = collectionForType(currentAddType);
        var existing = activeCollection.find(function (entry) { return entry.id === activeRecordId; });
        if (!existing) return;
        if (currentAddType === 'tasks') {
          existing.title = data.get('title');
          existing.category = data.get('category');
          existing.emoji = data.get('emoji') || existing.emoji || '💗';
        } else if (currentAddType === 'trips') {
          var editedTripPhoto = await croppedImageFromInput(recordForm.elements.photo);
          existing.city = data.get('city'); existing.date = data.get('date');
          existing.lat = Number(data.get('lat')); existing.lng = Number(data.get('lng'));
          existing.story = data.get('story');
          if (editedTripPhoto) existing.photo = editedTripPhoto;
          destroyMaps();
        } else if (currentAddType === 'timeline') {
          var editedTimelinePhoto = await croppedImageFromInput(recordForm.elements.photo);
          existing.title = data.get('title'); existing.date = data.get('date');
          existing.mood = data.get('mood'); existing.text = data.get('text');
          if (editedTimelinePhoto) existing.photo = editedTimelinePhoto;
        } else {
          var editedAlbumPhoto = await croppedImageFromInput(recordForm.elements.photo);
          existing.caption = data.get('caption'); existing.date = data.get('date'); existing.place = data.get('place');
          if (editedAlbumPhoto) existing.url = editedAlbumPhoto;
        }
        if (saveState()) {
          activeRecordId = null;
          closeDialog(); updateDashboard(); renderSection(currentSection, sectionSearch.value);
        }
        return;
      }
      if (currentAddType === 'tasks') item = { id: id, title: data.get('title'), category: data.get('category'), done: false, date: '', location: '', note: '', photo: '', emoji: data.get('emoji') || '💗' };
      else if (currentAddType === 'trips') {
        var tripPhoto = await croppedImageFromInput(recordForm.elements.photo);
        item = { id: id, city: data.get('city'), date: data.get('date'), lat: Number(data.get('lat')), lng: Number(data.get('lng')), story: data.get('story'), photo: tripPhoto };
        if (tripPhoto) state.media.unshift({ id: 'media-' + Date.now(), url: tripPhoto, caption: data.get('city') + '旅行', date: data.get('date'), place: data.get('city') });
      } else if (currentAddType === 'album') {
        item = { id: id, url: await croppedImageFromInput(recordForm.elements.photo), caption: data.get('caption'), date: data.get('date'), place: data.get('place') };
      } else if (currentAddType === 'timeline') {
        item = { id: id, title: data.get('title'), date: data.get('date'), mood: data.get('mood'), text: data.get('text'), photo: await croppedImageFromInput(recordForm.elements.photo) };
      }
      else if (currentAddType === 'diaries') item = { id: id, title: data.get('title'), date: data.get('date'), mood: data.get('mood'), content: data.get('content') };
      else if (currentAddType === 'anniversaries') item = { id: id, title: data.get('title'), date: data.get('date') };
      else if (currentAddType === 'gifts') item = { id: id, title: data.get('title'), date: data.get('date'), from: data.get('from'), story: data.get('story') };
      else if (currentAddType === 'messages') item = { id: id, from: data.get('from'), content: data.get('content'), date: new Date().toISOString().slice(0, 10) };
      else item = { id: id, title: data.get('title'), date: data.get('date') || '', status: data.get('status') || '想去' };
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

    function deleteRecord(type, id) {
      if (['tasks', 'trips', 'timeline', 'album'].indexOf(type) === -1) return;
      var collection = collectionForType(type);
      if (!collection) return;
      var item = collection.find(function (entry) { return entry.id === id; });
      var name = item ? (item.title || item.city || '这条记录') : '这条记录';
      if (!confirm('确定删除“' + name + '”吗？')) return;
      var nextCollection = collection.filter(function (entry) { return entry.id !== id; });
      if (type === 'album') state.media = nextCollection;
      else state[type] = nextCollection;
      if (type === 'trips') destroyMaps();
      if (saveState()) {
        updateDashboard();
        if (currentSection === type) renderSection(type, sectionSearch.value);
      }
    }

    function openPhoto(url, caption) {
      if (!url) return;
      lightboxImage.src = url;
      lightboxCaption.textContent = caption || '我们的回忆';
      if (typeof lightbox.showModal === 'function') lightbox.showModal(); else lightbox.setAttribute('open', '');
    }

    function closeLightbox() {
      if (typeof lightbox.close === 'function') lightbox.close(); else lightbox.removeAttribute('open');
    }

    function handleRootClick(event) {
      var navButton = event.target.closest('[data-love-section]');
      if (navButton) return activateSection(navButton.dataset.loveSection);
      var openSection = event.target.closest('[data-open-section]');
      if (openSection) return activateSection(openSection.dataset.openSection);
      var addType = event.target.closest('[data-add-type]');
      if (addType) return openDialog(addType.dataset.addType);
      var editRecord = event.target.closest('[data-record-edit]');
      if (editRecord) return openDialog(editRecord.dataset.recordType, editRecord.dataset.recordEdit);
      var deleteRecordButton = event.target.closest('[data-record-delete]');
      if (deleteRecordButton) return deleteRecord(deleteRecordButton.dataset.recordType, deleteRecordButton.dataset.recordDelete);
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
        trips.forEach(function (trip) {
          var marker = new window.AMap.Marker({ position: [trip.lng, trip.lat], title: trip.city, anchor: 'bottom-center' });
          marker.setMap(map);
        });
        if (trips.length) map.setFitView();
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
      requestAnimationFrame(function () { initMap('love-dashboard-map', state.trips); });
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
    recordForm.addEventListener('submit', saveRecord);
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
      recordForm.removeEventListener('submit', saveRecord);
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
    };
  });
})();
