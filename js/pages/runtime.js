(function () {
  if (window.BambooPageRuntime) return;

  var registry = {};
  var activeName = null;
  var activeCleanup = null;
  var loadingScripts = {};

  function loadScript(src, globalName) {
    if (globalName && window[globalName]) return Promise.resolve(window[globalName]);
    if (loadingScripts[src]) return loadingScripts[src];

    loadingScripts[src] = new Promise(function (resolve, reject) {
      var existing = document.querySelector('script[data-bamboo-src="' + src + '"]');
      if (existing) {
        if (globalName && window[globalName]) resolve(window[globalName]);
        else existing.addEventListener('load', function () { resolve(globalName ? window[globalName] : true); }, { once: true });
        return;
      }

      var script = document.createElement('script');
      script.src = src;
      script.async = true;
      script.dataset.bambooSrc = src;
      script.onload = function () { resolve(globalName ? window[globalName] : true); };
      script.onerror = function () {
        delete loadingScripts[src];
        reject(new Error('页面资源加载失败：' + src));
      };
      document.head.appendChild(script);
    });

    return loadingScripts[src];
  }

  function unmount() {
    if (typeof activeCleanup === 'function') {
      try { activeCleanup(); } catch (error) { console.warn('[BambooPage] cleanup failed', error); }
    }
    activeCleanup = null;
    activeName = null;
  }

  function syncHeaderOffset() {
    var header = document.getElementById('navHeader');
    var height = header ? Math.ceil(header.getBoundingClientRect().height) : 60;
    document.documentElement.style.setProperty('--bamboo-header-height', Math.max(48, height) + 'px');
  }

  function mount() {
    var root = document.querySelector('[data-bamboo-page]');
    if (!root) {
      unmount();
      return;
    }

    var name = root.getAttribute('data-bamboo-page');
    if (activeName === name && activeCleanup) return;
    unmount();
    activeName = name;

    if (!registry[name]) return;
    Promise.resolve(registry[name](root)).then(function (cleanup) {
      if (activeName === name) activeCleanup = typeof cleanup === 'function' ? cleanup : function () {};
      else if (typeof cleanup === 'function') cleanup();
    }).catch(function (error) {
      console.error('[BambooPage] mount failed', error);
      root.classList.add('page-load-error');
      var message = root.querySelector('[data-page-error]');
      if (message) message.textContent = '页面加载失败，请刷新后重试。';
    });
  }

  function register(name, factory) {
    registry[name] = factory;
    var root = document.querySelector('[data-bamboo-page="' + name + '"]');
    if (root) mount();
  }

  function navigate(url) {
    var target = new URL(url, window.location.href);
    if (target.origin === window.location.origin && window.pjax && typeof window.pjax.loadUrl === 'function') {
      window.pjax.loadUrl(target.href);
      return;
    }
    window.location.href = target.href;
  }

  window.BambooPageRuntime = {
    register: register,
    mount: mount,
    unmount: unmount,
    loadScript: loadScript,
    navigate: navigate
  };

  document.addEventListener('DOMContentLoaded', function () { syncHeaderOffset(); mount(); });
  window.addEventListener('resize', syncHeaderOffset);
  document.addEventListener('pjax:send', unmount);
  document.addEventListener('pjax:complete', function () { requestAnimationFrame(function () { syncHeaderOffset(); mount(); }); });
})();
