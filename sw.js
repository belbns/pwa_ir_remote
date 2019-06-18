importScripts('js/sw-toolbox.js');

toolbox.precache([
    'css/index.css',
    'js/main.js',
    'js/nipplejs.js',
    'js/companion.js',
    'index.html',
]);

toolbox.router.default = toolbox.networkFirst;
toolbox.options.networkTimeoutSeconds = 5;

toolbox.router.get('icons/*', toolbox.fastest);
