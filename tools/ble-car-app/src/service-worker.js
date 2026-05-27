// SvelteKit picks this up automatically and registers it in production
// builds. `$service-worker` exposes the build artifacts + static files we
// need to precache for offline.

/// <reference types="@sveltejs/kit" />
/* eslint-env serviceworker */

import { build, files, version } from '$service-worker';

const CACHE = `mm26-ble-${version}`;
const ASSETS = [...build, ...files];

self.addEventListener('install', (event) => {
    event.waitUntil(
        (async () => {
            const cache = await caches.open(CACHE);
            await cache.addAll(ASSETS);
            self.skipWaiting();
        })()
    );
});

self.addEventListener('activate', (event) => {
    event.waitUntil(
        (async () => {
            for (const key of await caches.keys()) {
                if (key !== CACHE) await caches.delete(key);
            }
            await self.clients.claim();
        })()
    );
});

self.addEventListener('fetch', (event) => {
    const req = event.request;
    if (req.method !== 'GET') return;

    const url = new URL(req.url);
    if (url.origin !== self.location.origin) return;

    event.respondWith(
        (async () => {
            const cache = await caches.open(CACHE);

            // Precached assets — cache-first.
            if (ASSETS.includes(url.pathname)) {
                const cached = await cache.match(url.pathname);
                if (cached) return cached;
            }

            // Navigations — fall back to the SPA index when offline.
            if (req.mode === 'navigate') {
                try {
                    return await fetch(req);
                } catch {
                    const root = await cache.match('/');
                    if (root) return root;
                    const fallback = await cache.match('/index.html');
                    if (fallback) return fallback;
                }
            }

            // Everything else — network with cache fallback.
            try {
                const res = await fetch(req);
                if (res.status === 200 && res.type === 'basic') {
                    cache.put(req, res.clone());
                }
                return res;
            } catch {
                const cached = await cache.match(req);
                if (cached) return cached;
                return new Response('offline', { status: 503 });
            }
        })()
    );
});
