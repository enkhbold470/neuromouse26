<script lang="ts">
    import { onMount } from 'svelte';
    import {
        connect as bleConnect,
        isWebBluetoothSupported,
        type BLECarHandle,
        type ConnState,
        type Cmd
    } from '$lib/ble';

    let supported = $state(true);
    let conn = $state<ConnState>({ kind: 'idle' });
    let handle: BLECarHandle | null = null;
    let active = $state<Cmd>('S');
    let lastSent = $state<string>('—');
    let log = $state<string[]>([]);

    const KEY_MAP: Record<string, Cmd> = {
        ArrowUp: 'F',
        ArrowDown: 'B',
        ArrowLeft: 'L',
        ArrowRight: 'R',
        w: 'F',
        W: 'F',
        s: 'B',
        S: 'B',
        a: 'L',
        A: 'L',
        d: 'R',
        D: 'R',
        ' ': 'S'
    };

    onMount(() => {
        supported = isWebBluetoothSupported();
    });

    async function onConnect() {
        try {
            handle = await bleConnect();
            handle.onState((s) => {
                conn = s;
                if (s.kind === 'idle') handle = null;
            });
            handle.onRx((m) => {
                log = [...log.slice(-19), m.trim()].filter((x) => x.length > 0);
            });
        } catch (e) {
            // requestDevice was cancelled or failed — already in error state.
            console.warn(e);
        }
    }

    function onDisconnect() {
        handle?.disconnect();
        handle = null;
    }

    async function send(cmd: Cmd) {
        active = cmd;
        lastSent = cmd;
        if (!handle) return;
        try {
            await handle.sendCmd(cmd);
        } catch (e) {
            console.error('[send] failed', e);
        }
    }

    function press(cmd: Cmd) {
        return (ev: Event) => {
            ev.preventDefault();
            send(cmd);
        };
    }

    function release() {
        return (ev: Event) => {
            ev.preventDefault();
            if (active !== 'S') send('S');
        };
    }

    function onKeyDown(ev: KeyboardEvent) {
        const cmd = KEY_MAP[ev.key];
        if (!cmd) return;
        if (ev.repeat) return;
        ev.preventDefault();
        send(cmd);
    }

    function onKeyUp(ev: KeyboardEvent) {
        const cmd = KEY_MAP[ev.key];
        if (!cmd) return;
        ev.preventDefault();
        if (cmd !== 'S') send('S');
    }

    const connected = $derived(conn.kind === 'connected');
</script>

<svelte:window on:keydown={onKeyDown} on:keyup={onKeyUp} />

<main>
    <header>
        <h1>mm26 BLE Car</h1>
        <div class="status" class:on={connected} class:err={conn.kind === 'error'}>
            {#if conn.kind === 'idle'}
                idle
            {:else if conn.kind === 'connecting'}
                connecting…
            {:else if conn.kind === 'connected'}
                linked · {conn.deviceName}
            {:else}
                {conn.message}
            {/if}
        </div>
    </header>

    {#if !supported}
        <p class="warn">
            Web Bluetooth is not available in this browser. Use Chrome or Edge on
            desktop, or Chrome on Android. iOS Safari does not support Web
            Bluetooth.
        </p>
    {/if}

    <div class="row">
        {#if !connected}
            <button class="primary" disabled={!supported || conn.kind === 'connecting'} onclick={onConnect}>
                {conn.kind === 'connecting' ? 'connecting…' : 'connect'}
            </button>
        {:else}
            <button class="secondary" onclick={onDisconnect}>disconnect</button>
        {/if}
        <span class="lastsent">last: <b>{lastSent}</b></span>
    </div>

    <div class="pad">
        <div class="pad-row">
            <div class="slot"></div>
            <button
                class="pad-btn"
                class:active={active === 'F'}
                onpointerdown={press('F')}
                onpointerup={release()}
                onpointerleave={release()}
                onpointercancel={release()}
                aria-label="forward"
            >▲</button>
            <div class="slot"></div>
        </div>
        <div class="pad-row">
            <button
                class="pad-btn"
                class:active={active === 'L'}
                onpointerdown={press('L')}
                onpointerup={release()}
                onpointerleave={release()}
                onpointercancel={release()}
                aria-label="left"
            >◀</button>
            <button
                class="pad-btn stop"
                class:active={active === 'S'}
                onpointerdown={press('S')}
                aria-label="stop"
            >■</button>
            <button
                class="pad-btn"
                class:active={active === 'R'}
                onpointerdown={press('R')}
                onpointerup={release()}
                onpointerleave={release()}
                onpointercancel={release()}
                aria-label="right"
            >▶</button>
        </div>
        <div class="pad-row">
            <div class="slot"></div>
            <button
                class="pad-btn"
                class:active={active === 'B'}
                onpointerdown={press('B')}
                onpointerup={release()}
                onpointerleave={release()}
                onpointercancel={release()}
                aria-label="backward"
            >▼</button>
            <div class="slot"></div>
        </div>
    </div>

    <p class="hint">
        hold a button (or W / A / S / D / arrow keys) to drive · release sends
        STOP automatically · spacebar = stop
    </p>

    {#if log.length > 0}
        <section class="log">
            <h2>from robot</h2>
            <ul>
                {#each log as line, i (i + line)}
                    <li>{line}</li>
                {/each}
            </ul>
        </section>
    {/if}
</main>

<style>
    :global(html, body) {
        margin: 0;
        padding: 0;
        background: #0b0e14;
        color: #e6e6e6;
        font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
        overscroll-behavior: none;
        touch-action: manipulation;
    }
    :global(*) {
        box-sizing: border-box;
    }

    main {
        max-width: 480px;
        margin: 0 auto;
        padding: 24px 20px 32px;
        display: flex;
        flex-direction: column;
        gap: 16px;
        min-height: 100vh;
        min-height: 100dvh;
    }

    header {
        display: flex;
        justify-content: space-between;
        align-items: baseline;
        gap: 12px;
    }
    h1 {
        font-size: 18px;
        font-weight: 600;
        margin: 0;
        letter-spacing: 0.3px;
    }
    .status {
        font-size: 12px;
        padding: 4px 10px;
        border-radius: 999px;
        background: #1c2230;
        color: #8a99b2;
    }
    .status.on {
        background: #143324;
        color: #5cd396;
    }
    .status.err {
        background: #3a1a1a;
        color: #ff7676;
    }

    .warn {
        background: #2a1b0a;
        color: #f3b066;
        border-radius: 8px;
        padding: 10px 12px;
        font-size: 13px;
        margin: 0;
    }

    .row {
        display: flex;
        align-items: center;
        gap: 14px;
    }

    button {
        font: inherit;
        cursor: pointer;
        border: none;
    }

    button.primary,
    button.secondary {
        padding: 10px 18px;
        border-radius: 8px;
        font-weight: 600;
        font-size: 14px;
    }
    button.primary {
        background: #4a8df5;
        color: white;
    }
    button.primary:disabled {
        background: #2a3a55;
        color: #7a8aa0;
        cursor: not-allowed;
    }
    button.secondary {
        background: #2a2f3d;
        color: #cfd6e4;
    }
    .lastsent {
        font-size: 13px;
        color: #8a99b2;
    }
    .lastsent b {
        color: #e6e6e6;
        font-family: ui-monospace, monospace;
    }

    .pad {
        display: flex;
        flex-direction: column;
        gap: 10px;
        margin-top: 8px;
    }
    .pad-row {
        display: grid;
        grid-template-columns: 1fr 1fr 1fr;
        gap: 10px;
    }
    .slot {
        visibility: hidden;
    }
    .pad-btn {
        aspect-ratio: 1 / 1;
        font-size: 36px;
        border-radius: 16px;
        background: #1e2533;
        color: #cfd6e4;
        user-select: none;
        -webkit-user-select: none;
        transition: background 60ms ease, transform 60ms ease;
        touch-action: none;
    }
    .pad-btn:hover {
        background: #262e40;
    }
    .pad-btn.active {
        background: #4a8df5;
        color: white;
        transform: scale(0.97);
    }
    .pad-btn.stop {
        background: #2a1f1f;
        color: #e88888;
    }
    .pad-btn.stop.active {
        background: #c44a4a;
        color: white;
    }

    .hint {
        text-align: center;
        font-size: 12px;
        color: #6a7790;
        margin: 4px 0 0;
        line-height: 1.5;
    }

    .log {
        margin-top: 8px;
        background: #11151e;
        border-radius: 10px;
        padding: 10px 14px;
    }
    .log h2 {
        font-size: 12px;
        font-weight: 600;
        margin: 0 0 6px;
        color: #8a99b2;
        text-transform: uppercase;
        letter-spacing: 0.6px;
    }
    .log ul {
        list-style: none;
        margin: 0;
        padding: 0;
        max-height: 160px;
        overflow-y: auto;
        font-family: ui-monospace, monospace;
        font-size: 12px;
        color: #b3c0d6;
    }
    .log li {
        padding: 2px 0;
    }
</style>
