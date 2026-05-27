// Web Bluetooth wrapper for the Micromouse26 BLE Car menu mode.
//
// Mirrors the Nordic UART Service (NUS) UUIDs used in
// include/BLECarControl.h. We write single-character commands to RX; the
// firmware latches them until the next write.
//
//   F = forward   B = backward   L = left   R = right   S = stop

export const NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
export const NUS_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
export const NUS_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';

export type Cmd = 'F' | 'B' | 'L' | 'R' | 'S';

export type ConnState =
    | { kind: 'idle' }
    | { kind: 'connecting' }
    | { kind: 'connected'; deviceName: string }
    | { kind: 'error'; message: string };

export interface BLECarHandle {
    sendCmd(cmd: Cmd): Promise<void>;
    disconnect(): void;
    onState(cb: (s: ConnState) => void): void;
    onRx(cb: (msg: string) => void): void;
}

export function isWebBluetoothSupported(): boolean {
    return typeof navigator !== 'undefined' && 'bluetooth' in navigator;
}

export async function connect(): Promise<BLECarHandle> {
    if (!isWebBluetoothSupported()) {
        throw new Error(
            'Web Bluetooth is unavailable. Use Chrome/Edge on desktop or Android.'
        );
    }

    const stateCbs: Array<(s: ConnState) => void> = [];
    const rxCbs: Array<(msg: string) => void> = [];
    const emitState = (s: ConnState) => stateCbs.forEach((cb) => cb(s));
    const emitRx = (m: string) => rxCbs.forEach((cb) => cb(m));

    emitState({ kind: 'connecting' });

    let device: BluetoothDevice;
    try {
        device = await navigator.bluetooth.requestDevice({
            // Match on EITHER the name prefix OR the NUS service UUID —
            // robust to advertising packets that only include one of the two.
            filters: [
                { namePrefix: 'Micromouse' },
                { services: [NUS_SERVICE] }
            ],
            optionalServices: [NUS_SERVICE]
        });
    } catch (e) {
        const msg = e instanceof Error ? e.message : String(e);
        emitState({ kind: 'error', message: msg });
        throw e;
    }

    const onDisc = () => emitState({ kind: 'idle' });
    device.addEventListener('gattserverdisconnected', onDisc);

    const server = await device.gatt!.connect();
    const service = await server.getPrimaryService(NUS_SERVICE);
    const rxChar = await service.getCharacteristic(NUS_RX_UUID);
    const txChar = await service.getCharacteristic(NUS_TX_UUID);

    await txChar.startNotifications();
    const decoder = new TextDecoder();
    txChar.addEventListener('characteristicvaluechanged', (ev) => {
        const target = ev.target as BluetoothRemoteGATTCharacteristic;
        if (!target.value) return;
        emitRx(decoder.decode(target.value));
    });

    emitState({ kind: 'connected', deviceName: device.name ?? 'Micromouse26' });

    const encoder = new TextEncoder();
    return {
        async sendCmd(cmd: Cmd) {
            const data = encoder.encode(cmd);
            // writeWithoutResponse is much lower latency than writeWithResponse;
            // a dropped command is harmless because we re-send on every press.
            const c = rxChar as BluetoothRemoteGATTCharacteristic & {
                writeValue?: (d: BufferSource) => Promise<void>;
            };
            if (typeof c.writeValueWithoutResponse === 'function') {
                await c.writeValueWithoutResponse(data);
            } else if (typeof c.writeValue === 'function') {
                await c.writeValue(data);
            }
        },
        disconnect() {
            device.removeEventListener('gattserverdisconnected', onDisc);
            if (device.gatt?.connected) device.gatt.disconnect();
            emitState({ kind: 'idle' });
        },
        onState(cb) {
            stateCbs.push(cb);
        },
        onRx(cb) {
            rxCbs.push(cb);
        }
    };
}
