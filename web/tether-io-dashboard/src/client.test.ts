import { afterEach, describe, expect, it, vi } from 'vitest';
import { TetherIOClient } from './client';
import { BinaryWriter, MessageType } from './protocol';

class MockWebSocket {
  static readonly OPEN = 1;
  static instances: MockWebSocket[] = [];
  binaryType = 'blob';
  readyState = 0;
  onopen: ((event: Event) => void) | null = null;
  onerror: ((event: Event) => void) | null = null;
  onclose: ((event: CloseEvent) => void) | null = null;
  onmessage: ((event: MessageEvent) => void) | null = null;
  sent: Uint8Array[] = [];

  constructor(_url: string) { MockWebSocket.instances.push(this); }
  send(data: ArrayBufferLike | Blob | ArrayBufferView): void {
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data as ArrayBuffer);
    this.sent.push(bytes);
    if (bytes[0] === MessageType.clientHello) queueMicrotask(() => this.receive(serverHello()));
    if (bytes[0] === MessageType.getParamReq) {
      const request = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      const response = new BinaryWriter(1 + 8 + 1 + 4).u8(MessageType.getParamResp)
        .u64(request.getBigUint64(1, true)).varint(4).bytes(Uint8Array.from([0x2a, 0, 0, 0])).finish();
      queueMicrotask(() => this.receive(response));
    }
    if (bytes[0] === MessageType.setParameterReq) {
      const request = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      const response = new BinaryWriter(9).u8(MessageType.setParameterResp).u64(request.getBigUint64(1, true)).finish();
      queueMicrotask(() => this.receive(response));
    }
    if (bytes[0] === MessageType.invokeExReq) {
      const request = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      const response = new BinaryWriter(17).u8(MessageType.invokeExResp)
        .u64(request.getBigUint64(1, true)).u8(0).u32(0).u16(0).u8(0).finish();
      queueMicrotask(() => this.receive(response));
    }
  }
  close(): void {
    this.readyState = 3;
    this.onclose?.({ code: 1000, reason: '', wasClean: true } as CloseEvent);
  }
  receive(bytes: Uint8Array): void {
    this.onmessage?.({ data: bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) } as MessageEvent);
  }
  open(): void { this.readyState = MockWebSocket.OPEN; this.onopen?.(new Event('open')); }
}

function serverHello(): Uint8Array {
  const writer = new BinaryWriter(38).u8(MessageType.serverHello).u8(6).u32(0);
  for (let i = 0; i < 6; i += 1) writer.u32(1024);
  return writer.u32(3).u32(0).finish();
}

const originalWebSocket = globalThis.WebSocket;
afterEach(() => {
  globalThis.WebSocket = originalWebSocket;
  MockWebSocket.instances = [];
  vi.restoreAllMocks();
});

describe('TetherIOClient V6 session', () => {
  it('negotiates schemas before resolving connect and decodes varint-framed values', async () => {
    globalThis.WebSocket = MockWebSocket as unknown as typeof WebSocket;
    const client = new TetherIOClient();
    const connect = client.connect('ws://test/tether-io');
    const socket = MockWebSocket.instances[0]!;
    socket.open();
    await connect;
    expect(socket.sent.map((frame) => frame[0])).toEqual([MessageType.clientHello, MessageType.schemaCommit]);
    expect(client.schemaCatalog?.epoch).toBe(3);

    const value = await client.get('param', 0x1234n);
    expect([...value]).toEqual([0x2a, 0, 0, 0]);
    await client.setParameter(0x1234n, Uint8Array.from([1, 2, 3, 4]));
    const setFrame = socket.sent.at(-1)!;
    expect([...setFrame]).toEqual([MessageType.setParameterReq, 0x34, 0x12, 0, 0, 0, 0, 0, 0, 4, 1, 2, 3, 4]);
    const call = await client.callFunction(0x7788n);
    expect(call.success).toBe(true);
    expect(call.functionId).toBe(0x7788n);
    expect(socket.sent.at(-1)?.[0]).toBe(MessageType.invokeExReq);
    client.disconnect();
  });
});
