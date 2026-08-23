import { describe, expect, it } from 'vitest';
import { MessageType, SlipDecoder, decodeValueBytes, slipEncode, ValueType } from './protocol';

describe('Tether IO protocol', () => {
  it('round trips fragmented escaped SLIP frames', () => {
    const encoded = slipEncode(Uint8Array.from([1, 0xc0, 2, 0xdb, 3]));
    const decoder = new SlipDecoder();
    expect(decoder.push(encoded.slice(0, 2))).toHaveLength(0);
    expect([...decoder.push(encoded.slice(2))][0]).toEqual(Uint8Array.from([1, 0xc0, 2, 0xdb, 3]));
  });
  it('decodes fixed scalar values', () => {
    const bytes = new Uint8Array(8);
    new DataView(bytes.buffer).setFloat64(0, 12.5, true);
    expect(decodeValueBytes(bytes, ValueType.F64)).toBe(12.5);
    expect(MessageType.streamData).toBe(7);
  });
  it('rejects oversized in-progress frames', () => {
    const decoder = new SlipDecoder(3);
    expect(decoder.push(Uint8Array.from([1, 2, 3, 4, 0xc0]))).toEqual([]);
  });
});
