// Minimal ESP-IDF NVS (v2) partition image generator: one namespace + several u8 keys.
function crc32le(bytes, init) {
  let crc = (~init) >>> 0;
  for (let i = 0; i < bytes.length; i++) {
    crc ^= bytes[i];
    for (let k = 0; k < 8; k++) crc = (crc >>> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return (~crc) >>> 0;
}
function nvsCrc(bytes) { return crc32le(bytes, 0xFFFFFFFF); }

// kv: [[key, value], ...]  (u8 values, up to 100 keys)
function makeNvsU8s(namespace, kv, size) {
  const img = new Uint8Array(size).fill(0xFF);
  const dv = new DataView(img.buffer);
  dv.setUint32(0, 0xFFFFFFFE, true);  // ACTIVE
  dv.setUint32(4, 0, true);           // seq
  img[8] = 0xFE;                      // version 2
  dv.setUint32(28, nvsCrc(img.subarray(4, 28)), true);
  const entry = (idx, ns, keyStr, val) => {
    const off = 64 + idx * 32;
    img[off] = ns; img[off + 1] = 0x01; img[off + 2] = 1; img[off + 3] = 0xFF;
    for (let i = 0; i < 16; i++) img[off + 8 + i] = 0;
    img.set(new TextEncoder().encode(keyStr).subarray(0, 15), off + 8);
    img[off + 24] = val & 0xFF;
    const tmp = new Uint8Array(28);
    tmp.set(img.subarray(off, off + 4), 0);
    tmp.set(img.subarray(off + 8, off + 32), 4);
    dv.setUint32(off + 4, nvsCrc(tmp), true);
    img[32 + (idx >> 2)] &= ~(1 << ((idx & 3) * 2));     // state: written (10b)
  };
  entry(0, 0, namespace, 1);
  kv.forEach(([k, v], i) => entry(i + 1, 1, k, v));
  return img;
}
function makeNvsU8(namespace, key, value, size) { return makeNvsU8s(namespace, [[key, value]], size); }
if (typeof module !== 'undefined') module.exports = { makeNvsU8, makeNvsU8s };
