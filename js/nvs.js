// Minimal ESP-IDF NVS (v2) partition image generator: one namespace, u8 and string values.
function crc32le(bytes, init) {
  let crc = (~init) >>> 0;
  for (let i = 0; i < bytes.length; i++) {
    crc ^= bytes[i];
    for (let k = 0; k < 8; k++) crc = (crc >>> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return (~crc) >>> 0;
}
function nvsCrc(bytes) { return crc32le(bytes, 0xFFFFFFFF); }

// items: [[key, value]]；value 為數字 = u8，字串 = string（UTF-8，最多 4000 bytes）
function makeNvs(namespace, items, size) {
  const img = new Uint8Array(size).fill(0xFF);
  const dv = new DataView(img.buffer);
  dv.setUint32(0, 0xFFFFFFFE, true);  // page state ACTIVE
  dv.setUint32(4, 0, true);           // seq
  img[8] = 0xFE;                      // version 2
  dv.setUint32(28, nvsCrc(img.subarray(4, 28)), true);
  let idx = 0;
  const mark = i => { img[32 + (i >> 2)] &= ~(1 << ((i & 3) * 2)); };   // written = 10b
  const header = (i, ns, type, span, keyStr, fillData) => {
    const off = 64 + i * 32;
    img[off] = ns; img[off + 1] = type; img[off + 2] = span; img[off + 3] = 0xFF;
    for (let k = 0; k < 16; k++) img[off + 8 + k] = 0;
    img.set(new TextEncoder().encode(keyStr).subarray(0, 15), off + 8);
    fillData(off + 24);
    const tmp = new Uint8Array(28);
    tmp.set(img.subarray(off, off + 4), 0);
    tmp.set(img.subarray(off + 8, off + 32), 4);
    dv.setUint32(off + 4, nvsCrc(tmp), true);
  };
  header(idx, 0, 0x01, 1, namespace, d => { img[d] = 1; }); mark(idx); idx++;
  for (const [key, val] of items) {
    if (typeof val === 'number') {
      header(idx, 1, 0x01, 1, key, d => { img[d] = val & 0xFF; }); mark(idx); idx++;
    } else {
      const data = new Uint8Array([...new TextEncoder().encode(String(val)), 0]);
      const span = 1 + Math.ceil(data.length / 32);
      if (idx + span > 126) throw new Error('NVS 內容太多');
      header(idx, 1, 0x21, span, key, d => {
        dv.setUint16(d, data.length, true); dv.setUint16(d + 2, 0xFFFF, true);
        dv.setUint32(d + 4, nvsCrc(data), true);
      });
      img.set(data, 64 + (idx + 1) * 32);
      for (let k = 0; k < span; k++) mark(idx + k);
      idx += span;
    }
  }
  return img;
}
function makeNvsU8s(namespace, kv, size) { return makeNvs(namespace, kv, size); }
function makeNvsU8(namespace, key, value, size) { return makeNvs(namespace, [[key, value]], size); }
if (typeof module !== 'undefined') module.exports = { makeNvs, makeNvsU8, makeNvsU8s };
