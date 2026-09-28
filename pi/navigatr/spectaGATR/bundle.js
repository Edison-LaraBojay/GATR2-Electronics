// bundle.js
// Reads a capture bundle (gatr2.capture/1): a ZIP of CSVs plus
// metadata.json. The Pi writes store-only ZIPs, so the reader is small; a
// deflated entry (a bundle re-zipped by hand) is inflated with the
// browser's DecompressionStream when it has one. Every entry's CRC-32 is
// checked; a mismatch is an error, never silently accepted.
//
// CSV: header row, comma separated, RFC 4180 quoting, \n or \r\n line
// ends. An empty field is missing (NaN in numeric columns, null in text).

const CRC_TABLE = (() => {
    const t = new Uint32Array(256);
    for (let n = 0; n < 256; ++n) {
        let c = n;
        for (let k = 0; k < 8; ++k) {
            c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
        }
        t[n] = c >>> 0;
    }
    return t;
})();

export function crc32(bytes) {
    let c = 0xffffffff;
    for (let i = 0; i < bytes.length; ++i) {
        c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
    }
    return (c ^ 0xffffffff) >>> 0;
}

async function inflateRaw(bytes) {
    if (typeof DecompressionStream !== 'function') {
        throw new Error('deflated ZIP entry and this browser cannot inflate; use the store-only bundle from the Pi');
    }
    const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate-raw'));
    return new Uint8Array(await new Response(stream).arrayBuffer());
}

// Map of entry name -> Uint8Array. Throws on a malformed archive.
export async function readZip(buffer) {
    const u8 = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer);
    const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
    // end of central directory: last 22 bytes plus up to 64 KiB of comment
    let eocd = -1;
    for (let i = u8.length - 22; i >= Math.max(0, u8.length - 22 - 65535); --i) {
        if (dv.getUint32(i, true) === 0x06054b50) {
            eocd = i;
            break;
        }
    }
    if (eocd < 0) {
        throw new Error('not a ZIP file (no end of central directory)');
    }
    const count = dv.getUint16(eocd + 10, true);
    const cdSize = dv.getUint32(eocd + 12, true);
    const cdOffset = dv.getUint32(eocd + 16, true);
    if (cdOffset + cdSize > u8.length) {
        throw new Error('ZIP central directory out of range');
    }
    const out = new Map();
    const decoder = new TextDecoder();
    let p = cdOffset;
    for (let e = 0; e < count; ++e) {
        if (dv.getUint32(p, true) !== 0x02014b50) {
            throw new Error(`ZIP central directory entry ${e} is malformed`);
        }
        const method = dv.getUint16(p + 10, true);
        const crc = dv.getUint32(p + 16, true);
        const csize = dv.getUint32(p + 20, true);
        const usize = dv.getUint32(p + 24, true);
        const nameLen = dv.getUint16(p + 28, true);
        const extraLen = dv.getUint16(p + 30, true);
        const commentLen = dv.getUint16(p + 32, true);
        const local = dv.getUint32(p + 42, true);
        const name = decoder.decode(u8.subarray(p + 46, p + 46 + nameLen));
        p += 46 + nameLen + extraLen + commentLen;
        if (dv.getUint32(local, true) !== 0x04034b50) {
            throw new Error(`ZIP local header of ${name} is malformed`);
        }
        const dataStart = local + 30 + dv.getUint16(local + 26, true) + dv.getUint16(local + 28, true);
        if (dataStart + csize > u8.length) {
            throw new Error(`ZIP entry ${name} is truncated`);
        }
        const raw = u8.subarray(dataStart, dataStart + csize);
        let data;
        if (method === 0) {
            data = raw;
        } else if (method === 8) {
            data = await inflateRaw(raw);
        } else {
            throw new Error(`ZIP entry ${name} uses compression method ${method}`);
        }
        if (data.length !== usize) {
            throw new Error(`ZIP entry ${name}: size ${data.length}, expected ${usize}`);
        }
        const got = crc32(data);
        if (got !== crc) {
            throw new Error(`ZIP entry ${name}: CRC-32 ${got.toString(16)}, expected ${crc.toString(16)}`);
        }
        if (!name.endsWith('/')) {
            out.set(name.replace(/^.*\//, ''), data);
        }
    }
    return out;
}

// Parses CSV text into {header, rows} with rows as arrays of strings; an
// empty field is ''. Quoted fields may hold commas, quotes ("") and
// newlines. Unquoted fields are sliced, not built a character at a time,
// so a capture of a few hundred thousand rows parses in well under a second.
export function parseCsvRows(text) {
    const rows = [];
    const n = text.length;
    let row = [];
    let i = 0;
    while (i < n) {
        let value;
        if (text.charCodeAt(i) === 34) {
            let j = i + 1;
            value = '';
            for (;;) {
                const q = text.indexOf('"', j);
                if (q < 0) {
                    value += text.slice(j); // unterminated: keep what is there
                    i = n;
                    break;
                }
                value += text.slice(j, q);
                if (text.charCodeAt(q + 1) === 34) {
                    value += '"';
                    j = q + 2;
                    continue;
                }
                i = q + 1;
                break;
            }
            while (i < n) {
                const c = text.charCodeAt(i);
                if (c === 44 || c === 10 || c === 13) {
                    break;
                }
                i += 1;
            }
        } else {
            let j = i;
            while (j < n) {
                const c = text.charCodeAt(j);
                if (c === 44 || c === 10 || c === 13) {
                    break;
                }
                j += 1;
            }
            value = text.slice(i, j);
            i = j;
        }
        row.push(value);
        if (i >= n) {
            break;
        }
        const c = text.charCodeAt(i);
        if (c === 44) {
            i += 1;
            if (i === n) {
                row.push('');
            }
            continue;
        }
        i += c === 13 && text.charCodeAt(i + 1) === 10 ? 2 : 1;
        if (!(row.length === 1 && row[0] === '')) {
            rows.push(row);
        }
        row = [];
    }
    if (row.length && !(row.length === 1 && row[0] === '')) {
        rows.push(row);
    }
    const header = rows.length ? rows.shift() : [];
    return { header, rows };
}

// A parsed CSV with lazy typed columns: num(name) is a Float64Array with
// NaN for missing or non-numeric fields, str(name) an array with null for
// missing. Booleans are 0/1 in the bundle, so num() covers them.
export class CsvTable {
    constructor(name, text) {
        this.name = name;
        const { header, rows } = parseCsvRows(text);
        this.header = header;
        this.rows = rows;
        this.n = rows.length;
        this.index = new Map(header.map((h, i) => [h, i]));
        this.nums = new Map();
        this.strs = new Map();
    }

    has(name) {
        return this.index.has(name);
    }

    // First of names present, or null.
    pick(...names) {
        for (const n of names) {
            if (this.index.has(n)) {
                return n;
            }
        }
        return null;
    }

    num(name) {
        if (name === null || name === undefined || !this.index.has(name)) {
            return null;
        }
        let a = this.nums.get(name);
        if (!a) {
            const i = this.index.get(name);
            a = new Float64Array(this.n);
            for (let r = 0; r < this.n; ++r) {
                const f = this.rows[r][i];
                a[r] = f === undefined || f === '' ? NaN : Number(f);
            }
            this.nums.set(name, a);
        }
        return a;
    }

    str(name) {
        if (name === null || name === undefined || !this.index.has(name)) {
            return null;
        }
        let a = this.strs.get(name);
        if (!a) {
            const i = this.index.get(name);
            a = this.rows.map((r) => (r[i] === undefined || r[i] === '' ? null : r[i]));
            this.strs.set(name, a);
        }
        return a;
    }

    // Columns whose non-empty fields are all numbers (at least one).
    numericColumns() {
        const out = [];
        for (const h of this.header) {
            const a = this.num(h);
            let any = false;
            let ok = true;
            const i = this.index.get(h);
            for (let r = 0; r < this.n; ++r) {
                const f = this.rows[r][i];
                if (f === undefined || f === '') {
                    continue;
                }
                if (Number.isNaN(a[r])) {
                    ok = false;
                    break;
                }
                any = true;
            }
            if (ok && any) {
                out.push(h);
            }
        }
        return out;
    }
}

// Unit from a column name suffix, per the bundle's naming rule.
export function unitOf(column) {
    const units = [
        ['_deg_s', 'deg/s'], ['_dps', 'deg/s'], ['_m_s', 'm/s'], ['_mm_s', 'mm/s'], ['_rpm', 'rpm'],
        ['_deg', 'deg'], ['_rad', 'rad'], ['_mm', 'mm'], ['_m', 'm'], ['_us', 'us'], ['_ms', 'ms'],
        ['_s', 's'], ['_hz', 'Hz'], ['_bytes', 'bytes'], ['_counts', 'counts'], ['_mdps', 'mdeg/s'],
        ['_mdeg', 'mdeg'], ['_cdeg', 'cdeg'],
    ];
    for (const [suffix, unit] of units) {
        if (column.endsWith(suffix)) {
            return unit;
        }
    }
    return '';
}
