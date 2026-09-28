// dom.js
// Keyed DOM updates for the panels. A row or badge is created once per key
// and afterwards only its changed text, class or title is written, so a
// panel refresh costs a few string compares instead of a table rebuild and
// the layout the browser has to redo.

export function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) {
        e.className = cls;
    }
    if (text !== undefined) {
        e.textContent = text;
    }
    return e;
}

// Writes only on change; the DOM compare is the expensive part, not ours.
export function setText(node, text) {
    const s = text === undefined || text === null ? '' : String(text);
    if (node.__text !== s) {
        node.__text = s;
        node.textContent = s;
    }
}

export function setClass(node, cls) {
    const s = cls || '';
    if (node.__cls !== s) {
        node.__cls = s;
        node.className = s;
    }
}

export function setTitle(node, title) {
    const s = title || '';
    if (node.__title !== s) {
        node.__title = s;
        node.title = s;
    }
}

export function setHidden(node, hidden) {
    const h = !!hidden;
    if (node.__hidden !== h) {
        node.__hidden = h;
        node.hidden = h;
    }
}

// A table with a fixed header and rows updated in place by key. A cell is
// a value or [value, className]. Rows not in the latest update are removed.
export class KeyedTable {
    constructor(table, header) {
        this.table = table;
        this.rows = new Map();
        this.seen = new Set();
        table.textContent = '';
        this.head = el('tr');
        this.setHeader(header || []);
        table.appendChild(this.head);
    }

    setHeader(header) {
        const key = header.join('\u0001');
        if (this.headerKey === key) {
            return;
        }
        this.headerKey = key;
        this.head.textContent = '';
        for (const h of header) {
            this.head.appendChild(el('th', undefined, h));
        }
    }

    // One row. Call begin(), then row() for each row in order, then end().
    begin() {
        this.seen.clear();
        this.position = 1; // after the header row
    }

    row(key, cells, opts) {
        let r = this.rows.get(key);
        if (!r) {
            r = { tr: el('tr'), tds: [] };
            this.rows.set(key, r);
            if (opts && opts.onCreate) {
                opts.onCreate(r.tr);
            }
        }
        while (r.tds.length < cells.length) {
            const td = el('td');
            r.tds.push(td);
            r.tr.appendChild(td);
        }
        while (r.tds.length > cells.length) {
            r.tr.removeChild(r.tds.pop());
        }
        for (let i = 0; i < cells.length; ++i) {
            const c = cells[i];
            const td = r.tds[i];
            if (Array.isArray(c)) {
                setText(td, c[0]);
                setClass(td, c[1]);
                if (c[2] !== undefined && td.colSpan !== c[2]) {
                    td.colSpan = c[2];
                }
            } else {
                setText(td, c);
                setClass(td, '');
            }
        }
        if (opts && opts.title !== undefined) {
            setTitle(r.tr, opts.title);
        }
        if (opts && opts.cls !== undefined) {
            setClass(r.tr, opts.cls);
        }
        const at = this.table.children[this.position];
        if (at !== r.tr) {
            this.table.insertBefore(r.tr, at || null);
        }
        this.position += 1;
        this.seen.add(key);
        return r.tr;
    }

    end() {
        for (const [key, r] of this.rows) {
            if (!this.seen.has(key)) {
                r.tr.remove();
                this.rows.delete(key);
            }
        }
    }

    clear() {
        this.begin();
        this.end();
    }
}

// Badges keyed like table rows; absent keys are hidden, not destroyed.
export class BadgeSet {
    constructor(container) {
        this.container = container;
        this.badges = new Map();
        this.seen = new Set();
    }

    begin() {
        this.seen.clear();
        this.position = 0;
    }

    badge(key, text, cls, title) {
        let b = this.badges.get(key);
        if (!b) {
            b = el('span');
            b.dataset.badge = key;
            this.badges.set(key, b);
        }
        setText(b, text);
        setClass(b, 'badge ' + (cls || 'info'));
        setTitle(b, title);
        setHidden(b, false);
        const at = this.container.children[this.position];
        if (at !== b) {
            this.container.insertBefore(b, at || null);
        }
        this.position += 1;
        this.seen.add(key);
        return b;
    }

    end() {
        for (const [key, b] of this.badges) {
            if (!this.seen.has(key)) {
                setHidden(b, true);
            }
        }
    }
}

// Lines of text keyed by position, for short kv blocks.
export class Lines {
    constructor(container) {
        this.container = container;
        this.nodes = [];
        this.count = 0;
    }

    begin() {
        this.count = 0;
    }

    line(text, cls) {
        let n = this.nodes[this.count];
        if (!n) {
            n = el('div');
            this.nodes.push(n);
            this.container.appendChild(n);
        }
        setText(n, text);
        setClass(n, cls);
        setHidden(n, false);
        this.count += 1;
        return n;
    }

    end() {
        for (let i = this.count; i < this.nodes.length; ++i) {
            setHidden(this.nodes[i], true);
        }
    }
}
