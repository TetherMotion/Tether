/**
 * @file components.ts
 * @brief Custom HTML elements (web components) for the Tether IO dashboard.
 *
 * Defines four custom elements:
 *
 *   - `<tether-catalog>`       — Filterable, selectable list of params/signals.
 *   - `<tether-value-card>`    — Read-once value display for a single entry.
 *   - `<tether-function-list>` — Read-only list of RPC functions.
 *   - `<tether-scope>`         — Canvas-based oscilloscope for live streams.
 *
 * These components are framework-agnostic (no React/Vue) and communicate
 * with the parent `<tether-app>` via properties and `CustomEvent`s.
 */

import {
  CatalogEntry,
  FunctionEntry,
  StreamRow,
  ValueType,
  decodeValueBytes,
  formatValue,
} from './protocol';
import { TetherIOClient } from './client';

/** Human-readable name for a ValueType enum value (e.g. `F64` → `"F64"`). */
const typeName = (type: number): string => ValueType[type] ?? `type ${type}`;

// ===========================================================================
// TetherCatalog — filterable entry list with checkboxes
// ===========================================================================

/**
 * Filterable, selectable list of catalog entries (parameters and/or signals).
 *
 * @fires selection-change — The set of checked entries changed.
 * @fires read-entry       — The "Read now" button on a row was clicked
 *                           (detail: `bigint` entry ID).
 */
export class TetherCatalog extends HTMLElement {
  /** Current entries to display. */
  private entries: CatalogEntry[] = [];
  /** Lowercased filter query (matched against name, group, description, kind). */
  private query = '';
  /** Set of currently checked entry IDs. */
  private selected = new Set<bigint>();

  /** Replace the displayed entries and re-render. */
  set items(value: CatalogEntry[]) {
    this.entries = value;
    this.render();
  }

  /** IDs of all currently checked entries. */
  get selectedIds(): bigint[] {
    return [...this.selected];
  }

  /** Lifecycle: element inserted into the DOM. */
  connectedCallback(): void {
    this.innerHTML =
      '<input class="catalog-filter" placeholder="Filter name, group, description…" ' +
      'aria-label="Filter catalog"><div class="catalog-list"></div>';
    this.querySelector('input')?.addEventListener('input', (event) => {
      this.query = (event.target as HTMLInputElement).value.toLowerCase();
      this.render();
    });
    this.render();
  }

  /** Re-render the list rows, applying the current filter and selection. */
  private render(): void {
    const list = this.querySelector('.catalog-list');
    if (!list) return;

    const rows = this.entries.filter((entry) =>
      `${entry.name} ${entry.group} ${entry.description} ${entry.kind}`
        .toLowerCase()
        .includes(this.query),
    );

    list.innerHTML = rows
      .map(
        (entry) =>
          `<label class="catalog-row">` +
          `<input type="checkbox" data-id="${entry.id}" ${this.selected.has(entry.id) ? 'checked' : ''}>` +
          `<span><strong>${entry.name}</strong>` +
          `<small>${entry.kind} · ${entry.group} · ${typeName(entry.type)}</small></span>` +
          `<button data-read="${entry.id}" title="Read now" type="button">↗</button>` +
          `</label>`,
      )
      .join('');

    // Wire up checkbox toggles → selection-change event.
    list.querySelectorAll<HTMLInputElement>('input').forEach((input) =>
      input.addEventListener('change', () => {
        const id = BigInt(input.dataset.id!);
        if (input.checked) this.selected.add(id);
        else this.selected.delete(id);
        this.dispatchEvent(new Event('selection-change'));
      }),
    );

    // Wire up "Read now" buttons → read-entry event.
    list.querySelectorAll<HTMLButtonElement>('[data-read]').forEach((button) =>
      button.addEventListener('click', (event) => {
        event.preventDefault();
        this.dispatchEvent(new CustomEvent('read-entry', { detail: BigInt(button.dataset.read!) }));
      }),
    );
  }
}
customElements.define('tether-catalog', TetherCatalog);

// ===========================================================================
// TetherValueCard — read-once value display
// ===========================================================================

/**
 * Card that reads and displays the current value of a single catalog entry.
 *
 * Set the `model` property to `{ entry, client }` and call `read()` to fetch
 * the value.  The card re-renders itself on success or failure.
 */
export class TetherValueCard extends HTMLElement {
  private entry?: CatalogEntry;
  private client?: TetherIOClient;
  /** Last read value (formatted string) or an error message. */
  private value = '—';

  /** Provide the entry and client, then trigger an initial render. */
  set model(value: { entry: CatalogEntry; client: TetherIOClient }) {
    this.entry = value.entry;
    this.client = value.client;
    this.render();
  }

  /** Lifecycle: element inserted into the DOM. */
  connectedCallback(): void {
    this.render();
  }

  /** Render the card HTML and wire up the Read button. */
  private render(): void {
    if (!this.entry) return;
    this.innerHTML =
      `<div class="value-card-head"><span>${this.entry.name}</span>` +
      `<button class="read-button">Read</button></div>` +
      `<strong class="value">${this.value}</strong>` +
      `<small>${this.entry.group} · ${typeName(this.entry.type)}</small>`;
    this.querySelector('button')?.addEventListener('click', () => void this.read());
  }

  /**
   * Fetch the current value from the server and update the display.
   * Uses `entry.kind` to decide whether to call `getParam` or `getSignal`.
   */
  async read(): Promise<void> {
    if (!this.entry || !this.client) return;
    try {
      const bytes = await this.client.get(this.entry.kind, this.entry.id);
      this.value = formatValue(decodeValueBytes(bytes, this.entry.type));
      this.render();
    } catch (error) {
      this.value = error instanceof Error ? error.message : 'Read failed';
      this.render();
    }
  }
}
customElements.define('tether-value-card', TetherValueCard);

// ===========================================================================
// TetherFunctionList — read-only function catalog display
// ===========================================================================

/**
 * Read-only list of remotely callable functions.
 *
 * Set the `items` property to an array of `FunctionEntry` to render.
 */
export class TetherFunctionList extends HTMLElement {
  private functions: FunctionEntry[] = [];

  /** Replace the displayed functions and re-render. */
  set items(value: FunctionEntry[]) {
    this.functions = value;
    this.render();
  }

  /** Lifecycle: element inserted into the DOM. */
  connectedCallback(): void {
    this.render();
  }

  /** Render the function list (or an empty-state message). */
  private render(): void {
    this.innerHTML = this.functions.length
      ? this.functions
          .map(
            (fn) =>
              `<article class="function-row">` +
              `<div><strong>${fn.name}</strong>` +
              `<small>${fn.description || fn.group}</small></div>` +
              `<span>${fn.parameters.length} parameter${fn.parameters.length === 1 ? '' : 's'}</span>` +
              `</article>`,
          )
          .join('')
      : '<p class="empty">No functions advertised.</p>';
  }
}
customElements.define('tether-function-list', TetherFunctionList);

// ===========================================================================
// TetherScope — canvas-based oscilloscope
// ===========================================================================

/**
 * Canvas-based oscilloscope that plots streamed values as a scrolling line.
 *
 * Call `push(row, type)` for each `StreamRow` received.  The scope keeps a
 * ring buffer of up to 1200 points and auto-scales to the min/max of the
 * visible window.
 */
export class TetherScope extends HTMLElement {
  /** Ring buffer of numeric sample points (max 1200). */
  private points: number[] = [];
  /** Cached 2D rendering context (lazily acquired on connect). */
  private context?: CanvasRenderingContext2D | null;
  /** ResizeObserver for high-DPI canvas resizing. */
  private resizeObserver?: ResizeObserver;

  /** Lifecycle: element inserted into the DOM. */
  connectedCallback(): void {
    this.innerHTML = '<canvas aria-label="Live signal plot"></canvas>';
    const canvas = this.querySelector('canvas')!;
    this.context = canvas.getContext('2d');
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(() => this.resize());
      this.resizeObserver.observe(this);
    }
    this.resize();
  }

  /** Lifecycle: element removed from the DOM. */
  disconnectedCallback(): void {
    this.resizeObserver?.disconnect();
  }

  /**
   * Push one stream row into the ring buffer and redraw.
   *
   * Only the first value of the row is plotted (single-channel scope).
   * Non-numeric values are silently dropped.
   *
   * @param row  The StreamRow from a StreamData message.
   * @param type Value type for decoding the raw bytes.
   */
  push(row: StreamRow, type: ValueType): void {
    const bytes = row.values[0];
    if (!bytes) return;
    const value = decodeValueBytes(bytes, type);
    const numeric =
      typeof value === 'bigint'
        ? Number(value)
        : typeof value === 'boolean'
          ? value
            ? 1
            : 0
          : typeof value === 'number'
            ? value
            : Number(value);
    if (!Number.isFinite(numeric)) return;
    this.points.push(numeric);
    if (this.points.length > 1200) this.points.splice(0, this.points.length - 1200);
    this.draw();
  }

  /** Resize the canvas to match the element's CSS size (high-DPI aware). */
  private resize(): void {
    const canvas = this.querySelector('canvas') as HTMLCanvasElement | null;
    if (!canvas) return;
    const rect = this.getBoundingClientRect();
    const scale = typeof devicePixelRatio === 'number' ? devicePixelRatio : 1;
    canvas.width = Math.max(1, rect.width * scale);
    canvas.height = Math.max(1, rect.height * scale);
    this.draw();
  }

  /** Redraw the oscilloscope: grid lines + the value trace. */
  private draw(): void {
    if (!this.context || !this.canvas) return;
    const canvas = this.canvas;
    const ctx = this.context;
    const width = canvas.width;
    const height = canvas.height;

    // Clear and draw horizontal grid lines.
    ctx.clearRect(0, 0, width, height);
    ctx.strokeStyle = 'rgba(130,180,220,.15)';
    for (let y = height / 4; y < height; y += height / 4) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    if (this.points.length < 2) return;

    // Auto-scale to the current min/max.
    const min = Math.min(...this.points);
    const max = Math.max(...this.points);
    const range = max - min || 1;

    // Draw the trace.
    ctx.strokeStyle = '#80f4d0';
    ctx.lineWidth = 2 * devicePixelRatio;
    ctx.beginPath();
    this.points.forEach((point, index) => {
      const x = (index / (this.points.length - 1)) * width;
      const y =
        height - ((point - min) / range) * (height - 20 * devicePixelRatio) - 10 * devicePixelRatio;
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  /** The inner <canvas> element (lazily queried). */
  private get canvas(): HTMLCanvasElement | null {
    return this.querySelector('canvas');
  }
}
customElements.define('tether-scope', TetherScope);
