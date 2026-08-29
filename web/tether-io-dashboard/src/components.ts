/**
 * @file components.ts
 * @brief Custom HTML elements (web components) for the Tether IO dashboard.
 *
 * Defines three custom elements:
 *
 *   - `<tether-catalog>`       — Filterable, selectable list of params/signals.
 *   - `<tether-value-card>`    — Read-once value display for a single entry.
 *   - `<tether-function-list>` — Read-only list of RPC functions.
 *
 * The WebGPU-based oscilloscope (`<tether-webgpu-scope>`) is defined in
 * {@link ./webgpu-scope} and re-exported here for convenience.
 *
 * These components are framework-agnostic (no React/Vue) and communicate
 * with the parent `<tether-app>` via properties and `CustomEvent`s.
 */

import { CatalogEntry, FunctionEntry, ValueType, decodeValueBytes, formatValue } from './protocol';
import { TetherIOClient } from './client';

// Re-export the WebGPU oscilloscope so importing 'components' registers it.
export { WebGPUScope } from './webgpu-scope';

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
// TetherParamPanel — editable parameter grid
// ===========================================================================

/**
 * Editable grid of F64 parameters.
 *
 * Set the `model` property to `{ params, client }` to render one row per
 * parameter.  Changes to a numeric input are encoded as F64 and sent to
 * the server via `TetherIOClient.setParameter()`.
 */
export class TetherParamPanel extends HTMLElement {
  private params: CatalogEntry[] = [];
  private client?: TetherIOClient;

  /** Provide the parameter list and client, then trigger an initial render. */
  set model(value: { params: CatalogEntry[]; client: TetherIOClient }) {
    this.params = value.params;
    this.client = value.client;
    this.render();
  }

  /** Lifecycle: element inserted into the DOM. */
  connectedCallback(): void {
    this.render();
  }

  /** Encode a JavaScript number as an 8-byte little-endian F64. */
  private encodeF64(value: number): Uint8Array {
    const out = new Uint8Array(8);
    new DataView(out.buffer).setFloat64(0, value, true);
    return out;
  }

  /** Render the parameter rows and wire up input → setParameter(). */
  private render(): void {
    if (this.params.length === 0) {
      this.innerHTML = '<p class="empty">No parameters available.</p>';
      return;
    }
    this.innerHTML =
      `<div class="param-panel-head"><span class="eyebrow">Parameters</span>` +
      `<h2>Live tuning</h2></div>` +
      this.params
        .map(
          (p) =>
            `<label class="param-row" data-id="${p.id}">` +
            `<span><strong>${p.name}</strong><small>${p.group} · ${typeName(p.type)}</small></span>` +
            `<input type="number" step="any" value="" ${p.type === ValueType.F64 ? '' : 'disabled'}>` +
            `</label>`,
        )
        .join('');

    // Read current values to populate the inputs.
    for (const p of this.params) {
      void this.loadValue(p);
    }

    // Wire up change events.
    this.querySelectorAll<HTMLInputElement>('input[type="number"]').forEach((input) =>
      input.addEventListener('change', () => {
        const id = BigInt(input.closest('.param-row')?.getAttribute('data-id') ?? '0');
        const value = parseFloat(input.value);
        if (Number.isNaN(value) || !this.client) return;
        void this.client.setParameter(id, this.encodeF64(value));
      }),
    );
  }

  /** Fetch the current value of a parameter and display it in the input. */
  private async loadValue(entry: CatalogEntry): Promise<void> {
    if (!this.client) return;
    try {
      const bytes = await this.client.get('param', entry.id);
      const decoded = decodeValueBytes(bytes, entry.type);
      const input = this.querySelector<HTMLInputElement>(`.param-row[data-id="${entry.id}"] input`);
      if (input && typeof decoded === 'number') input.value = String(decoded);
    } catch {
      // Leave the input empty; the user can still type a new value.
    }
  }
}
customElements.define('tether-param-panel', TetherParamPanel);

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
