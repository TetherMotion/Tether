import { CatalogEntry, FunctionEntry, StreamRow, ValueType, decodeValueBytes, formatValue } from './protocol';
import { TetherIoClient } from './client';

const typeName = (type: number): string => ValueType[type] ?? `type ${type}`;

export class TetherCatalog extends HTMLElement {
  private entries: CatalogEntry[] = [];
  private query = '';
  private selected = new Set<bigint>();
  set items(value: CatalogEntry[]) { this.entries = value; this.render(); }
  get selectedIds(): bigint[] { return [...this.selected]; }
  connectedCallback(): void { this.innerHTML = `<input class="catalog-filter" placeholder="Filter name, group, description…" aria-label="Filter catalog"><div class="catalog-list"></div>`; this.querySelector('input')?.addEventListener('input', event => { this.query = (event.target as HTMLInputElement).value.toLowerCase(); this.render(); }); this.render(); }
  private render(): void { const list = this.querySelector('.catalog-list'); if (!list) return; const rows = this.entries.filter(entry => `${entry.name} ${entry.group} ${entry.description}`.toLowerCase().includes(this.query)); list.innerHTML = rows.map(entry => `<label class="catalog-row"><input type="checkbox" data-id="${entry.id}" ${this.selected.has(entry.id) ? 'checked' : ''}><span><strong>${entry.name}</strong><small>${entry.group} · ${typeName(entry.type)}</small></span><button data-read="${entry.id}" title="Read now" type="button">↗</button></label>`).join(''); list.querySelectorAll<HTMLInputElement>('input').forEach(input => input.addEventListener('change', () => { const id = BigInt(input.dataset.id!); if (input.checked) this.selected.add(id); else this.selected.delete(id); this.dispatchEvent(new Event('selection-change')); })); list.querySelectorAll<HTMLButtonElement>('[data-read]').forEach(button => button.addEventListener('click', event => { event.preventDefault(); this.dispatchEvent(new CustomEvent('read-entry', { detail: BigInt(button.dataset.read!) })); })); }
}
customElements.define('tether-catalog', TetherCatalog);

export class TetherValueCard extends HTMLElement {
  private entry?: CatalogEntry;
  private client?: TetherIoClient;
  private value = '—';
  set model(value: { entry: CatalogEntry; client: TetherIoClient }) { this.entry = value.entry; this.client = value.client; this.render(); }
  connectedCallback(): void { this.render(); }
  private render(): void { if (!this.entry) return; this.innerHTML = `<div class="value-card-head"><span>${this.entry.name}</span><button class="read-button">Read</button></div><strong class="value">${this.value}</strong><small>${this.entry.group} · ${typeName(this.entry.type)}</small>`; this.querySelector('button')?.addEventListener('click', () => void this.read()); }
  async read(): Promise<void> { if (!this.entry || !this.client) return; try { const bytes = await this.client.get(this.entry.name.includes('signal') ? 'signal' : 'signal', this.entry.id); this.value = formatValue(decodeValueBytes(bytes, this.entry.type)); this.render(); } catch (error) { this.value = error instanceof Error ? error.message : 'Read failed'; this.render(); } }
}
customElements.define('tether-value-card', TetherValueCard);

export class TetherFunctionList extends HTMLElement {
  private functions: FunctionEntry[] = [];
  set items(value: FunctionEntry[]) { this.functions = value; this.render(); }
  connectedCallback(): void { this.render(); }
  private render(): void { this.innerHTML = this.functions.length ? this.functions.map(fn => `<article class="function-row"><div><strong>${fn.name}</strong><small>${fn.description || fn.group}</small></div><span>${fn.parameters.length} parameter${fn.parameters.length === 1 ? '' : 's'}</span></article>`).join('') : '<p class="empty">No functions advertised.</p>'; }
}
customElements.define('tether-function-list', TetherFunctionList);

export class TetherScope extends HTMLElement {
  private points: number[] = [];
  private context?: CanvasRenderingContext2D | null;
  private resizeObserver?: ResizeObserver;
  connectedCallback(): void { this.innerHTML = '<canvas aria-label="Live signal plot"></canvas>'; const canvas = this.querySelector('canvas')!; this.context = canvas.getContext('2d'); if (typeof ResizeObserver !== 'undefined') { this.resizeObserver = new ResizeObserver(() => this.resize()); this.resizeObserver.observe(this); } this.resize(); }
  disconnectedCallback(): void { this.resizeObserver?.disconnect(); }
  push(row: StreamRow, type: ValueType): void { const bytes = row.values[0]; if (!bytes) return; const value = decodeValueBytes(bytes, type); const numeric = typeof value === 'bigint' ? Number(value) : typeof value === 'boolean' ? (value ? 1 : 0) : typeof value === 'number' ? value : Number(value); if (!Number.isFinite(numeric)) return; this.points.push(numeric); if (this.points.length > 1200) this.points.splice(0, this.points.length - 1200); this.draw(); }
  private resize(): void { const canvas = this.querySelector('canvas') as HTMLCanvasElement | null; if (!canvas) return; const rect = this.getBoundingClientRect(); const scale = typeof devicePixelRatio === 'number' ? devicePixelRatio : 1; canvas.width = Math.max(1, rect.width * scale); canvas.height = Math.max(1, rect.height * scale); this.draw(); }
  private draw(): void { if (!this.context || !this.canvas) return; const canvas = this.canvas; const ctx = this.context; const width = canvas.width; const height = canvas.height; ctx.clearRect(0, 0, width, height); ctx.strokeStyle = 'rgba(130,180,220,.15)'; for (let y = height / 4; y < height; y += height / 4) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke(); } if (this.points.length < 2) return; const min = Math.min(...this.points); const max = Math.max(...this.points); const range = max - min || 1; ctx.strokeStyle = '#80f4d0'; ctx.lineWidth = 2 * devicePixelRatio; ctx.beginPath(); this.points.forEach((point, index) => { const x = index / (this.points.length - 1) * width; const y = height - ((point - min) / range) * (height - 20 * devicePixelRatio) - 10 * devicePixelRatio; if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); }); ctx.stroke(); }
  private get canvas(): HTMLCanvasElement | null { return this.querySelector('canvas'); }
}
customElements.define('tether-scope', TetherScope);
