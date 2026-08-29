/**
 * @file main.ts
 * @brief Top-level application component for the Tether IO dashboard.
 *
 * `<tether-app>` is the root custom element that orchestrates the entire
 * dashboard: it owns the {@link TetherIOClient} instance, renders the
 * static HTML shell, wires up event handlers, and manages tab switching
 * and stream lifecycle.
 *
 * On connection it fetches both the parameter and signal catalogs in
 * parallel and displays them combined under the "All" tab.  The user can
 * then filter to just signals, just parameters, or view the function list.
 */

import './components';
import { TetherIOClient } from './client';
import {
  CatalogEntry,
  FunctionEntry,
  StreamLayoutEntry,
  StreamRow,
  ValueType,
  decodeValueBytes,
} from './protocol';
import './style.css';

/** Type alias for the `<tether-webgpu-scope>` element's public interface. */
type TetherScope = HTMLElement & {
  setChannels: (channels: { name: string; color: [number, number, number] }[]) => void;
  push: (timestampUs: bigint, values: number[]) => void;
  clear: () => void;
  togglePause: () => boolean;
  resetView: () => void;
};

/** Tab identifiers for the catalog navigation. */
type Tab = 'all' | 'signals' | 'params' | 'functions';

/** Default color palette for stream channels (ggplot-like). */
const STREAM_COLORS: [number, number, number][] = [
  [0.0, 0.45, 0.75], // blue
  [0.85, 0.33, 0.1], // orange
  [0.0, 0.62, 0.45], // green
  [0.8, 0.1, 0.2], // red
  [0.58, 0.4, 0.74], // purple
  [0.91, 0.59, 0.09], // brown
  [0.75, 0.31, 0.5], // pink
  [0.4, 0.4, 0.4], // gray
];

/**
 * Root dashboard component.
 *
 * @customElement tether-app
 */
class TetherApp extends HTMLElement {
  /** WebSocket protocol client (owned by this element). */
  private readonly client = new TetherIOClient();
  /** Cached signal catalog entries. */
  private signals: CatalogEntry[] = [];
  /** Cached parameter catalog entries. */
  private params: CatalogEntry[] = [];
  /** Cached function catalog entries. */
  private functions: FunctionEntry[] = [];
  /** Layout of the currently configured stream (set by configureStream). */
  private streamLayout: StreamLayoutEntry[] = [];
  /** Whether a stream is currently active. */
  private streamActive = false;

  // ---- Lifecycle --------------------------------------------------------

  /** Lifecycle: element inserted into the DOM — render, bind, auto-connect. */
  connectedCallback(): void {
    // Default to light mode unless the user previously chose dark.
    const saved = localStorage.getItem('tether-theme');
    if (saved === 'dark' || saved === 'light') {
      document.documentElement.dataset.theme = saved;
    } else {
      document.documentElement.dataset.theme = 'light';
    }
    this.render();
    this.bind();
    void this.connect();
  }

  // ---- Rendering --------------------------------------------------------

  /**
   * Render the static HTML shell of the dashboard.
   *
   * The WebSocket URL is derived from `window.location` so the dashboard
   * works regardless of the host/port it is served from.
   */
  private render(): void {
    const wsProto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProto}//${window.location.host}/tether-io`;

    this.innerHTML = `
      <main class="shell">
        <header class="topbar">
          <div class="brand">
            <span class="brand-mark">T</span>
            <div>
              <strong>Tether IO</strong>
              <small>Realtime control cockpit</small>
            </div>
          </div>
          <div class="connection">
            <input id="url" value="${wsUrl}" aria-label="WebSocket URL">
            <button id="connect">Connect</button>
            <button id="theme-toggle" class="theme-toggle" aria-label="Toggle theme" title="Toggle light/dark">
              ${document.documentElement.dataset.theme === 'dark' ? '\u2600\ufe0f' : '\u{1F319}'}
            </button>
            <span id="status" class="status">Offline</span>
          </div>
        </header>

        <section class="hero">
          <div>
            <p class="eyebrow">EtherCAT observability</p>
            <h1>See the machine breathe.</h1>
            <p class="lede">
              Explore live parameters, signals, functions and PDO telemetry
              through one calm, glassy control surface.
            </p>
          </div>
          <div class="hero-orb">
            <span>1 kHz</span>
            <small>stream-ready</small>
          </div>
        </section>

        <section class="workspace">
          <!-- Left panel: catalog with tabs -->
          <aside class="panel catalog-panel">
            <div class="panel-title">
              <div>
                <span class="eyebrow">Catalog</span>
                <h2>Everything at hand</h2>
              </div>
              <span id="catalog-count" class="badge">0</span>
            </div>
            <nav class="tabs">
              <button class="tab active" data-tab="all">All</button>
              <button class="tab" data-tab="signals">Signals</button>
              <button class="tab" data-tab="params">Parameters</button>
              <button class="tab" data-tab="functions">Functions</button>
            </nav>
            <tether-catalog id="catalog"></tether-catalog>
          </aside>

          <!-- Right panel: scope + params + functions -->
          <section class="content">
            <div class="metrics">
              <article class="metric panel">
                <small>Connection</small>
                <strong id="metric-connection">Offline</strong>
                <span>WebSocket binary</span>
              </article>
              <article class="metric panel">
                <small>Selected points</small>
                <strong id="metric-points">0</strong>
                <span>ring-buffered locally</span>
              </article>
              <article class="metric panel">
                <small>Transport</small>
                <strong>SLIP / IO v5</strong>
                <span>bounded frames</span>
              </article>
            </div>

            <section class="scope panel">
              <div class="scope-head">
                <div>
                  <span class="eyebrow">Oscilloscope</span>
                  <h2>Signal trace</h2>
                </div>
                <div class="scope-actions">
                  <span class="live-dot" id="live-dot">● LIVE</span>
                  <button id="pause-btn" class="secondary">Pause</button>
                  <button id="reset-zoom-btn" class="secondary" hidden>Reset zoom</button>
                </div>
              </div>
              <tether-webgpu-scope id="scope"></tether-webgpu-scope>
            </section>

            <section class="panel param-panel" id="param-panel">
              <tether-param-panel id="params-editor"></tether-param-panel>
            </section>

            <section class="panel function-panel">
              <div class="panel-title">
                <div>
                  <span class="eyebrow">RPC surface</span>
                  <h2>Functions</h2>
                </div>
              </div>
              <tether-function-list id="functions"></tether-function-list>
            </section>
          </section>
        </section>

        <footer>
          <span>Static TypeScript web components</span>
          <span>Designed for loss-tolerant realtime telemetry</span>
        </footer>

        <div id="toast-host" class="toast-host" aria-live="polite"></div>
      </main>
    `;
  }

  // ---- Event binding ----------------------------------------------------

  /**
   * Wire up all event listeners: buttons, tabs, catalog events, and
   * client events (connected/disconnected/stream/error).
   */
  private bind(): void {
    // Connect button
    this.querySelector<HTMLButtonElement>('#connect')!.addEventListener(
      'click',
      () => void this.connect(),
    );

    // Theme toggle (light/dark)
    this.querySelector<HTMLButtonElement>('#theme-toggle')!.addEventListener('click', () =>
      this.toggleTheme(),
    );

    // Pause / Reset zoom buttons.
    this.querySelector<HTMLButtonElement>('#pause-btn')!.addEventListener('click', () => {
      const scope = this.querySelector<TetherScope>('tether-webgpu-scope');
      if (!scope) return;
      const paused = scope.togglePause();
      const btn = this.querySelector<HTMLButtonElement>('#pause-btn')!;
      btn.textContent = paused ? 'Resume' : 'Pause';
      const dot = this.querySelector('#live-dot')!;
      dot.textContent = paused ? '⏸ PAUSED' : '● LIVE';
      // Reset zoom button is only relevant while paused.
      this.querySelector<HTMLButtonElement>('#reset-zoom-btn')!.hidden = !paused;
    });
    this.querySelector<HTMLButtonElement>('#reset-zoom-btn')!.addEventListener('click', () => {
      const scope = this.querySelector<TetherScope>('tether-webgpu-scope');
      if (!scope) return;
      // Only reset zoom — do NOT resume.  The view stays paused at the
      // full 5-second window.
      scope.resetView();
    });

    // Tab buttons
    this.querySelectorAll<HTMLButtonElement>('.tab').forEach((tab) =>
      tab.addEventListener('click', () => void this.switchTab(tab.dataset.tab as Tab)),
    );

    // Catalog selection changes → auto-(re)start stream with the new
    // selection of signals.  No manual start/stop buttons.
    this.querySelector('tether-catalog')?.addEventListener('selection-change', () => {
      void this.onSelectionChange();
    });

    // Client lifecycle events
    this.client.addEventListener('connected', () => this.setStatus('Online', true));
    this.client.addEventListener('disconnected', () => this.setStatus('Offline', false));
    this.client.addEventListener('error-message', (event: Event) => {
      const message =
        (event as CustomEvent<{ message?: string }>).detail.message ?? 'Protocol error';
      this.setStatus(message, false);
      this.showToast(message, 'error');
    });

    // Stream data → decode all channels and push into the WebGPU oscilloscope
    this.client.addEventListener('stream', (event: Event) => {
      const row = (event as CustomEvent<StreamRow>).detail;
      const scope = this.querySelector<TetherScope>('tether-webgpu-scope');
      if (scope && this.streamLayout.length > 0) {
        // Decode each channel's value from the raw bytes.
        const values = row.values.map((bytes, i) => {
          const type = this.streamLayout[i]?.type ?? ValueType.F32;
          const decoded = decodeValueBytes(bytes, type);
          if (typeof decoded === 'number') return decoded;
          if (typeof decoded === 'bigint') return Number(decoded);
          if (typeof decoded === 'boolean') return decoded ? 1 : 0;
          return 0;
        });
        scope.push(row.timestampUs, values);
      }
      this.querySelector('#metric-points')!.textContent = String(row.values.length);
    });
  }

  // ---- Connection & catalog loading -------------------------------------

  /**
   * Connect to the WebSocket server and load the initial catalog.
   *
   * Reads the URL from the `#url` input field, connects, then fetches
   * both params and signals in parallel.
   */
  private async connect(): Promise<void> {
    const url = this.querySelector<HTMLInputElement>('#url')!.value;
    try {
      await this.client.connect(url);
      await this.loadAll();
    } catch (error) {
      this.setStatus(error instanceof Error ? error.message : 'Connection failed', false);
    }
  }

  /**
   * Fetch parameters and signals in parallel and render the combined
   * catalog under the "All" tab.
   */
  private async loadAll(): Promise<void> {
    try {
      const [params, signals] = await Promise.all([
        this.client.list('params'),
        this.client.list('signals'),
      ]);
      this.params = params;
      this.signals = signals;
      this.renderCatalog([...params, ...signals]);
      this.renderParamPanel();
    } catch (error) {
      this.setStatus(error instanceof Error ? error.message : 'Catalog failed', false);
    }
  }

  // ---- Tab switching ----------------------------------------------------

  /**
   * Switch the catalog view to the given tab.
   *
   * - `'all'`       — Show cached params + signals (no refetch).
   * - `'signals'`   — Fetch and show only signals.
   * - `'params'`    — Fetch and show only parameters.
   * - `'functions'` — Fetch and show the function list.
   */
  private async switchTab(tab: Tab): Promise<void> {
    // Update active-tab styling
    this.querySelectorAll('.tab').forEach((item) =>
      item.classList.toggle('active', (item as HTMLElement).dataset.tab === tab),
    );

    // Functions tab: fetch function catalog
    if (tab === 'functions') {
      try {
        this.functions = await this.client.listFunctions();
        (
          this.querySelector('#functions') as HTMLElement & {
            items: FunctionEntry[];
          }
        ).items = this.functions;
      } catch (error) {
        this.setStatus(error instanceof Error ? error.message : 'Function catalog failed', false);
      }
      return;
    }

    // All tab: show cached data (no refetch)
    if (tab === 'all') {
      this.renderCatalog([...this.params, ...this.signals]);
      this.renderParamPanel();
      return;
    }

    // Signals / Params tab: fetch (or refetch) and display
    try {
      const entries = await this.client.list(tab);
      if (tab === 'signals') this.signals = entries;
      else this.params = entries;
      this.renderCatalog(entries);
      this.renderParamPanel();
    } catch (error) {
      this.setStatus(error instanceof Error ? error.message : 'Catalog failed', false);
    }
  }

  // ---- Catalog rendering ------------------------------------------------

  /**
   * Render the given entries into the `<tether-catalog>` element and
   * update the count badge.
   */
  private renderCatalog(entries: CatalogEntry[]): void {
    const catalog = this.querySelector('tether-catalog') as HTMLElement & {
      items: CatalogEntry[];
      selectedIds: bigint[];
    };
    catalog.items = entries;
    this.querySelector('#catalog-count')!.textContent = String(entries.length);
  }

  /**
   * Render the editable parameter panel below the value card.
   */
  private renderParamPanel(): void {
    const panel = this.querySelector('tether-param-panel') as HTMLElement & {
      model: { params: CatalogEntry[]; client: TetherIOClient };
    };
    panel.model = { params: this.params, client: this.client };
  }

  /**
   * Handle catalog selection changes: automatically (re)start the stream
   * with the currently selected signals.  If no signals are selected, any
   * active stream is stopped.
   */
  private async onSelectionChange(): Promise<void> {
    const catalog = this.querySelector('tether-catalog') as HTMLElement & {
      selectedIds: bigint[];
    };
    const selectedIds = catalog.selectedIds;

    // Filter to just signal IDs — parameters cannot be streamed.
    const signalIds = new Set(this.signals.map((s) => s.id));
    const selectedSignalIds = selectedIds.filter((id) => signalIds.has(id));

    // Stop any active stream first.
    if (this.streamActive) {
      await this.client.stopStream();
      this.streamActive = false;
    }

    if (selectedSignalIds.length === 0) {
      // Nothing to stream — clear the scope.
      const scope = this.querySelector<TetherScope>('tether-webgpu-scope');
      if (scope) scope.setChannels([]);
      return;
    }

    // Configure + start a new stream over the selected signals.
    // 1 kHz (1 ms interval), chunks of 20 rows for smooth throughput.
    await this.client.configureStream(selectedSignalIds, 1, 20);
    this.streamLayout = this.client.currentStreamLayout;
    const scope = this.querySelector<TetherScope>('tether-webgpu-scope');
    if (scope) {
      const allEntries = [...this.signals, ...this.params];
      const channels = this.streamLayout.map((entry, i) => {
        const catalogEntry = allEntries.find((e) => e.id === entry.id);
        const name = catalogEntry?.name ?? `ch${i}`;
        const color = STREAM_COLORS[i % STREAM_COLORS.length] ?? ([0.5, 0.5, 0.5] as const);
        return { name, color };
      });
      scope.setChannels(channels);
      scope.clear();
    }
    await this.client.startStream();
    this.streamActive = true;
  }

  // ---- Theme ---------------------------------------------------------------

  /**
   * Toggle between light and dark themes.
   *
   * Persists the choice to `localStorage` and updates the toggle button icon.
   * The WebGPU oscilloscope reads CSS variables on its next frame, so no
   * explicit re-render is needed.
   */
  private toggleTheme(): void {
    const current = document.documentElement.dataset.theme ?? 'light';
    const next = current === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    localStorage.setItem('tether-theme', next);
    const btn = this.querySelector<HTMLButtonElement>('#theme-toggle');
    if (btn) btn.textContent = next === 'dark' ? '\u2600\ufe0f' : '\u{1F319}';
  }

  // ---- Status display ---------------------------------------------------

  /**
   * Update the connection status text and styling in both the top bar
   * and the metrics panel.
   */
  private setStatus(text: string, online: boolean): void {
    this.querySelector('#status')!.textContent = text;
    this.querySelector('#metric-connection')!.textContent = text;
    this.querySelector('#status')!.classList.toggle('online', online);
  }

  // ---- Toast notifications ---------------------------------------------

  /**
   * Show a transient toast notification in the bottom-right corner.
   *
   * @param message  Text to display.
   * @param kind     `'error'` (red) or `'info'` (neutral).  Errors stay
   *                 visible longer than info toasts.
   */
  private showToast(message: string, kind: 'error' | 'info' = 'info'): void {
    const host = this.querySelector<HTMLElement>('#toast-host');
    if (!host) return;
    const toast = document.createElement('div');
    toast.className = `toast toast-${kind}`;
    toast.textContent = message;
    host.appendChild(toast);
    // Animate in on the next frame.
    requestAnimationFrame(() => toast.classList.add('toast-visible'));
    const ttl = kind === 'error' ? 6000 : 3000;
    window.setTimeout(() => {
      toast.classList.remove('toast-visible');
      window.setTimeout(() => toast.remove(), 300);
    }, ttl);
  }
}

customElements.define('tether-app', TetherApp);
