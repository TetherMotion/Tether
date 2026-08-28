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
import { CatalogEntry, FunctionEntry, ValueType } from './protocol';
import './style.css';

/** Type alias for the `<tether-scope>` element's public interface. */
type TetherScope = HTMLElement & {
  push: (row: unknown, type: ValueType) => void;
};

/** Tab identifiers for the catalog navigation. */
type Tab = 'all' | 'signals' | 'params' | 'functions';

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
  /** Currently selected entry (for Read / Stream actions). */
  private active?: CatalogEntry;
  /** Kind of the currently active entry (determines get vs getSignal). */
  private activeKind: 'param' | 'signal' = 'signal';
  /** Cached function catalog entries. */
  private functions: FunctionEntry[] = [];

  // ---- Lifecycle --------------------------------------------------------

  /** Lifecycle: element inserted into the DOM — render, bind, auto-connect. */
  connectedCallback(): void {
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

          <!-- Right panel: monitor + scope + functions -->
          <section class="content">
            <div class="toolbar panel">
              <div>
                <span class="eyebrow">Live monitor</span>
                <h2 id="selection-title">Select a signal to inspect</h2>
              </div>
              <div class="actions">
                <button id="read">Read once</button>
                <button id="stream" class="primary">Start stream</button>
              </div>
            </div>

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
                <span class="live-dot">● LIVE</span>
              </div>
              <tether-scope id="scope"></tether-scope>
            </section>

            <section class="value-grid" id="value-grid"></section>

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

    // Read / Stream buttons
    this.querySelector<HTMLButtonElement>('#read')!.addEventListener(
      'click',
      () => void this.readActive(),
    );
    this.querySelector<HTMLButtonElement>('#stream')!.addEventListener(
      'click',
      () => void this.toggleStream(),
    );

    // Tab buttons
    this.querySelectorAll<HTMLButtonElement>('.tab').forEach((tab) =>
      tab.addEventListener('click', () => void this.switchTab(tab.dataset.tab as Tab)),
    );

    // Catalog "read-entry" event (from the ↗ button on each row)
    this.querySelector('tether-catalog')?.addEventListener('read-entry', (event) => {
      const id = (event as CustomEvent<bigint>).detail;
      this.active = [...this.signals, ...this.params].find((entry) => entry.id === id);
      if (this.active) this.activeKind = this.active.kind;
      void this.readActive();
    });

    // Client lifecycle events
    this.client.addEventListener('connected', () => this.setStatus('Online', true));
    this.client.addEventListener('disconnected', () => this.setStatus('Offline', false));
    this.client.addEventListener('error-message', (event: Event) =>
      this.setStatus(
        (event as CustomEvent<{ message?: string }>).detail.message ?? 'Protocol error',
        false,
      ),
    );

    // Stream data → push into the oscilloscope
    this.client.addEventListener('stream', (event: Event) => {
      const row = (event as CustomEvent).detail;
      this.querySelector<TetherScope>('tether-scope')?.push(
        row,
        this.active?.type ?? ValueType.F32,
      );
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
      return;
    }

    // Signals / Params tab: fetch (or refetch) and display
    this.activeKind = tab === 'params' ? 'param' : 'signal';
    try {
      const entries = await this.client.list(tab);
      if (tab === 'signals') this.signals = entries;
      else this.params = entries;
      this.renderCatalog(entries);
    } catch (error) {
      this.setStatus(error instanceof Error ? error.message : 'Catalog failed', false);
    }
  }

  // ---- Catalog rendering ------------------------------------------------

  /**
   * Render the given entries into the `<tether-catalog>` element and
   * update the count badge.
   *
   * Also wires up the `selection-change` event to update `this.active`
   * and `this.activeKind` when the user checks an entry.
   */
  private renderCatalog(entries: CatalogEntry[]): void {
    const catalog = this.querySelector('tether-catalog') as HTMLElement & {
      items: CatalogEntry[];
      selectedIds: bigint[];
    };
    catalog.items = entries;
    this.querySelector('#catalog-count')!.textContent = String(entries.length);

    catalog.addEventListener('selection-change', () => {
      const selected = catalog.selectedIds;
      this.active = entries.find((entry) => entry.id === selected[0]);
      if (this.active) this.activeKind = this.active.kind;
      this.querySelector('#selection-title')!.textContent =
        this.active?.name ?? 'Select a signal to inspect';
    });
  }

  // ---- Read / Stream actions -------------------------------------------

  /**
   * Read the current value of the active entry and display it in a
   * `<tether-value-card>`.
   */
  private async readActive(): Promise<void> {
    if (!this.active) return;
    try {
      const card = document.createElement('tether-value-card') as HTMLElement & {
        model: { entry: CatalogEntry; client: TetherIOClient };
        read: () => Promise<void>;
      };
      card.model = { entry: this.active, client: this.client };
      this.querySelector('#value-grid')!.replaceChildren(card);
      await card.read();
    } catch {
      this.setStatus('Read failed', false);
    }
  }

  /**
   * Toggle the stream on or off for the currently selected signal(s).
   *
   * Only works when the active entry is a signal (parameters cannot be
   * streamed).  Uses the checked entries from the catalog as the stream
   * configuration.
   */
  private async toggleStream(): Promise<void> {
    if (!this.active || this.activeKind !== 'signal') return;
    const button = this.querySelector<HTMLButtonElement>('#stream')!;

    if (button.dataset.running) {
      // Stop the active stream
      await this.client.stopStream();
      delete button.dataset.running;
      button.textContent = 'Start stream';
    } else {
      // Start a new stream over all checked entries
      const selectedIds = (
        this.querySelector('tether-catalog') as HTMLElement & { selectedIds: bigint[] }
      ).selectedIds;
      await this.client.configureStream(selectedIds, 10, 1);
      await this.client.startStream();
      button.dataset.running = '1';
      button.textContent = 'Stop stream';
    }
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
}

customElements.define('tether-app', TetherApp);
