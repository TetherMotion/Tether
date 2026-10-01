/** Machine-oriented shell for the browser Tether IO client. */
import './components';
import { TetherIOClient } from './client';
import { discoverEventService, EventServiceAvailability, MachineEvent, readEventCursor, readEventPage } from './domain/event-history';
import { EventStore } from './stores/event-store';
import {
  MachineDescriptorView,
  MachineProfileAvailability,
  MachineSnapshotView,
  discoverMachineProfile,
  readDriveSnapshot,
  readMachineDescriptor,
  readMachineSnapshot,
} from './domain/machine-profile';
import { CatalogEntry, FunctionEntry, StreamRow } from './protocol';
import { NavigationItem, machineShellTemplate } from './views/machine-shell';
import { DriveSnapshotRecord, openDriveDetail, renderDriveViews } from './views/drives';
import './style.css';

type ScopeElement = HTMLElement & {
  setChannels(channels: { name: string; color: [number, number, number] }[]): void;
  push(timestampUs: bigint, values: number[]): void;
  clear(): void;
  togglePause(): boolean;
  resetView(): void;
};

type ViewId = 'overview' | 'drives' | 'motion' | 'trends' | 'alarms' | 'diagnostics' | 'commissioning' | 'recipes' | 'explore' | 'settings';
type CatalogTab = 'all' | 'signals' | 'params' | 'functions';

const views: NavigationItem[] = [
  { id: 'overview', label: 'Overview' }, { id: 'drives', label: 'Drives' },
  { id: 'motion', label: 'Motion' }, { id: 'trends', label: 'Trends' },
  { id: 'alarms', label: 'Alarms & events' }, { id: 'diagnostics', label: 'Diagnostics' },
  { id: 'commissioning', label: 'Commissioning' }, { id: 'recipes', label: 'Recipes' },
  { id: 'explore', label: 'Explore' }, { id: 'settings', label: 'Settings' },
];
const colors: [number, number, number][] = [
  [0, 0.45, 0.75], [0.85, 0.33, 0.1], [0, 0.62, 0.45], [0.8, 0.1, 0.2],
  [0.58, 0.4, 0.74], [0.91, 0.59, 0.09], [0.75, 0.31, 0.5], [0.4, 0.4, 0.4],
];
class TetherApp extends HTMLElement {
  private readonly client = new TetherIOClient();
  private params: CatalogEntry[] = [];
  private signals: CatalogEntry[] = [];
  private functions: FunctionEntry[] = [];
  private eventService: EventServiceAvailability = { available: false };
  private readonly eventStore = new EventStore(200);
  private profile: MachineProfileAvailability = {
    profile: 'unavailable', descriptor: false, machineSnapshot: false, driveSnapshots: [],
    controlsAvailable: false, explanation: 'Connect to a V6 server to discover available typed machine services.',
  };
  private readonly driveData = new Map<bigint, DriveSnapshotRecord>();
  private machineDescriptor?: MachineDescriptorView;
  private machineSnapshot?: MachineSnapshotView;
  private machineSnapshotReceivedAt?: number;
  private streamLayout: { id: bigint }[] = [];
  private streamActive = false;
  private polling = false;
  private pollingEvents = false;
  private drivePollTimer?: number;
  private activeView: ViewId = 'overview';

  connectedCallback(): void {
    const savedTheme = localStorage.getItem('tether-theme');
    document.documentElement.dataset.theme = savedTheme === 'dark' ? 'dark' : 'light';
    this.renderShell();
    this.bindEvents();
    void this.connect();
  }

  disconnectedCallback(): void {
    if (this.drivePollTimer !== undefined) window.clearInterval(this.drivePollTimer);
    this.client.disconnect();
  }

  private renderShell(): void {
    this.innerHTML = machineShellTemplate(views);
    const url = this.querySelector<HTMLInputElement>('#url');
    if (url) {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      url.value = `${protocol}//${window.location.host}/tether-io`;
    }
    this.renderPage();
  }

  private bindEvents(): void {
    this.querySelector<HTMLButtonElement>('#connect')?.addEventListener('click', () => void this.connect());
    this.querySelector<HTMLButtonElement>('#theme-toggle')?.addEventListener('click', () => this.toggleTheme());
    this.querySelectorAll<HTMLButtonElement>('[data-view]').forEach((button) => button.addEventListener('click', () => {
      const view = button.dataset.view as ViewId | undefined;
      if (view) this.openView(view);
    }));
    this.querySelectorAll<HTMLButtonElement>('[data-open-view]').forEach((button) => button.addEventListener('click', () => this.openView('drives')));
    this.querySelectorAll<HTMLButtonElement>('.tab').forEach((button) => button.addEventListener('click', () => this.switchCatalogTab(button.dataset.tab as CatalogTab)));
    this.querySelector('tether-catalog')?.addEventListener('selection-change', () => void this.onSelectionChange());
    this.querySelector<HTMLButtonElement>('#pause-btn')?.addEventListener('click', () => this.toggleScopePause());
    this.querySelector<HTMLButtonElement>('#reset-zoom-btn')?.addEventListener('click', () => this.querySelector<ScopeElement>('#scope')?.resetView());
    this.client.addEventListener('connected', () => {
      this.setStatus('Connected · V6 schemas verified', true);
      this.querySelector('#role-label')!.textContent = 'Role: unverified';
      this.querySelector('#freshness-label')!.textContent = 'Data: waiting for snapshots';
    });
    this.client.addEventListener('disconnected', () => {
      this.setStatus('Disconnected', false);
      this.querySelector('#freshness-label')!.textContent = 'Data: stale / connection lost';
      if (this.drivePollTimer !== undefined) window.clearInterval(this.drivePollTimer);
      this.drivePollTimer = undefined;
      this.renderDriveViews();
    });
    this.client.addEventListener('catalog-changed', () => {
      this.profile = discoverMachineProfile(undefined, []);
      this.eventService = { available: false };
      this.eventStore.reset();
      this.driveData.clear();
      this.machineDescriptor = undefined;
      this.machineSnapshot = undefined;
      this.machineSnapshotReceivedAt = undefined;
      this.setStatus('Schema changed · reconnect required', false);
      this.querySelector('#freshness-label')!.textContent = 'Data: invalidated';
      this.renderMachineSummary();
      this.showToast('The schema epoch changed. Data was invalidated; reconnect to renegotiate before reading again.', 'error');
    });
    this.client.addEventListener('error-message', (event: Event) => {
      const detail = (event as CustomEvent<{ message?: string }>).detail;
      this.showToast(detail.message ?? 'Protocol error', 'error');
    });
    this.client.addEventListener('stream', (event: Event) => this.onStream((event as CustomEvent<StreamRow>).detail));
  }

  private async connect(): Promise<void> {
    const url = this.querySelector<HTMLInputElement>('#url')?.value ?? '';
    this.setStatus('Connecting…', false);
    try {
      await this.client.connect(url);
      await this.loadCatalogs();
      await this.refreshProfile();
      this.drivePollTimer = window.setInterval(() => {
        void this.pollDrives();
        void this.pollEvents();
      }, 1000);
    } catch (error) {
      this.setStatus(error instanceof Error ? error.message : 'Connection failed', false);
      this.showToast(error instanceof Error ? error.message : 'Connection failed', 'error');
    }
  }

  private async loadCatalogs(): Promise<void> {
    [this.params, this.signals, this.functions] = await Promise.all([
      this.client.list('params'), this.client.list('signals'), this.client.listFunctions(),
    ]);
    await this.loadEntryMetadata([...this.params, ...this.signals]);
    this.renderCatalog([...this.params, ...this.signals]);
    const functionList = this.querySelector<HTMLElement & { items: FunctionEntry[] }>('#functions');
    if (functionList) functionList.items = this.functions;
  }

  private async loadEntryMetadata(entries: CatalogEntry[]): Promise<void> {
    const batchSize = 24;
    const batches: CatalogEntry[][] = [];
    for (let offset = 0; offset < entries.length; offset += batchSize) {
      batches.push(entries.slice(offset, offset + batchSize));
    }
    await batches.reduce<Promise<void>>(
      (previous, batch) => previous.then(async () => {
        await Promise.allSettled(batch.map(async (entry) => {
          entry.metadata = await this.client.getMetadata(entry.id);
        }));
      }),
      Promise.resolve(),
    );
  }

  private async refreshProfile(): Promise<void> {
    this.profile = discoverMachineProfile(this.client.schemaCatalog, this.signals);
    this.eventService = discoverEventService(this.client.schemaCatalog, this.signals, this.functions);
    this.eventStore.reset();
    this.machineDescriptor = undefined;
    this.machineSnapshot = undefined;
    if (this.profile.descriptorEntry) {
      try {
        this.machineDescriptor = await readMachineDescriptor(this.client, this.profile.descriptorEntry);
      } catch (error) {
        this.showToast(error instanceof Error ? `Machine descriptor rejected: ${error.message}` : 'Machine descriptor rejected', 'error');
      }
    }
    this.renderMachineSummary();
    await this.pollDrives();
    await this.pollEvents();
  }

  private async pollEvents(): Promise<void> {
    const catalog = this.client.schemaCatalog;
    const service = this.eventService;
    if (this.client.state !== 'connected' || !catalog || !service.available ||
        !service.cursorEntry || this.pollingEvents) return;
    this.pollingEvents = true;
    try {
      const latest = await readEventCursor(this.client, catalog, service.cursorEntry);
      if (latest !== this.eventStore.cursor || this.eventStore.cursor > latest) {
        const page = await readEventPage(this.client, catalog, service, this.eventStore.cursor, 50);
        this.eventStore.apply(page);
        this.renderEventTimeline();
      }
      const label = this.querySelector<HTMLElement>('#alarm-label');
      if (label) label.textContent = `Events: ${this.eventStore.events.length} retained${this.eventStore.gapDetected ? ' · gap detected' : ''}`;
    } catch (error) {
      const label = this.querySelector<HTMLElement>('#alarm-label');
      if (label) label.textContent = 'Events: service read failed';
      console.warn('[TetherIO] event history poll failed', error);
    } finally {
      this.pollingEvents = false;
    }
  }

  private renderEventTimeline(): void {
    const timeline = this.querySelector<HTMLElement & { model: { events: readonly MachineEvent[]; gapDetected: boolean; available: boolean } }>('#event-timeline');
    if (timeline) timeline.model = {
      events: this.eventStore.events,
      gapDetected: this.eventStore.gapDetected,
      available: this.eventService.available,
    };
    const count = this.querySelector<HTMLElement>('#event-count-summary');
    if (count) count.textContent = this.eventService.available ? String(this.eventStore.events.length) : 'Unavailable';
    const copy = this.querySelector<HTMLElement>('#event-summary-copy');
    if (copy) {
      if (!this.eventService.available) copy.textContent = 'Typed event history service not advertised';
      else {
        const historyState = this.eventStore.gapDetected ? 'history gap detected' : 'read-only history';
        copy.textContent = `${this.eventStore.events.length} retained · ${historyState}`;
      }
    }
  }

  private async pollDrives(): Promise<void> {
    const entries = this.profile.driveSnapshots;
    if (this.client.state !== 'connected' || this.polling ||
        (!entries.length && !this.profile.machineSnapshotEntry)) return;
    this.polling = true;
    const tasks: Promise<{ kind: 'machine'; snapshot: MachineSnapshotView } | { kind: 'drive'; id: bigint; snapshot: DriveSnapshotRecord['snapshot'] }>[] = [];
    if (this.profile.machineSnapshotEntry) {
      tasks.push(readMachineSnapshot(this.client, this.profile.machineSnapshotEntry)
        .then((snapshot) => ({ kind: 'machine' as const, snapshot })));
    }
    for (const entry of entries) {
      tasks.push(readDriveSnapshot(this.client, entry)
        .then((snapshot) => ({ kind: 'drive' as const, id: entry.id, snapshot })));
    }
    const results = await Promise.allSettled(tasks);
    let machineReceived = false;
    for (const result of results) {
      if (result.status !== 'fulfilled') continue;
      if (result.value.kind === 'machine') {
        this.machineSnapshot = result.value.snapshot;
        this.machineSnapshotReceivedAt = Date.now();
        machineReceived = true;
      } else {
        this.driveData.set(result.value.id, { snapshot: result.value.snapshot, receivedAt: Date.now() });
      }
    }
    const failedCount = results.filter((result) => result.status === 'rejected').length;
    let freshness = 'Data: live drive snapshots';
    if (machineReceived) freshness = 'Data: live · coherent machine snapshot';
    if (failedCount > 0) {
      const noun = failedCount === 1 ? 'read' : 'reads';
      freshness = `Data: partial · ${failedCount} ${noun} failed`;
    }
    this.querySelector('#freshness-label')!.textContent = freshness;
    this.renderMachineSummary();
    this.renderDriveViews();
    this.polling = false;
  }

  private renderMachineSummary(): void {
    this.renderProfileSummary();
    this.renderTelemetrySummary();
    this.renderEventTimeline();
    this.renderDriveViews();
  }

  private renderProfileSummary(): void {
    const badge = this.querySelector<HTMLElement>('#profile-badge');
    if (badge) badge.textContent = this.profile.profile === 'machine.cia402.v1' ? 'CiA 402 profile' : 'Profile incomplete';
    const explanation = this.querySelector<HTMLElement>('#profile-explanation');
    if (explanation) explanation.textContent = this.profile.explanation;
    const count = this.querySelector<HTMLElement>('#drive-count');
    if (count) count.textContent = String(this.machineDescriptor?.axes.length ?? this.profile.driveSnapshots.length);
    const summary = this.querySelector<HTMLElement>('#drive-summary');
    if (summary) summary.textContent = this.profile.driveSnapshots.length > 0
      ? `${this.driveData.size} current · ${this.profile.driveSnapshots.length} discovered`
      : 'No schema-backed drive snapshots';
  }

  private renderTelemetrySummary(): void {
    const message = this.querySelector<HTMLElement>('#overview-message');
    const machineName = this.querySelector<HTMLElement>('#machine-display-name');
    if (machineName && this.machineDescriptor) machineName.textContent = this.machineDescriptor.displayName;
    const health = this.querySelector<HTMLElement>('#machine-health');
    const healthCopy = this.querySelector<HTMLElement>('#machine-health-copy');
    this.renderMasterWidget();
    if (!this.machineSnapshot) {
      this.renderUnavailableTelemetry(health, healthCopy, message);
      return;
    }
    const state = this.machineSnapshot;
    const snapshotAgeMs = this.machineSnapshotReceivedAt === undefined ? Infinity : Date.now() - this.machineSnapshotReceivedAt;
    const degraded = !state.linkUp || state.actualWkc !== state.expectedWkc || state.staleCount > 0 ||
      state.faultCount > 0 || snapshotAgeMs > 3000;
    if (health) health.textContent = degraded ? 'Degraded telemetry' : 'Telemetry nominal';
    if (healthCopy) healthCopy.textContent = this.formatMachineSnapshot(state, snapshotAgeMs);
    if (message) message.textContent = `${this.machineDescriptor?.displayName ?? 'Machine'} · generation ${state.stateGeneration.toString()}. Telemetry summary only; it does not assert safety or motion readiness.`;
  }

  private renderMasterWidget(): void {
    const widget = this.querySelector<HTMLElement & { model: { snapshot: MachineSnapshotView; ageMs: number } | undefined }>('#machine-state-widget');
    if (!widget) return;
    widget.model = this.machineSnapshot
      ? { snapshot: this.machineSnapshot, ageMs: Math.max(0, Date.now() - (this.machineSnapshotReceivedAt ?? Date.now())) }
      : undefined;
  }

  private renderUnavailableTelemetry(
    health: HTMLElement | null,
    healthCopy: HTMLElement | null,
    message: HTMLElement | null,
  ): void {
    if (health) health.textContent = 'Unknown';
    if (healthCopy) healthCopy.textContent = 'No coherent machine snapshot received.';
    if (message) message.textContent = this.profile.explanation;
  }

  private formatMachineSnapshot(state: MachineSnapshotView, ageMs: number): string {
    const source = state.simulated ? 'Simulated' : 'Live';
    const freshness = Number.isFinite(ageMs) ? `${Math.max(0, Math.round(ageMs))} ms old` : 'stale';
    return `${source} · ${state.enabledCount}/${state.axisCount} enabled · ${state.faultCount} faults · ${state.warningCount} warnings · ${state.staleCount} stale · WKC ${state.actualWkc}/${state.expectedWkc} · ${freshness}`;
  }

  private renderDriveViews(): void {
    const axesByStableId = new Map((this.machineDescriptor?.axes ?? []).map((axis) => [axis.stableId, axis]));
    renderDriveViews(this, this.profile.driveSnapshots, this.driveData, (entry, snapshot, ageMs) => {
      this.openView('drives');
      openDriveDetail(this, entry, snapshot, ageMs, () => undefined,
        axesByStableId.get(entry.metadata?.['resource.stable_id'] ?? ''));
    }, axesByStableId);
  }

  private openView(view: ViewId): void {
    this.activeView = view;
    this.querySelectorAll<HTMLElement>('.app-nav-item').forEach((button) => button.classList.toggle('active', button.dataset.view === view));
    this.renderPage();
  }

  private renderPage(): void {
    const nav = views.find((candidate) => candidate.id === this.activeView)!;
    this.querySelector('#view-title')!.textContent = nav.label;
    this.querySelector('#view-kicker')!.textContent = this.activeView === 'overview' ? 'Machine overview' : 'Machine console';
    const known: Partial<Record<ViewId, string>> = {
      overview: 'overview-page', drives: 'drives-page', motion: 'motion-page', trends: 'trends-page',
      alarms: 'alarms-page', explore: 'explore-page',
    };
    const pageId = known[this.activeView];
    this.querySelectorAll<HTMLElement>('.machine-page').forEach((page) => { page.hidden = page.id !== pageId; });
    if (!pageId) {
      const placeholder = this.querySelector<HTMLElement>('#placeholder-page')!;
      placeholder.hidden = false;
      this.querySelector('#placeholder-title')!.textContent = `${nav.label} service not advertised`;
      this.querySelector('#placeholder-copy')!.textContent = this.activeView === 'settings'
        ? 'Identity, roles, connection profiles, retention, and export policy must be provided by the authenticated deployment.'
        : `${nav.label} requires typed server-side event, diagnostic, capture, configuration, or recipe services. Generic registry names are not treated as an authoritative contract.`;
    } else {
      const placeholder = this.querySelector<HTMLElement>('#placeholder-page');
      if (placeholder) placeholder.hidden = true;
    }
    if (pageId === 'explore-page') this.renderCatalog([...this.params, ...this.signals]);
  }

  private renderCatalog(entries: CatalogEntry[]): void {
    const catalog = this.querySelector<HTMLElement & { items: CatalogEntry[] }>('#catalog');
    if (catalog) catalog.items = entries;
    const count = this.querySelector<HTMLElement>('#catalog-count');
    if (count) count.textContent = String(entries.length);
  }

  private switchCatalogTab(tab: CatalogTab): void {
    this.querySelectorAll<HTMLButtonElement>('.tab').forEach((button) => button.classList.toggle('active', button.dataset.tab === tab));
    if (tab === 'functions') {
      const list = this.querySelector<HTMLElement & { items: FunctionEntry[] }>('#functions');
      if (list) list.items = this.functions;
      return;
    }
    let entries: CatalogEntry[];
    if (tab === 'all') entries = [...this.params, ...this.signals];
    else if (tab === 'signals') entries = this.signals;
    else entries = this.params;
    this.renderCatalog(entries);
  }

  private async onSelectionChange(): Promise<void> {
    const catalog = this.querySelector<HTMLElement & { selectedIds: bigint[] }>('#catalog');
    const signalIds = new Set(this.signals.map((signal) => signal.id));
    const selected = (catalog?.selectedIds ?? []).filter((id) => signalIds.has(id));
    if (this.streamActive) {
      await this.client.stopStream();
      this.streamActive = false;
    }
    if (!selected.length) {
      this.querySelectorAll<ScopeElement>('tether-webgpu-scope').forEach((scope) => scope.setChannels([]));
      return;
    }
    await this.client.configureStream(selected, 10, 20);
    this.streamLayout = this.client.currentStreamLayout.map(({ id }) => ({ id }));
    this.querySelectorAll<ScopeElement>('tether-webgpu-scope').forEach((scope) => {
      scope.setChannels(this.streamLayout.map(({ id }, index) => ({
        name: this.signals.find((signal) => signal.id === id)?.name ?? `ch${index}`,
        color: colors[index % colors.length] ?? [0.5, 0.5, 0.5],
      })));
      scope.clear();
    });
    await this.client.startStream();
    this.streamActive = true;
  }

  private onStream(row: StreamRow): void {
    const values = row.values.map((bytes) => {
      if (bytes.length === 8) return new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength).getFloat64(0, true);
      if (bytes.length === 4) return new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength).getFloat32(0, true);
      return bytes.length ? bytes[0]! : 0;
    });
    this.querySelectorAll<ScopeElement>('tether-webgpu-scope').forEach((scope) => scope.push(row.timestampUs, values));
  }

  private toggleScopePause(): void {
    const scope = this.querySelector<ScopeElement>('#scope');
    if (!scope) return;
    const paused = scope.togglePause();
    this.querySelector('#pause-btn')!.textContent = paused ? 'Resume' : 'Pause';
    const live = this.querySelector<HTMLElement>('#live-dot')!;
    live.textContent = paused ? 'Ⅱ PAUSED' : '● LIVE';
    live.classList.toggle('paused', paused);
    this.querySelector<HTMLButtonElement>('#reset-zoom-btn')!.hidden = !paused;
  }

  private toggleTheme(): void {
    const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    localStorage.setItem('tether-theme', next);
  }

  private setStatus(text: string, online: boolean): void {
    const status = this.querySelector<HTMLElement>('#status');
    if (status) status.textContent = text;
    status?.classList.toggle('online', online);
    const age = this.querySelector<HTMLElement>('#freshness-label');
    if (!online && age && !age.textContent?.includes('invalidated')) age.textContent = 'Data: stale / offline';
  }

  private showToast(message: string, kind: 'error' | 'info' = 'info'): void {
    const host = this.querySelector<HTMLElement>('#toast-host');
    if (!host) return;
    const toast = document.createElement('div');
    toast.className = `toast toast-${kind}`;
    toast.textContent = message;
    host.append(toast);
    requestAnimationFrame(() => toast.classList.add('toast-visible'));
    window.setTimeout(() => { toast.classList.remove('toast-visible'); window.setTimeout(() => toast.remove(), 300); }, kind === 'error' ? 6000 : 3000);
  }
}

customElements.define('tether-app', TetherApp);
