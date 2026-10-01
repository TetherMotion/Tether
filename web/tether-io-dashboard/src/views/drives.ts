import { DriveSnapshotView, MachineAxisDescriptor, alStateLabel, ds402StateLabel } from '../domain/machine-profile';
import { CatalogEntry } from '../protocol';

export interface DriveSnapshotRecord {
  snapshot: DriveSnapshotView;
  receivedAt: number;
}

type DriveSelection = (entry: CatalogEntry, snapshot: DriveSnapshotView, ageMs: number) => void;
type StateWidget = HTMLElement & { model?: { name: string; snapshot: DriveSnapshotView; ageMs: number } };
interface DriveRowModel {
  entry: CatalogEntry;
  name: string;
  axis?: MachineAxisDescriptor;
  snapshot: DriveSnapshotView;
  ageMs: number;
  fault: boolean;
  warning: boolean;
  stale: boolean;
}

export function renderDriveViews(
  root: ParentNode,
  entries: CatalogEntry[],
  driveData: Map<bigint, DriveSnapshotRecord>,
  onSelect: DriveSelection,
  axesByStableId: ReadonlyMap<string, MachineAxisDescriptor> = new Map(),
): void {
  const body = root.querySelector<HTMLTableSectionElement>('#drive-table-body');
  const summary = root.querySelector<HTMLElement>('#overview-drive-list');
  const widgets = root.querySelector<HTMLElement>('#drive-state-widgets');
  if (!body || !summary) return;
  body.replaceChildren();
  summary.replaceChildren();
  widgets?.replaceChildren();
  const axisFor = (entry: CatalogEntry) => axesByStableId.get(entry.metadata?.['resource.stable_id'] ?? '');
  const displayName = (entry: CatalogEntry) => axisFor(entry)?.name ?? entry.name;
  const sortedEntries = [...entries].sort((a, b) => displayName(a).localeCompare(displayName(b)));
  let enabledCount = 0;
  let faultCount = 0;
  for (const entry of sortedEntries) {
    const axis = axisFor(entry);
    const name = displayName(entry);
    const data = driveData.get(entry.id);
    if (widgets) appendStateWidgets(widgets, name, data);
    if (!data) {
      appendWaitingRow(body, name);
      continue;
    }
    const { snapshot, receivedAt } = data;
    const ageMs = Math.max(0, Date.now() - receivedAt);
    const fault = snapshot.faultCode !== 0 || snapshot.ds402State === 7;
    const warning = (snapshot.statusWord & (1 << 7)) !== 0;
    const stale = (snapshot.qualityFlags & 1) !== 0;
    if (snapshot.ds402State === 4) enabledCount += 1;
    if (fault) faultCount += 1;
    const model: DriveRowModel = { entry, name, axis, snapshot, ageMs, fault, warning, stale };
    appendDriveRow(body, model, onSelect);
    appendOverviewDrive(summary, model, onSelect);
  }
  const driveSummary = root.querySelector<HTMLElement>('#drive-summary');
  if (driveSummary && entries.length) {
    const liveCount = [...driveData.keys()].filter((id) => entries.some((entry) => entry.id === id)).length;
    driveSummary.textContent = `${liveCount} live · ${enabledCount} enabled · ${faultCount} faulted`;
  }
  if (widgets && sortedEntries.length === 0) {
    const empty = document.createElement('p');
    empty.className = 'widget-empty';
    empty.textContent = 'No schema-backed drive snapshots are available for this machine.';
    widgets.append(empty);
  }
}

function appendStateWidgets(container: HTMLElement, name: string, data?: DriveSnapshotRecord): void {
  const group = document.createElement('div');
  group.className = 'drive-widget-group';
  const ethercat = document.createElement('tether-ethercat-state') as StateWidget;
  const cia402 = document.createElement('tether-cia402-state') as StateWidget;
  if (data) {
    const model = { name, snapshot: data.snapshot, ageMs: Math.max(0, Date.now() - data.receivedAt) };
    ethercat.model = model;
    cia402.model = model;
  }
  group.append(ethercat, cia402);
  container.append(group);
}

function appendWaitingRow(body: HTMLTableSectionElement, name: string): void {
  const row = body.insertRow();
  const cell = row.insertCell();
  cell.colSpan = 12;
  cell.textContent = `${name} · waiting for a coherent snapshot`;
}

function appendDriveRow(body: HTMLTableSectionElement, model: DriveRowModel, onSelect: DriveSelection): void {
  const { entry, name, axis, snapshot, ageMs, fault, warning, stale } = model;
  const row = body.insertRow();
  row.className = fault ? 'drive-row fault' : 'drive-row';
  let faultSummary = 'None reported';
  if (warning) faultSummary = 'Warning';
  if (fault) faultSummary = `Fault 0x${snapshot.faultCode.toString(16).padStart(4, '0')}`;
  const cells = [
    name, alStateLabel(snapshot.alState), ds402StateLabel(snapshot.ds402State), modeLabel(snapshot.displayMode),
    snapshot.ds402State === 4 ? 'Enabled' : 'Disabled', faultSummary,
    formatPosition(snapshot.targetPosition, axis), formatPosition(snapshot.actualPosition, axis), formatPosition(snapshot.followingError, axis),
    'Not available', `${ageMs} ms${stale ? ' · stale' : ''}`, 'Not advertised',
  ];
  cells.forEach((text) => { const cell = row.insertCell(); cell.textContent = text; });
  row.tabIndex = 0;
  row.setAttribute('role', 'button');
  row.setAttribute('aria-label', `Inspect ${name}, ${ds402StateLabel(snapshot.ds402State)}`);
  row.addEventListener('click', () => onSelect(entry, snapshot, ageMs));
  row.addEventListener('keydown', (event) => {
    if (event.key !== 'Enter' && event.key !== ' ') return;
    event.preventDefault();
    onSelect(entry, snapshot, ageMs);
  });
}

function appendOverviewDrive(container: HTMLElement, model: DriveRowModel, onSelect: DriveSelection): void {
  const { entry, name, axis, snapshot, ageMs, fault, warning } = model;
  const card = document.createElement('button');
  card.className = 'overview-drive-row';
  let condition = '';
  if (warning) condition = ' · WARNING';
  if (fault) condition = ' · FAULT';
  card.textContent = `${name} · ${ds402StateLabel(snapshot.ds402State)} · actual ${formatPosition(snapshot.actualPosition, axis)}${condition}`;
  card.addEventListener('click', () => onSelect(entry, snapshot, ageMs));
  container.append(card);
}

export function openDriveDetail(
  root: ParentNode,
  entry: CatalogEntry,
  snapshot: DriveSnapshotView,
  ageMs: number,
  onClose: () => void,
  axis?: MachineAxisDescriptor,
): void {
  const drawer = root.querySelector<HTMLElement>('#drive-drawer');
  if (!drawer) return;
  const bits: [string, number][] = [
    ['Ready to switch on', 0], ['Switched on', 1], ['Operation enabled', 2], ['Fault', 3],
    ['Voltage enabled', 4], ['Quick stop', 5], ['Switch-on disabled', 6], ['Warning', 7],
    ['Remote', 9], ['Target reached', 10], ['Internal limit active', 11],
  ];
  drawer.hidden = false;
  drawer.replaceChildren();

  const header = document.createElement('div');
  header.className = 'drawer-head';
  const titleGroup = document.createElement('div');
  const eyebrow = document.createElement('span');
  eyebrow.className = 'eyebrow';
  eyebrow.textContent = `Drive detail · ${axis?.name ?? entry.name}`;
  const title = document.createElement('h2');
  title.textContent = ds402StateLabel(snapshot.ds402State);
  titleGroup.append(eyebrow, title);
  const closeButton = document.createElement('button');
  closeButton.id = 'close-drawer';
  closeButton.className = 'secondary';
  closeButton.textContent = 'Close';
  header.append(titleGroup, closeButton);
  drawer.append(header);

  const source = document.createElement('p');
  source.textContent = `Snapshot generation ${snapshot.stateGeneration.toString()} · received ${ageMs} ms ago · source timestamp ${snapshot.timestampUs.toString()} μs`;
  drawer.append(source);

  const metrics = document.createElement('div');
  metrics.className = 'detail-grid';
  appendMetric(metrics, 'AL state / code', `${alStateLabel(snapshot.alState)} / 0x${snapshot.alStatusCode.toString(16).padStart(4, '0')}`);
  appendMetric(metrics, 'Mode', `${modeLabel(snapshot.displayMode)} (${snapshot.displayMode})`);
  appendMetric(metrics, 'Target / actual', `${formatPosition(snapshot.targetPosition, axis)} / ${formatPosition(snapshot.actualPosition, axis)}`);
  appendMetric(metrics, 'Following error', formatPosition(snapshot.followingError, axis));
  appendMetric(metrics, 'Controlword', `0x${snapshot.controlWord.toString(16).padStart(4, '0')}`);
  appendMetric(metrics, 'Fault code', `0x${snapshot.faultCode.toString(16).padStart(4, '0')}`);
  drawer.append(metrics);

  const bitsHeading = document.createElement('h3');
  bitsHeading.textContent = 'Statusword bits';
  drawer.append(bitsHeading);
  const table = document.createElement('table');
  table.className = 'bit-table';
  const tableHead = table.createTHead();
  const headerRow = tableHead.insertRow();
  for (const label of ['Bit', 'Meaning', 'Value']) {
    const cell = document.createElement('th');
    cell.textContent = label;
    headerRow.append(cell);
  }
  const tableBody = table.createTBody();
  for (const [label, bit] of bits) {
    const row = tableBody.insertRow();
    for (const value of [String(bit), label, snapshot.statusWord & (1 << bit) ? 'Set' : 'Clear']) {
      const cell = row.insertCell();
      cell.textContent = value;
    }
  }
  drawer.append(table);

  const note = document.createElement('p');
  note.className = 'muted';
  note.textContent = 'Transition history, alarm cause, PDO mapping, recovery, and capture services are not advertised. This view is observational; no drive command is available.';
  drawer.append(note);

  closeButton.addEventListener('click', () => {
    drawer.hidden = true;
    onClose();
  });
  drawer.focus();
}

function appendMetric(container: HTMLElement, label: string, value: string): void {
  const metric = document.createElement('div');
  const title = document.createElement('strong');
  title.textContent = label;
  const detail = document.createElement('span');
  detail.textContent = value;
  metric.append(title, detail);
  container.append(metric);
}

function modeLabel(mode: number): string {
  return ({ 1: 'PP', 3: 'PV', 4: 'PT', 6: 'Homing', 7: 'Interpolated', 8: 'CSP', 9: 'CSV', 10: 'CST' } as Record<number, string>)[mode] ?? `Mode ${mode}`;
}

function formatPosition(value: number, axis?: MachineAxisDescriptor): string {
  if (!axis) return String(value);
  const converted = value * axis.positionScale;
  return `${Number.isFinite(converted) ? converted.toFixed(3) : '—'} ${axis.positionUnit}`;
}

