export interface NavigationItem {
  id: string;
  label: string;
}

export function machineShellTemplate(views: NavigationItem[]): string {
  return `
      <main class="shell machine-shell">
        <header class="topbar">
          <div class="brand"><span class="brand-mark">T</span><div><strong id="machine-display-name">Tether Machine Console</strong><small>EtherCAT observability · browser controls are not safety functions</small></div></div>
          <div class="connection"><input id="url" aria-label="WebSocket URL"><button id="connect">Connect</button><button id="theme-toggle" class="theme-toggle" aria-label="Toggle theme">◐</button><span id="status" class="status">Offline</span></div>
        </header>
        <nav class="app-nav" aria-label="Machine console navigation">${views.map((view) => `<button class="app-nav-item${view.id === 'overview' ? ' active' : ''}" data-view="${view.id}">${view.label}</button>`).join('')}</nav>
        <div class="safety-notice" role="note"><strong>Read-only session</strong><span>This interface is not an E-stop, STO, safety PLC, or safety-rated control. Use the machine's independent hardware safety system.</span><div class="global-state"><span id="role-label">Role: unverified</span><span id="authority-label">Control owner: none</span><span id="freshness-label">Data: waiting</span><span id="alarm-label">Alarms: unavailable</span></div></div>
        <section id="machine-view" class="machine-view" aria-live="polite">
          <div class="view-heading"><div><p class="eyebrow" id="view-kicker">Machine overview</p><h1 id="view-title">Overview</h1></div><span id="profile-badge" class="badge">Profile unavailable</span></div>
          <p id="profile-explanation" class="profile-explanation"></p>
          <section id="overview-page" class="machine-page">
            <div class="overview-grid">
              <article class="machine-card"><small>Telemetry health</small><strong id="machine-health">Unknown</strong><span id="machine-health-copy">No coherent machine snapshot received.</span></article>
              <article class="machine-card"><small>Drive snapshots</small><strong id="drive-count">0</strong><span id="drive-summary">Waiting for profile data</span></article>
              <article class="machine-card"><small>Event journal</small><strong id="event-count-summary">Unavailable</strong><span id="event-summary-copy">Typed event history service not advertised</span></article>
              <article class="machine-card"><small>Motion authority</small><strong>None</strong><span>Control lease service not advertised</span></article>
            </div>
            <section class="widget-section" aria-labelledby="state-widgets-title">
              <div class="panel-title"><div><span class="eyebrow">Live typed telemetry</span><h2 id="state-widgets-title">Machine state widgets</h2></div><span class="widget-source">MachineSnapshotV1 + DriveSnapshotV1 · read-only</span></div>
              <tether-machine-state id="machine-state-widget"></tether-machine-state>
              <div id="drive-state-widgets" class="drive-state-widgets"><p class="widget-empty">Connect to a machine profile to load EtherCAT and CiA 402 widgets.</p></div>
            </section>
            <article class="machine-card overview-callout"><h2>System status</h2><p id="overview-message">Connect to a schema-negotiated machine application to load coherent fleet state.</p></article>
            <article class="machine-card"><div class="panel-title"><div><span class="eyebrow">Fleet health</span><h2>Drive summary</h2></div><button class="secondary" data-open-view="drives">Open drives</button></div><div id="overview-drive-list" class="overview-drive-list"></div></article>
          </section>
          <section id="drives-page" class="machine-page" hidden><div class="table-wrap"><table class="drive-table"><thead><tr><th>Axis / resource</th><th>EtherCAT</th><th>CiA 402</th><th>Mode</th><th>Enabled</th><th>Fault / warning</th><th>Target</th><th>Actual</th><th>Following error</th><th>Owner</th><th>Data age</th><th>Recovery</th></tr></thead><tbody id="drive-table-body"></tbody></table></div><aside id="drive-drawer" class="drive-drawer" hidden></aside></section>
          <section id="alarms-page" class="machine-page" hidden><article class="machine-card"><div class="panel-title"><div><span class="eyebrow">Read-only journal</span><h2>Alarms & events</h2></div><span class="widget-source">Server cursor · bounded history</span></div><p class="event-history-note">This is an immutable event history view. It does not acknowledge, clear, or suppress alarms; those lifecycle operations require a separately authorized server service.</p><tether-event-timeline id="event-timeline"></tether-event-timeline></article></section>
          <section id="motion-page" class="machine-page" hidden><article class="machine-card unavailable-card"><span class="state-label">Controls unavailable</span><h2>No server-validated control authority</h2><p>Motion commands stay disabled until the authenticated application provides ownership leases, live interlocks, generation checks, bounded command services, structured blockers, and audited results.</p><p>Browser timing is never used for cyclic CSP, CSV, or CST setpoints. The native cyclic controller and independent safety system remain authoritative.</p></article></section>
          <section id="trends-page" class="machine-page" hidden><article class="machine-card"><span class="eyebrow">Live signals</span><h2>Trend workspace</h2><p>Choose schema-backed signals in Explore. Capture, replay, derived traces, and server-side evidence are not advertised by this connection.</p><tether-webgpu-scope id="trend-scope"></tether-webgpu-scope></article></section>
          <section id="explore-page" class="machine-page" hidden><div class="workspace explore-workspace"><aside class="panel catalog-panel"><div class="panel-title"><div><span class="eyebrow">Expert tool</span><h2>Generic catalog</h2></div><span id="catalog-count" class="badge">0</span></div><nav class="tabs"><button class="tab active" data-tab="all">All</button><button class="tab" data-tab="signals">Signals</button><button class="tab" data-tab="params">Parameters</button><button class="tab" data-tab="functions">Functions</button></nav><tether-catalog id="catalog"></tether-catalog></aside><section class="content"><div class="scope panel"><div class="scope-head"><div><span class="eyebrow">Oscilloscope</span><h2>Selected signal trace</h2></div><div class="scope-actions"><span id="live-dot" class="live-dot">● LIVE</span><button id="pause-btn" class="secondary">Pause</button><button id="reset-zoom-btn" class="secondary" hidden>Reset zoom</button></div></div><tether-webgpu-scope id="scope"></tether-webgpu-scope></div><article class="machine-card"><h2>Generic function catalog</h2><p>Functions are descriptive only here. Motion-affecting actions are not invoked from this expert catalog.</p><tether-function-list id="functions"></tether-function-list></article></section></div></section>
          <section id="placeholder-page" class="machine-page" hidden><article class="machine-card unavailable-card"><span class="state-label">Service unavailable</span><h2 id="placeholder-title">Not advertised</h2><p id="placeholder-copy">No state is inferred from generic registry names. This server does not provide the typed, versioned service required for this view.</p></article></section>
        </section>
        <footer><span>Typed V6 telemetry · server remains authoritative</span><span>Hardware safety functions are independent</span></footer>
        <div id="toast-host" class="toast-host" aria-live="polite"></div>
      </main>`;
}
