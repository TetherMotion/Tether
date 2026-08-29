/**
 * @file webgpu-scope.ts
 * @brief WebGPU-based real-time oscilloscope for streaming Tether IO data.
 *
 * Based on the WebGPU oscilloscope from
 * https://techoverflow.net/2026/08/19/real-time-1-khz-webgpu-oscilloscope-with-ring-buffer-auto-scroll/
 * adapted for the Tether IO dashboard.
 *
 * Architecture:
 *   - Data arrives via {@link WebGPUScope.push} (called from StreamData events).
 *   - Samples are stored in a GPU ring buffer (timestamp + value per channel,
 *     interleaved) and uploaded asynchronously via `queue.writeBuffer`.
 *   - The GPU renders a grid background + one line strip per channel in a
 *     single instanced draw call with MSAA anti-aliasing.
 *   - Auto-scroll: the X viewport is always `[currentTime - windowSec, currentTime]`;
 *     old samples map off-screen left and are clipped by the rasterizer.
 *   - Axis labels and legend are drawn on a 2D canvas overlay.
 *
 * @license CC0-1.0 (original source by Uli Köhler, TechOverflow.net)
 */

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/** A single sample row: one timestamp + one value per channel. */
interface ScopeRow {
  /** Timestamp in seconds (relative or absolute — only deltas matter). */
  t: number;
  /** One numeric value per channel. */
  values: number[];
}

/** Per-channel metadata for rendering. */
interface ChannelInfo {
  /** Display name (e.g. the signal/param name). */
  name: string;
  /** RGB color in [0, 1]. */
  color: [number, number, number];
}

// ---------------------------------------------------------------------------
// Configuration constants
// ---------------------------------------------------------------------------

/** Visible time window in seconds (auto-scroll width). */
const WINDOW_SEC = 5.0;

/** Maximum sample rate the buffer can hold (used to size the ring buffer). */
const MAX_SAMPLE_RATE = 1000; // 1 kHz

/** Number of samples in the visible window. */
const WINDOW_SAMPLES = MAX_SAMPLE_RATE * WINDOW_SEC; // 5000

/** Extra slots beyond the window for chunk alignment headroom. */
const BUFFER_PADDING = 100;

/** Total ring buffer capacity in sample slots. */
const BUFFER_SAMPLES = WINDOW_SAMPLES + BUFFER_PADDING;

/** MSAA sample counts to probe (highest supported is used). */
const MSAA_SAMPLE_COUNTS = [1, 2, 4, 8, 16];

/** Target MSAA sample count (prefers 4× to balance quality/performance). */
const TARGET_MSAA = 4;

/** Plot margins in CSS pixels. */
const MARGIN_LEFT = 64;
const MARGIN_RIGHT = 100; // extra for legend
const MARGIN_TOP = 16;
const MARGIN_BOTTOM = 48;

/** Maximum channels supported (ring buffer is pre-allocated for this). */
const MAX_CHANNELS = 16;

/** Default ggplot-like color palette. */
const DEFAULT_COLORS: [number, number, number][] = [
  [0.0, 0.45, 0.75], // blue
  [0.85, 0.33, 0.1], // orange
  [0.0, 0.62, 0.45], // green
  [0.8, 0.1, 0.2], // red
  [0.58, 0.4, 0.74], // purple
  [0.91, 0.59, 0.09], // brown
  [0.75, 0.31, 0.5], // pink
  [0.4, 0.4, 0.4], // gray
  [0.0, 0.0, 0.0], // black
  [0.2, 0.6, 0.8], // light blue
  [0.55, 0.23, 0.12], // dark brown
  [0.34, 0.71, 0.91], // sky blue
  [0.62, 0.85, 0.34], // lime
  [0.89, 0.47, 0.76], // magenta
  [0.5, 0.5, 0.0], // olive
  [0.0, 0.5, 0.5], // teal
];

// ---------------------------------------------------------------------------
// WGSL shader
// ---------------------------------------------------------------------------

/**
 * WebGPU shader source.  Two pipelines:
 *   1. `vs_line` / `fs_line` — renders N line strips (one per channel) with MSAA.
 *   2. `vs_grid` / `fs_grid` — renders the plot background with anti-aliased grid lines.
 *
 * The ring buffer stores `(timestamp, value)` pairs for all channels,
 * interleaved: sample `i` has channel `ch` at index `(i * MAX_CHANNELS + ch) * 2`.
 *
 * Auto-scroll: the uniform `currentTime` is the timestamp of the most recent
 * sample.  The X viewport is `[currentTime - windowSec, currentTime]`.
 * Samples older than `currentTime - windowSec` map to X < -1 (off-screen).
 */
const SHADER = /* wgsl */ `
  struct RenderUniforms {
    currentTime   : f32,
    windowSec     : f32,
    yMin          : f32,
    yMax          : f32,
    plotX         : f32,
    plotY         : f32,
    plotW         : f32,
    plotH         : f32,
    canvasW       : f32,
    canvasH       : f32,
    xMajorStep    : f32,
    yMajorStep    : f32,
    writeIndex    : u32,
    bufferSamples : u32,
    // Theme colors (packed as vec4s for 16-byte alignment).
    marginColor   : vec4<f32>,  // offset 56
    plotBgColor   : vec4<f32>,  // offset 72
    minorColor    : vec4<f32>,  // offset 88
    majorColor    : vec4<f32>,  // offset 104
  };

  @group(0) @binding(0) var<uniform> ru : RenderUniforms;
  @group(0) @binding(1) var<storage, read> ringBuffer : array<f32>;
  @group(0) @binding(2) var<storage, read> channelColors : array<vec4<f32>>;

  fn worldToClip(t : f32, v : f32) -> vec4<f32> {
    let xMin = ru.currentTime - ru.windowSec;
    let xMax = ru.currentTime;
    let plotPx = (t - xMin) / (xMax - xMin) * ru.plotW;
    let plotPy = (ru.yMax - v) / (ru.yMax - ru.yMin) * ru.plotH;
    let canvasPx = ru.plotX + plotPx;
    let canvasPy = ru.plotY + plotPy;
    return vec4<f32>(
      (canvasPx / ru.canvasW) * 2.0 - 1.0,
      1.0 - (canvasPy / ru.canvasH) * 2.0,
      0.0, 1.0
    );
  }

  // ── Line strip pipeline ─────────────────────────────────────────────
  struct LineVSOut {
    @builtin(position) clipPos : vec4<f32>,
    @location(0)       color   : vec3<f32>,
  };

  @vertex
  fn vs_line(@builtin(vertex_index) vi : u32,
             @builtin(instance_index) ch : u32) -> LineVSOut {
    let sampleIdx = (ru.writeIndex + ru.bufferSamples - ${WINDOW_SAMPLES}u + vi) % ru.bufferSamples;
    let base = (sampleIdx * ${MAX_CHANNELS}u + ch) * 2u;
    let t = ringBuffer[base];
    let v = ringBuffer[base + 1u];

    var out : LineVSOut;
    out.clipPos = worldToClip(t, v);
    out.color = channelColors[ch].rgb;
    return out;
  }

  @fragment
  fn fs_line(in : LineVSOut) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
  }

  // ── Grid pipeline ───────────────────────────────────────────────────
  struct GridVSOut {
    @builtin(position) clipPos : vec4<f32>,
    @location(0)       canvasPx : vec2<f32>,
  };

  @vertex
  fn vs_grid(@builtin(vertex_index) vi : u32) -> GridVSOut {
    var pos = array<vec2<f32>, 6>(
      vec2<f32>(-1.0, -1.0),
      vec2<f32>( 1.0, -1.0),
      vec2<f32>( 1.0,  1.0),
      vec2<f32>(-1.0, -1.0),
      vec2<f32>( 1.0,  1.0),
      vec2<f32>(-1.0,  1.0),
    );
    var out : GridVSOut;
    out.clipPos = vec4<f32>(pos[vi], 0.0, 1.0);
    out.canvasPx = vec2<f32>(
      (pos[vi].x * 0.5 + 0.5) * ru.canvasW,
      (1.0 - pos[vi].y * 0.5 - 0.5) * ru.canvasH,
    );
    return out;
  }

  fn gridLineDistPx(coord : f32, step : f32, pxPerWorld : f32) -> f32 {
    let p = coord / step;
    let frac_p = fract(p);
    // dWorld is in step fractions; multiply by step to get back to world units,
    // then convert to pixels.
    let dWorld = min(frac_p, 1.0 - frac_p) * step;
    return dWorld * pxPerWorld;
  }

  fn gridLineIntensity(distPx : f32, width : f32) -> f32 {
    return 1.0 - smoothstep(0.0, max(width, 0.0001), distPx);
  }

  @fragment
  fn fs_grid(in : GridVSOut) -> @location(0) vec4<f32> {
    let marginColor = ru.marginColor.rgb;
    let plotBg = ru.plotBgColor.rgb;

    let px = in.canvasPx.x;
    let py = in.canvasPx.y;
    let inside = px >= ru.plotX && px < ru.plotX + ru.plotW
              && py >= ru.plotY && py < ru.plotY + ru.plotH;

    if (!inside) {
      return vec4<f32>(marginColor, 1.0);
    }

    let xMin = ru.currentTime - ru.windowSec;
    let worldX = xMin + (px - ru.plotX) / ru.plotW * ru.windowSec;
    let worldY = ru.yMax - (py - ru.plotY) / ru.plotH * (ru.yMax - ru.yMin);

    let pxPerWorldX = ru.plotW / ru.windowSec;
    let pxPerWorldY = ru.plotH / (ru.yMax - ru.yMin);

    let minorD = min(gridLineDistPx(worldX, ru.xMajorStep * 0.5, pxPerWorldX),
                     gridLineDistPx(worldY, ru.yMajorStep * 0.5, pxPerWorldY));
    let majorD = min(gridLineDistPx(worldX, ru.xMajorStep, pxPerWorldX),
                     gridLineDistPx(worldY, ru.yMajorStep, pxPerWorldY));

    let xMinor = gridLineIntensity(minorD, 1.0);
    let xMajor = gridLineIntensity(majorD, 1.0);

    var color = plotBg;
    color = mix(color, ru.minorColor.rgb, xMinor * 0.35);
    color = mix(color, ru.majorColor.rgb, xMajor * 0.45);

    return vec4<f32>(color, 1.0);
  }
`;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Nice tick step algorithm: picks a "round" step (1, 2, 5 × 10^n) that
 * produces approximately `targetCount` ticks across `range`.
 */
function niceStep(range: number, targetCount: number): number {
  const raw = range / targetCount;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  let step: number;
  if (norm <= 1.5) step = 1;
  else if (norm <= 3) step = 2;
  else if (norm <= 7) step = 5;
  else step = 10;
  return step * mag;
}

/** Format a tick value for display (strips trailing zeros). */
function formatTick(v: number): string {
  const abs = Math.abs(v);
  if (abs === 0) return '0';
  if (abs >= 1000 || abs < 1e-3) return v.toExponential(1);
  let s = v.toFixed(3);
  s = s.replace(/0+$/, '').replace(/\.$/, '');
  return s;
}

// ---------------------------------------------------------------------------
// WebGPUScope
// ---------------------------------------------------------------------------

/**
 * WebGPU-based real-time oscilloscope web component.
 *
 * Replaces the canvas-based `TetherScope`.  Uses a GPU ring buffer with
 * modulo addressing and auto-scroll, MSAA anti-aliased line strips, and a
 * 2D canvas overlay for axis labels and legend.
 *
 * Based on https://techoverflow.net/2026/08/19/real-time-1-khz-webgpu-oscilloscope-with-ring-buffer-auto-scroll/
 *
 * @customElement tether-webgpu-scope
 */
export class WebGPUScope extends HTMLElement {
  // ---- WebGPU state ----
  private device?: GPUDevice;
  private ctx?: GPUCanvasContext;
  private format?: GPUTextureFormat;
  private ringBuffer?: GPUBuffer;
  private renderUniforms?: GPUBuffer;
  private colorBuffer?: GPUBuffer;
  private bindGroup?: GPUBindGroup;
  private linePipeline?: GPURenderPipeline;
  private gridPipeline?: GPURenderPipeline;
  private msaaTexture?: GPUTexture | null;
  private msaaSC = 0;
  private msaaW = 0;
  private msaaH = 0;
  private canvasEl?: HTMLCanvasElement;
  private overlayEl?: HTMLCanvasElement;
  private octx?: CanvasRenderingContext2D | null;
  private rafId?: number;
  private initialized = false;

  // ---- Ring buffer state ----
  /** Next sample slot to write (0..BUFFER_SAMPLES-1). */
  private writeIndex = 0;
  /** Timestamp of the most recent sample (seconds). */
  private currentTime = 0.0;
  /** Total samples written so far (for firstVertex calculation). */
  private sampleCounter = 0;
  /** Pending samples not yet uploaded to the GPU. */
  private pendingRows: ScopeRow[] = [];

  // ---- Channel configuration ----
  /** Number of active channels (set by setChannels). */
  private numChannels = 0;
  /** Per-channel info (name + color). */
  private channels: ChannelInfo[] = [];

  // ---- Y auto-scale ----
  /** Rolling min/max of visible samples for Y auto-scaling. */
  private yMin = -1.2;
  private yMax = 1.2;

  // ---- Pre-allocated upload buffer ----
  /** Reused Float32Array for batch uploads (avoids GC pressure). */
  private chunkData?: Float32Array;

  // =========================================================================
  // Lifecycle
  // =========================================================================

  /** Lifecycle: element inserted into the DOM. */
  async connectedCallback(): Promise<void> {
    this.innerHTML =
      '<canvas class="realtime-plot-canvas" style="width:100%;height:100%;display:block;"></canvas>';
    this.style.position = 'relative';
    this.canvasEl = this.querySelector('canvas')!;

    await this.initWebGPU();
    if (this.initialized) {
      this.startRenderLoop();
    }
  }

  /** Lifecycle: element removed from the DOM. */
  disconnectedCallback(): void {
    if (this.rafId) cancelAnimationFrame(this.rafId);
    this.msaaTexture?.destroy();
  }

  // =========================================================================
  // Public API
  // =========================================================================

  /**
   * Set the channel configuration (names and colors).
   *
   * Must be called before pushing data.  Allocates the color buffer on the GPU.
   *
   * @param channels Array of channel info (name + RGB color).
   */
  setChannels(channels: ChannelInfo[]): void {
    this.channels = channels;
    this.numChannels = channels.length;
    if (this.device && this.colorBuffer) {
      this.writeColors();
    }
  }

  /**
   * Push a stream row into the oscilloscope.
   *
   * The row is queued and uploaded to the GPU ring buffer on the next
   * animation frame (batched for efficiency).
   *
   * @param timestampUs Timestamp in microseconds (from StreamRow).
   * @param values      One numeric value per channel.
   */
  push(timestampUs: bigint, values: number[]): void {
    if (this.numChannels === 0) return;
    const t = Number(timestampUs) / 1e6; // convert µs → seconds
    this.pendingRows.push({ t, values });
    this.currentTime = t;
  }

  /** Clear all data from the oscilloscope. */
  clear(): void {
    this.pendingRows = [];
    this.writeIndex = 0;
    this.currentTime = 0.0;
    this.sampleCounter = 0;
    this.yMin = -1.2;
    this.yMax = 1.2;
    if (this.device && this.ringBuffer) {
      // Re-initialize ring buffer with sentinel timestamps.
      const initBuf = new Float32Array(new ArrayBuffer(BUFFER_SAMPLES * MAX_CHANNELS * 2 * 4));
      for (let i = 0; i < BUFFER_SAMPLES; i++) {
        for (let ch = 0; ch < MAX_CHANNELS; ch++) {
          const idx = (i * MAX_CHANNELS + ch) * 2;
          initBuf[idx] = -1e30;
          initBuf[idx + 1] = 0.0;
        }
      }
      this.device.queue.writeBuffer(this.ringBuffer, 0, initBuf);
    }
  }

  // =========================================================================
  // WebGPU initialization
  // =========================================================================

  /** Initialize WebGPU device, buffers, pipelines, and overlay canvas. */
  private async initWebGPU(): Promise<void> {
    const canvas = this.canvasEl!;

    if (!navigator.gpu) {
      canvas.replaceWith(document.createTextNode('WebGPU is not supported in this browser.'));
      return;
    }

    const adapter = await navigator.gpu.requestAdapter();
    if (!adapter) {
      canvas.replaceWith(document.createTextNode('No WebGPU adapter available.'));
      return;
    }

    const device = await adapter.requestDevice();
    this.device = device;
    this.ctx = canvas.getContext('webgpu')!;
    this.format = navigator.gpu.getPreferredCanvasFormat();
    this.ctx.configure({ device, format: this.format, alphaMode: 'premultiplied' });

    const module = device.createShaderModule({ code: SHADER });

    // Bind group layout: uniforms + ring buffer + color buffer.
    const renderLayout = device.createBindGroupLayout({
      entries: [
        {
          binding: 0,
          visibility: GPUShaderStage.VERTEX | GPUShaderStage.FRAGMENT,
          buffer: { type: 'uniform' },
        },
        {
          binding: 1,
          visibility: GPUShaderStage.VERTEX,
          buffer: { type: 'read-only-storage' },
        },
        {
          binding: 2,
          visibility: GPUShaderStage.VERTEX,
          buffer: { type: 'read-only-storage' },
        },
      ],
    });

    // Probe MSAA sample counts and create pipelines.
    const supportedSC: number[] = [];
    const linePipelines: GPURenderPipeline[] = [];
    const gridPipelines: GPURenderPipeline[] = [];
    const layout = device.createPipelineLayout({ bindGroupLayouts: [renderLayout] });

    for (const sc of MSAA_SAMPLE_COUNTS) {
      try {
        const [linePipe, gridPipe] = await Promise.all([
          device.createRenderPipelineAsync({
            layout,
            vertex: { module, entryPoint: 'vs_line' },
            fragment: { module, entryPoint: 'fs_line', targets: [{ format: this.format }] },
            primitive: { topology: 'line-strip' },
            multisample: { count: sc },
          }),
          device.createRenderPipelineAsync({
            layout,
            vertex: { module, entryPoint: 'vs_grid' },
            fragment: { module, entryPoint: 'fs_grid', targets: [{ format: this.format }] },
            primitive: { topology: 'triangle-list' },
            multisample: { count: sc },
          }),
        ]);
        supportedSC.push(sc);
        linePipelines.push(linePipe);
        gridPipelines.push(gridPipe);
      } catch {
        // Unsupported sample count — skip.
      }
    }

    // Prefer 4× MSAA; fall back to highest available.
    let msaaIndex = supportedSC.length - 1;
    const targetIdx = supportedSC.indexOf(TARGET_MSAA);
    if (targetIdx !== -1) msaaIndex = targetIdx;

    // Ring buffer: (t, v) for MAX_CHANNELS channels, interleaved per sample.
    const bufferFloats = BUFFER_SAMPLES * MAX_CHANNELS * 2;
    const bufferBytes = bufferFloats * 4;
    this.ringBuffer = device.createBuffer({
      size: bufferBytes,
      usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST,
    });

    // Render uniforms (56 bytes: 12 × f32 + 2 × u32).
    this.renderUniforms = device.createBuffer({
      size: 128, // 56 bytes (12 f32 + 2 u32) + 64 bytes (4 vec4 theme colors) padded to 16
      usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
    });

    // Channel colors storage buffer: MAX_CHANNELS × vec4<f32>.
    this.colorBuffer = device.createBuffer({
      size: MAX_CHANNELS * 16,
      usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST,
    });

    // Initialize ring buffer with sentinel timestamps (far off-screen).
    const initBuf = new Float32Array(new ArrayBuffer(bufferFloats * 4));
    for (let i = 0; i < BUFFER_SAMPLES; i++) {
      for (let ch = 0; ch < MAX_CHANNELS; ch++) {
        const idx = (i * MAX_CHANNELS + ch) * 2;
        initBuf[idx] = -1e30;
        initBuf[idx + 1] = 0.0;
      }
    }
    device.queue.writeBuffer(this.ringBuffer, 0, initBuf);

    // Write channel colors (defaults if setChannels not yet called).
    this.writeColors();

    this.bindGroup = device.createBindGroup({
      layout: renderLayout,
      entries: [
        { binding: 0, resource: { buffer: this.renderUniforms } },
        { binding: 1, resource: { buffer: this.ringBuffer } },
        { binding: 2, resource: { buffer: this.colorBuffer } },
      ],
    });

    this.linePipeline = linePipelines[msaaIndex]!;
    this.gridPipeline = gridPipelines[msaaIndex]!;
    this.msaaSC = supportedSC[msaaIndex] ?? 1;

    // Pre-allocate chunk upload buffer (explicit ArrayBuffer for WebGPU compat).
    this.chunkData = new Float32Array(new ArrayBuffer(MAX_CHANNELS * 2 * 4));

    // 2D canvas overlay for axis labels + legend.
    this.overlayEl = document.createElement('canvas');
    this.overlayEl.style.position = 'absolute';
    this.overlayEl.style.left = '0';
    this.overlayEl.style.top = '0';
    this.overlayEl.style.width = '100%';
    this.overlayEl.style.height = '100%';
    this.overlayEl.style.pointerEvents = 'none';
    this.appendChild(this.overlayEl);
    this.octx = this.overlayEl.getContext('2d');

    this.initialized = true;
  }

  /** Write channel colors to the GPU color buffer. */
  private writeColors(): void {
    if (!this.device || !this.colorBuffer) return;
    const colorData = new Float32Array(new ArrayBuffer(MAX_CHANNELS * 4 * 4));
    for (let ch = 0; ch < MAX_CHANNELS; ch++) {
      const color: [number, number, number] =
        ch < this.channels.length ? this.channels[ch]!.color : [0.5, 0.5, 0.5];
      colorData[ch * 4] = color[0]!;
      colorData[ch * 4 + 1] = color[1]!;
      colorData[ch * 4 + 2] = color[2]!;
      colorData[ch * 4 + 3] = 1.0;
    }
    this.device.queue.writeBuffer(this.colorBuffer, 0, colorData);
  }

  // =========================================================================
  // Data upload
  // =========================================================================

  /**
   * Upload all pending rows to the GPU ring buffer.
   * Called once per animation frame (batches multiple push() calls).
   */
  private flushPending(): void {
    if (!this.device || !this.ringBuffer || !this.chunkData) return;
    if (this.pendingRows.length === 0) return;

    const sampleStride = MAX_CHANNELS * 2 * 4; // bytes per sample

    for (const row of this.pendingRows) {
      // Fill chunkData with this sample's (t, v) for all MAX_CHANNELS slots.
      for (let ch = 0; ch < MAX_CHANNELS; ch++) {
        const idx = ch * 2;
        const hasValue = ch < row.values.length;
        this.chunkData[idx] = hasValue ? row.t : -1e30;
        this.chunkData[idx + 1] = hasValue ? (row.values[ch] ?? 0.0) : 0.0;
      }

      // Write to ring buffer at writeIndex (handle wraparound).
      if (this.writeIndex < BUFFER_SAMPLES) {
        const offset = this.writeIndex * sampleStride;
        this.device.queue.writeBuffer(
          this.ringBuffer,
          offset,
          this.chunkData as Float32Array<ArrayBuffer>,
        );
      }
      // Note: since we write one sample at a time, wraparound is simple.
      this.writeIndex = (this.writeIndex + 1) % BUFFER_SAMPLES;
      this.sampleCounter++;
    }

    this.pendingRows = [];
  }

  // =========================================================================
  // Y auto-scale
  // =========================================================================

  /**
   * Compute Y min/max from the visible window of samples.
   * Called each frame to auto-scale the Y axis.
   */
  private updateYScale(): void {
    if (this.numChannels === 0 || this.sampleCounter === 0) return;
    // For simplicity, use a fixed Y range based on all pushed data.
    // A more sophisticated approach would scan only the visible window.
    // For now, we use a rolling estimate.
    let min = Infinity;
    let max = -Infinity;
    for (const row of this.pendingRows) {
      for (let ch = 0; ch < this.numChannels; ch++) {
        const v = row.values[ch];
        if (v !== undefined && Number.isFinite(v)) {
          if (v < min) min = v;
          if (v > max) max = v;
        }
      }
    }
    if (Number.isFinite(min) && Number.isFinite(max)) {
      // Smoothly update the rolling min/max.
      const range = max - min || 1;
      const padding = range * 0.1;
      const targetMin = min - padding;
      const targetMax = max + padding;
      // Lerp towards the target for smooth scaling.
      this.yMin = this.yMin + (targetMin - this.yMin) * 0.1;
      this.yMax = this.yMax + (targetMax - this.yMax) * 0.1;
    }
  }

  // =========================================================================
  // MSAA texture management
  // =========================================================================

  /** Ensure the MSAA texture matches the canvas size and sample count. */
  private ensureMsaa(w: number, h: number, sc: number): void {
    if (!this.device || !this.format) return;
    if (sc === 1) {
      if (this.msaaTexture) {
        this.msaaTexture.destroy();
        this.msaaTexture = null;
      }
      this.msaaSC = 1;
      this.msaaW = w;
      this.msaaH = h;
      return;
    }
    if (this.msaaTexture && this.msaaSC === sc && this.msaaW === w && this.msaaH === h) return;
    if (this.msaaTexture) this.msaaTexture.destroy();
    this.msaaTexture = this.device.createTexture({
      size: [w, h, 1],
      format: this.format,
      usage: GPUTextureUsage.RENDER_ATTACHMENT,
      sampleCount: sc,
    });
    this.msaaSC = sc;
    this.msaaW = w;
    this.msaaH = h;
  }

  // =========================================================================
  // Render loop
  // =========================================================================

  /** Start the requestAnimationFrame render loop. */
  private startRenderLoop(): void {
    const frame = () => {
      this.flushPending();
      this.updateYScale();
      this.render();
      this.rafId = requestAnimationFrame(frame);
    };
    this.rafId = requestAnimationFrame(frame);
  }

  /** Render one frame: grid + line strips + axis overlay. */
  private render(): void {
    if (
      !this.device ||
      !this.ctx ||
      !this.canvasEl ||
      !this.renderUniforms ||
      !this.bindGroup ||
      !this.linePipeline ||
      !this.gridPipeline
    )
      return;

    const canvas = this.canvasEl;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const cw = Math.max(1, Math.floor(canvas.clientWidth * dpr));
    const ch = Math.max(1, Math.floor(canvas.clientHeight * dpr));
    if (canvas.width !== cw || canvas.height !== ch) {
      canvas.width = cw;
      canvas.height = ch;
    }

    const ml = MARGIN_LEFT * dpr;
    const mr = MARGIN_RIGHT * dpr;
    const mt = MARGIN_TOP * dpr;
    const mb = MARGIN_BOTTOM * dpr;
    const plotX = ml;
    const plotY = mt;
    const plotW = Math.max(1, cw - ml - mr);
    const plotH = Math.max(1, ch - mt - mb);

    const sc = this.msaaSC;
    const xMajorStep = niceStep(WINDOW_SEC, 8);
    const yMajorStep = niceStep(this.yMax - this.yMin, 6);

    // Write render uniforms (12 × f32).
    const ruData = new Float32Array(new ArrayBuffer(12 * 4));
    ruData[0] = this.currentTime;
    ruData[1] = WINDOW_SEC;
    ruData[2] = this.yMin;
    ruData[3] = this.yMax;
    ruData[4] = plotX;
    ruData[5] = plotY;
    ruData[6] = plotW;
    ruData[7] = plotH;
    ruData[8] = cw;
    ruData[9] = ch;
    ruData[10] = xMajorStep;
    ruData[11] = yMajorStep;
    this.device.queue.writeBuffer(this.renderUniforms, 0, ruData);

    // Write writeIndex and bufferSamples as u32 at offset 48 (bytes).
    const bufInfo = new Uint32Array(new ArrayBuffer(2 * 4));
    bufInfo[0] = this.writeIndex;
    bufInfo[1] = BUFFER_SAMPLES;
    this.device.queue.writeBuffer(this.renderUniforms, 48, bufInfo);

    // Write theme colors as 4 × vec4<f32> at offset 56 (bytes).
    // Colors are read from CSS variables so the plot matches the active theme.
    const theme = this.readThemeColors();
    const colorData = new Float32Array(new ArrayBuffer(16 * 4));
    colorData.set(theme.margin, 0);
    colorData.set(theme.plotBg, 4);
    colorData.set(theme.minor, 8);
    colorData.set(theme.major, 12);
    this.device.queue.writeBuffer(this.renderUniforms, 56, colorData);

    this.ensureMsaa(cw, ch, sc);

    const encoder = this.device.createCommandEncoder();
    const canvasView = this.ctx.getCurrentTexture().createView();
    let colorAttachment: GPURenderPassColorAttachment;
    if (sc > 1 && this.msaaTexture) {
      colorAttachment = {
        view: this.msaaTexture.createView(),
        resolveTarget: canvasView,
        clearValue: { r: 0.94, g: 0.94, b: 0.94, a: 1 },
        loadOp: 'clear',
        storeOp: 'discard',
      };
    } else {
      colorAttachment = {
        view: canvasView,
        clearValue: { r: 0.94, g: 0.94, b: 0.94, a: 1 },
        loadOp: 'clear',
        storeOp: 'store',
      };
    }

    const rp = encoder.beginRenderPass({ colorAttachments: [colorAttachment] });

    // 1) Grid background.
    rp.setPipeline(this.gridPipeline);
    rp.setBindGroup(0, this.bindGroup);
    rp.draw(6);

    // 2) Line strips — single instanced draw, one instance per channel.
    if (this.numChannels > 0) {
      rp.setPipeline(this.linePipeline);
      rp.setBindGroup(0, this.bindGroup);
      const validSamples = Math.min(this.sampleCounter, WINDOW_SAMPLES);
      const firstVertex = WINDOW_SAMPLES - validSamples;
      rp.draw(validSamples, this.numChannels, firstVertex, 0);
    }

    rp.end();
    this.device.queue.submit([encoder.finish()]);

    // Draw axis labels + legend on the 2D overlay.
    this.drawAxes(canvas.clientWidth, canvas.clientHeight, dpr);
  }

  // =========================================================================
  // Theme color reading
  // =========================================================================

  /**
   * Read theme colors from CSS custom properties.
   *
   * Returns 4 RGBA tuples (as `[r, g, b, a]` arrays) for the plot margin,
   * plot background, minor grid lines, and major grid lines.  The colors
   * adapt to the active light/dark theme automatically.
   *
   * @returns Object with `margin`, `plotBg`, `minor`, `major` — each a
   *          4-element array of floats in [0, 1].
   */
  private readThemeColors(): {
    margin: [number, number, number, number];
    plotBg: [number, number, number, number];
    minor: [number, number, number, number];
    major: [number, number, number, number];
  } {
    const root = document.documentElement;
    const styles = getComputedStyle(root);
    const isDark = root.getAttribute('data-theme') === 'dark' || styles.colorScheme === 'dark';

    // Parse rgb(...), rgba(...), #rrggbb, #rgb, or #rrggbbaa to [r, g, b].
    const parseColor = (h: string): [number, number, number] => {
      const s = h.trim();
      const rgbMatch = s.match(
        /^rgba?\(\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)/,
      );
      if (rgbMatch) {
        return [
          Math.min(1, Math.max(0, Number(rgbMatch[1]) / 255)),
          Math.min(1, Math.max(0, Number(rgbMatch[2]) / 255)),
          Math.min(1, Math.max(0, Number(rgbMatch[3]) / 255)),
        ];
      }
      const hexMatch = s.match(/^#([0-9a-f]{3,4}|[0-9a-f]{6}|[0-9a-f]{8})$/i);
      if (hexMatch) {
        const hex = hexMatch[1]!;
        if (hex.length === 3) {
          return [
            parseInt(hex[0]! + hex[0]!, 16) / 255,
            parseInt(hex[1]! + hex[1]!, 16) / 255,
            parseInt(hex[2]! + hex[2]!, 16) / 255,
          ];
        }
        if (hex.length === 4) {
          return [
            parseInt(hex[0]! + hex[0]!, 16) / 255,
            parseInt(hex[1]! + hex[1]!, 16) / 255,
            parseInt(hex[2]! + hex[2]!, 16) / 255,
          ];
        }
        if (hex.length === 6 || hex.length === 8) {
          return [
            parseInt(hex.slice(0, 2), 16) / 255,
            parseInt(hex.slice(2, 4), 16) / 255,
            parseInt(hex.slice(4, 6), 16) / 255,
          ];
        }
      }
      return [0.92, 0.92, 0.93];
    };

    // Read CSS variables; fall back to sensible defaults.
    const cssVar = (name: string) => styles.getPropertyValue(name).trim();
    const marginRaw = cssVar('--plot-margin') || (isDark ? '#0a1420' : '#e8eef5');
    const plotBgRaw = cssVar('--plot-bg') || (isDark ? '#0d1a28' : '#ffffff');

    const [mr, mg, mb] = parseColor(marginRaw);
    const [pr, pg, pb] = parseColor(plotBgRaw);

    // Grid line colors from CSS, or subtle defaults.
    const minorRaw = cssVar('--plot-grid-minor') || (isDark ? '#162636' : '#e2e8f0');
    const majorRaw = cssVar('--plot-grid-major') || (isDark ? '#253b52' : '#cbd5e1');
    const [mir, mig, mib] = parseColor(minorRaw);
    const [mar, mag, mab] = parseColor(majorRaw);

    return {
      margin: [mr, mg, mb, 1],
      plotBg: [pr, pg, pb, 1],
      minor: [mir, mig, mib, 1],
      major: [mar, mag, mab, 1],
    };
  }

  // =========================================================================
  // Axis labels & legend (2D canvas overlay)
  // =========================================================================

  /** Draw axis tick labels, axis titles, and channel legend on the overlay. */
  private drawAxes(cssW: number, cssH: number, dpr: number): void {
    if (!this.overlayEl || !this.octx || !this.canvasEl) return;
    const canvas = this.canvasEl;
    const overlay = this.overlayEl;
    const octx = this.octx;
    const cw = canvas.width;
    const ch = canvas.height;
    if (overlay.width !== cw || overlay.height !== ch) {
      overlay.width = cw;
      overlay.height = ch;
    }
    octx.clearRect(0, 0, cw, ch);
    octx.save();
    octx.scale(dpr, dpr);

    // Read theme-aware text/axis colors from CSS variables.
    const styles = getComputedStyle(this);
    const textColor = styles.getPropertyValue('--text').trim() || '#1a2a3a';
    const textMuted = styles.getPropertyValue('--text-muted').trim() || '#5a7088';

    const ml = MARGIN_LEFT;
    const mr = MARGIN_RIGHT;
    const mt = MARGIN_TOP;
    const mb = MARGIN_BOTTOM;
    const px = ml;
    const py = mt;
    const pw = cssW - ml - mr;
    const ph = cssH - mt - mb;

    // Plot axes (ggplot style: only bottom and left axis lines).
    octx.strokeStyle = textColor;
    octx.lineWidth = 1;
    octx.beginPath();
    octx.moveTo(px, py);
    octx.lineTo(px, py + ph);
    octx.lineTo(px + pw, py + ph);
    octx.stroke();

    octx.font = '11px sans-serif';
    octx.fillStyle = textMuted;

    // X axis: relative time labels.
    const xStep = niceStep(WINDOW_SEC, 8);
    const xStart = Math.ceil((this.currentTime - WINDOW_SEC) / xStep) * xStep;
    octx.textAlign = 'center';
    octx.textBaseline = 'top';
    for (let x = xStart; x <= this.currentTime + xStep * 0.001; x += xStep) {
      const sx = px + ((x - (this.currentTime - WINDOW_SEC)) / WINDOW_SEC) * pw;
      if (sx < px - 1 || sx > px + pw + 1) continue;
      octx.beginPath();
      octx.moveTo(sx, py + ph);
      octx.lineTo(sx, py + ph + 4);
      octx.stroke();
      const label = x === this.currentTime ? '0s' : (x - this.currentTime).toFixed(1) + 's';
      octx.fillText(label, sx, py + ph + 8);
    }

    // Y axis.
    const yStep = niceStep(this.yMax - this.yMin, 6);
    const yStart = Math.ceil(this.yMin / yStep) * yStep;
    octx.textAlign = 'right';
    octx.textBaseline = 'middle';
    for (let y = yStart; y <= this.yMax + yStep * 0.001; y += yStep) {
      const sy = py + ((this.yMax - y) / (this.yMax - this.yMin)) * ph;
      if (sy < py - 1 || sy > py + ph + 1) continue;
      octx.beginPath();
      octx.moveTo(px, sy);
      octx.lineTo(px - 4, sy);
      octx.stroke();
      octx.fillText(formatTick(y), px - 8, sy);
    }

    // Axis titles.
    octx.font = '13px sans-serif';
    octx.fillStyle = textColor;
    octx.textAlign = 'center';
    octx.textBaseline = 'bottom';
    octx.fillText('t (s, relative)', ml + pw / 2, cssH - 4);
    octx.save();
    octx.translate(12, mt + ph / 2);
    octx.rotate(-Math.PI / 2);
    octx.textAlign = 'center';
    octx.textBaseline = 'top';
    octx.fillText('value', 0, 0);
    octx.restore();

    // Legend (right margin).
    const legendX = ml + pw + 8;
    const legendY = mt + 4;
    octx.font = '11px sans-serif';
    octx.textAlign = 'left';
    octx.textBaseline = 'middle';
    for (let i = 0; i < this.numChannels; i++) {
      const [r, g, b] = this.channels[i]!.color;
      const ly = legendY + i * 18;
      octx.fillStyle = `rgb(${Math.round(r * 255)},${Math.round(g * 255)},${Math.round(b * 255)})`;
      octx.fillRect(legendX, ly - 5, 12, 10);
      octx.fillStyle = textMuted;
      const name = this.channels[i]!.name;
      octx.fillText(name, legendX + 16, ly);
    }

    octx.restore();
  }
}

customElements.define('tether-webgpu-scope', WebGPUScope);
