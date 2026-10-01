import {
  ChannelInfo,
  formatTick,
  MARGIN_BOTTOM,
  MARGIN_LEFT,
  MARGIN_RIGHT,
  MARGIN_TOP,
  niceStep,
  WINDOW_SEC,
} from './config';

export interface ScopeOverlayState {
  canvas: HTMLCanvasElement;
  overlay: HTMLCanvasElement;
  context: CanvasRenderingContext2D;
  cssWidth: number;
  cssHeight: number;
  pixelRatio: number;
  activeCurrentTime: number;
  viewTimeMin: number | null;
  viewTimeSpan: number;
  yMin: number;
  yMax: number;
  zoomYMin: number | null;
  zoomYMax: number | null;
  channels: ChannelInfo[];
  highlightedChannel: number | null;
  dragActive: boolean;
  dragStartX: number;
  dragStartY: number;
  dragCurrentX: number;
  dragCurrentY: number;
}

export function drawScopeOverlay(state: ScopeOverlayState): void {
  const { canvas, overlay, context, pixelRatio: dpr } = state;
  const width = canvas.width;
  const height = canvas.height;
  if (overlay.width !== width || overlay.height !== height) {
    overlay.width = width;
    overlay.height = height;
  }
  context.clearRect(0, 0, width, height);
  context.save();
  context.scale(dpr, dpr);

  const styles = getComputedStyle(canvas.parentElement ?? canvas);
  const textColor = styles.getPropertyValue('--text').trim() || '#1a2a3a';
  const mutedColor = styles.getPropertyValue('--text-muted').trim() || '#5a7088';
  const plotX = MARGIN_LEFT;
  const plotY = MARGIN_TOP;
  const plotWidth = state.cssWidth - MARGIN_LEFT - MARGIN_RIGHT;
  const plotHeight = state.cssHeight - MARGIN_TOP - MARGIN_BOTTOM;
  const viewMin = state.viewTimeMin ?? state.activeCurrentTime - WINDOW_SEC;
  const viewSpan = state.viewTimeSpan;
  const yMin = state.zoomYMin ?? state.yMin;
  const yMax = state.zoomYMax ?? state.yMax;

  context.strokeStyle = textColor;
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(plotX, plotY);
  context.lineTo(plotX, plotY + plotHeight);
  context.lineTo(plotX + plotWidth, plotY + plotHeight);
  context.stroke();

  context.font = '11px sans-serif';
  context.fillStyle = mutedColor;
  const xStep = niceStep(viewSpan, 8);
  const xStart = Math.ceil(viewMin / xStep) * xStep;
  context.textAlign = 'center';
  context.textBaseline = 'top';
  for (let value = xStart; value <= viewMin + viewSpan + xStep * 0.001; value += xStep) {
    const screenX = plotX + ((value - viewMin) / viewSpan) * plotWidth;
    if (screenX < plotX - 1 || screenX > plotX + plotWidth + 1) continue;
    context.beginPath();
    context.moveTo(screenX, plotY + plotHeight);
    context.lineTo(screenX, plotY + plotHeight + 4);
    context.stroke();
    context.fillText(`${(value - viewMin).toFixed(2)}s`, screenX, plotY + plotHeight + 8);
  }

  const yStep = niceStep(yMax - yMin, 6);
  const yStart = Math.ceil(yMin / yStep) * yStep;
  context.textAlign = 'right';
  context.textBaseline = 'middle';
  for (let value = yStart; value <= yMax + yStep * 0.001; value += yStep) {
    const screenY = plotY + ((yMax - value) / (yMax - yMin)) * plotHeight;
    if (screenY < plotY - 1 || screenY > plotY + plotHeight + 1) continue;
    context.beginPath();
    context.moveTo(plotX, screenY);
    context.lineTo(plotX - 4, screenY);
    context.stroke();
    context.fillText(formatTick(value), plotX - 8, screenY);
  }

  context.font = '13px sans-serif';
  context.fillStyle = textColor;
  context.textAlign = 'center';
  context.textBaseline = 'bottom';
  context.fillText('t (s, relative)', MARGIN_LEFT + plotWidth / 2, state.cssHeight - 4);
  context.save();
  context.translate(12, MARGIN_TOP + plotHeight / 2);
  context.rotate(-Math.PI / 2);
  context.textAlign = 'center';
  context.textBaseline = 'top';
  context.fillText('value', 0, 0);
  context.restore();

  drawLegend(context, state, plotX + plotWidth + 8, plotY + 4, plotWidth, textColor, mutedColor);
  drawDragSelection(context, state);
  context.restore();
}

function drawLegend(
  context: CanvasRenderingContext2D,
  state: ScopeOverlayState,
  legendX: number,
  legendY: number,
  plotWidth: number,
  textColor: string,
  mutedColor: string,
): void {
  context.font = '11px sans-serif';
  context.textAlign = 'left';
  context.textBaseline = 'middle';
  for (let index = 0; index < state.channels.length; index += 1) {
    const channel = state.channels[index]!;
    const isHighlighted = state.highlightedChannel === index;
    const isDimmed = state.highlightedChannel !== null && !isHighlighted;
    const [red, green, blue] = isDimmed ? dimColor(channel.color, 0.7) : channel.color;
    const rowY = legendY + index * 18;
    if (isHighlighted) {
      context.fillStyle = 'rgba(0,0,0,0.06)';
      context.fillRect(legendX - 4, rowY - 9, plotWidth + 12 - (legendX - MARGIN_LEFT), 18);
    }
    context.fillStyle = `rgb(${Math.round(red * 255)},${Math.round(green * 255)},${Math.round(blue * 255)})`;
    context.fillRect(legendX, rowY - 5, 12, 10);
    context.fillStyle = isDimmed ? mutedColor : textColor;
    context.font = isHighlighted ? 'bold 11px sans-serif' : '11px sans-serif';
    context.fillText(channel.name, legendX + 16, rowY);
  }
}

function drawDragSelection(context: CanvasRenderingContext2D, state: ScopeOverlayState): void {
  if (!state.dragActive) return;
  const x0 = Math.min(state.dragStartX, state.dragCurrentX);
  const x1 = Math.max(state.dragStartX, state.dragCurrentX);
  const y0 = Math.min(state.dragStartY, state.dragCurrentY);
  const y1 = Math.max(state.dragStartY, state.dragCurrentY);
  context.fillStyle = 'rgba(0, 100, 200, 0.15)';
  context.fillRect(x0, y0, x1 - x0, y1 - y0);
  context.strokeStyle = 'rgba(0, 100, 200, 0.8)';
  context.lineWidth = 1;
  context.strokeRect(x0, y0, x1 - x0, y1 - y0);
}

function dimColor(color: [number, number, number], factor: number): [number, number, number] {
  const background = 0.97;
  const gray = 0.299 * color[0] + 0.587 * color[1] + 0.114 * color[2];
  return color.map((channel) => {
    const desaturated = 0.5 * channel + 0.5 * gray;
    return desaturated + (background - desaturated) * factor;
  }) as [number, number, number];
}
