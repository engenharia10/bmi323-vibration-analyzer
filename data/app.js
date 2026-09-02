/* BMI323 Vibration Analyzer - interface
   Espectro/forma de onda chegam por WebSocket (binario), status por JSON.
   A cadeia de filtros e a do Betaflight; as formulas abaixo sao as mesmas de
   lib/Filtros (bf_filter.cpp), entao a curva |H(f)| desenhada aqui e exatamente
   a do filtro rodando no ESP32.                                              */
'use strict';

const $ = (id) => document.getElementById(id);
const SPECTRUM_MS = 200;
// Quanto tempo um pico continua na lista depois de sumir da deteccao.
// Sem isso as linhas trocam a 5 fps e nao da tempo de ler.
const PEAK_HOLD_MS = 1000;

// Os 6 canais da IMU. A cor vale para o espectro, para a legenda e para as
// faixas da janela de onda filtrada.
const CH = [
  { id: 'aX', label: 'acel X', color: '#22d3ee', unit: 'mg' },
  { id: 'aY', label: 'acel Y', color: '#34d399', unit: 'mg' },
  { id: 'aZ', label: 'acel Z', color: '#60a5fa', unit: 'mg' },
  { id: 'gX', label: 'giro X', color: '#fb923c', unit: '\u00b0/s' },
  { id: 'gY', label: 'giro Y', color: '#a3e635', unit: '\u00b0/s' },
  { id: 'gZ', label: 'giro Z', color: '#c084fc', unit: '\u00b0/s' },
];

const S = {
  cfg: null,
  spec: null,          // {bins, binHz, fs, mask, ch:[], filt}
  fscope: null,        // {pts, dt, mask, data:[]}  ondas filtradas
  rawscope: null,      // {pts, dt, mask, data:[]}  leitura crua do sensor
  showFilt: true,      // curva do sinal filtrado no espectro
  hold: null,
  scope: null,         // {pts, dt, raw, filt}
  peaks: [],
  peaksHeld: [],   // {f, a, t} com o instante da ultima deteccao
  dyn: [],             // centros rastreados pelo notch dinamico
  rpm: [],             // [{f, w}] harmonicas do filtro RPM
  dcCh: [], rmsCh: [], // resumo do trecho cru: nivel medio e RMS AC por eixo
  lsbCh: [],           // contagem de 16 bits do ultimo quadro do FIFO
  lpf1Cut: 0,
  fmax: 800,
  offline: false,       // sem firmware: previa gerada no navegador
  offlineTimer: null,
  serialPort: null, serialWriter: null, serialReader: null, serialFrames: 0,
  simF: [],             // frequencia instantanea de cada fonte (vem do firmware)
  att: null,            // {r,p,y,gx,gy,gz,g,cal,ok}
  t0: performance.now() / 1000,
};

/* ======================================================== WebSocket ===== */
let ws = null;
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => $('link-dot').classList.add('on');
  ws.onclose = () => { $('link-dot').classList.remove('on'); setTimeout(connect, 1500); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') return onJson(JSON.parse(ev.data));
    onBinary(ev.data);
  };
}

function onJson(m) {
  if (m.t === 'config') { applyConfig(m.cfg); return; }
  if (m.t !== 'status') return;
  logRecord(m);
  // com log aberto a tela e do log: o vivo continua chegando e sendo gravado,
  // mas nao desenha por cima do instante que o usuario esta olhando
  if (LOG.open) return;
  $('m-fs').textContent = m.fs.toFixed(0);
  $('m-meas').textContent = m.measured.toFixed(0);
  $('m-rms').textContent = m.rmsRaw.toFixed(1);
  $('m-rmsf').textContent = m.rmsFilt.toFixed(1);
  $('m-pkpk').textContent = m.pkpk.toFixed(0);
  $('m-temp').textContent = m.temp.toFixed(1);
  $('m-heap').textContent = (m.heap / 1024).toFixed(0) + ' kB';
  $('m-drop').textContent = m.drops;
  mergePeaks(m.peaks);
  S.dyn = m.dyn || [];
  S.rpm = m.rpm || [];
  S.lpf1Cut = m.lpf1Cut || 0;
  S.dcCh = m.dcCh || [];
  S.lsbCh = m.lsbCh || [];
  S.rmsCh = m.rmsCh || [];
  renderAxes();
  S.simF = m.simF || [];
  S.att = m.att || null;
  renderAttitude();
  setMode(m.sim ? 'sim' : (S.serialWriter ? 'serial' : ''));
  renderPeaks();
  renderTracking();
  renderSimLive();
}

function onBinary(buf) {
  if (LOG.open) return;
  const dv = new DataView(buf);
  if (dv.getUint8(0) !== 0xA5) return;
  const type   = dv.getUint8(1);
  const axis   = dv.getUint8(2);
  const flags  = dv.getUint8(3);
  const count  = dv.getUint16(4, true);
  const series = dv.getUint16(6, true);
  const stepX  = dv.getFloat32(8, true);
  const fs     = dv.getFloat32(12, true);
  const f = new Float32Array(buf, 16, count * series);
  const at = (i) => f.subarray(i * count, (i + 1) * count);

  if (type === 1) {
    // uma serie por canal habilitado, na ordem crescente, e a filtrada por ultimo
    const mask = flags & 0x3F;
    const ch = new Array(6).fill(null);
    let i = 0;
    for (let c = 0; c < 6; c++) if (mask & (1 << c)) ch[c] = at(i++);
    const filt = (flags & 0x80) ? at(i++) : null;
    S.spec = { bins: count, binHz: stepX, fs, mask, axis, ch, filt };

    $('res-note').textContent =
      stepX.toFixed(2) + ' Hz/bin \u00b7 ' + count + ' bins \u00b7 Nyquist ' + (fs / 2).toFixed(0) + ' Hz';
    pushWaterfall();
  } else if (type === 2) {
    S.scope = { pts: count, dt: stepX, axis, raw: at(0), filt: at(1) };
    $('scope-span').textContent = (count * stepX * 1000).toFixed(0) + ' ms';
  } else if (type === 3 || type === 4) {
    const mask = flags & 0x3F;
    const data = new Array(6).fill(null);
    let i = 0;
    for (let c = 0; c < 6; c++) if (mask & (1 << c)) data[c] = at(i++);
    const span = (count * stepX * 1000).toFixed(0) + ' ms';
    if (type === 3) {
      S.fscope = { pts: count, dt: stepX, mask, data };
      $('fscope-span').textContent = span;
    } else {
      S.rawscope = { pts: count, dt: stepX, mask, data };
      $('raw-span').textContent = span + ' \u00b7 ' + (1 / stepX).toFixed(0) + ' pontos/s';
    }
  }
}

function setMode(mode) {
  const b = $('mode-badge');
  if (mode === 'off')      { b.hidden = false; b.className = 'mode off'; b.textContent = 'OFFLINE'; }
  else if (mode === 'sim') { b.hidden = false; b.className = 'mode';     b.textContent = 'SIMULAÇÃO'; }
  else if (mode === 'serial') { b.hidden = false; b.className = 'mode ser'; b.textContent = 'SERIAL'; }
  else                     { b.hidden = true; }
}

/* ============================== filtros (espelho de lib/Filtros) ======== */
const CUTOFF_CORRECTION_PT2 = 1.553773974;
const CUTOFF_CORRECTION_PT3 = 1.961459177;
const BIQUAD_Q = 0.70710678;
const LPF_NAMES = ['PT1', 'BIQUAD', 'PT2', 'PT3'];

const pt1FilterGain = (fCut, dT) => { const w = 2 * Math.PI * fCut * dT; return w / (w + 1); };

// Q de um notch a partir do centro e da frequencia de corte inferior
function filterGetNotchQ(centerFreq, cutoffFreq) {
  if (!(cutoffFreq > 0) || cutoffFreq >= centerFreq) return 0;
  return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

// H de um PT1 (1 polo) como numero complexo
function pt1H(k, w) {
  const a = 1 - k;
  const dr = 1 - a * Math.cos(w), di = a * Math.sin(w);
  const d2 = dr * dr + di * di;
  return { re: k * dr / d2, im: -k * di / d2 };
}
const cabs = (c) => Math.hypot(c.re, c.im);

function biquadCoefs(type, freq, fs, q) {
  const nyq = fs * 0.5;
  freq = Math.min(Math.max(freq, 0.1), nyq * 0.98);
  q = Math.max(q, 0.05);
  const w = 2 * Math.PI * freq / fs, sn = Math.sin(w), cs = Math.cos(w), alpha = sn / (2 * q);
  let b0, b1, b2, a1, a2;
  if (type === 'lpf') { b1 = 1 - cs; b0 = b1 * 0.5; b2 = b0; a1 = -2 * cs; a2 = 1 - alpha; }
  else { b0 = 1; b1 = -2 * cs; b2 = 1; a1 = b1; a2 = 1 - alpha; }   // notch
  const a0 = 1 + alpha;
  return { b0: b0 / a0, b1: b1 / a0, b2: b2 / a0, a1: a1 / a0, a2: a2 / a0 };
}

function biquadMag(bq, w) {
  const cw = Math.cos(w), sw = Math.sin(w), c2 = Math.cos(2 * w), s2 = Math.sin(2 * w);
  const nr = bq.b0 + bq.b1 * cw + bq.b2 * c2, ni = -(bq.b1 * sw + bq.b2 * s2);
  const dr = 1 + bq.a1 * cw + bq.a2 * c2, di = -(bq.a1 * sw + bq.a2 * s2);
  const num = { re: nr, im: ni }, den = { re: dr, im: di };
  return cabs(num) / Math.max(cabs(den), 1e-10);
}

// applyDF1Weighted: y = weight*H + (1-weight)*x  =>  H_eq = w*H + (1-w)
function biquadMagWeighted(bq, w, weight) {
  const cw = Math.cos(w), sw = Math.sin(w), c2 = Math.cos(2 * w), s2 = Math.sin(2 * w);
  const nr = bq.b0 + bq.b1 * cw + bq.b2 * c2, ni = -(bq.b1 * sw + bq.b2 * s2);
  const dr = 1 + bq.a1 * cw + bq.a2 * c2, di = -(bq.a1 * sw + bq.a2 * s2);
  const d2 = Math.max(dr * dr + di * di, 1e-20);
  const hr = (nr * dr + ni * di) / d2, hi = (ni * dr - nr * di) / d2;
  return Math.hypot(weight * hr + (1 - weight), weight * hi);
}

// H de um biquad como numero complexo - o atraso de grupo precisa da fase
function biquadH(bq, w) {
  const cw = Math.cos(w), sw = Math.sin(w), c2 = Math.cos(2 * w), s2 = Math.sin(2 * w);
  const nr = bq.b0 + bq.b1 * cw + bq.b2 * c2, ni = -(bq.b1 * sw + bq.b2 * s2);
  const dr = 1 + bq.a1 * cw + bq.a2 * c2, di = -(bq.a1 * sw + bq.a2 * s2);
  const d2 = Math.max(dr * dr + di * di, 1e-20);
  return { re: (nr * dr + ni * di) / d2, im: (ni * dr - nr * di) / d2 };
}
const cmul = (a, b) => ({ re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re });
function cpow(a, n) { let r = { re: 1, im: 0 }; for (let i = 0; i < n; i++) r = cmul(r, a); return r; }

// Cortes de LPF sao escalados pelo multiplicador global
const mult = () => (S.cfg ? S.cfg.mult / 100 : 1);

// Monta a lista de estagios na mesma ordem do firmware:
// DC -> LPF2 -> RPM -> notch1 -> notch2 -> LPF1 -> notch dinamico
function chain() {
  const c = S.cfg; const out = [];
  if (!c) return out;
  const fs = c.odr, dT = 1 / fs;

  if (c.dc && c.dcf > 0) {
    const k = pt1FilterGain(c.dcf, dT);
    const dcH = (w) => { const h = pt1H(k, w); return { re: 1 - h.re, im: -h.im }; };
    out.push({ name: 'dcblock', kind: 'dc', k, hz: c.dcf,
               h: dcH, mag: (w) => cabs(dcH(w)) });
  }

  const lowpass = (label, type, hz) => {
    hz *= mult();
    if (!(hz > 0) || hz >= fs * 0.5) return;
    if (type === 1) {
      const bq = biquadCoefs('lpf', hz, fs, BIQUAD_Q);
      out.push({ name: label, kind: 'biquad', bq, hz, type,
                 h: (w) => biquadH(bq, w), mag: (w) => biquadMag(bq, w) });
    } else {
      const n = type === 0 ? 1 : type === 2 ? 2 : 3;
      const corr = n === 1 ? 1 : n === 2 ? CUTOFF_CORRECTION_PT2 : CUTOFF_CORRECTION_PT3;
      const k = pt1FilterGain(hz * corr, dT);
      out.push({ name: label, kind: 'pt' + n, k, n, hz, type,
                 h: (w) => cpow(pt1H(k, w), n), mag: (w) => Math.pow(cabs(pt1H(k, w)), n) });
    }
  };

  lowpass('lpf2', c.lpf2Type, c.lpf2Hz);

  // filtro RPM: uma notch por harmonica, com peso (crossfade entrada/saida)
  if (c.rpmSrc > 0 && c.rpmHarm > 0) {
    S.rpm.slice(0, c.rpmHarm).forEach((hm, i) => {
      if (!(hm.f > 0) || hm.w <= 0) return;
      const bq = biquadCoefs('notch', hm.f, fs, c.rpmQ / 100);
      out.push({ name: 'rpm' + (i + 1), kind: 'biquad', bq, q: c.rpmQ / 100, freq: hm.f, weight: hm.w,
                 rpm: true,
                 h: (w) => { const g = biquadH(bq, w);
                             return { re: hm.w * g.re + (1 - hm.w), im: hm.w * g.im }; },
                 mag: (w) => biquadMagWeighted(bq, w, hm.w) });
    });
  }

  [[c.n1h, c.n1c, 'notch1'], [c.n2h, c.n2c, 'notch2']].forEach(([hz, cut, label]) => {
    const q = filterGetNotchQ(hz, cut);
    if (hz > 0 && q > 0) {
      const bq = biquadCoefs('notch', hz, fs, q);
      out.push({ name: label, kind: 'biquad', bq, q, freq: hz,
                 h: (w) => biquadH(bq, w), mag: (w) => biquadMag(bq, w) });
    }
  });

  lowpass('lpf1', c.lpf1Type, c.lpf1Dyn ? (S.lpf1Cut / mult() || c.lpf1DynMin) : c.lpf1Hz);

  // notches dinamicos, nos centros que o firmware esta rastreando agora
  const dq = c.dnQ / 100;
  S.dyn.slice(0, c.dnCount).forEach((f0, i) => {
    if (!(f0 > 0)) return;
    const bq = biquadCoefs('notch', f0, fs, dq);
    out.push({ name: 'dynNotch' + (i + 1), kind: 'biquad', bq, q: dq, freq: f0, dyn: true,
               h: (w) => biquadH(bq, w), mag: (w) => biquadMag(bq, w) });
  });

  return out;
}

function respDb(f, fs, ch) {
  const w = 2 * Math.PI * f / fs;
  let db = 0;
  for (const st of ch) db += 20 * Math.log10(Math.max(st.mag(w), 1e-7));
  return db;
}

// Atraso de grupo em DC: -dfase/dw por diferenca finita sobre H(w+d)*conj(H(w)),
// o que dispensa desenrolar a fase. O DC-block fica de fora de proposito: sendo
// passa-alta, o atraso dele explode perto de DC e nao diz nada sobre a faixa
// que esta sendo medida.
function groupDelayMs(ch, fs) {
  const d = 1e-4;
  const tot = (w) => ch.reduce(
    (acc, st) => (st.kind === 'dc' ? acc : cmul(acc, st.h(w))), { re: 1, im: 0 });
  const a = tot(0), b = tot(d);
  const p = cmul(b, { re: a.re, im: -a.im });
  return -Math.atan2(p.im, p.re) / d / fs * 1000;
}

/* ====================================================== canvas helpers == */
function fitCanvas(cv) {
  const r = cv.getBoundingClientRect();
  const d = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(1, Math.round(r.width * d)), h = Math.max(1, Math.round(r.height * d));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const ctx = cv.getContext('2d');
  ctx.setTransform(d, 0, 0, d, 0, 0);
  return { ctx, w: r.width, h: r.height };
}

function ticks(min, max, want) {
  const span = max - min;
  if (span <= 0) return [min];
  const raw = span / want;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / mag;
  const step = (n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10) * mag;
  const out = [];
  for (let v = Math.ceil(min / step) * step; v <= max + step * 1e-6; v += step) out.push(v);
  return out;
}

// arredonda para cima em passos 1/1.5/2/3/5/8 - escala bem mais justa que meia decada
function niceCeil(v) {
  if (!(v > 0)) return 1;
  const e = Math.pow(10, Math.floor(Math.log10(v))), n = v / e;
  const step = n <= 1 ? 1 : n <= 1.5 ? 1.5 : n <= 2 ? 2 : n <= 3 ? 3 : n <= 5 ? 5 : n <= 8 ? 8 : 10;
  return step * e;
}

const fmtHz = (v) => (v >= 1000 ? (v / 1000).toFixed(v % 1000 ? 2 : 0) + 'k' : String(+v.toFixed(2)));
const fmtMg = (v) => (v >= 100 ? v.toFixed(0) : v >= 10 ? v.toFixed(1) : v >= 1 ? v.toFixed(2) : v.toFixed(3));

function placeholder(ctx, g, txt) {
  ctx.fillStyle = '#4d6076'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.font = '12px system-ui,sans-serif';
  ctx.fillText(txt, (g.x0 + g.x1) / 2, (g.y0 + g.y1) / 2);
}

/* ==================================================== leitura no tempo == */
// O painel de cima nao mostra mais espectro: mostra a leitura que sai do
// sensor, ponto a ponto, do jeito que chegou pelo barramento. O espectro
// continua existindo - alimenta o waterfall e a tabela de picos - mas ele
// confunde na hora de conferir o sinal, entao saiu daqui.
//
// Uma faixa por canal, cada uma com sua propria escala: e o que deixa mg e
// graus/s conviverem no mesmo painel sem uma esmagar a outra.

function drawLanes(cvId, src, emptyMsg) {
  const cv = $(cvId); const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);

  const chans = [];
  if (src) for (let c = 0; c < 6; c++) if (src.data[c]) chans.push(c);
  if (!chans.length) {
    placeholder(ctx, { x0: 0, x1: w, y0: 0, y1: h }, emptyMsg);
    return;
  }

  const x0 = 52, x1 = w - 8, y0 = 4, y1 = h - 4;
  const laneH = (y1 - y0) / chans.length;
  ctx.font = '9px ui-monospace,monospace';

  chans.forEach((c, k) => {
    const arr = src.data[c];
    const top = y0 + laneH * k;
    const cy = top + laneH / 2;
    const half = laneH / 2 - 3;

    // cada faixa se escala sozinha pelo proprio pico do trecho
    let amp = 1e-6;
    for (let i = 0; i < src.pts; i++) amp = Math.max(amp, Math.abs(arr[i]));
    amp = niceCeil(amp * 1.15);

    ctx.strokeStyle = '#1b2534'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x0, Math.round(cy) + 0.5); ctx.lineTo(x1, Math.round(cy) + 0.5);
    ctx.stroke();
    if (k) {
      ctx.strokeStyle = '#151d27';
      ctx.beginPath();
      ctx.moveTo(0, Math.round(top) + 0.5); ctx.lineTo(w, Math.round(top) + 0.5);
      ctx.stroke();
    }

    ctx.strokeStyle = CH[c].color; ctx.lineWidth = 1.2;
    ctx.beginPath();
    for (let i = 0; i < src.pts; i++) {
      const x = x0 + (i / (src.pts - 1)) * (x1 - x0);
      const y = cy - (arr[i] / amp) * half;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = CH[c].color; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText(CH[c].id, 5, cy - 5);
    ctx.fillStyle = '#5b6c81';
    ctx.fillText('\u00b1' + (amp >= 100 ? amp.toFixed(0) : amp.toFixed(1)) + ' ' + CH[c].unit,
                 5, cy + 6);
  });
}

// Distingue "ainda nao chegou nada" de "chegou espectro mas nao a leitura
// crua". O segundo caso e placa rodando firmware antigo: o quadro type 4 so
// passou a existir quando este painel virou a leitura da IMU, e sem essa
// distincao a tela fica dizendo "aguardando" para sempre.
function rawEmptyMsg() {
  if (S.rawscope) return 'nenhum canal selecionado';
  if (S.spec) return 'placa conectada, mas sem leitura crua — grave o firmware novo';
  return 'aguardando dados do sensor…';
}

const drawRawScope = () =>
  (LOG.open ? drawTrend() : drawLanes('cv-raw', S.rawscope, rawEmptyMsg()));


/* ========================================================= waterfall ==== */
let wfImg = null, wfW = 0, wfH = 0, wfFresh = true, wfRow = 2;
// altura da linha proporcional ao painel: ~20 s para encher em qualquer tamanho
const wfRowFor = (h) => Math.max(2, Math.round(h / 110));

function pushWaterfall() {
  const cv = $('cv-wf');
  const r = cv.getBoundingClientRect();
  const W = Math.max(1, Math.round(r.width)), H = Math.max(1, Math.round(r.height));
  if (!wfImg || wfW !== W || wfH !== H) {
    wfImg = document.createElement('canvas');
    wfImg.width = W; wfImg.height = H; wfW = W; wfH = H; wfFresh = true;
    wfRow = wfRowFor(H);
  }
  const c = wfImg.getContext('2d');
  if (wfFresh) {
    // pinta o "sem dados ainda" com a cor mais baixa do mapa de calor
    const [R, G, B] = heat(0);
    c.fillStyle = `rgb(${R},${G},${B})`;
    c.fillRect(0, 0, W, H);
    wfFresh = false;
  } else {
    c.drawImage(wfImg, 0, -wfRow);
  }

  const sp = S.spec;
  const src = sp.ch[sp.axis] || sp.ch.find((x) => x);
  if (!src) return;
  const fmax = Math.max(10, Math.min(S.fmax, sp.fs / 2));
  const row = c.createImageData(W, wfRow);
  for (let x = 0; x < W; x++) {
    const f = (x / W) * fmax;
    const bin = Math.max(1, Math.min(sp.bins - 1, Math.round(f / sp.binHz)));
    const db = 20 * Math.log10(Math.max(src[bin], 1e-4));
    const t = Math.max(0, Math.min(1, (db + 40) / 70));
    const [R, G, B] = heat(t);
    for (let y = 0; y < wfRow; y++) {
      const o = (y * W + x) * 4;
      row.data[o] = R; row.data[o + 1] = G; row.data[o + 2] = B; row.data[o + 3] = 255;
    }
  }
  c.putImageData(row, 0, H - wfRow);
}

function heat(t) {
  const stops = [[8, 12, 20], [16, 60, 120], [34, 211, 238], [251, 191, 36], [255, 255, 235]];
  const s = t * (stops.length - 1), i = Math.min(stops.length - 2, Math.floor(s)), k = s - i;
  return [0, 1, 2].map((j) => Math.round(stops[i][j] + (stops[i + 1][j] - stops[i][j]) * k));
}

function drawWaterfall() {
  const cv = $('cv-wf'); const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);
  if (!wfImg) { placeholder(ctx, { x0: 0, x1: w, y0: 0, y1: h }, 'sem dados'); return; }
  ctx.drawImage(wfImg, 0, 0, w, h);
  const fmax = Math.max(10, Math.min(S.fmax, S.spec ? S.spec.fs / 2 : S.fmax));
  ctx.font = '10px ui-monospace,monospace';
  ctx.textBaseline = 'bottom';
  ctx.lineJoin = 'round';
  for (const f of ticks(0, fmax, 6)) {
    const x = (f / fmax) * w;
    ctx.strokeStyle = 'rgba(255,255,255,.09)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    if (f <= 0) continue;
    // encosta o ultimo rotulo na borda em vez de deixar metade sair do canvas
    const near = x > w - 26;
    ctx.textAlign = near ? 'right' : 'center';
    const tx = near ? w - 3 : x;
    const label = fmtHz(f) + ' Hz';
    ctx.strokeStyle = 'rgba(0,0,0,.75)'; ctx.lineWidth = 3;
    ctx.strokeText(label, tx, h - 3);
    ctx.fillStyle = 'rgba(233,240,248,.92)';
    ctx.fillText(label, tx, h - 3);
  }
  ctx.lineWidth = 1;
}

/* ======================================================= osciloscopio === */
function drawScope() {
  const cv = $('cv-scope'); const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);
  const g = { x0: 44, x1: w - 8, y0: 10, y1: h - 18 };
  const sc = S.scope;

  let amp = 1;
  if (sc) for (let i = 0; i < sc.pts; i++) amp = Math.max(amp, Math.abs(sc.raw[i]), Math.abs(sc.filt[i]));
  amp = niceCeil(amp * 1.1);

  ctx.font = '10px ui-monospace,monospace';
  ctx.strokeStyle = '#1b2534'; ctx.fillStyle = '#6f819a';
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (const v of ticks(-amp, amp, 4)) {
    const y = Math.round(g.y1 - ((v + amp) / (2 * amp)) * (g.y1 - g.y0)) + 0.5;
    ctx.beginPath(); ctx.moveTo(g.x0, y); ctx.lineTo(g.x1, y); ctx.stroke();
    ctx.fillText(Math.abs(v) < 1e-9 ? '0' : v.toFixed(Math.abs(v) >= 10 ? 0 : 1), g.x0 - 6, y);
  }
  ctx.textAlign = 'left'; ctx.fillStyle = '#4d6076';
  ctx.fillText('mg', 6, g.y0 + 2);

  if (!sc) { placeholder(ctx, g, 'sem dados'); return; }

  const line = (arr, color, width) => {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
    for (let i = 0; i < sc.pts; i++) {
      const x = g.x0 + (i / (sc.pts - 1)) * (g.x1 - g.x0);
      const y = g.y1 - ((arr[i] + amp) / (2 * amp)) * (g.y1 - g.y0);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
  };
  line(sc.raw, 'rgba(34,211,238,.55)', 1);
  line(sc.filt, '#fbbf24', 1.4);
}

/* ============================================================= picos ==== */
// Junta a deteccao nova com a anterior: um pico que reaparece perto da
// mesma frequencia e o MESMO pico, so atualiza. Os que somem ficam na lista
// ate PEAK_HOLD_MS, o que estabiliza a leitura sem esconder que sumiram
// (a linha vai apagando).
// Descarta o que passou do tempo de retencao. Roda tambem no desenho: se o
// fluxo de dados parar, a lista precisa esvaziar sozinha em vez de congelar.
function prunePeaks() {
  const now = performance.now();
  S.peaksHeld = S.peaksHeld.filter((q) => now - q.t < PEAK_HOLD_MS);
  S.peaksHeld.sort((x, y) => y.a - x.a);
  if (S.peaksHeld.length > 8) S.peaksHeld.length = 8;
  S.peaks = S.peaksHeld.slice(0, 5);
}

function mergePeaks(incoming) {
  const now = performance.now();
  // tolerancia de dois bins: abaixo disso e a mesma raia espectral
  const tol = S.spec ? Math.max(2 * S.spec.binHz, 1.0) : 3;

  for (const p of incoming || []) {
    const m = S.peaksHeld.find((q) => Math.abs(q.f - p.f) <= tol);
    if (m) { m.f = p.f; m.a = p.a; m.t = now; }
    else S.peaksHeld.push({ f: p.f, a: p.a, t: now });
  }
  prunePeaks();
}

function renderPeaks() {
  prunePeaks();
  const tb = $('peak-rows');
  if (!S.peaks.length) {
    tb.innerHTML = '<tr class="empty"><td colspan="4">nenhum pico acima do ruído</td></tr>';
    return;
  }
  const now = performance.now();
  tb.innerHTML = S.peaks.slice(0, 5).map((p, i) => `
    <tr class="${p.t !== undefined && now - p.t > PEAK_HOLD_MS / 2 ? 'fading' : ''}">
      <td>${i + 1}</td>
      <td class="f">${p.f.toFixed(2)} Hz</td>
      <td>${fmtMg(p.a)} mg</td>
      <td><div class="pk-btns"><button class="mini" data-add="1" data-freq="${p.f}"
        title="adicionar a lista de oscilacoes">+ filtrar</button></div></td>
    </tr>`).join('');
  tb.querySelectorAll('button[data-add]').forEach((b) => {
    b.onclick = () => { if (tgAdd(+b.dataset.freq)) tgApply(); };
  });
}

/* ======================================================== oscilacoes ==== */
/* A entrada do painel deixa de ser "centro e corte de cada notch" e passa a ser
   "quais frequencias incomodam". O alocador decide onde cada alvo cai: familia
   harmonica -> filtro RPM, alvo fixo -> notch 1/2, alvo que varia -> notch
   dinamico. Centro, corte e Q continuam la embaixo, agora derivados. */

const TG = { list: [], q: 3, drift: false, seeded: false };
const TG_QS = [5, 3, 1.5];                 // estreita, media, larga
const TG_QWORDS = ['estreita', 'média', 'larga'];
// o chip mostra a largura por nome; o valor exato fica no nivel avancado
const qWord = (q) => TG_QWORDS[TG_QS.reduce(
  (p, x, i) => (Math.abs(x - q) < Math.abs(TG_QS[p] - q) ? i : p), 0)];
const RPM_Q_MIN = 2.5;                     // o firmware clampa rpm_filter_q em 250

// Corte inferior do notch a partir do Q, invertendo filterGetNotchQ()
const cutoffForQ = (f0, q) => f0 * (Math.sqrt(1 + 4 * q * q) - 1) / (2 * q);

function tgAdd(freq, drift, q) {
  const f = +(+freq).toFixed(1);
  if (!(f > 0) || !S.cfg || f >= S.cfg.odr * 0.5) return false;
  if (TG.list.some((t) => Math.abs(t.f - f) < 0.5)) return false;
  if (TG.list.length >= 6) return false;
  TG.list.push({ f, q: q || TG.q, drift: !!drift });
  TG.list.sort((a, b) => a.f - b.f);
  return true;
}

// Na primeira carga a lista vem da config que ja esta na placa; senao o painel
// mostraria "nenhuma oscilacao" com os notches ligados.
function tgSeedFromCfg(c) {
  if (TG.seeded) return;
  TG.seeded = true;
  if (c.rpmSrc === 1 && c.rpmHarm > 0 && c.rpmBase > 0) {
    for (let h = 1; h <= c.rpmHarm; h++) tgAdd(c.rpmBase * h, false, c.rpmQ / 100);
  }
  [[c.n1h, c.n1c], [c.n2h, c.n2c]].forEach((pair) => {
    const q = filterGetNotchQ(pair[0], pair[1]);
    if (pair[0] > 0 && q > 0) tgAdd(pair[0], false, +q.toFixed(1));
  });
}

// Alvos -> blocos da cadeia, sem efeito colateral. Quem escreve e tgWrite().
// Separado porque a UI recalcula o plano a cada quadro para mostrar o resumo,
// e recalcular nao pode reescrever a config por baixo de um ajuste manual.
function tgPlan() {
  const c = S.cfg; if (!c) return null;
  const fixed = TG.list.filter((t) => !t.drift);
  const drift = TG.list.filter((t) => t.drift);

  // familia harmonica: 2x e 3x da fundamental, +/-3% -> um filtro RPM so
  let rpm = null, used = [];
  for (const base of fixed) {
    const harm = [base];
    for (let h = 2; h <= 3; h++) {
      const want = base.f * h;
      const m = fixed.find((t) => t !== base && !harm.includes(t) &&
                                  Math.abs(t.f - want) / want < 0.03);
      if (!m) break;
      harm.push(m);
    }
    if (harm.length >= 2) {
      rpm = { base: base.f, harm: harm.length, q: Math.max(base.q, RPM_Q_MIN) };
      used = harm;
      break;
    }
  }

  const rest = fixed.filter((t) => !used.includes(t));
  const notches = rest.slice(0, 2);
  const overflow = (rest.length - notches.length) + Math.max(0, drift.length - 3);

  const dyn = drift.slice(0, 3);
  const nyq = c.odr * 0.5;
  const dynRange = dyn.length
    ? { min: Math.min(250, Math.max(20, Math.round(dyn[0].f * 0.7))),
        max: Math.min(nyq, Math.max(100, Math.round(dyn[dyn.length - 1].f * 1.4))),
        q: Math.round(Math.max.apply(null, dyn.map((t) => t.q)) * 100) }
    : null;

  return { rpm: rpm, notches: notches, dyn: dyn, range: dynRange, overflow: overflow };
}

// Escreve o plano na config. So roda quando a lista de oscilacoes muda - nunca
// no caminho de render, senao apagaria um ajuste manual do painel avancado.
function tgWrite(plan) {
  const c = S.cfg; if (!c || !plan) return;

  if (plan.rpm) {
    c.rpmSrc = 1; c.rpmBase = plan.rpm.base; c.rpmHarm = plan.rpm.harm;
    c.rpmQ = Math.round(plan.rpm.q * 100);
  } else if (c.rpmSrc === 1) {
    c.rpmSrc = 0;                          // so desliga o que o alocador ligou
  }

  [0, 1].forEach((i) => {
    const t = plan.notches[i];
    c['n' + (i + 1) + 'h'] = t ? t.f : 0;
    c['n' + (i + 1) + 'c'] = t ? +cutoffForQ(t.f, t.q).toFixed(1) : 0;
  });

  if (plan.range) {
    c.dnCount = plan.dyn.length;
    c.dnQ = plan.range.q;
    c.dnMin = plan.range.min;
    c.dnMax = plan.range.max;
  }
}

// Chamado sempre que a lista muda: aloca, atualiza a UI e manda para a placa
function tgApply() {
  tgWrite(tgPlan());
  applyConfig(S.cfg);
  sendFilters();
}

function renderTargets(plan) {
  const box = $('tg-box'), inp = $('tg-in');
  box.querySelectorAll('.chip').forEach((e) => e.remove());

  TG.list.forEach((t, i) => {
    const el = document.createElement('span');
    el.className = 'chip' + (t.drift ? ' drift' : '');

    const lbl = document.createElement('span');
    lbl.textContent = t.f.toFixed(1) + ' Hz';
    el.appendChild(lbl);

    const q = document.createElement('span');
    q.className = 'q';
    q.textContent = qWord(t.q);
    q.title = 'clique para trocar a largura';
    q.onclick = () => {
      const k = TG_QS.indexOf(t.q);
      t.q = TG_QS[(k < 0 ? 1 : k + 1) % TG_QS.length];
      tgApply();
    };
    el.appendChild(q);

    const dr = document.createElement('button');
    dr.type = 'button';
    dr.textContent = '~';
    dr.title = t.drift ? 'fixar a frequencia' : 'a frequencia varia (notch dinamico)';
    dr.setAttribute('aria-label', dr.title);
    dr.onclick = () => { t.drift = !t.drift; tgApply(); };
    el.appendChild(dr);

    const rm = document.createElement('button');
    rm.type = 'button';
    rm.textContent = '×';
    rm.title = 'remover';
    rm.setAttribute('aria-label', 'remover ' + t.f + ' Hz');
    rm.onclick = () => { TG.list.splice(i, 1); tgApply(); };
    el.appendChild(rm);

    box.insertBefore(el, inp);
  });

  const lines = [];
  if (plan) {
    if (plan.rpm) {
      lines.push('<b>' + plan.rpm.base.toFixed(1) + ' Hz</b> e mais ' + (plan.rpm.harm - 1) +
                 ' múltiplo(s): família do motor, seguida junto');
    }
    plan.notches.forEach((t) => {
      lines.push('<b>' + t.f.toFixed(1) + ' Hz</b>: cortada fixa, faixa ' + qWord(t.q));
    });
    if (plan.range) {
      lines.push('<b>' + plan.dyn.length + ' que varia(m)</b>: perseguida(s) entre ' +
                 plan.range.min + ' e ' + plan.range.max + ' Hz');
    }
    if (plan.overflow) {
      lines.push('<span class="warn">' + plan.overflow + ' alvo(s) sem lugar: só cabem 2 cortes ' +
                 'fixos. Marque um com “~” para o rastreio automático pegar.</span>');
    }
  }
  const el = $('tg-plan');
  el.classList.toggle('idle', !lines.length);
  el.innerHTML = lines.length ? lines.join('<br>') : 'nenhuma oscilacao na lista';
}

function renderTracking() {
  const c = S.cfg;
  renderCost();
  const dyn = $('dyn-track');
  const onDyn = c && c.dnCount > 0 && S.dyn.length;
  dyn.classList.toggle('idle', !onDyn);
  dyn.textContent = onDyn
    ? 'rastreando  ' + S.dyn.slice(0, c.dnCount).map((f) => f.toFixed(1)).join('  ·  ') + '  Hz'
    : (c && c.dnCount > 0 ? 'aguardando espectro…' : 'desligado');

  const rpm = $('rpm-track');
  const onRpm = c && c.rpmSrc > 0 && S.rpm.length;
  rpm.classList.toggle('idle', !onRpm);
  rpm.textContent = onRpm
    ? S.rpm.slice(0, c.rpmHarm).map((h, i) =>
        `H${i + 1} ${h.f.toFixed(1)}Hz ${(h.w * 100).toFixed(0)}%`).join('  ·  ')
    : (c && c.rpmSrc > 0 ? 'aguardando fundamental…' : 'desligado');

  const l1 = $('l1-track');
  if (c && c.lpf1Dyn) {
    const cut = S.lpf1Cut || c.lpf1DynMin * mult();
    l1.classList.remove('idle');
    l1.textContent = `corte atual  ${cut.toFixed(0)} Hz`;
  } else {
    l1.classList.add('idle');
    l1.textContent = '—';
  }
}

/* ======================================================== configuracao == */
const FILTER_MAP = {
  'f-mult': 'mult', 'f-dc': 'dc', 'f-dcf': 'dcf',
  'f-lpf1t': 'lpf1Type', 'f-lpf1f': 'lpf1Hz', 'f-lpf1dyn': 'lpf1Dyn',
  'f-l1dmin': 'lpf1DynMin', 'f-l1dmax': 'lpf1DynMax', 'f-l1dexp': 'lpf1DynExpo',
  'f-lpf2t': 'lpf2Type', 'f-lpf2f': 'lpf2Hz',
  'f-n1h': 'n1h', 'f-n1c': 'n1c', 'f-n2h': 'n2h', 'f-n2c': 'n2c',
  'f-dnc': 'dnCount', 'f-dnq': 'dnQ', 'f-dnmin': 'dnMin', 'f-dnmax': 'dnMax',
  'f-rpmsrc': 'rpmSrc', 'f-rpmbase': 'rpmBase', 'f-rpmharm': 'rpmHarm',
  'f-rpmq': 'rpmQ', 'f-rpmmin': 'rpmMin', 'f-rpmfade': 'rpmFade', 'f-rpmlpf': 'rpmLpf',
};
const RPM_W_IDS = ['f-rpmw0', 'f-rpmw1', 'f-rpmw2'];

function applyConfig(c) {
  S.cfg = c;
  tgSeedFromCfg(c);
  $('s-axis').value = c.axis;
  $('s-odr').value = String(Math.round(c.odr));
  $('s-range').value = String(c.rangeG);
  $('s-avg').value = String(c.avg);
  $('s-bw').value = String(c.bw);
  $('s-gyro').value = String(c.gyro ? 1 : 0);
  $('s-gyrrange').value = String(c.gyrRange);
  if (c.chan === undefined) c.chan = 0x07;
  if (c.attLpf !== undefined) $('s-attlpf').value = c.attLpf;
  if (c.attKp !== undefined) $('s-attkp').value = c.attKp;
  if (c.attSrc !== undefined) $('s-attsrc').value = String(c.attSrc);
  syncLegend();
  $('s-fft').value = String(c.fft);
  $('s-win').value = String(c.window);
  $('s-alpha').value = nearestOption('s-alpha', c.alpha);

  for (const [id, key] of Object.entries(FILTER_MAP)) {
    const el = $(id);
    if (el.type === 'checkbox') el.checked = !!c[key];
    else el.value = c[key];
  }
  RPM_W_IDS.forEach((id, i) => { $(id).value = c.rpmW[i]; });
  $('f-load').value = Math.round((c.load || 0) * 100);

  for (const [id, key] of Object.entries(SIM_MAP)) {
    const el = $(id);
    if (el.type === 'checkbox') el.checked = !!c[key];
    else el.value = c[key];
  }
  buildSimUI();
  c.simSrc.forEach((o, i) => {
    $(`s${i}en`).checked = !!o.en;
    $(`s${i}f`).value = o.f;  $(`s${i}rpm`).value = Math.round(o.f * 60);
    $(`s${i}a`).value = o.a;  $(`s${i}h`).value = String(o.h);
    $(`s${i}d`).value = o.d;  $(`s${i}dr`).value = o.dr;
    $(`s${i}dp`).value = o.dp; $(`s${i}jt`).value = o.jt;
    $(`s${i}ax`).value = String(o.ax);
  });

  const nyq = Math.round(c.odr / 2);
  ['f-lpf1f', 'f-lpf2f', 'f-n1h', 'f-n1c', 'f-n2h', 'f-n2c', 'f-dnmax',
   'f-l1dmin', 'f-l1dmax', 'f-rpmbase', 'sim-resf'].forEach((id) => { $(id).max = nyq; });
  for (let i = 0; i < SIM_SOURCES; i++) $(`s${i}f`).max = nyq;

  syncFilterUI();
  syncSimUI();
  showManualIfUnexplained();
}

// O nivel avancado fica recolhido, mas nao pode esconder ajuste manual: se a
// config que chegou tem bloco ligado que a lista de oscilacoes nao explica, a
// gaveta abre uma vez. O notch dinamico nos valores de fabrica nao conta - ele
// vem ligado de serie e abriria a gaveta sempre.
/* ------------------------------------------------------------------ abas
   A propria tira da cadeia e a navegacao: cada estagio e uma aba, e abrir uma
   mostra so as opcoes daquele filtro. A lista de oscilacoes e a intensidade
   ficam FORA das abas, sempre visiveis - sao o caminho curto, e escondê-las
   atras de um clique desfaria a simplificacao.

   Cuidado com os dois estados do chip, que sao independentes: ".on" diz que o
   estagio esta agindo no sinal, aria-selected diz qual esta aberto. */
const FSTAGES = {
  'st-dc': 'g-dc', 'st-lpf2': 'g-lpf2', 'st-rpm': 'g-rpm', 'st-n1': 'g-n1',
  'st-n2': 'g-n2', 'st-lpf1': 'g-lpf1', 'st-dyn': 'g-dyn',
};
// estagios que a lista de oscilacoes escreve: merecem o aviso de sobrescrita
const FOWNED = ['st-rpm', 'st-n1', 'st-n2', 'st-dyn'];

function setFilterTab(tab, remember) {
  if (!FSTAGES[tab]) tab = 'st-lpf1';
  for (const [t, g] of Object.entries(FSTAGES)) {
    const on = t === tab;
    $(t).setAttribute('aria-selected', String(on));
    $(g).hidden = !on;
  }
  $('tg-owned').hidden = !FOWNED.includes(tab);
  // aba escolhida sobrevive ao reload; em aba anonima o storage joga excecao
  if (remember !== false) { try { localStorage.setItem('bmi-ftab', tab); } catch (e) { /* sem storage */ } }
}

// Se a placa chegar com corte que a lista de oscilacoes nao explica, o usuario
// precisa ver de onde ele veio: mostra a aba manual, uma vez so. O notch
// dinamico nos valores de fabrica nao conta - vem ligado de serie, e sem essa
// excecao a aba trocaria em toda carga. Nao grava a preferencia: foi decisao
// do programa, nao do usuario.
let advOpened = false;
function showManualIfUnexplained() {
  const c = S.cfg;
  if (advOpened || !c) return;
  const plan = tgPlan(); if (!plan) return;
  const D = DEFAULT_CFG;
  const dynStock = c.dnCount === D.dnCount && c.dnQ === D.dnQ &&
                   c.dnMin === D.dnMin && c.dnMax === D.dnMax;
  const manual = (c.rpmSrc > 0 && c.rpmHarm > 0 && !plan.rpm) ||
                 (c.n1h > 0 && !plan.notches[0]) ||
                 (c.n2h > 0 && !plan.notches[1]) ||
                 (c.dnCount > 0 && !plan.range && !dynStock);
  // abre justamente o estagio que a lista nao explica, para o usuario ver de onde veio
  let culpado = null;
  if (c.rpmSrc > 0 && c.rpmHarm > 0 && !plan.rpm) culpado = 'st-rpm';
  else if (c.n1h > 0 && !plan.notches[0]) culpado = 'st-n1';
  else if (c.n2h > 0 && !plan.notches[1]) culpado = 'st-n2';
  else if (c.dnCount > 0 && !plan.range && !dynStock) culpado = 'st-dyn';
  if (manual && culpado) { setFilterTab(culpado, false); advOpened = true; }
}

function nearestOption(id, v) {
  const opts = [...$(id).options].map((o) => +o.value);
  return String(opts.reduce((p, x) => (Math.abs(x - v) < Math.abs(p - v) ? x : p)));
}

// Le a UI -> S.cfg, atualiza rotulos e o diagrama da cadeia
function syncFilterUI() {
  const c = S.cfg; if (!c) return;
  for (const [id, key] of Object.entries(FILTER_MAP)) {
    const el = $(id);
    c[key] = el.type === 'checkbox' ? (el.checked ? 1 : 0) : +el.value;
  }
  c.rpmW = RPM_W_IDS.map((id) => +$(id).value);
  c.load = +$('f-load').value / 100;

  const set = (id, txt) => { $(id).textContent = txt; };
  set('o-mult', c.mult.toFixed(0));
  set('o-dcf', c.dcf.toFixed(1));
  set('o-lpf1f', c.lpf1Hz ? c.lpf1Hz.toFixed(0) : 'off');
  set('o-lpf2f', c.lpf2Hz ? c.lpf2Hz.toFixed(0) : 'off');
  set('o-l1dmin', c.lpf1DynMin.toFixed(0));
  set('o-l1dmax', c.lpf1DynMax.toFixed(0));
  set('o-l1dexp', c.lpf1DynExpo.toFixed(0));
  set('o-load', (c.load * 100).toFixed(0));
  set('o-n1h', c.n1h ? c.n1h.toFixed(1) : 'off');
  set('o-n1c', c.n1c.toFixed(1));
  set('o-n2h', c.n2h ? c.n2h.toFixed(1) : 'off');
  set('o-n2c', c.n2c.toFixed(1));
  set('o-dnmin', c.dnMin.toFixed(0));
  set('o-dnmax', c.dnMax.toFixed(0));
  set('o-dnq', 'Q ' + (c.dnQ / 100).toFixed(2));
  set('o-rpmbase', c.rpmBase.toFixed(1));
  set('o-rpmmin', c.rpmMin.toFixed(0));
  set('o-rpmfade', c.rpmFade.toFixed(0));
  set('o-rpmlpf', c.rpmLpf > 0 ? c.rpmLpf.toFixed(0) : 'off');
  set('o-rpmq', 'Q ' + (c.rpmQ / 100).toFixed(2));
  set('o-rpmw', c.rpmW.join('/'));

  const q1 = filterGetNotchQ(c.n1h, c.n1c), q2 = filterGetNotchQ(c.n2h, c.n2c);
  set('o-q1', q1 > 0 ? 'Q ' + q1.toFixed(2) : 'off');
  set('o-q2', q2 > 0 ? 'Q ' + q2.toFixed(2) : 'off');

  const rpmOn = c.rpmSrc > 0 && c.rpmHarm > 0;
  const on = {
    'st-dc': c.dc, 'st-lpf2': c.lpf2Hz > 0, 'st-rpm': rpmOn, 'st-n1': q1 > 0,
    'st-n2': q2 > 0, 'st-lpf1': c.lpf1Dyn || c.lpf1Hz > 0, 'st-dyn': c.dnCount > 0,
  };
  Object.entries(on).forEach(([id, v]) => $(id).classList.toggle('on', !!v));
  $('g-dc').classList.toggle('off', !c.dc);
  $('g-lpf1').classList.toggle('off', !(c.lpf1Dyn || c.lpf1Hz > 0));
  $('g-lpf1').classList.toggle('dyn', !!c.lpf1Dyn);
  $('g-lpf2').classList.toggle('off', !(c.lpf2Hz > 0));
  $('g-rpm').classList.toggle('off', !rpmOn);
  $('ctl-rpmbase').style.display = c.rpmSrc === 1 ? '' : 'none';
  $('g-n1').classList.toggle('on', q1 > 0);
  $('g-n2').classList.toggle('on', q2 > 0);
  $('g-dyn').classList.toggle('off', !(c.dnCount > 0));
  renderTargets(tgPlan());
  renderTracking();
}

// Filtrar custa atraso. Sem esse numero o multiplicador global e so uma
// porcentagem: com ele vira uma troca explicita entre limpeza e atraso.
function renderCost() {
  const c = S.cfg; if (!c) return;
  const ch = chain();
  const notches = ch.filter((st) => st.freq > 0).length;
  const cut = c.lpf1Dyn ? (S.lpf1Cut || c.lpf1DynMin * mult()) : c.lpf1Hz * mult();
  $('o-cost-cut').textContent = cut > 0 ? cut.toFixed(0) : 'off';
  $('o-cost-del').textContent = groupDelayMs(ch, c.odr).toFixed(2);
  $('o-cost-n').textContent = notches;
}

/* ------------------------------------------------------------- envio ---- */
// Um caminho so para configurar, escolhendo o transporte disponivel:
// serial > HTTP > previa local.
async function api(path, params) {
  const q = new URLSearchParams(params).toString();
  if (S.serialWriter) {
    // a resposta chega enquadrada e cai em applyConfig pelo parser
    await serialSend('api ' + path.replace(/^api\//, '') + ' ' + q);
    return S.cfg;
  }
  if (S.offline) return S.cfg;
  try {
    const r = await fetch(path + (q ? '?' + q : ''));
    const c = await r.json();
    if (c.odr !== undefined) applyConfig(c);
    return c;
  } catch (e) { return S.cfg; }
}

function sendFilters() {
  const c = S.cfg; if (!c) return;
  const p = {};
  for (const key of Object.values(FILTER_MAP)) p[key] = c[key];
  c.rpmW.forEach((v, i) => { p['rpmW' + i] = v; });
  p.load = c.load;
  api('api/filters', p);
}
const sendSensor = () => api('api/sensor', {
  odr: $('s-odr').value, range: $('s-range').value, avg: $('s-avg').value, bw: $('s-bw').value,
  gyro: $('s-gyro').value, gyrRange: $('s-gyrrange').value,
});
const sendAnalysis = () => api('api/analysis', {
  fft: $('s-fft').value, window: $('s-win').value, alpha: $('s-alpha').value,
  axis: $('s-axis').value, chan: S.cfg ? S.cfg.chan : 7,
});

/* --------------------------------------------------- digitar em vez de arrastar
   Todo output ao lado de um slider vira campo de texto no clique. E o unico
   jeito de cravar 118.4 Hz - o numero que o analisador acabou de medir e que
   slider nenhum alcanca. Enter confirma, Esc cancela, sair do campo confirma. */
// Um .ctl pode ter dois pares slider+output (drift/periodo do simulador), entao
// o par certo e sempre o slider imediatamente antes do output.
function sliderOf(out) {
  const prev = out.previousElementSibling;
  return (prev && prev.tagName === 'INPUT' && prev.type === 'range') ? prev : null;
}

// Chamado de novo depois que o simulador monta as fontes dele
function markEditable(root) {
  (root || document).querySelectorAll('.ctl > output:not([data-noedit])').forEach((out) => {
    if (!sliderOf(out)) return;
    out.classList.add('editable');
    out.title = 'clique para digitar';
  });
}

function wireTypeIn() {
  markEditable();
  document.addEventListener('click', (e) => {
    const out = e.target.closest('.ctl > output.editable');
    if (!out || out.dataset.editing) return;
    openTypeIn(out, sliderOf(out));
  });
}

function openTypeIn(out, rng) {
  out.dataset.editing = '1';
  out.hidden = true;

  const inp = document.createElement('input');
  inp.type = 'text';
  inp.className = 'numedit';
  inp.inputMode = 'decimal';
  inp.value = rng.value;
  inp.setAttribute('aria-label', 'valor');
  out.after(inp);
  inp.focus();
  inp.select();

  let done = false;
  const close = (commit) => {
    if (done) return;
    done = true;
    if (commit) {
      const v = parseFloat(inp.value.replace(',', '.'));
      if (!isNaN(v)) {
        rng.value = Math.min(Math.max(v, +rng.min), +rng.max);
        rng.dispatchEvent(new Event('input', { bubbles: true }));
        rng.dispatchEvent(new Event('change', { bubbles: true }));
      }
    }
    inp.remove();
    out.hidden = false;
    delete out.dataset.editing;
  };

  inp.onkeydown = (ev) => {
    if (ev.key === 'Enter') { ev.preventDefault(); close(true); }
    else if (ev.key === 'Escape') { ev.preventDefault(); close(false); }
  };
  inp.onblur = () => close(true);
}

/* =========================================================== controles == */
function wire() {
  wireTypeIn();

  wireScrub();
  $('bt-log-rec').onclick = logToggleRec;
  $('bt-log-save').onclick = logSave;
  $('bt-log-open').onclick = () => $('log-file').click();
  $('log-file').onchange = (e) => {
    const f = e.target.files && e.target.files[0];
    if (f) logLoad(f);
    e.target.value = '';        // permite reabrir o mesmo arquivo
  };
  $('log-pos').oninput = (e) => logSeek(+e.target.value);
  $('bt-log-close').onclick = logClose;

  Object.keys(FSTAGES).forEach((t) => { $(t).onclick = () => setFilterTab(t); });
  let startTab = 'st-lpf1';
  try { startTab = localStorage.getItem('bmi-ftab') || startTab; } catch (e) { /* sem storage */ }
  setFilterTab(startTab, false);

  // Enter aceita uma ou varias frequencias de uma vez: "118" ou "118 236 354"
  $('tg-in').addEventListener('keydown', (e) => {
    if (e.key !== 'Enter') return;
    e.preventDefault();
    let added = false;
    e.target.value.split(/[,;\s]+/).forEach((tok) => {
      const v = parseFloat(tok.replace(',', '.'));
      if (!isNaN(v) && tgAdd(v, $('tg-drift').checked)) added = true;
    });
    e.target.value = '';
    if (added) tgApply();
  });

  $('tg-width').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-q]');
    if (!b) return;
    TG.q = +b.dataset.q;
    $('tg-width').querySelectorAll('button').forEach((x) => {
      x.setAttribute('aria-pressed', String(x === b));
    });
  });
  $('in-fmax').onchange = (e) => { S.fmax = Math.max(10, +e.target.value || 800); wfImg = null; wfFresh = true; };
  $('bt-wf-clear').onclick = () => { wfImg = null; wfFresh = true; };

  [...Object.keys(FILTER_MAP), ...RPM_W_IDS, 'f-load'].forEach((id) => {
    const el = $(id);
    el.oninput = syncFilterUI;              // rotulos e curva respondem na hora
    el.onchange = () => { syncFilterUI(); sendFilters(); };
  });

  simInputIds().forEach((id) => {
    const el = $(id);
    el.oninput = syncSimUI;
    el.onchange = () => { syncSimUI(); sendSim(); };
  });
  $('sim-apply').onclick = () => {
    applyPreset($('sim-preset').value);
    applyConfig(S.cfg);
    sendSim();
  };

  ['s-odr', 's-range', 's-avg', 's-bw', 's-gyro', 's-gyrrange'].forEach((id) => $(id).onchange = sendSensor);

  ['s-attlpf', 's-attkp', 's-attsrc'].forEach((id) => {
    $(id).onchange = () => {
      if (S.cfg) S.cfg.attSrc = +$('s-attsrc').value;
      api('api/attitude', {
        attLpf: $('s-attlpf').value, attKp: $('s-attkp').value, attSrc: $('s-attsrc').value,
      });
    };
  });

  $('bt-att-cal').onclick = () => {
    api('api/attitude', { cal: 1 });
    $('a-rates').textContent = 'calibrando o giroscópio — deixe parado…';
  };
  $('bt-att-zero').onclick = () => {
    if (S.offline) { S.attYaw0 = (S.att ? S.att.y : 0); return; }
    api('api/attitude', { zero: 1 });
  };
  ['s-fft', 's-win', 's-alpha', 's-axis'].forEach((id) => $(id).onchange = sendAnalysis);

  $('bt-auto').onclick = () => {
    const ps = S.peaks.slice(0, 2);
    if (!ps.length) return;
    ps.forEach((pk) => tgAdd(pk.f));
    tgApply();
  };

  $('bt-clear-filters').onclick = () => {
    TG.list.length = 0;
    Object.assign(S.cfg, { n1h: 0, n1c: 0, n2h: 0, n2c: 0, dnCount: 0, rpmSrc: 0, lpf1Dyn: 0 });
    applyConfig(S.cfg);
    sendFilters();
  };

  $('bt-serial').onclick = connectSerial;

  $('bt-save').onclick = async () => {
    await api('api/save', {});
    $('bt-save').textContent = 'salvo ✓';
    setTimeout(() => ($('bt-save').textContent = 'Salvar na flash'), 1500);
  };

  $('bt-code').onclick = () => {
    const out = $('code-out');
    out.hidden = !out.hidden;
    if (!out.hidden) out.textContent = genCode();
  };

}

/* ============================================================== log ===== */
/* Gravar a sessao e reabrir depois. Fica tudo no navegador: a flash do ESP32
   mal cabe a interface, e o PC tem disco de sobra.

   Uma linha por mensagem de status (1 Hz) com os escalares, a leitura dos 6
   eixos e o espectro do canal em detalhe. 512 bins arredondados dao ~3 kB por
   linha, entao 10 min de gravacao ficam perto de 2 MB - grande, mas e o que
   permite reconstruir o waterfall inteiro depois. */

const LOG = {
  rec: false,          // gravando
  t0: 0,
  rows: [],
  head: null,          // config e escalas no inicio da gravacao
  open: null,          // log carregado: { head, rows }
  dur: 30,             // segundos da janela; 0 = ate o usuario parar
};

function logRecord(m) {
  if (!LOG.rec) return;
  if (!LOG.rows.length) {
    LOG.t0 = Date.now();
    LOG.head = {
      v: 1,
      inicio: new Date().toISOString(),
      cfg: S.cfg ? JSON.parse(JSON.stringify(S.cfg)) : null,
      fs: m.fs,
      bins: S.spec ? S.spec.bins : 0,
      binHz: S.spec ? S.spec.binHz : 0,
      axis: S.spec ? S.spec.axis : (S.cfg ? S.cfg.axis : 0),
    };
  }
  // espectro do canal em detalhe, arredondado: e o que vira waterfall na volta
  let spec = null;
  if (S.spec && S.spec.ch[S.spec.axis]) {
    const src = S.spec.ch[S.spec.axis];
    spec = new Array(src.length);
    for (let i = 0; i < src.length; i++) spec[i] = +src[i].toFixed(3);
  }
  LOG.rows.push({
    t: +((Date.now() - LOG.t0) / 1000).toFixed(2),
    fs: m.fs, meas: m.measured, rms: m.rmsRaw, rmsF: m.rmsFilt,
    pkpk: m.pkpk, temp: m.temp,
    lsb: m.lsbCh || [], dc: m.dcCh || [], acRms: m.rmsCh || [],
    peaks: (m.peaks || []).map((q) => ({ f: +q.f.toFixed(2), a: +q.a.toFixed(3) })),
    dyn: m.dyn || [], att: m.att || null,
    spec,
  });
  // janela fechada: para sozinho quando completa o tempo escolhido, para o
  // usuario nao ter que cronometrar nem lembrar de apertar parar
  if (LOG.dur > 0 && LOG.rows[LOG.rows.length - 1].t >= LOG.dur) {
    logToggleRec();
    $('log-stat').textContent =
      'janela de ' + fmtDur(LOG.dur) + ' completa · ' + LOG.rows.length + ' amostras';
    return;
  }
  logStat();
}

function logStat() {
  const n = LOG.rows.length;
  if (LOG.rec) {
    const seg = n ? LOG.rows[n - 1].t : 0;
    // estimativa: serializar tudo a cada segundo so para medir sairia caro.
    // 3350 B/linha com espectro de 512 bins, 330 B sem - medido, nao chutado.
    const mb = (n * (LOG.rows[0] && LOG.rows[0].spec ? 3350 : 330)) / 1048576;
    const alvo = LOG.dur > 0 ? ' / ' + fmtDur(LOG.dur) : '';
    $('log-stat').textContent = 'gravando  ' + fmtDur(seg) + alvo +
      '  ·  ' + n + ' amostras  ·  ~' + mb.toFixed(1) + ' MB';
  } else if (n) {
    $('log-stat').textContent = n + ' amostras gravadas (' + fmtDur(LOG.rows[n - 1].t) + ')';
  } else {
    $('log-stat').textContent = 'nenhum log gravado';
  }
  $('bt-log-save').disabled = !n;
}

const fmtDur = (s) => {
  const m = Math.floor(s / 60), r = Math.floor(s % 60);
  return m + ':' + String(r).padStart(2, '0');
};

function logToggleRec() {
  LOG.rec = !LOG.rec;
  if (LOG.rec) {
    LOG.rows = []; LOG.head = null;
    LOG.dur = +$('log-dur').value || 0;
    if (LOG.open) logClose();      // gravar e reproduzir ao mesmo tempo confunde
  }
  $('bt-log-rec').textContent = LOG.rec ? 'Parar gravação' : 'Iniciar log';
  $('bt-log-rec').classList.toggle('rec', LOG.rec);
  logStat();
}

function logSave() {
  if (!LOG.rows.length) return;
  const doc = Object.assign({}, LOG.head, { rows: LOG.rows });
  const blob = new Blob([JSON.stringify(doc)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'bmi323-' + (LOG.head.inicio || '').replace(/[:.]/g, '-').slice(0, 19) + '.json';
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}

function logLoad(file) {
  const fr = new FileReader();
  fr.onload = () => {
    let doc;
    try { doc = JSON.parse(fr.result); } catch (e) { doc = null; }
    if (!doc || !Array.isArray(doc.rows) || !doc.rows.length) {
      $('log-stat').textContent = 'arquivo não é um log válido deste analisador';
      return;
    }
    logOpen(doc, file.name);
  };
  fr.readAsText(file);
}

function logOpen(doc, nome) {
  LOG.open = doc;
  if (LOG.rec) logToggleRec();

  $('logbar').hidden = false;
  $('cv-raw').classList.add('scrub');
  $('log-name').textContent = nome + '  ·  ' + doc.rows.length + ' amostras';
  $('log-pos').max = doc.rows.length - 1;
  $('log-pos').value = 0;
  $('raw-title').innerHTML = 'Tendência do log <small id="raw-span"></small>';
  $('raw-hint').textContent = 'RMS e pico dominante ao longo da gravação';

  // o waterfall vira o espectrograma da sessao inteira: empurra tudo de uma vez
  wfImg = null; wfFresh = true;
  doc.rows.forEach((r) => {
    if (!r.spec) return;
    S.spec = { bins: r.spec.length, binHz: doc.binHz, fs: doc.fs,
               mask: 1 << doc.axis, axis: doc.axis, ch: [], filt: null };
    S.spec.ch[doc.axis] = r.spec;
    pushWaterfall();
  });

  logSeek(0);
}

function logClose() {
  LOG.open = null;
  $('logbar').hidden = true;
  $('cv-raw').classList.remove('scrub');
  $('raw-title').innerHTML = 'Leitura da IMU <small id="raw-span"></small>';
  $('raw-hint').textContent = 'valor que sai do sensor, sem filtro';
  wfImg = null; wfFresh = true;
}

// Leva a tela para o instante escolhido do log
function logSeek(i) {
  const doc = LOG.open; if (!doc) return;
  const r = doc.rows[Math.max(0, Math.min(doc.rows.length - 1, i))];
  if (!r) return;

  $('log-t').textContent = fmtDur(r.t) + ' / ' + fmtDur(doc.rows[doc.rows.length - 1].t);
  $('m-fs').textContent = r.fs.toFixed(0);
  $('m-meas').textContent = r.meas.toFixed(0);
  $('m-rms').textContent = r.rms.toFixed(1);
  $('m-rmsf').textContent = r.rmsF.toFixed(1);
  $('m-pkpk').textContent = r.pkpk.toFixed(0);
  $('m-temp').textContent = r.temp.toFixed(1);

  S.lsbCh = r.lsb; S.dcCh = r.dc; S.rmsCh = r.acRms;
  renderAxes();

  // renderPeaks() passa por prunePeaks(), que reescreve S.peaks a partir de
  // S.peaksHeld - entao os picos do log entram por la, com carimbo novo
  const agora = performance.now();
  S.peaksHeld = r.peaks.map((q) => ({ f: q.f, a: q.a, t: agora }));
  renderPeaks();

  S.dyn = r.dyn || [];
  S.att = r.att || null;
  renderAttitude();

  if (r.spec) {
    S.spec = { bins: r.spec.length, binHz: doc.binHz, fs: doc.fs,
               mask: 1 << doc.axis, axis: doc.axis, ch: [], filt: null };
    S.spec.ch[doc.axis] = r.spec;
  }
}

/* Tendencia: o que aconteceu ao longo da gravacao. Tres faixas - RMS cru,
   RMS filtrado e frequencia do pico dominante - com a linha do instante
   selecionado por cima. E a vista que responde "quando piorou?". */
function drawTrend() {
  const doc = LOG.open;
  const cv = $('cv-raw'); const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);
  if (!doc) return;

  const rows = doc.rows;
  const tMax = rows[rows.length - 1].t || 1;
  const series = [
    { nome: 'rms cru', cor: '#22d3ee', un: 'mg', get: (r) => r.rms },
    { nome: 'rms filtrado', cor: '#fbbf24', un: 'mg', get: (r) => r.rmsF },
    { nome: 'pico', cor: '#f472b6', un: 'Hz', get: (r) => (r.peaks[0] ? r.peaks[0].f : 0) },
  ];

  const x0 = 58, x1 = w - 8, y0 = 4, y1 = h - 16;
  const laneH = (y1 - y0) / series.length;
  const xOf = (t) => x0 + (t / tMax) * (x1 - x0);
  ctx.font = '9px ui-monospace,monospace';

  series.forEach((sr, k) => {
    const top = y0 + laneH * k, cy = top + laneH / 2, half = laneH / 2 - 4;
    let amp = 1e-6;
    rows.forEach((r) => { amp = Math.max(amp, sr.get(r)); });
    amp = niceCeil(amp * 1.1);

    ctx.strokeStyle = '#1b2534'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x0, Math.round(top + laneH) + 0.5); ctx.lineTo(x1, Math.round(top + laneH) + 0.5);
    ctx.stroke();

    ctx.strokeStyle = sr.cor; ctx.lineWidth = 1.3; ctx.beginPath();
    rows.forEach((r, i) => {
      const x = xOf(r.t), y = top + laneH - 3 - (sr.get(r) / amp) * (half * 2 - 3);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();

    ctx.fillStyle = sr.cor; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText(sr.nome, 5, cy - 5);
    ctx.fillStyle = '#5b6c81';
    ctx.fillText('máx ' + (amp >= 100 ? amp.toFixed(0) : amp.toFixed(1)) + ' ' + sr.un, 5, cy + 6);
  });

  // instante selecionado
  const r = rows[+$('log-pos').value];
  if (r) {
    const x = Math.round(xOf(r.t)) + 0.5;
    ctx.strokeStyle = 'rgba(244,114,182,.7)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke();
  }

  ctx.fillStyle = '#4d6076'; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  for (const t of ticks(0, tMax, 6)) ctx.fillText(fmtDur(t), xOf(t), y1 + 3);
}

/* Arrastar no grafico de tendencia move o instante. O slider continua ali,
   mas ninguem procura um slider quando esta olhando a curva - a mao vai no
   grafico. As setas do teclado fazem o passo fino. */
function logScrubAt(clientX) {
  const doc = LOG.open; if (!doc) return;
  const cv = $('cv-raw'), r = cv.getBoundingClientRect();
  const x0 = 58, x1 = r.width - 8;                 // mesmas margens de drawTrend()
  const f = (clientX - r.left - x0) / Math.max(1, x1 - x0);
  const tMax = doc.rows[doc.rows.length - 1].t || 1;
  const alvo = Math.max(0, Math.min(1, f)) * tMax;

  // a linha do log e por tempo, nao por indice: acha a mais proxima
  let melhor = 0, dist = Infinity;
  doc.rows.forEach((row, i) => {
    const d = Math.abs(row.t - alvo);
    if (d < dist) { dist = d; melhor = i; }
  });
  $('log-pos').value = melhor;
  logSeek(melhor);
}

function logStep(delta) {
  const doc = LOG.open; if (!doc) return;
  const i = Math.max(0, Math.min(doc.rows.length - 1, +$('log-pos').value + delta));
  $('log-pos').value = i;
  logSeek(i);
}

function wireScrub() {
  const cv = $('cv-raw');
  let arrastando = false;

  cv.addEventListener('pointerdown', (e) => {
    if (!LOG.open) return;
    arrastando = true;
    cv.setPointerCapture(e.pointerId);
    logScrubAt(e.clientX);
  });
  cv.addEventListener('pointermove', (e) => {
    if (arrastando) logScrubAt(e.clientX);
  });
  const solta = (e) => {
    if (!arrastando) return;
    arrastando = false;
    try { cv.releasePointerCapture(e.pointerId); } catch (err) { /* ja soltou */ }
  };
  cv.addEventListener('pointerup', solta);
  cv.addEventListener('pointercancel', solta);

  // setas so respondem quando nao se esta digitando num campo
  document.addEventListener('keydown', (e) => {
    if (!LOG.open) return;
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
    const alvo = e.target;
    if (alvo && /^(INPUT|SELECT|TEXTAREA)$/.test(alvo.tagName)) return;
    e.preventDefault();
    logStep(e.key === 'ArrowRight' ? 1 : -1);
  });
}

/* ===================================================== exportar codigo == */
function genCode() {
  const c = S.cfg;
  // notches dinamicos e RPM mudam de coeficiente em tempo real: nao entram
  // no codigo estatico, so ficam registrados no comentario.
  const ch = chain().filter((s) => !s.dyn && !s.rpm);
  if (!ch.length) return '// nenhum filtro estatico ativo';

  let s = `// Cadeia de filtros gerada pelo analisador BMI323\n`;
  s += `// Formulas do Betaflight (src/main/common/filter.c)\n`;
  s += `// fs = ${c.odr.toFixed(2)} Hz   eixo ${'XYZ|'[c.axis]}   multiplicador ${c.mult}%\n`;
  s += `// ordem: ${ch.map((x) => x.name).join(' -> ')}\n`;
  if (c.rpmSrc > 0 && c.rpmHarm > 0) {
    s += `// + filtro RPM: ${c.rpmHarm} harmonica(s), Q ${(c.rpmQ / 100).toFixed(2)}, ` +
         `min ${c.rpmMin} Hz, fade ${c.rpmFade} Hz, suav ${c.rpmLpf > 0 ? c.rpmLpf + ' Hz' : 'off'}, pesos ${c.rpmW.join('/')}\n`;
  }
  if (c.dnCount > 0) {
    s += `// + notch dinamico: ${c.dnCount} notch(es), Q ${(c.dnQ / 100).toFixed(2)}, ` +
         `${c.dnMin}-${c.dnMax} Hz (SDFT)\n`;
  }
  s += `\n`;

  s += `typedef struct { float state, k; } pt1_t;\n`;
  s += `typedef struct { float b0,b1,b2,a1,a2, x1,x2,y1,y2; } biquad_t;\n\n`;
  s += `static inline float pt1_apply(pt1_t *f, float x){ f->state += f->k * (x - f->state); return f->state; }\n`;
  s += `static inline float biquad_df1(biquad_t *f, float x){\n`;
  s += `  float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2;\n`;
  s += `  f->x2 = f->x1; f->x1 = x; f->y2 = f->y1; f->y1 = y; return y;\n}\n\n`;

  const decl = [], body = [];
  ch.forEach((st) => {
    if (st.kind === 'dc') {
      decl.push(`static pt1_t ${st.name} = { 0.0f, ${st.k.toFixed(9)}f };   // ${st.hz} Hz`);
      body.push(`  x = x - pt1_apply(&${st.name}, x);   // DC-block`);
    } else if (st.kind.startsWith('pt')) {
      for (let i = 0; i < st.n; i++) decl.push(`static pt1_t ${st.name}_${i} = { 0.0f, ${st.k.toFixed(9)}f };`);
      body.push(`  // ${st.name}: ${LPF_NAMES[st.type]} @ ${st.hz.toFixed(1)} Hz`);
      for (let i = 0; i < st.n; i++) body.push(`  x = pt1_apply(&${st.name}_${i}, x);`);
    } else {
      const b = st.bq;
      decl.push(`static biquad_t ${st.name} = { ${b.b0.toFixed(9)}f, ${b.b1.toFixed(9)}f, ${b.b2.toFixed(9)}f, ` +
                `${b.a1.toFixed(9)}f, ${b.a2.toFixed(9)}f, 0,0,0,0 };` +
                (st.freq ? `  // ${st.freq.toFixed(1)} Hz, Q ${st.q.toFixed(2)}`
                         : `  // LPF ${st.hz.toFixed(1)} Hz`));
      body.push(`  x = biquad_df1(&${st.name}, x);`);
    }
  });

  s += decl.join('\n') + `\n\n`;
  s += `float filtros_apply(float x){\n` + body.join('\n') + `\n  return x;\n}\n`;
  return s;
}

/* ============================================ leitura dos 6 eixos ======= */
// A leitura em si, sem estatistica nenhuma no meio: lsbCh e o ultimo quadro
// que saiu do FIFO do BMI323, contagem de 16 bits, do jeito que veio pelo
// barramento. Abaixo dela o mesmo numero convertido pelo fundo de escala em
// vigor (lsbG / lsbDps, que a placa informa no /api/config).
//
// Nao passa por filtro, por media nem por FFT - se aparecer valor estranho
// aqui, o problema e do sensor ou do barramento, nao do processamento.

// Contagem -> unidade de engenharia, com o fundo de escala corrente
function lsbToUnit(lsb, gyro) {
  const c = S.cfg; if (!c) return null;
  const k = gyro ? c.lsbDps : c.lsbG;
  return k > 0 ? lsb / k : null;
}

function buildAxes() {
  $('axes-row').innerHTML = CH.map((c, i) =>
    '<div class="ax" id="ax' + i + '" style="--c:' + c.color + '" title="' + c.label + '">' +
    '<b>' + c.id + '</b>' +
    '<span class="v" id="axr' + i + '">—</span>' +
    '<span class="dc" id="axd' + i + '">—</span></div>').join('');
}

function renderAxes() {
  for (let i = 0; i < 6; i++) {
    const lsb = S.lsbCh[i], gy = i >= 3;
    if (lsb === undefined) {
      $('axr' + i).textContent = '—';
      $('axd' + i).textContent = '—';
    } else {
      const u = lsbToUnit(lsb, gy);
      $('axr' + i).innerHTML = lsb + ' <i>LSB</i>';
      $('axd' + i).innerHTML = u === null ? '—'
        : gy ? u.toFixed(2) + ' <i>°/s</i>' : u.toFixed(4) + ' <i>g</i>';
    }
    // canal escondido no espectro continua sendo lido, so fica apagado
    $('ax' + i).classList.toggle('off', !(S.cfg && (S.cfg.chan & (1 << i))));
  }
}

/* ============================================== legenda / canais ======== */
// Os botoes da legenda mudam a mascara de canais NO FIRMWARE: canal escondido
// nao gasta FFT nem banda de rede.

function buildLegend() {
  const el = $('legend');
  el.innerHTML = CH.map((c, i) =>
    '<span class="k" id="leg' + i + '" data-ch="' + i + '" style="--c:' + c.color +
    '" title="' + c.label + '">' + c.id + '</span>').join('');

  el.querySelectorAll('[data-ch]').forEach((e) => {
    e.onclick = () => {
      const c = +e.dataset.ch;
      S.cfg.chan ^= (1 << c);
      if (!S.cfg.chan) S.cfg.chan = (1 << c);   // pelo menos um canal ligado
      syncLegend();
      sendAnalysis();
    };
  });
}

function syncLegend() {
  const m = S.cfg ? (S.cfg.chan | 0) : 0;
  CH.forEach((c, i) => $('leg' + i).classList.toggle('on', !!(m & (1 << i))));
}

/* =================================================== onda filtrada ====== */
// Mesmo desenho do painel de cima, so que depois da cadeia: comparar os dois
// e o jeito direto de ver o que o filtro tirou.

const drawFilteredScope = () =>
  drawLanes('cv-fscope', S.fscope, 'nenhum canal selecionado');

/* ============================================================ atitude === */
// roll e pitch vem do filtro complementar do firmware (giroscopio corrigido
// pela gravidade). yaw e integracao pura de gz: DERIVA, nao e rumo absoluto.

function renderAttitude() {
  const a = S.att;
  const set = (id, v, d) => { $(id).textContent = (a && a.ok !== undefined) ? v.toFixed(d) : '—'; };
  if (!a) { ['a-roll', 'a-pitch', 'a-yaw', 'a-g'].forEach((id) => { $(id).textContent = '—'; }); return; }
  set('a-roll', a.r, 1);
  set('a-pitch', a.p, 1);
  set('a-yaw', a.y, 1);
  set('a-g', a.g, 3);

  const t = a.trust === undefined ? 0 : a.trust;
  $('a-trustbar').style.width = (t * 100).toFixed(0) + '%';
  $('a-trust').textContent = (t * 100).toFixed(0) + '%';

  const el = $('a-rates');
  el.classList.toggle('cal', !!a.cal);
  el.classList.toggle('bad', !a.ok && !a.cal);
  if (a.cal) el.textContent = 'calibrando o giroscópio — deixe parado…';
  else if (!a.ok) el.textContent = 'giroscópio desligado · yaw indisponível';
  else {
    // deixa claro de onde saiu o ângulo: o acelerômetro cai para "cru" sozinho
    // quando o DC-block está ligado, porque senão não sobra gravidade
    const src = a.gftd ? (a.af ? 'giro+acel filtrados' : 'giro filtrado') : 'sinal cru';
    el.textContent = `gx ${a.gx.toFixed(1)}  gy ${a.gy.toFixed(1)}  gz ${a.gz.toFixed(1)} °/s` +
      (a.vib !== undefined ? `   vib ${a.vib.toFixed(0)} mg` : '') + `   · ${src}`;
  }
}

function drawAttitude() {
  const cv = $('cv-att'); const { ctx, w, h } = fitCanvas(cv);
  ctx.clearRect(0, 0, w, h);
  const a = S.att;

  // dois mostradores lado a lado; empilha se o painel ficar estreito
  const side = w > h * 1.5;
  const cell = side ? { cw: w / 2, ch: h } : { cw: w, ch: h / 2 };
  const LBL = 14;   // faixa reservada para a legenda embaixo de cada mostrador
  const R = Math.max(20, Math.min(cell.cw, cell.ch - LBL) / 2 - 6);
  const cyTop = (cell.ch - LBL) / 2;
  const hc = side ? { x: cell.cw / 2, y: cyTop } : { x: w / 2, y: cyTop };
  const yc = side ? { x: cell.cw * 1.5, y: cyTop } : { x: w / 2, y: cell.ch + cyTop };

  if (!a) {
    placeholder(ctx, { x0: 0, x1: w, y0: 0, y1: h }, 'sem dados de atitude');
    return;
  }

  drawHorizon(ctx, hc.x, hc.y, R, a.r, a.p);
  drawCompass(ctx, yc.x, yc.y, R, a.y, a.ok);
}

function drawHorizon(ctx, cx, cy, R, roll, pitch) {
  const pxPerDeg = R / 35;   // +/-35 graus cabem no mostrador
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 7); ctx.clip();

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-roll * Math.PI / 180);
  ctx.translate(0, pitch * pxPerDeg);

  const big = R * 2.4;
  ctx.fillStyle = '#12324a';                       // céu
  ctx.fillRect(-big, -big, big * 2, big);
  ctx.fillStyle = '#3d3220';                       // solo
  ctx.fillRect(-big, 0, big * 2, big);
  ctx.strokeStyle = '#dbe4ee'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(-big, 0); ctx.lineTo(big, 0); ctx.stroke();

  // escada de pitch
  ctx.font = '9px ui-monospace,monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  for (let d = -30; d <= 30; d += 10) {
    if (d === 0) continue;
    const y = -d * pxPerDeg;
    const half = d % 20 === 0 ? R * 0.34 : R * 0.2;
    ctx.strokeStyle = 'rgba(219,228,238,.55)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(-half, y); ctx.lineTo(half, y); ctx.stroke();
    // so escreve o numero se ele couber dentro do circulo do instrumento
    if (d % 20 === 0 && Math.hypot(half + 16, y) < R) {
      ctx.fillStyle = 'rgba(219,228,238,.6)';
      ctx.fillText(String(Math.abs(d)), half + 11, y);
    }
  }
  ctx.restore();
  ctx.restore();

  // moldura
  ctx.strokeStyle = '#2b3a4d'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 7); ctx.stroke();

  // arco de roll com marcas fixas
  ctx.strokeStyle = 'rgba(219,228,238,.35)';
  for (const d of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
    const ang = (-90 + d) * Math.PI / 180;
    const r1 = R, r2 = R - (d % 30 === 0 ? 8 : 5);
    ctx.beginPath();
    ctx.moveTo(cx + r1 * Math.cos(ang), cy + r1 * Math.sin(ang));
    ctx.lineTo(cx + r2 * Math.cos(ang), cy + r2 * Math.sin(ang));
    ctx.stroke();
  }
  // ponteiro do roll
  const pa = (-90 - roll) * Math.PI / 180;
  ctx.fillStyle = '#22d3ee';
  ctx.beginPath();
  ctx.moveTo(cx + (R - 2) * Math.cos(pa), cy + (R - 2) * Math.sin(pa));
  ctx.lineTo(cx + (R - 11) * Math.cos(pa - 0.055), cy + (R - 11) * Math.sin(pa - 0.055));
  ctx.lineTo(cx + (R - 11) * Math.cos(pa + 0.055), cy + (R - 11) * Math.sin(pa + 0.055));
  ctx.closePath(); ctx.fill();

  // simbolo fixo do corpo
  ctx.strokeStyle = '#fbbf24'; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx - R * 0.42, cy); ctx.lineTo(cx - R * 0.14, cy);
  ctx.moveTo(cx + R * 0.14, cy); ctx.lineTo(cx + R * 0.42, cy);
  ctx.moveTo(cx, cy - R * 0.09); ctx.lineTo(cx, cy + R * 0.09);
  ctx.stroke();

  ctx.fillStyle = '#5b6c81'; ctx.font = '9px ui-monospace,monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  ctx.fillText('ROLL / PITCH', cx, cy + R + 11);
}

function drawCompass(ctx, cx, cy, R, yaw, ok) {
  ctx.save();
  ctx.strokeStyle = '#2b3a4d'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 7); ctx.stroke();

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-yaw * Math.PI / 180);
  ctx.font = '9px ui-monospace,monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  for (let d = 0; d < 360; d += 15) {
    const ang = (d - 90) * Math.PI / 180;
    const major = d % 45 === 0;
    const r2 = R - (major ? 10 : 5);
    ctx.strokeStyle = major ? 'rgba(219,228,238,.5)' : 'rgba(219,228,238,.2)';
    ctx.beginPath();
    ctx.moveTo(R * Math.cos(ang), R * Math.sin(ang));
    ctx.lineTo(r2 * Math.cos(ang), r2 * Math.sin(ang));
    ctx.stroke();
    // Num instrumento pequeno os rotulos cardinais colidem com a leitura
    // central; abaixo de 55 px de raio ficam so os ticks e a marca do zero.
    if (d % 90 === 0 && R >= 55) {
      const lbl = ['0', '90', '180', '270'][d / 90];
      ctx.fillStyle = d === 0 ? '#34d399' : 'rgba(219,228,238,.6)';
      const rt = R - 20;
      ctx.save();
      ctx.translate(rt * Math.cos(ang), rt * Math.sin(ang));
      ctx.rotate(yaw * Math.PI / 180);
      ctx.fillText(lbl, 0, 0);
      ctx.restore();
    }
  }
  ctx.restore();

  // ponteiro fixo no topo
  ctx.fillStyle = ok ? '#22d3ee' : '#5b6c81';
  ctx.beginPath();
  ctx.moveTo(cx, cy - R + 1);
  ctx.lineTo(cx - 5, cy - R + 11);
  ctx.lineTo(cx + 5, cy - R + 11);
  ctx.closePath(); ctx.fill();

  ctx.fillStyle = ok ? '#dbe4ee' : '#4d6076';
  ctx.font = '600 ' + Math.max(10, Math.min(15, Math.round(R * 0.3))) + 'px ui-monospace,monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(ok ? yaw.toFixed(0) + '°' : '—', cx, cy);

  ctx.fillStyle = '#5b6c81'; ctx.font = '9px ui-monospace,monospace';
  ctx.textBaseline = 'bottom';
  ctx.fillText(ok ? 'YAW (deriva)' : 'YAW', cx, cy + R + 11);
  ctx.restore();
}

// Sem firmware, a atitude sai da mesma configuracao de movimento do simulador.
function offlineAttitude(t) {
  const c = S.cfg;
  // a vibracao simulada e o que derruba a confianca na gravidade
  const vibMg = (S.spec && S.spec.ch[2]) ? (c.simNoise || 0) * 3 : 0;
  const trust = Math.max(0, Math.min(1, 1 - (vibMg / 1000) / (c.attTol || 0.35)));

  if (!c.simOn || !c.simAtt) {
    S.att = { r: 0, p: 0, y: 0, gx: 0, gy: 0, gz: 0, g: 1, gf: 1,
              trust, vib: vibMg, gftd: c.attSrc >= 1 ? 1 : 0,
              af: c.attSrc === 2 && !c.dc ? 1 : 0, cal: 0, ok: 0 };
    renderAttitude();
    return;
  }
  const w = 2 * Math.PI / Math.max(c.simAttP, 0.01);
  let y = (c.simYaw * t - (S.attYaw0 || 0)) % 360;
  if (y > 180) y -= 360;
  if (y < -180) y += 360;
  S.att = {
    r: c.simRoll * Math.sin(w * t),
    p: c.simPitch * Math.sin(w * t * 0.73 + 0.9),
    y,
    gx: c.simRoll * w * Math.cos(w * t),
    gy: c.simPitch * w * 0.73 * Math.cos(w * t * 0.73 + 0.9),
    gz: c.simYaw,
    g: 1, gf: 1, trust, vib: vibMg,
    gftd: c.attSrc >= 1 ? 1 : 0, af: c.attSrc === 2 && !c.dc ? 1 : 0,
    cal: 0, ok: 1,
  };
  renderAttitude();
}

/* ======================================================= enlace serial === */
// Alternativa ao WiFi: o navegador fala direto com a porta USB da placa
// (Web Serial API) e recebe exatamente os mesmos pacotes do WebSocket.
// Util quando o PC nao esta na mesma rede da placa.
//
// Exige contexto seguro: funciona em http://localhost / 127.0.0.1 e em https,
// NAO funciona abrindo o arquivo direto (file://).

const SERIAL_BAUD = 2000000;
let rxBuf = new Uint8Array(0);

function serialSupported() {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

async function serialSend(line) {
  if (!S.serialWriter) return;
  await S.serialWriter.write(new TextEncoder().encode(line + '\n'));
}

// Mensagem ao lado do botao. Sem isso uma falha de abertura da porta parece
// simplesmente 'o botao nao faz nada'.
function serialMsg(txt, kind) {
  const el = $('serial-msg');
  el.textContent = txt || '';
  el.className = 'topmsg' + (kind ? ' ' + kind : '');
}

async function connectSerial() {
  if (S.serialPort) return disconnectSerial();
  if (!serialSupported()) {
    serialMsg(window.isSecureContext
      ? 'este navegador não tem Web Serial — use Chrome ou Edge'
      : 'abra por http://localhost ou 127.0.0.1 (file:// não permite Web Serial)', 'err');
    return;
  }

  let port;
  try {
    port = await navigator.serial.requestPort();
  } catch (e) {
    serialMsg('nenhuma porta escolhida');
    return;
  }

  serialMsg('abrindo…');
  try {
    await port.open({ baudRate: SERIAL_BAUD });
  } catch (e) {
    // o caso mais comum de longe: monitor serial aberto segurando a porta
    $('bt-serial').classList.add('err');
    serialMsg('não abriu: ' + (e && e.message ? e.message : e) +
              ' — feche o monitor serial e tente de novo', 'err');
    return;
  }

  S.serialPort = port;
  S.serialWriter = port.writable.getWriter();
  rxBuf = new Uint8Array(0);

  // sai da prévia local: agora os dados vem da placa
  if (S.offlineTimer) { clearInterval(S.offlineTimer); S.offlineTimer = null; }
  S.offline = false;
  setMode('serial');
  $('link-dot').classList.add('on');
  $('bt-serial').classList.add('on');
  $('bt-serial').textContent = 'serial ✓';

  // limpa o que a previa local deixou: durante o boot da placa a tela mostraria
  // numeros do simulador com a mensagem "aguardando", o que confunde
  S.spec = S.scope = S.fscope = S.rawscope = S.att = null;
  S.peaks = []; S.peaksHeld = []; S.dyn = []; S.rpm = [];
  S.dcCh = []; S.rmsCh = []; S.lsbCh = [];
  wfImg = null; wfFresh = true;
  ['m-fs', 'm-meas', 'm-rms', 'm-rmsf', 'm-pkpk', 'm-temp'].forEach((id) => {
    $(id).textContent = '—';
  });
  // os rotulos de taxa sobreviviam ao reset e passavam a mentir: sem quadro
  // novo eles continuavam anunciando a cadencia da previa local
  ['raw-span', 'fscope-span', 'scope-span', 'res-note'].forEach((id) => {
    $(id).textContent = '';
  });
  renderAxes(); renderPeaks(); renderAttitude();

  S.serialFrames = 0;
  readSerialLoop(port);
  armStream(port);
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// Abrir a porta faz a placa REINICIAR: o circuito de auto-reset do ESP32 usa
// DTR/RTS, que o navegador aciona ao abrir. O boot leva alguns segundos (WiFi,
// LittleFS, varredura I2C) e um "stream 1" mandado antes disso se perde. Por
// isso o pedido e repetido ate o primeiro quadro chegar.
async function armStream(port) {
  // se der, solta as linhas de handshake para nao segurar a placa em reset
  try { await port.setSignals({ dataTerminalReady: false, requestToSend: false }); } catch (e) {}

  const t0 = performance.now();
  let tentativas = 0;
  while (S.serialPort === port && performance.now() - t0 < 15000) {
    if (S.serialFrames > 0) { serialMsg(''); return; }
    tentativas++;
    serialMsg('a placa reinicia ao abrir a porta — aguardando o boot (' + tentativas + ')', 'ok');
    try { await serialSend('stream 1'); } catch (e) { return; }
    await sleep(900);
  }
  if (S.serialPort !== port) return;
  if (S.serialFrames > 0) { serialMsg(''); return; }
  serialMsg('sem quadros em 15 s — o firmware gravado é o novo? o baud é ' +
            SERIAL_BAUD + '?', 'err');
}

async function disconnectSerial() {
  try { await serialSend('stream 0'); } catch (e) { /* porta ja caiu */ }
  try { if (S.serialReader) await S.serialReader.cancel(); } catch (e) {}
  try { if (S.serialWriter) S.serialWriter.releaseLock(); } catch (e) {}
  try { if (S.serialPort) await S.serialPort.close(); } catch (e) {}
  S.serialPort = null; S.serialWriter = null; S.serialReader = null;
  $('link-dot').classList.remove('on');
  $('bt-serial').classList.remove('on');
  $('bt-serial').classList.remove('err');
  $('bt-serial').textContent = 'serial';
  serialMsg('');
  setMode('off');
}

async function readSerialLoop(port) {
  const reader = port.readable.getReader();
  S.serialReader = reader;
  try {
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value) feedSerial(value);
    }
  } catch (e) {
    // cabo removido ou porta fechada
  } finally {
    try { reader.releaseLock(); } catch (e) {}
    if (S.serialPort) disconnectSerial();
  }
}

// Quadro: AB CD <len:u16 LE> <payload> <xor>. O SOF de dois bytes deixa o
// parser se realinhar mesmo se cair no meio de uma linha de log em texto.
function feedSerial(chunk) {
  const b = new Uint8Array(rxBuf.length + chunk.length);
  b.set(rxBuf); b.set(chunk, rxBuf.length);
  rxBuf = b;

  let pos = 0;
  for (;;) {
    let i = pos;
    while (i + 1 < rxBuf.length && !(rxBuf[i] === 0xAB && rxBuf[i + 1] === 0xCD)) i++;
    if (i + 1 >= rxBuf.length) { pos = Math.max(pos, rxBuf.length - 1); break; }
    if (rxBuf.length < i + 4) { pos = i; break; }

    const len = rxBuf[i + 2] | (rxBuf[i + 3] << 8);
    const end = i + 4 + len + 1;
    if (len > 262144) { pos = i + 2; continue; }   // tamanho absurdo: SOF falso
    if (rxBuf.length < end) { pos = i; break; }

    const payload = rxBuf.slice(i + 4, i + 4 + len);
    let x = 0;
    for (let k = 0; k < len; k++) x ^= payload[k];
    if (x === rxBuf[i + 4 + len]) { S.serialFrames++; handleSerialPayload(payload); }
    pos = end;
  }
  rxBuf = pos > 0 ? rxBuf.slice(pos) : rxBuf;
  if (rxBuf.length > 1 << 20) rxBuf = new Uint8Array(0);   // dessincronizou feio
}

function handleSerialPayload(p) {
  if (p[0] === 0xA5) {
    onBinary(p.buffer);   // slice() acima ja deu um buffer proprio, offset 0
    return;
  }
  let m;
  try {
    m = JSON.parse(new TextDecoder().decode(p));
  } catch (e) {
    return;
  }
  if (m.t === 'status') onJson(m);
  else if (m.odr !== undefined) applyConfig(m);
}

/* ========================================================= simulador ==== */
// O simulador roda NO FIRMWARE: as amostras sinteticas entram no mesmo caminho
// das amostras do BMI323, entao a FFT, os filtros e o notch dinamico sao os
// reais. Sem firmware por tras (abrir data/index.html direto), a mesma
// configuracao alimenta uma previa analitica gerada aqui no navegador.

const SIM_SOURCES = 6;
const SIM_MAP = {
  'sim-on': 'simOn', 'sim-noise': 'simNoise', 'sim-grav': 'simGrav',
  'sim-reson': 'simResOn', 'sim-resf': 'simResHz', 'sim-resq': 'simResQ', 'sim-resg': 'simResG',
  'sim-att': 'simAtt', 'sim-roll': 'simRoll', 'sim-pitch': 'simPitch',
  'sim-attp': 'simAttP', 'sim-yaw': 'simYaw',
};
const SRC_KEYS = ['en', 'f', 'a', 'h', 'd', 'dr', 'dp', 'jt', 'ax'];
const AXIS_NAMES = ['X', 'Y', 'Z', 'todos'];

const simInputIds = () => [
  ...Object.keys(SIM_MAP),
  ...Array.from({ length: SIM_SOURCES }, (_, i) =>
    ['en', 'f', 'rpm', 'a', 'h', 'd', 'dr', 'dp', 'jt', 'ax'].map((k) => 's' + i + k)).flat(),
];

let simUIBuilt = false;
function buildSimUI() {
  if (simUIBuilt) return;
  $('sim-sources').innerHTML = Array.from({ length: SIM_SOURCES }, (_, i) => `
    <div class="fgroup src" id="gs${i}">
      <div class="nlabel" data-toggle="${i}">
        <label class="sw" data-stop="1"><input type="checkbox" id="s${i}en"><span></span></label>
        <span class="tag">FONTE ${i + 1}</span>
        <b class="live" id="os${i}live"></b>
        <span class="chev">&#9654;</span>
      </div>
      <div class="ctl"><span>freq</span><input type="range" id="s${i}f" min="1" max="800" step="0.5"><output id="os${i}f"></output><i>Hz</i></div>
      <div class="ctl"><span>rpm</span><input type="number" class="rpm" id="s${i}rpm" min="60" max="48000" step="60">
        <span class="unit">amp</span><input type="range" id="s${i}a" min="0" max="400" step="1"><output id="os${i}a"></output><i>mg</i></div>
      <div class="more">
        <div class="ctl"><span>harm</span>
          <select class="tsel" id="s${i}h"><option value="1">1</option><option value="2">2</option><option value="3">3</option><option value="4">4</option></select>
          <span class="unit">queda</span><input type="range" id="s${i}d" min="0" max="100" step="5"><output id="os${i}d"></output><i>%</i></div>
        <div class="ctl"><span>drift</span><input type="range" id="s${i}dr" min="0" max="120" step="1"><output id="os${i}dr"></output><i>Hz</i>
          <span class="unit">em</span><input type="range" id="s${i}dp" min="1" max="60" step="1"><output id="os${i}dp"></output><i>s</i></div>
        <div class="ctl"><span>jitter</span><input type="range" id="s${i}jt" min="0" max="10" step="0.1"><output id="os${i}jt"></output><i>Hz</i>
          <span class="unit">eixo</span>
          <select class="tsel" id="s${i}ax">${AXIS_NAMES.map((n, k) => `<option value="${k}">${n}</option>`).join('')}</select></div>
      </div>
    </div>`).join('');

  $('sim-sources').querySelectorAll('[data-stop]').forEach((el) => {
    el.addEventListener('click', (e) => e.stopPropagation());
  });
  $('sim-sources').querySelectorAll('[data-toggle]').forEach((el) => {
    el.onclick = () => $('gs' + el.dataset.toggle).classList.toggle('open');
  });
  $('gs0').classList.add('open');
  markEditable($('sim-sources'));
  simUIBuilt = true;
}

// Le a UI -> S.cfg e atualiza os rotulos do simulador
function syncSimUI() {
  const c = S.cfg; if (!c) return;
  for (const [id, key] of Object.entries(SIM_MAP)) {
    const el = $(id);
    c[key] = el.type === 'checkbox' ? (el.checked ? 1 : 0) : +el.value;
  }
  for (let i = 0; i < SIM_SOURCES; i++) {
    const o = c.simSrc[i];
    o.en = $('s' + i + 'en').checked ? 1 : 0;
    // rpm e freq sao a mesma grandeza: quem foi mexido por ultimo manda
    if (document.activeElement === $('s' + i + 'rpm')) {
      o.f = +(+$('s' + i + 'rpm').value / 60).toFixed(2);
      $('s' + i + 'f').value = o.f;
    } else {
      o.f = +$('s' + i + 'f').value;
      $('s' + i + 'rpm').value = Math.round(o.f * 60);
    }
    o.a = +$('s' + i + 'a').value;   o.h = +$('s' + i + 'h').value;
    o.d = +$('s' + i + 'd').value;   o.dr = +$('s' + i + 'dr').value;
    o.dp = +$('s' + i + 'dp').value; o.jt = +$('s' + i + 'jt').value;
    o.ax = +$('s' + i + 'ax').value;

    $('os' + i + 'f').textContent = o.f.toFixed(1);
    $('os' + i + 'a').textContent = o.a.toFixed(0);
    $('os' + i + 'd').textContent = o.d.toFixed(0);
    $('os' + i + 'dr').textContent = o.dr.toFixed(0);
    $('os' + i + 'dp').textContent = o.dp.toFixed(0);
    $('os' + i + 'jt').textContent = o.jt.toFixed(1);
    $('gs' + i).classList.toggle('on', !!o.en);
  }
  $('o-simnoise').textContent = c.simNoise.toFixed(1);
  $('o-simresf').textContent = c.simResHz.toFixed(0);
  $('o-simresq').textContent = c.simResQ.toFixed(1);
  $('o-simresg').textContent = c.simResG.toFixed(0);
  $('o-simroll').textContent = c.simRoll.toFixed(0);
  $('o-simpitch').textContent = c.simPitch.toFixed(0);
  $('o-simattp').textContent = c.simAttP.toFixed(1);
  $('o-simyaw').textContent = c.simYaw.toFixed(0);
  $('g-simres').classList.toggle('off', !c.simResOn);
  $('g-simatt').classList.toggle('off', !c.simAtt);

  // Simulador desligado: o painel inteiro some. Nao ha o que configurar,
  // e a barra lateral se reorganiza para nao ficar com um buraco.
  $('p-sim').hidden = !c.simOn;
  $('sim-toggle').classList.toggle('on', !!c.simOn);
  document.querySelector('.side').classList.toggle('no-sim', !c.simOn);
  $('sim-note').textContent = c.simOn
    ? (S.offline ? 'prévia local' : 'substituindo o sensor')
    : (S.offline ? 'sem firmware' : 'sensor real');
  renderSimLive();
}

// frequencia instantanea de cada fonte (com deriva), reportada pelo firmware
function renderSimLive() {
  const c = S.cfg; if (!c || !simUIBuilt) return;
  for (let i = 0; i < SIM_SOURCES; i++) {
    const o = c.simSrc[i];
    const live = S.simF[i];
    $('os' + i + 'live').textContent =
      o.en && c.simOn && live > 0 ? live.toFixed(1) + ' Hz · ' + Math.round(live * 60) + ' rpm' : '';
  }
}

function sendSim() {
  const c = S.cfg; if (!c) return;
  const p = {};
  for (const key of Object.values(SIM_MAP)) p[key] = c[key];
  c.simSrc.forEach((o, i) => { for (const k of SRC_KEYS) p['s' + i + k] = o[k]; });
  api('api/sim', p);
}

/* ------------------------------------------------------------ presets -- */
const mkSrc = (o) => Object.assign(
  { en: 0, f: 100, a: 30, h: 1, d: 50, dr: 0, dp: 10, jt: 0, ax: 3 }, o || {});

const SIM_PRESETS = {
  // 4 motores nunca giram exatamente igual: dai as 4 raias vizinhas
  drone5: { noise: 3, grav: 1, res: [1, 320, 5, 10], src: [
    mkSrc({ en: 1, f: 118, a: 55, h: 2, d: 45, dr: 8, dp: 9, jt: 0.6 }),
    mkSrc({ en: 1, f: 124, a: 48, h: 2, d: 45, dr: 9, dp: 11, jt: 0.6 }),
    mkSrc({ en: 1, f: 112, a: 52, h: 2, d: 45, dr: 7, dp: 13, jt: 0.6 }),
    mkSrc({ en: 1, f: 129, a: 44, h: 2, d: 45, dr: 10, dp: 8, jt: 0.6 }),
    mkSrc(), mkSrc()] },
  drone7: { noise: 4, grav: 1, res: [1, 190, 4, 12], src: [
    mkSrc({ en: 1, f: 74, a: 90, h: 3, d: 50, dr: 6, dp: 12, jt: 0.5 }),
    mkSrc({ en: 1, f: 79, a: 82, h: 3, d: 50, dr: 5, dp: 14, jt: 0.5 }),
    mkSrc({ en: 1, f: 70, a: 86, h: 3, d: 50, dr: 6, dp: 10, jt: 0.5 }),
    mkSrc({ en: 1, f: 83, a: 76, h: 3, d: 50, dr: 7, dp: 16, jt: 0.5 }),
    mkSrc(), mkSrc()] },
  // 1770 rpm no eixo + 2x a frequencia da rede (ruido magnetico)
  motor4p: { noise: 1.5, grav: 1, res: [1, 420, 8, 6], src: [
    mkSrc({ en: 1, f: 29.5, a: 95, h: 3, d: 55, dr: 0.3, dp: 20, jt: 0.05 }),
    mkSrc({ en: 1, f: 120, a: 28, h: 2, d: 40 }),
    mkSrc(), mkSrc(), mkSrc(), mkSrc()] },
  // rotacao + passagem de pa (5 pas x 24 Hz)
  fan: { noise: 2, grav: 1, res: [1, 260, 6, 8], src: [
    mkSrc({ en: 1, f: 24, a: 60, h: 2, d: 50, dr: 1.5, dp: 15, jt: 0.2 }),
    mkSrc({ en: 1, f: 120, a: 42, h: 2, d: 60, dr: 7.5, dp: 15, jt: 0.2 }),
    mkSrc(), mkSrc(), mkSrc(), mkSrc()] },
  // folga mecanica gera muitas harmonicas com queda lenta
  pump: { noise: 2.5, grav: 1, res: [1, 380, 7, 9], src: [
    mkSrc({ en: 1, f: 24.5, a: 115, h: 4, d: 72, dr: 0.5, dp: 25, jt: 0.1 }),
    mkSrc({ en: 1, f: 98, a: 35, h: 2, d: 50 }),
    mkSrc(), mkSrc(), mkSrc(), mkSrc()] },
  unbal: { noise: 1, grav: 1, res: [0, 300, 6, 10], src: [
    mkSrc({ en: 1, f: 50, a: 130, h: 1 }), mkSrc(), mkSrc(), mkSrc(), mkSrc(), mkSrc()] },
  // varredura larga: e o caso em que o notch dinamico mostra servico
  sweep: { noise: 2, grav: 1, res: [1, 350, 5, 8], src: [
    mkSrc({ en: 1, f: 300, a: 90, h: 2, d: 50, dr: 200, dp: 20, jt: 0.5 }),
    mkSrc(), mkSrc(), mkSrc(), mkSrc(), mkSrc()] },
};

function applyPreset(name) {
  const p = SIM_PRESETS[name];
  if (!p) return;                       // "personalizado": nao mexe em nada
  const c = S.cfg;
  c.simNoise = p.noise; c.simGrav = p.grav;
  c.simResOn = p.res[0]; c.simResHz = p.res[1]; c.simResQ = p.res[2]; c.simResG = p.res[3];
  c.simSrc = p.src.map((o) => Object.assign({}, o));
  const nyq = c.odr / 2;
  c.simSrc.forEach((o) => { if (o.f > nyq) o.f = Math.round(nyq * 0.9); });
}

/* ================================================== previa sem firmware = */
// Sem backend, monta o espectro analiticamente a partir da mesma configuracao
// do simulador e aplica a resposta da cadeia de filtros.
const DEFAULT_CFG = {
  odr: 1600, odrCode: 12, rangeG: 16, avg: 0, bw: 0,
  fft: 1024, window: 1, alpha: 0.35, axis: 0,
  dc: 1, dcf: 2, mult: 100,
  lpf1Type: 0, lpf1Hz: 250, lpf1Dyn: 0, lpf1DynMin: 250, lpf1DynMax: 500, lpf1DynExpo: 5,
  lpf2Type: 0, lpf2Hz: 500,
  n1h: 0, n1c: 0, n2h: 0, n2c: 0,
  dnCount: 3, dnQ: 300, dnMin: 100, dnMax: 600,
  rpmSrc: 0, rpmBase: 118, rpmHarm: 3, rpmQ: 500, rpmMin: 100, rpmFade: 50, rpmLpf: 150, rpmW: [100, 100, 100],
  lsbG: 2048, lsbDps: 16.4,
  load: 0,
  simOn: 1, simNoise: 3, simGrav: 1, simResOn: 1, simResHz: 320, simResQ: 5, simResG: 10,
  simAtt: 1, simRoll: 12, simPitch: 8, simAttP: 6, simYaw: 20,
  chan: 0x3F,
  attLpf: 3, attKp: 1.5, attKi: 0.05, attTol: 0.35, attSrc: 1,
  gyro: 1, gyrRange: 500,
  simSrc: [],
};

function startOffline() {
  S.offline = true;
  setMode('off');
  $('bt-csv').removeAttribute('download');
  const c = JSON.parse(JSON.stringify(DEFAULT_CFG));
  S.cfg = c;
  applyPreset('drone5');
  applyConfig(c);
  S.offlineTimer = setInterval(offlineTick, SPECTRUM_MS);
  offlineTick();
}

// modulo do passa-faixa da ressonancia, pico 1.0 em f0
const bpfMag = (f, f0, q) => 1 / Math.sqrt(1 + Math.pow(q * (f / f0 - f0 / f), 2));

// Vibracao nunca chega igual nos tres eixos, e o giroscopio sente uma fracao
// dela (a vibracao linear vira rotacional pela geometria da fixacao).
const AXW = [0.85, 0.70, 1.00];
const GYRO_COUPLE = 0.035;   // graus/s por mg

function srcWeight(o, ch) {
  const ax = ch % 3;
  let w = (o.ax < 3) ? (o.ax === ax ? 1 : 0) : AXW[ax];
  if (ch >= 3) w *= GYRO_COUPLE;
  return w;
}

// Raias {f, a, ax} que o simulador produziria neste instante. A amplitude "a"
// ainda nao tem peso de eixo: quem usa aplica srcWeight() por canal.
function simTones(t) {
  const c = S.cfg, out = [];
  if (!c.simOn) return out;
  const nyq = c.odr / 2;
  const resAmp = c.simResOn ? Math.pow(10, c.simResG / 20) - 1 : 0;

  c.simSrc.forEach((o) => {
    if (!o.en || o.a <= 0) return;
    let f = o.f;
    if (o.dr > 0 && o.dp > 0.01) f += o.dr * Math.sin(2 * Math.PI * t / o.dp);
    if (f < 0.5) f = 0.5;

    for (let h = 0; h < o.h; h++) {
      const fh = f * (h + 1);
      if (fh >= nyq) break;
      let a = o.a * Math.pow(o.d / 100, h);
      if (resAmp > 0) a *= 1 + resAmp * bpfMag(fh, c.simResHz, c.simResQ);
      out.push({ f: fh, a, ax: o.ax });
    }
  });
  return out;
}

function offlineTick() {
  const c = S.cfg, bins = c.fft / 2, binHz = c.odr / c.fft;
  const t = performance.now() / 1000 - S.t0;
  const tones = simTones(t);
  const mask = c.chan | 0;

  // --------------------------------------------------- espectro por canal
  const spCh = new Array(6).fill(null);
  for (let ch = 0; ch < 6; ch++) {
    if (!(mask & (1 << ch)) && ch !== c.axis) continue;
    const arr = new Float32Array(bins);
    const nf = (c.simNoise || 0) * (ch >= 3 ? GYRO_COUPLE * 3 : 1);
    const perBin = nf * 2 / Math.sqrt(bins) + (ch >= 3 ? 0.0015 : 0.004);
    for (let i = 1; i < bins; i++) arr[i] = perBin * (0.45 + 0.9 * Math.random());

    for (const tone of tones) {
      const a = tone.a * srcWeight(tone, ch);
      if (a <= 0) continue;
      const b0 = tone.f / binHz;
      for (let k = -3; k <= 3; k++) {
        const i = Math.round(b0) + k;
        if (i < 1 || i >= bins) continue;
        arr[i] += a * Math.exp(-Math.pow((i - b0) / 1.1, 2));
      }
    }
    spCh[ch] = arr;
  }

  // canal em detalhe (pode ser o modulo, canal 6)
  const detCh = c.axis < 6 ? c.axis : 0;
  const det = spCh[detCh] || spCh.find((x) => x);

  // ------------------------------------ rastreamento que o firmware faria
  const ranked = tones.slice().sort((x, y) => y.a - x.a);
  S.simF = c.simSrc.map((o) => {
    if (!o.en) return 0;
    let f = o.f;
    if (o.dr > 0 && o.dp > 0.01) f += o.dr * Math.sin(2 * Math.PI * t / o.dp);
    return f;
  });

  if (c.dnCount > 0) {
    const cand = ranked.filter((x) => x.f >= c.dnMin && x.f <= c.dnMax).slice(0, c.dnCount);
    S.dyn = cand.map((x, i) => {
      const prev = S.dyn[i] || x.f;
      return prev + (x.f - prev) * 0.3;
    });
  } else S.dyn = [];

  if (c.rpmSrc > 0 && c.rpmHarm > 0) {
    const base = c.rpmSrc === 1 ? c.rpmBase : (ranked[0] ? ranked[0].f : 0);
    S.rpm = Array.from({ length: c.rpmHarm }, (_, h) => {
      const f = Math.min(Math.max((h + 1) * base, c.rpmMin), 0.48 * c.odr);
      const margin = f - c.rpmMin;
      let w = c.rpmW[h] / 100;
      if (c.rpmFade > 0 && margin < c.rpmFade) w *= Math.max(0, margin / c.rpmFade);
      return { f, w };
    });
  } else S.rpm = [];

  if (c.lpf1Dyn) {
    const expof = c.lpf1DynExpo / 10, x = c.load;
    S.lpf1Cut = ((c.lpf1DynMax - c.lpf1DynMin) * (x * (1 - x) * expof + x) + c.lpf1DynMin) * mult();
  } else S.lpf1Cut = c.lpf1Hz * mult();

  // ------------------------------------------------- resposta dos filtros
  const ch = chain();
  const gainAt = (f) => Math.pow(10, respDb(Math.max(f, 0.01), c.odr, ch) / 20);

  const filt = new Float32Array(bins);
  if (det) for (let i = 0; i < bins; i++) filt[i] = det[i] * gainAt(i * binHz);

  S.spec = { bins, binHz, fs: c.odr, mask, axis: detCh, ch: spCh, filt };
  $('res-note').textContent =
    binHz.toFixed(2) + ' Hz/bin · ' + bins + ' bins · Nyquist ' + (c.odr / 2).toFixed(0) + ' Hz';
  pushWaterfall();

  // ------------------------------------------- ondas no tempo (cru/filtrado)
  const pts = 256, dt = 4 / c.odr;
  const sr = new Float32Array(pts), sf = new Float32Array(pts);
  for (let i = 0; i < pts; i++) {
    const tt = t + i * dt;
    let v = (Math.random() - 0.5) * (c.simNoise || 0) * 2;
    let vf = 0;
    for (const tone of tones) {
      const a = tone.a * srcWeight(tone, detCh);
      if (a <= 0) continue;
      const ph = 2 * Math.PI * tone.f * tt;
      v += a * Math.sin(ph);
      vf += a * gainAt(tone.f) * Math.sin(ph);
    }
    sr[i] = v; sf[i] = vf;
  }
  S.scope = { pts, dt, axis: detCh, raw: sr, filt: sf };
  $('scope-span').textContent = (pts * dt * 1000).toFixed(0) + ' ms';

  // -------------------------------- ondas filtradas, uma por canal visivel
  const fdata = new Array(6).fill(null);
  for (let cc = 0; cc < 6; cc++) {
    if (!(mask & (1 << cc))) continue;
    const arr = new Float32Array(pts);
    for (let i = 0; i < pts; i++) {
      const tt = t + i * dt;
      let v = 0;
      for (const tone of tones) {
        const a = tone.a * srcWeight(tone, cc);
        if (a <= 0) continue;
        v += a * gainAt(tone.f) * Math.sin(2 * Math.PI * tone.f * tt);
      }
      arr[i] = v;
    }
    fdata[cc] = arr;
  }
  S.fscope = { pts, dt, mask, data: fdata };
  $('fscope-span').textContent = (pts * dt * 1000).toFixed(0) + ' ms';

  // ------------------------------ mesma coisa sem filtro: a leitura "do sensor"
  const rdata = new Array(6).fill(null);
  for (let cc = 0; cc < 6; cc++) {
    if (!(mask & (1 << cc))) continue;
    const arr = new Float32Array(pts);
    for (let i = 0; i < pts; i++) {
      const tt = t + i * dt;
      let v = (Math.random() - 0.5) * (c.simNoise || 0) * 2;
      for (const tone of tones) {
        const a = tone.a * srcWeight(tone, cc);
        if (a <= 0) continue;
        v += a * Math.sin(2 * Math.PI * tone.f * tt);
      }
      arr[i] = v;
    }
    rdata[cc] = arr;
  }
  S.rawscope = { pts, dt, mask, data: rdata };
  $('raw-span').textContent = (pts * dt * 1000).toFixed(0) + ' ms \u00b7 ' +
                              (1 / dt).toFixed(0) + ' pontos/s';

  // ------------------------------------------------------ picos e metricas
  S.peaks = [];
  if (det) {
    let sum = 0;
    for (let i = 2; i < bins; i++) sum += det[i];
    const floorAmp = (sum / (bins - 2)) * 3;
    for (let i = 2; i < bins - 1; i++) {
      if (det[i] > floorAmp && det[i] > det[i - 1] && det[i] >= det[i + 1]) {
        S.peaks.push({ f: i * binHz, a: det[i] });
      }
    }
    const found = S.peaks;
    S.peaks = [];
    mergePeaks(found);
  }
  // mesma leitura por eixo que o firmware manda, a partir do espectro local
  S.dcCh = []; S.rmsCh = []; S.lsbCh = [];
  for (let cc = 0; cc < 6; cc++) {
    const arr = spCh[cc];
    if (!arr) { S.dcCh.push(0); S.rmsCh.push(0); S.lsbCh.push(0); continue; }
    let best = 2, sum = 0;
    for (let i = 3; i < bins - 1; i++) { if (arr[i] > arr[best]) best = i; }
    for (let i = 2; i < bins; i++) sum += arr[i] * arr[i];
    S.dcCh.push(cc === 2 && c.simGrav ? 1.0 : 0);
    S.rmsCh.push(Math.sqrt(sum / 2));
    // sem placa nao existe FIFO: a contagem vem do nivel simulado
    S.lsbCh.push(Math.round((cc === 2 && c.simGrav ? 1.0 : 0) *
                            (cc < 3 ? (c.lsbG || 2048) : (c.lsbDps || 16.4))));
  }
  renderAxes();

  renderPeaks();
  renderTracking();
  renderSimLive();
  offlineAttitude(t);

  const rms = (arr) => { let x = 0; for (const v of arr) x += v * v; return Math.sqrt(x / 2); };
  $('m-fs').textContent = c.odr.toFixed(0);
  $('m-meas').textContent = c.odr.toFixed(0);
  $('m-rms').textContent = det ? rms(det).toFixed(1) : '—';
  $('m-rmsf').textContent = rms(filt).toFixed(1);
  $('m-pkpk').textContent = det ? (rms(det) * 2.9).toFixed(0) : '—';
  $('m-temp').textContent = '—';
  $('m-heap').textContent = '—';
}

/* ============================================================== loop ==== */
function frame() {
  drawRawScope();
  drawWaterfall();
  drawScope();
  drawFilteredScope();
  drawAttitude();
  requestAnimationFrame(frame);
}

buildLegend();
buildAxes();
buildSimUI();   // as fontes precisam existir antes de wire() ligar os handlers
wire();
fetch('api/config')
  .then((r) => (r.ok ? r.json() : Promise.reject()))
  .then((c) => { applyConfig(c); connect(); })
  .catch(startOffline);   // sem firmware: previa local com a mesma configuracao
requestAnimationFrame(frame);
