#!/usr/bin/env python3
"""12-channel digital oscilloscope digital twin (PyQt + pyqtgraph).

    python scope12.py                     # listens on UDP 127.0.0.1:9100, auto-probes bldc_sim on :9000
    python scope12.py --sim-decim 4       # ask the motor sim for every 4th sample (lighter)
    python scope12.py --depth 1000000     # memory depth per source (samples)

Probes: every signal published by any program with scope_probe.ScopeProbe (your ESC) and every internal
signal of the motor sim (currents, terminal voltages, star point, hall, gates, BEMF, rpm, torque, bus...).
Time axis = SIMULATION time, so waveforms are correct even when the sim runs in slow lock-step.

Mouse on the screen:  wheel = timebase | Shift+wheel = delay | Ctrl+wheel = V/div of selected channel
                      drag left/right = move waveform in time | drag up/down = move selected channel
Keys (plot focused):  Left/Right timebase  Up/Down V/div  PgUp/PgDn position  Home delay=0
Global keys:          F5 run/stop  F6 single  F7 autoset selected  F8 autoset all  Ctrl+S save capture
"""
import argparse
import json
import os
import sys
import time
from dataclasses import asdict

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scope_probe import DEFAULT_SCOPE_PORT
from scope_core import (NCH, HDIV, VDIV_N, VDIVS, TDIVS, Ch, Acquisition, Snapshot, ceil125, default_setup,
                        eng, find_trigger, measure, nearest_idx, peak_decimate, sanitize_setup, split_unit)

COLORS = ["#FFE000", "#00E5FF", "#FF4FD8", "#4CFF6A", "#FF9A2E", "#5C8DFF",
          "#FF5252", "#E8E8E8", "#B6FF4A", "#FF9CC8", "#3FD9BE", "#B58CFF"]
EDGES = ["Rising ↑", "Falling ↓", "Either ↕"]
TMODES = ["Auto", "Normal", "Single"]
Qt = QtCore.Qt
QShortcut = getattr(QtGui, "QShortcut", None) or QtWidgets.QShortcut


# ============================================================ view box
class ScopeVB(pg.ViewBox):
    def __init__(self, win, digital=False):
        super().__init__(enableMenu=False)
        self.win, self.digital = win, digital
        self.setMouseEnabled(False, False)

    def wheelEvent(self, ev, axis=None):
        try:
            d = ev.delta()
        except AttributeError:
            a = ev.angleDelta()
            d = a.y() or a.x()
        if d:
            self.win.on_wheel(1 if d > 0 else -1, ev.modifiers())
        ev.accept()

    def mouseDragEvent(self, ev, axis=None):
        if ev.button() != Qt.MouseButton.LeftButton:
            ev.ignore()
            return
        ev.accept()
        p0, p1 = self.mapToView(ev.lastPos()), self.mapToView(ev.pos())
        self.win.on_drag(p1.x() - p0.x(), 0.0 if self.digital else p1.y() - p0.y())

    def mouseClickEvent(self, ev):
        ev.accept()


# ============================================================ main window
class ScopeWindow(QtWidgets.QMainWindow):
    def __init__(self, acq, args):
        super().__init__()
        self.acq, self.args = acq, args
        self.setWindowTitle("12-CH Oscilloscope - digital twin")
        self.setup_path = args.setup
        st = default_setup()
        if os.path.exists(self.setup_path) and not args.default_setup:
            try:
                with open(self.setup_path) as f:
                    loaded = json.load(f)
                if isinstance(loaded, dict):
                    st.update(loaded)
            except (OSError, ValueError) as ex:
                print("setup load failed:", ex)
        st = sanitize_setup(st)
        self.ch = [Ch(**c) for c in st["ch"]]
        self.tdiv, self.delay = st["tdiv"], st["delay"]
        self.trig_ch, self.edge, self.level, self.tmode = st["trig_ch"], st["edge"], st["level"], st["tmode"]
        self.sel = st["sel"]
        self.running = True
        self.snap = None
        self.t_trig = None
        self.trig_state = "WAIT"
        self.last_trig_wall = -1e9
        self.single_after = -np.inf
        self.raw = [None] * NCH
        self.sig_version = -1
        self.last_meas = 0.0
        self._xr = None
        self._ticks_key = None
        self._nd = -1
        self._tl_ch = -1
        self._sym = [False] * NCH
        self.fps = self.frame_ms = 0.0
        self._last_tick = time.monotonic()
        self.font_mono = QtGui.QFont("DejaVu Sans Mono")
        self.font_mono.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        self.build_ui()
        self.sync_all()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    # ------------------------------------------------------------ UI build
    def build_ui(self):
        pg.setConfigOptions(antialias=False, background="#0b0d12", foreground="#b8bcc8")
        split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)
        self.setCentralWidget(split)

        # ---------- left: status, analog plot, digital plot, measurement table
        left = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        top = QtWidgets.QWidget()
        tl = QtWidgets.QVBoxLayout(top)
        tl.setContentsMargins(4, 4, 4, 0)
        tl.setSpacing(2)
        self.status = QtWidgets.QLabel()
        self.status.setFont(self.font_mono)
        self.status.setTextFormat(Qt.TextFormat.RichText)
        tl.addWidget(self.status)

        self.vb_a = ScopeVB(self)
        self.pw_a = pg.PlotWidget(viewBox=self.vb_a)
        self.pw_d = pg.PlotWidget(viewBox=ScopeVB(self, digital=True))
        for pw in (self.pw_a, self.pw_d):
            pi = pw.getPlotItem()
            pi.hideButtons()
            pi.setMenuEnabled(False)
            pi.showGrid(x=True, y=True, alpha=0.22)
            pi.getAxis("left").setWidth(118)
            pw.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.pw_a.setYRange(-VDIV_N / 2, VDIV_N / 2, padding=0)
        self.pw_d.setXLink(self.pw_a)
        tl.addWidget(self.pw_a, 1)
        tl.addWidget(self.pw_d, 0)
        left.addWidget(top)

        self.curves_a, self.curves_d, self.markers = [], [], []
        for i in range(NCH):
            pen = pg.mkPen(COLORS[i], width=1)
            ca = self.pw_a.plot(pen=pen)
            cd = self.pw_d.plot(pen=pen)
            for c in (ca, cd):
                if hasattr(c, "setSkipFiniteCheck"):
                    c.setSkipFiniteCheck(True)
            self.curves_a.append(ca)
            self.curves_d.append(cd)
            mk = pg.TextItem(text=f"{i + 1}", color="#000000", fill=pg.mkBrush(COLORS[i]), anchor=(0, 0.5))
            mk.setZValue(20)
            self.pw_a.addItem(mk, ignoreBounds=True)
            self.markers.append(mk)
        self.trig_tline = pg.InfiniteLine(pos=0, angle=90, movable=False,
                                          pen=pg.mkPen("#8a8f9c", style=Qt.PenStyle.DashLine))
        self.pw_a.addItem(self.trig_tline, ignoreBounds=True)
        self.trig_line = pg.InfiniteLine(angle=0, movable=True, label="T", labelOpts=dict(position=0.98, color="#fff"))
        self.trig_line.setZValue(30)
        self.trig_line.sigDragged.connect(self.on_trig_line_drag)
        self.pw_a.addItem(self.trig_line, ignoreBounds=True)
        cp = pg.mkPen("#ffffff", width=1, style=Qt.PenStyle.DashDotLine)
        self.cx = [pg.InfiniteLine(angle=90, movable=True, pen=cp, label=n, labelOpts=dict(position=0.95, color="#fff"))
                   for n in ("X1", "X2")]
        self.cy = [pg.InfiniteLine(angle=0, movable=True, pen=cp, label=n, labelOpts=dict(position=0.03, color="#fff"))
                   for n in ("Y1", "Y2")]
        for c in self.cx + self.cy:
            c.setZValue(25)
            c.hide()
            self.pw_a.addItem(c, ignoreBounds=True)

        self.meas = QtWidgets.QTableWidget(NCH, 10)
        self.meas.setHorizontalHeaderLabels(["Ch", "Signal", "Freq", "Period", "Duty+", "Vpp", "Max", "Min", "Mean", "RMS"])
        self.meas.verticalHeader().setVisible(False)
        self.meas.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.meas.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.NoSelection)
        self.meas.setFont(self.font_mono)
        self.meas.verticalHeader().setDefaultSectionSize(19)
        self.meas.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.meas.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
        self.meas.setMinimumHeight(19 * NCH + 30)
        self.meas.cellClicked.connect(lambda r, c: self.select(r))
        for r in range(NCH):
            for c in range(10):
                it = QtWidgets.QTableWidgetItem("")
                if c == 0:
                    it.setText(f"CH{r + 1}")
                    it.setForeground(QtGui.QBrush(QtGui.QColor("#000")))
                    it.setBackground(QtGui.QBrush(QtGui.QColor(COLORS[r])))
                self.meas.setItem(r, c, it)
        left.addWidget(self.meas)
        left.setStretchFactor(0, 4)
        left.setStretchFactor(1, 1)
        split.addWidget(left)

        # ---------- right: controls
        panel = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(panel)
        pl.setContentsMargins(6, 6, 6, 6)

        g = QtWidgets.QGroupBox("Channels  (select = scale shown on left axis / cursor Y / keys)")
        gl = QtWidgets.QGridLayout(g)
        gl.setVerticalSpacing(2)
        gl.setHorizontalSpacing(4)
        for c, h in enumerate(["", "On", "Probe (source/signal)", "Scale /div", "Pos div", "Offset / Thr", "Cpl", "Mode"]):
            lab = QtWidgets.QLabel(f"<b>{h}</b>")
            gl.addWidget(lab, 0, c)
        self.rb_group = QtWidgets.QButtonGroup(self)
        self.w_sel, self.w_on, self.w_src, self.w_vdiv, self.w_pos, self.w_ofs, self.w_cpl, self.w_mode = ([] for _ in range(8))
        for i in range(NCH):
            rb = QtWidgets.QRadioButton(f"CH{i + 1}")
            rb.setStyleSheet(f"color:{COLORS[i]}; font-weight:bold")
            self.rb_group.addButton(rb, i)
            rb.toggled.connect(lambda on, i=i: on and self.select(i, from_widget=True))
            cb = QtWidgets.QCheckBox()
            cb.toggled.connect(lambda v, i=i: self.set_ch(i, on=v))
            src = QtWidgets.QComboBox()
            src.setMinimumWidth(190)
            src.setMaxVisibleItems(30)
            src.currentTextChanged.connect(lambda txt, i=i: self.on_src(i, txt))
            vd = QtWidgets.QComboBox()
            for v in VDIVS:
                vd.addItem(eng(v, ""))
            vd.currentIndexChanged.connect(lambda k, i=i: self.set_ch(i, vdiv=VDIVS[k]) if k >= 0 else None)
            ps = QtWidgets.QDoubleSpinBox()
            ps.setRange(-VDIV_N / 2, VDIV_N / 2)
            ps.setSingleStep(0.1)
            ps.setDecimals(2)
            ps.valueChanged.connect(lambda v, i=i: self.set_ch(i, pos=v))
            of = QtWidgets.QDoubleSpinBox()
            of.setRange(-1e7, 1e7)
            of.setDecimals(4)
            of.setMinimumWidth(95)
            of.valueChanged.connect(lambda v, i=i: self.on_ofs(i, v))
            cp_ = QtWidgets.QComboBox()
            cp_.addItems(["DC", "AC", "GND"])
            cp_.currentTextChanged.connect(lambda v, i=i: self.set_ch(i, coup=v))
            md = QtWidgets.QComboBox()
            md.addItems(["Analog", "Digital"])
            md.currentIndexChanged.connect(lambda k, i=i: self.set_ch(i, digital=(k == 1)))
            for c, w in enumerate((rb, cb, src, vd, ps, of, cp_, md)):
                gl.addWidget(w, i + 1, c)
            for lst, w in zip((self.w_sel, self.w_on, self.w_src, self.w_vdiv, self.w_pos, self.w_ofs, self.w_cpl, self.w_mode),
                              (rb, cb, src, vd, ps, of, cp_, md)):
                lst.append(w)
        pl.addWidget(g)

        row = QtWidgets.QHBoxLayout()
        gh = QtWidgets.QGroupBox("Horizontal")
        hl = QtWidgets.QFormLayout(gh)
        self.w_tdiv = QtWidgets.QComboBox()
        for v in TDIVS:
            self.w_tdiv.addItem(eng(v, "s") + "/div")
        self.w_tdiv.currentIndexChanged.connect(lambda k: k >= 0 and self.set_tdiv(TDIVS[k]))
        self.w_delay = QtWidgets.QDoubleSpinBox()
        self.w_delay.setRange(-1e6, 1e6)
        self.w_delay.setDecimals(4)
        self.w_delay.setSuffix(" ms")
        self.w_delay.valueChanged.connect(lambda v: self.set_delay(v * 1e-3, from_widget=True))
        bz = QtWidgets.QPushButton("Delay = 0")
        bz.clicked.connect(lambda: self.set_delay(0.0))
        hl.addRow("Timebase", self.w_tdiv)
        hl.addRow("Delay (center)", self.w_delay)
        hl.addRow(bz)
        row.addWidget(gh)

        gt = QtWidgets.QGroupBox("Trigger")
        tlay = QtWidgets.QGridLayout(gt)
        self.w_tsrc = QtWidgets.QComboBox()
        self.w_tsrc.addItems([f"CH{i + 1}" for i in range(NCH)])
        self.w_tsrc.currentIndexChanged.connect(self.on_tsrc)
        self.w_edge = QtWidgets.QComboBox()
        self.w_edge.addItems(EDGES)
        self.w_edge.currentIndexChanged.connect(lambda k: setattr(self, "edge", k))
        self.w_level = QtWidgets.QDoubleSpinBox()
        self.w_level.setRange(-1e7, 1e7)
        self.w_level.setDecimals(4)
        self.w_level.valueChanged.connect(self.on_level)
        b50 = QtWidgets.QPushButton("50%")
        b50.clicked.connect(self.level_50)
        self.w_tmode = QtWidgets.QComboBox()
        self.w_tmode.addItems(TMODES)
        self.w_tmode.currentIndexChanged.connect(self.on_tmode)
        self.b_run = QtWidgets.QPushButton("RUN")
        self.b_run.clicked.connect(self.toggle_run)
        self.b_single = QtWidgets.QPushButton("SINGLE")
        self.b_single.clicked.connect(self.arm_single)
        tlay.addWidget(QtWidgets.QLabel("Source"), 0, 0)
        tlay.addWidget(self.w_tsrc, 0, 1)
        tlay.addWidget(QtWidgets.QLabel("Edge"), 0, 2)
        tlay.addWidget(self.w_edge, 0, 3)
        tlay.addWidget(QtWidgets.QLabel("Level"), 1, 0)
        tlay.addWidget(self.w_level, 1, 1)
        tlay.addWidget(b50, 1, 2)
        tlay.addWidget(self.w_tmode, 1, 3)
        tlay.addWidget(self.b_run, 2, 0, 1, 2)
        tlay.addWidget(self.b_single, 2, 2, 1, 2)
        row.addWidget(gt)
        pl.addLayout(row)

        row2 = QtWidgets.QHBoxLayout()
        gc = QtWidgets.QGroupBox("Cursors")
        cl = QtWidgets.QVBoxLayout(gc)
        cr = QtWidgets.QHBoxLayout()
        self.w_cx = QtWidgets.QCheckBox("Time X1/X2")
        self.w_cy = QtWidgets.QCheckBox("Level Y1/Y2 (sel. ch)")
        self.w_cx.toggled.connect(self.toggle_cx)
        self.w_cy.toggled.connect(self.toggle_cy)
        cr.addWidget(self.w_cx)
        cr.addWidget(self.w_cy)
        cl.addLayout(cr)
        self.cur_lab = QtWidgets.QLabel("")
        self.cur_lab.setFont(self.font_mono)
        self.cur_lab.setMinimumHeight(70)
        cl.addWidget(self.cur_lab)
        row2.addWidget(gc, 3)
        gtool = QtWidgets.QGroupBox("Tools")
        tg = QtWidgets.QGridLayout(gtool)
        for k, (txt, fn) in enumerate([("Autoset sel (F7)", lambda: self.autoset(self.sel)),
                                       ("Autoset all (F8)", self.autoset_all),
                                       ("Save PNG+CSV", self.save_capture), ("Clear memory", self.acq.clear),
                                       ("Default setup", self.load_default), ("Save setup", self.save_setup)]):
            b = QtWidgets.QPushButton(txt)
            b.clicked.connect(fn)
            tg.addWidget(b, k // 2, k % 2)
        row2.addWidget(gtool, 2)
        pl.addLayout(row2)

        gs = QtWidgets.QGroupBox("Probe sources (UDP :%d)" % self.args.port)
        sl = QtWidgets.QVBoxLayout(gs)
        self.src_lab = QtWidgets.QLabel("")
        self.src_lab.setFont(self.font_mono)
        sl.addWidget(self.src_lab)
        pl.addWidget(gs)
        hint = QtWidgets.QLabel("Screen: wheel = timebase, Shift+wheel = delay, Ctrl+wheel = V/div (sel)\n"
                                "drag ↔ = move in time, drag ↕ = move selected channel, drag T line = trigger level\n"
                                "F5 run/stop  F6 single  F7/F8 autoset  Ctrl+S save   (click plot, then arrows/PgUp/PgDn/Home)")
        hint.setStyleSheet("color:#8a8f9c")
        pl.addWidget(hint)
        pl.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(660)
        split.addWidget(scroll)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 0)

        def sc(seq, fn, ctx=Qt.ShortcutContext.ApplicationShortcut, parent=self):
            s = QShortcut(QtGui.QKeySequence(seq), parent)
            s.setContext(ctx)
            s.activated.connect(fn)
        sc("F5", self.toggle_run)
        sc("F6", self.arm_single)
        sc("F7", lambda: self.autoset(self.sel))
        sc("F8", self.autoset_all)
        sc("Ctrl+S", self.save_capture)
        for pw in (self.pw_a, self.pw_d):
            w = Qt.ShortcutContext.WidgetShortcut
            sc("Left", lambda: self.step_tdiv(+1), w, pw)
            sc("Right", lambda: self.step_tdiv(-1), w, pw)
            sc("Up", lambda: self.step_vdiv(self.sel, -1), w, pw)
            sc("Down", lambda: self.step_vdiv(self.sel, +1), w, pw)
            sc("PgUp", lambda: self.set_ch(self.sel, pos=self.ch[self.sel].pos + 0.2, sync=True), w, pw)
            sc("PgDown", lambda: self.set_ch(self.sel, pos=self.ch[self.sel].pos - 0.2, sync=True), w, pw)
            sc("Home", lambda: self.set_delay(0.0), w, pw)

    # ------------------------------------------------------------ model <-> widgets
    def sync_row(self, i):
        c = self.ch[i]
        ws = (self.w_on[i], self.w_src[i], self.w_vdiv[i], self.w_pos[i], self.w_ofs[i], self.w_cpl[i], self.w_mode[i])
        for w in ws:
            w.blockSignals(True)
        self.w_on[i].setChecked(c.on)
        txt = c.key or "(none)"
        if self.w_src[i].findText(txt) < 0:
            self.w_src[i].addItem(txt)
        self.w_src[i].setCurrentText(txt)
        self.w_vdiv[i].setCurrentIndex(nearest_idx(VDIVS, c.vdiv))
        self.w_vdiv[i].setEnabled(not c.digital)
        self.w_pos[i].setValue(c.pos)
        self.w_pos[i].setEnabled(not c.digital)
        self.w_ofs[i].setSingleStep(0.05 if c.digital else c.vdiv / 10)
        self.w_ofs[i].setValue(c.thr if c.digital else c.ofs)
        self.w_ofs[i].setToolTip("logic threshold" if c.digital else "offset (value at the Pos line)")
        self.w_cpl[i].setCurrentText(c.coup)
        self.w_mode[i].setCurrentIndex(1 if c.digital else 0)
        for w in ws:
            w.blockSignals(False)

    def sync_horizontal(self):
        for w in (self.w_tdiv, self.w_delay):
            w.blockSignals(True)
        self.w_tdiv.setCurrentIndex(nearest_idx(TDIVS, self.tdiv))
        self.w_delay.setSingleStep(self.tdiv * 1e3 / 5)
        self.w_delay.setValue(self.delay * 1e3)
        for w in (self.w_tdiv, self.w_delay):
            w.blockSignals(False)

    def sync_trigger(self):
        ws = (self.w_tsrc, self.w_edge, self.w_level, self.w_tmode)
        for w in ws:
            w.blockSignals(True)
        self.w_tsrc.setCurrentIndex(self.trig_ch)
        self.w_edge.setCurrentIndex(self.edge)
        c = self.ch[self.trig_ch]
        self.w_level.setEnabled(not c.digital)
        self.w_level.setSingleStep(c.vdiv / 10)
        self.w_level.setValue(c.thr if c.digital else self.level)
        self.w_tmode.setCurrentIndex(self.tmode)
        for w in ws:
            w.blockSignals(False)

    def sync_all(self):
        self.refresh_sources(force=True)
        for i in range(NCH):
            self.sync_row(i)
        self.w_sel[self.sel].setChecked(True)
        self.sync_horizontal()
        self.sync_trigger()
        self.update_run_button()

    def refresh_sources(self, force=False):
        if not force and self.acq.version == self.sig_version:
            return
        self.sig_version = self.acq.version
        items = ["(none)"] + self.acq.signal_list()
        for i in range(NCH):
            w = self.w_src[i]
            w.blockSignals(True)
            w.clear()
            w.addItems(items)
            txt = self.ch[i].key or "(none)"
            if w.findText(txt) < 0:
                w.addItem(txt)
            w.setCurrentText(txt)
            w.blockSignals(False)

    def set_ch(self, i, sync=False, **kw):
        c = self.ch[i]
        for k, v in kw.items():
            if k == "pos":
                v = max(-VDIV_N / 2, min(VDIV_N / 2, v))
            setattr(c, k, v)
        if "digital" in kw or "vdiv" in kw or sync:
            self.sync_row(i)
        if i == self.trig_ch:
            self.sync_trigger()
        self._ticks_key = None

    def on_src(self, i, txt):
        self.set_ch(i, key="" if txt == "(none)" else txt, autoset_pending=(txt != "(none)"), on=True, sync=True)

    def on_ofs(self, i, v):
        if self.ch[i].digital:
            self.set_ch(i, thr=v)
        else:
            self.set_ch(i, ofs=v)

    def select(self, i, from_widget=False):
        self.sel = i
        if not from_widget:
            self.w_sel[i].setChecked(True)
        self._ticks_key = None

    def step_vdiv(self, i, d):
        k = max(0, min(len(VDIVS) - 1, nearest_idx(VDIVS, self.ch[i].vdiv) + d))
        self.set_ch(i, vdiv=VDIVS[k])

    def set_tdiv(self, v):
        self.tdiv = v
        self.sync_horizontal()

    def step_tdiv(self, d):
        k = max(0, min(len(TDIVS) - 1, nearest_idx(TDIVS, self.tdiv) + d))
        self.set_tdiv(TDIVS[k])

    def set_delay(self, v, from_widget=False):
        self.delay = v
        if not from_widget:
            self.sync_horizontal()

    def on_tsrc(self, k):
        self.trig_ch = k
        self.sync_trigger()

    def on_level(self, v):
        if not self.ch[self.trig_ch].digital:
            self.level = v

    def on_tmode(self, k):
        self.tmode = k
        if k == 2:
            self.arm_single()

    def on_trig_line_drag(self):
        c = self.ch[self.trig_ch]
        self.level = c.ofs + (self.trig_line.value() - c.pos) * c.vdiv
        self.sync_trigger()

    def level_50(self):
        r = self.raw[self.trig_ch]
        if r is not None and len(r[1]):
            c = self.ch[self.trig_ch]
            mid = 0.5 * (float(np.max(r[1])) + float(np.min(r[1])))
            if c.digital:
                c.thr = mid
                self.sync_row(self.trig_ch)
            else:
                self.level = mid
            self.sync_trigger()

    def on_wheel(self, d, mods):
        if mods & Qt.KeyboardModifier.ControlModifier:
            self.step_vdiv(self.sel, -d)
        elif mods & Qt.KeyboardModifier.ShiftModifier:
            self.set_delay(self.delay + d * self.tdiv)
        else:
            self.step_tdiv(-d)

    def on_drag(self, dx, dy):
        if dx:
            self.set_delay(self.delay - dx)
        if dy and not self.ch[self.sel].digital:
            self.set_ch(self.sel, pos=self.ch[self.sel].pos + dy, sync=True)

    # ------------------------------------------------------------ run control
    def update_run_button(self):
        if self.running:
            self.b_run.setText("RUN  (click = STOP)")
            self.b_run.setStyleSheet("background:#1f7a3a; color:white; font-weight:bold")
        else:
            self.b_run.setText("STOP (click = RUN)")
            self.b_run.setStyleSheet("background:#9b2226; color:white; font-weight:bold")

    def toggle_run(self):
        if self.running:
            self.stop()
        else:
            self.running, self.snap = True, None
            if self.tmode == 2:                 # RUN after a single shot -> back to continuous Auto
                self.tmode = 0
                self.sync_trigger()
        self.update_run_button()

    def stop(self):
        self.running = False
        self.snap = Snapshot(self.acq)
        self.trig_state = "STOP"
        self.update_run_button()

    def arm_single(self):
        self.tmode = 2
        self.sync_trigger()
        self.running, self.snap = True, None
        s, k = self.acq.lookup(self.ch[self.trig_ch].key)
        with self.acq.lock:
            if s is not None:
                t, _ = s.view()
                self.single_after = float(t[-1]) if len(t) else -np.inf
            else:
                self.single_after = -np.inf
        self.trig_state = "ARMED"
        self.update_run_button()

    # ------------------------------------------------------------ autoset
    def autoset(self, i, quiet=False):
        c = self.ch[i]
        r = self.raw[i]
        if r is None or len(r[1]) < 2:
            if not quiet:
                self.statusBar().showMessage(f"CH{i + 1}: no data in window to autoset", 3000)
            return False
        v = r[1]
        lo, hi = float(np.min(v)), float(np.max(v))
        if c.digital:
            c.thr = 0.5 * (lo + hi) if hi > lo else c.thr
        else:
            pp = hi - lo
            big = max(abs(lo), abs(hi), 1e-12)
            if lo <= 0.0 <= hi or pp > 0.5 * big:
                pos = c.pos
                need_up = hi / min(3.0, VDIV_N / 2 - 0.2 - pos) if hi > 0 and pos < VDIV_N / 2 - 0.3 else 0.0
                need_dn = -lo / min(3.0, pos + VDIV_N / 2 - 0.2) if lo < 0 and pos > -VDIV_N / 2 + 0.3 else 0.0
                if (hi > 0 and pos >= VDIV_N / 2 - 0.3) or (lo < 0 and pos <= -VDIV_N / 2 + 0.3):
                    c.pos = 0.0
                    need_up, need_dn = hi / 3.8, -lo / 3.8
                c.ofs = 0.0
                c.vdiv = ceil125(max(need_up, need_dn, 1e-6))
            else:
                c.vdiv = ceil125(max(pp / 3.0, big * 1e-4, 1e-6))
                q = c.vdiv / 10
                c.ofs = round(0.5 * (lo + hi) / q) * q
        c.autoset_pending = False
        self.sync_row(i)
        if i == self.trig_ch:
            self.sync_trigger()
        self._ticks_key = None
        return True

    def autoset_all(self):
        for i, c in enumerate(self.ch):
            if c.on and c.key:
                self.autoset(i, quiet=True)

    # ------------------------------------------------------------ cursors
    def toggle_cx(self, on):
        if on:
            self.cx[0].setValue(self.delay - 2 * self.tdiv)
            self.cx[1].setValue(self.delay + 2 * self.tdiv)
        for c in self.cx:
            c.setVisible(on)

    def toggle_cy(self, on):
        if on:
            self.cy[0].setValue(1.0)
            self.cy[1].setValue(-1.0)
        for c in self.cy:
            c.setVisible(on)

    def cursor_text(self):
        lines = []
        c = self.ch[self.sel]
        unit = split_unit(c.key.split("/", 1)[-1])[1] if c.key else ""
        r = self.raw[self.sel]
        if self.w_cx.isChecked():
            x1, x2 = self.cx[0].value(), self.cx[1].value()
            dt = x2 - x1
            lines.append(f"X1 {eng(x1, 's'):>11}  X2 {eng(x2, 's'):>11}")
            lines.append(f"Δt {eng(dt, 's'):>11}  1/Δt {eng(1 / dt if dt else float('nan'), 'Hz'):>11}")
            if r is not None and self.t_trig is not None and len(r[0]) > 1:
                v1 = float(np.interp(self.t_trig + x1, r[0], r[1]))
                v2 = float(np.interp(self.t_trig + x2, r[0], r[1]))
                lines.append(f"CH{self.sel + 1}@X1 {eng(v1, unit)}  @X2 {eng(v2, unit)}  Δ {eng(v2 - v1, unit)}")
        if self.w_cy.isChecked() and not c.digital:
            y1 = c.ofs + (self.cy[0].value() - c.pos) * c.vdiv
            y2 = c.ofs + (self.cy[1].value() - c.pos) * c.vdiv
            lines.append(f"CH{self.sel + 1} Y1 {eng(y1, unit)}  Y2 {eng(y2, unit)}  ΔY {eng(y2 - y1, unit)}")
        return "\n".join(lines)

    # ------------------------------------------------------------ main loop
    def tick(self):
        now = time.monotonic()
        t_start = time.perf_counter()
        self.refresh_sources()
        store = self.snap if self.snap is not None else self.acq
        span = HDIV * self.tdiv
        tc = self.ch[self.trig_ch]
        level = tc.thr if tc.digital else self.level
        extracted = [None] * NCH
        with store.lock:
            if self.running:
                ts, k = store.lookup(tc.key)
                found = None
                if ts is not None:
                    t, d = ts.view()
                    if len(t) > 1:
                        # newest time for which ALL live displayed sources have data (they arrive with
                        # different latencies) - so the right edge of the screen is never empty
                        horizon = float(t[-1])
                        for c in self.ch:
                            s, _ = store.lookup(c.key) if c.on else (None, -1)
                            if s is not None and s is not ts and now - s.last_wall < 0.5:
                                tt, _ = s.view()
                                if len(tt):
                                    horizon = min(horizon, float(tt[-1]))
                        right = max(0.0, self.delay + span / 2)
                        after = self.single_after if self.tmode == 2 else -np.inf
                        found = find_trigger(t, d[k], level, self.edge, horizon - right,
                                             max(4 * span, 0.25), after)
                if found is not None:
                    self.t_trig, self.trig_state, self.last_trig_wall = found, "TRIG'D", now
                elif self.tmode == 0 and now - self.last_trig_wall > 0.3:
                    newest = None
                    for c in self.ch:
                        s, _ = store.lookup(c.key) if c.on else (None, -1)
                        if s is not None:
                            tt, _ = s.view()
                            if len(tt):
                                newest = float(tt[-1]) if newest is None else max(newest, float(tt[-1]))
                    if newest is not None:
                        self.t_trig = newest - span / 2 - self.delay
                        self.trig_state = "AUTO"
                elif self.tmode != 0:
                    self.trig_state = "ARMED" if self.tmode == 2 else "WAIT"
        if self.t_trig is not None:
            tl = self.t_trig + self.delay - span / 2
            tr = tl + span
            for i, c in enumerate(self.ch):
                if not c.key or not (c.on or i == self.trig_ch or i == self.sel):
                    continue
                with store.lock:          # per channel, so the UDP receiver is never blocked for long
                    s, k = store.lookup(c.key)
                    if s is None:
                        continue
                    t, d = s.view()
                    if len(t) == 0:
                        continue
                    i0 = max(0, int(np.searchsorted(t, tl)) - 1)
                    i1 = min(len(t), int(np.searchsorted(t, tr, "right")) + 1)
                    if i1 - i0 >= 1:
                        extracted[i] = (t[i0:i1].copy(), d[k, i0:i1].astype(np.float64))
        if self.running and self.tmode == 2 and self.trig_state == "TRIG'D":
            self.stop()
        self.raw = extracted
        for i, c in enumerate(self.ch):
            if c.autoset_pending and c.on and extracted[i] is not None and len(extracted[i][1]) > 10:
                self.autoset(i, quiet=True)
        self.draw(extracted, span)
        self.fps = 0.95 * self.fps + 0.05 / max(1e-3, now - self._last_tick)
        self._last_tick = now
        self.frame_ms = 0.95 * self.frame_ms + 0.05 * (time.perf_counter() - t_start) * 1e3
        if now - self.last_meas > 0.2:
            self.last_meas = now
            self.update_measurements()
            self.update_side_info()

    def draw(self, ex, span):
        x0, x1 = self.delay - span / 2, self.delay + span / 2
        if self._xr != (x0, x1):
            self._xr = (x0, x1)
            self.pw_a.setXRange(x0, x1, padding=0)
            ticks = [(x0 + k * self.tdiv, eng(x0 + k * self.tdiv, "s")) for k in range(HDIV + 1)]
            for pw in (self.pw_a, self.pw_d):
                pw.getAxis("bottom").setTicks([ticks, []])
        dig = [i for i, c in enumerate(self.ch) if c.on and c.digital and c.key]
        nd = len(dig)
        ncol = max(200, int(self.pw_a.width()))
        sc = self.ch[self.sel]
        tkey = (self.sel, sc.vdiv, sc.pos, sc.ofs, sc.digital, sc.key, tuple(dig))
        if tkey != self._ticks_key:
            self._ticks_key = tkey
            unit = split_unit(sc.key.split("/", 1)[-1])[1] if sc.key else ""
            if sc.digital or not sc.key:
                yt = [(float(dv), "") for dv in range(-VDIV_N // 2, VDIV_N // 2 + 1)]
            else:
                yt = [(float(dv), eng(sc.ofs + (dv - sc.pos) * sc.vdiv, unit))
                      for dv in range(-VDIV_N // 2, VDIV_N // 2 + 1)]
            self.pw_a.getAxis("left").setTicks([yt, []])
            self.pw_a.getAxis("left").setLabel(f"CH{self.sel + 1}  {eng(sc.vdiv, unit)}/div",
                                               color=COLORS[self.sel])
            lanes = [(nd - 1 - j + 0.5, f"CH{i + 1} {self.ch[i].key.split('/', 1)[-1]}"[:16]) for j, i in enumerate(dig)]
            self.pw_d.getAxis("left").setTicks([lanes, [(float(j), "") for j in range(nd + 1)]])
        if nd != self._nd:
            self._nd = nd
            self.pw_d.setVisible(nd > 0)
            self.pw_d.setFixedHeight(40 + 24 * nd)
            self.pw_d.setYRange(0, max(1, nd), padding=0)
            self.pw_a.getAxis("bottom").setStyle(showValues=(nd == 0))

        lane = {i: nd - 1 - j for j, i in enumerate(dig)}
        for i, c in enumerate(self.ch):
            ca, cd, mk = self.curves_a[i], self.curves_d[i], self.markers[i]
            r = ex[i]
            if not c.on or r is None or self.t_trig is None:
                ca.setData([], [])
                cd.setData([], [])
                mk.setVisible(bool(c.on and c.key and not c.digital))
                if mk.isVisible():
                    mk.setPos(x0, c.pos)
                continue
            t, v = r
            x = t - self.t_trig
            if c.digital:
                ca.setData([], [])
                mk.setVisible(False)
                b = (v > c.thr).astype(np.float64)
                if len(x) <= 2 * ncol:
                    X, Y = np.repeat(x, 2)[1:], np.repeat(b, 2)[:-1]
                else:
                    X, Y = peak_decimate(x, b, x0, x1, ncol)
                cd.setData(X, lane[i] + 0.15 + 0.7 * Y)
            else:
                cd.setData([], [])
                mk.setVisible(True)
                mk.setPos(x0, c.pos)
                if c.coup == "GND":
                    ca.setData([x0, x1], [c.pos, c.pos])
                    continue
                ref = float(np.mean(v)) if c.coup == "AC" else c.ofs
                X, Y = peak_decimate(x, v, x0, x1, ncol)
                Y = np.clip(c.pos + (Y - ref) / c.vdiv, -VDIV_N, VDIV_N)
                sparse = len(x) < ncol / 25          # zoomed past the sample rate: mark real samples
                if sparse != self._sym[i]:
                    self._sym[i] = sparse
                    ca.setSymbol("o" if sparse else None)
                    ca.setSymbolSize(4)
                    ca.setSymbolBrush(COLORS[i])
                    ca.setSymbolPen(None)
                ca.setData(X, Y)
        tc = self.ch[self.trig_ch]
        show = bool(tc.key) and not tc.digital
        self.trig_line.setVisible(show)
        if show and not self.trig_line.moving:
            if self._tl_ch != self.trig_ch:
                self._tl_ch = self.trig_ch
                self.trig_line.setPen(pg.mkPen(COLORS[self.trig_ch], style=Qt.PenStyle.DashLine))
            self.trig_line.setValue(tc.pos + (self.level - tc.ofs) / tc.vdiv)
        # status line
        run = "<span style='color:#4cff6a'>RUN</span>" if self.running else "<span style='color:#ff5252'>STOP</span>"
        stc = {"TRIG'D": "#4cff6a", "AUTO": "#ffe000", "WAIT": "#ff9a2e", "ARMED": "#ff9a2e", "STOP": "#ff5252"}
        tunit = split_unit(tc.key.split("/", 1)[-1])[1] if tc.key else ""
        lvl = tc.thr if tc.digital else self.level
        tt = "--" if self.t_trig is None else f"{self.t_trig:.6f} s"
        self.status.setText(
            f"<b>{run}&nbsp;&nbsp;<span style='color:{stc.get(self.trig_state, '#fff')}'>{self.trig_state}</span>"
            f"&nbsp;&nbsp;|&nbsp;&nbsp;{eng(self.tdiv, 's')}/div&nbsp;&nbsp;delay {eng(self.delay, 's')}"
            f"&nbsp;&nbsp;|&nbsp;&nbsp;T: <span style='color:{COLORS[self.trig_ch]}'>CH{self.trig_ch + 1}</span> "
            f"{EDGES[self.edge].split()[1]} {eng(lvl, tunit)} {TMODES[self.tmode]}"
            f"&nbsp;&nbsp;|&nbsp;&nbsp;t<sub>trig</sub> = {tt}</b>")

    def update_measurements(self):
        for i, c in enumerate(self.ch):
            name = c.key.split("/", 1)[-1] if c.key else ""
            unit = split_unit(name)[1]
            cells = [name] + [""] * 8
            if c.on and c.key and c.coup != "GND" and self.raw[i] is not None and self.t_trig is not None:
                t, v = self.raw[i]
                span = HDIV * self.tdiv
                tl = self.t_trig + self.delay - span / 2
                msk = (t >= tl) & (t <= tl + span)
                t, v = t[msk], v[msk]
                if c.digital:
                    v = (v > c.thr).astype(np.float64)
                    unit = ""
                m = measure(t, v)
                if m:
                    f = m["freq"]
                    cells = [name, eng(f, "Hz"), eng(1 / f, "s") if f else "--",
                             f"{100 * m['duty']:.1f} %" if m["duty"] is not None else "--",
                             eng(m["pp"], unit), eng(m["max"], unit), eng(m["min"], unit),
                             eng(m["mean"], unit), eng(m["rms"], unit)]
            for k, txt in enumerate(cells):
                self.meas.item(i, k + 1).setText(txt)
        self.cur_lab.setText(self.cursor_text())

    def update_side_info(self):
        lines = []
        now = time.monotonic()
        with self.acq.lock:
            for n in sorted(self.acq.sources):
                s = self.acq.sources[n]
                t, _ = s.view()
                depth = (t[-1] - t[0]) if len(t) > 1 else 0.0
                live = "live" if now - s.last_wall < 1.0 else "idle"
                drop = f"  GAPS {s.gaps}" if s.gaps else ""
                lines.append(f"{n:<8} {len(s.sigs):>2} sig  {eng(s.rate, 'S/s'):>10}  mem {eng(depth, 's'):>9}{drop}  "
                             f"t={t[-1] if len(t) else 0:9.4f}s  {live}")
        if not lines:
            lines = ["no data yet - start bldc_sim.py (auto-probed) or publish with scope_probe.ScopeProbe"]
        bad = f", {self.acq.errors} rejected" if self.acq.errors else ""
        lines.append(f"display {self.fps:4.1f} fps, {self.frame_ms:4.1f} ms/frame, {self.acq.packets} packets{bad}, "
                     f"UDP buffer {self.acq.rcvbuf // 1024} kB")
        if self.snap is not None:
            lines.append("[STOPPED: showing frozen acquisition memory - zoom/pan still work]")
        self.src_lab.setText("\n".join(lines))

    # ------------------------------------------------------------ files
    def setup_dict(self):
        return dict(ch=[{k: v for k, v in asdict(c).items() if k != "autoset_pending"} for c in self.ch],
                    tdiv=self.tdiv, delay=self.delay, trig_ch=self.trig_ch, edge=self.edge,
                    level=self.level, tmode=self.tmode if self.tmode != 2 else 0, sel=self.sel)

    def save_setup(self):
        try:
            with open(self.setup_path, "w") as f:
                json.dump(self.setup_dict(), f, indent=1)
            self.statusBar().showMessage(f"setup saved to {self.setup_path}", 3000)
        except OSError as ex:
            self.statusBar().showMessage(f"setup save failed: {ex}", 5000)

    def load_default(self):
        st = default_setup()
        self.ch = [Ch(**c) for c in st["ch"]]
        self.tdiv, self.delay = st["tdiv"], st["delay"]
        self.trig_ch, self.edge, self.level, self.tmode = st["trig_ch"], st["edge"], st["level"], st["tmode"]
        self.sync_all()

    def save_capture(self):
        os.makedirs(self.args.capture_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.args.capture_dir, f"scope_{stamp}")
        self.grab().save(base + ".png")
        store = self.snap if self.snap is not None else self.acq
        span = HDIV * self.tdiv
        files = [base + ".png"]
        if self.t_trig is not None:
            tl = self.t_trig + self.delay - span / 2
            by_src = {}
            for i, c in enumerate(self.ch):
                if c.on and c.key:
                    by_src.setdefault(c.key.split("/", 1)[0], []).append((i, c.key.split("/", 1)[1]))
            with store.lock:
                for sname, lst in by_src.items():
                    s = store.sources.get(sname)
                    if s is None:
                        continue
                    t, d = s.view()
                    i0, i1 = np.searchsorted(t, tl), np.searchsorted(t, tl + span, "right")
                    cols = [t[i0:i1] - self.t_trig]
                    hdr = ["t_rel_trigger[s]"]
                    for i, sig in lst:
                        if sig in s.sigs:
                            cols.append(d[s.sigs.index(sig), i0:i1])
                            hdr.append(f"CH{i + 1}:{sig}")
                    fn = f"{base}_{sname}.csv"
                    np.savetxt(fn, np.column_stack(cols), delimiter=",", header=",".join(hdr), comments="",
                               fmt="%.9g")
                    files.append(fn)
        self.statusBar().showMessage("saved " + ", ".join(os.path.basename(f) for f in files), 6000)
        return files

    def closeEvent(self, ev):
        if not self.args.no_save_setup:
            self.save_setup()
        self.acq.stop()
        super().closeEvent(ev)


def parse_target(s):
    h, p = s.rsplit(":", 1)
    return h, int(p)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="12-channel oscilloscope digital twin")
    ap.add_argument("--port", type=int, default=DEFAULT_SCOPE_PORT, help="UDP port probes send to")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--depth", type=int, default=400_000, help="memory depth per source (samples)")
    ap.add_argument("--sim", action="append", default=None, help="motor sim(s) to auto-probe host:port (default 127.0.0.1:9000)")
    ap.add_argument("--no-sim", action="store_true", help="do not auto-probe the motor sim")
    ap.add_argument("--sim-decim", type=int, default=1, help="ask the sim for every Nth physics sample")
    ap.add_argument("--setup", default=os.path.join(here, "scope_setup.json"))
    ap.add_argument("--default-setup", action="store_true")
    ap.add_argument("--no-save-setup", action="store_true")
    ap.add_argument("--capture-dir", default="captures")
    ap.add_argument("--opengl", action="store_true", help="use OpenGL for plotting")
    ap.add_argument("--run-seconds", type=float, default=0, help="(testing) quit after N s")
    ap.add_argument("--screenshot", default=None, help="(testing) save window PNG before quitting")
    ap.add_argument("--dump", action="store_true", help="(testing) print measurements before quitting")
    args = ap.parse_args()
    targets = [] if args.no_sim else [parse_target(s) for s in (args.sim or ["127.0.0.1:9000"])]

    if args.opengl:
        pg.setConfigOptions(useOpenGL=True)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    for role, col in [("Window", "#1b1e26"), ("WindowText", "#d6d9e0"), ("Base", "#12141a"),
                      ("AlternateBase", "#1b1e26"), ("Text", "#d6d9e0"), ("Button", "#262a35"),
                      ("ButtonText", "#d6d9e0"), ("Highlight", "#3d5afe"), ("HighlightedText", "#ffffff")]:
        pal.setColor(getattr(QtGui.QPalette.ColorRole, role), QtGui.QColor(col))
    app.setPalette(pal)
    try:
        acq = Acquisition(args.bind, args.port, args.depth, targets, max(1, args.sim_decim))
    except OSError as ex:
        print(f"cannot bind UDP {args.bind}:{args.port}: {ex}  (another scope running?)")
        sys.exit(1)
    win = ScopeWindow(acq, args)
    win.resize(1680, 980)
    win.show()
    if args.run_seconds:
        def finish():
            win.update_measurements()
            win.update_side_info()
            if args.dump:
                print("STATUS:", win.trig_state, "t_trig", win.t_trig)
                print(win.src_lab.text())
                for r in range(NCH):
                    print(" | ".join(win.meas.item(r, c).text() for c in range(10)))
            if args.screenshot:
                win.grab().save(args.screenshot)
            win.close()
        QtCore.QTimer.singleShot(int(args.run_seconds * 1000), finish)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
