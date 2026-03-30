"""
Robot Arm Teach Pendant — PyQt5
4 DOF: Base (rotazione orizzontale) + 3 giunti nel piano verticale
Comunicazione seriale con Arduino via PCA9685

Modifiche:
  - Gomito specchiato: l'angolo J2 viene inviato negato ad Arduino
  - Pannello WRIST: slider dedicato per il servo polso (J3)
  - Interruttore elettromagnete con indicatore LED visivo
"""

import sys
import math
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QPushButton, QComboBox, QSlider, QGridLayout,
    QStatusBar, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal, QTimer
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QPainterPath
)

# ─── Costanti braccio ───────────────────────────────────────────────────────
SEG_LENGTHS     = [120, 100, 70]
BASE_ANGLE_MIN  = 0
BASE_ANGLE_MAX  = 180
JOINT_ANGLE_MIN = -90
JOINT_ANGLE_MAX = 90

# ─── Palette colori ─────────────────────────────────────────────────────────
C_BG      = QColor("#0d0f14")
C_PANEL   = QColor("#13161e")
C_BORDER  = QColor("#1e2330")
C_ACCENT  = QColor("#00e5ff")
C_ACCENT2 = QColor("#ff6b35")
C_JOINT   = QColor("#ffffff")
C_SEG1    = QColor("#00e5ff")
C_SEG2    = QColor("#0099bb")
C_SEG3    = QColor("#006688")
C_GRIP    = QColor("#ff6b35")
C_TEXT    = QColor("#c8d0e0")
C_TEXTDIM = QColor("#4a5568")
C_GREEN   = QColor("#00ff88")
C_RED     = QColor("#ff3355")
C_MAGNET  = QColor("#ff2266")


# ═══════════════════════════════════════════════════════════════════════════
#  Widget Semicirconferenza — controllo BASE
# ═══════════════════════════════════════════════════════════════════════════
class BaseWidget(QWidget):
    angleChanged = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 170)
        self._angle    = 90.0
        self._dragging = False

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, v):
        self._angle = max(BASE_ANGLE_MIN, min(BASE_ANGLE_MAX, v))
        self.update()

    def _center(self):
        return QPointF(self.width() / 2, self.height() - 24)

    def _radius(self):
        return min(self.width() / 2 - 20, self.height() - 40)

    def _angle_to_point(self, deg):
        c   = self._center()
        r   = self._radius()
        rad = math.radians(180 - deg)
        return QPointF(c.x() + r * math.cos(rad), c.y() + r * math.sin(rad))

    def _point_to_angle(self, pos):
        c  = self._center()
        dx = pos.x() - c.x()
        dy = c.y() - pos.y()
        if dy < -5:
            return None
        deg = math.degrees(math.atan2(dy, dx))
        deg = 180 - deg
        return max(0.0, min(180.0, deg))

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c        = self._center()
        r        = self._radius()
        arc_rect = QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r)

        p.setPen(QPen(C_BORDER, 2))
        p.setBrush(Qt.NoBrush)
        p.drawArc(arc_rect, 0, 180 * 16)

        for deg in range(0, 181, 30):
            rad    = math.radians(180 - deg)
            pt_out = self._angle_to_point(deg)
            pt_in  = QPointF(c.x() + (r - 14) * math.cos(rad),
                             c.y() + (r - 14) * math.sin(rad))
            p.setPen(QPen(C_TEXTDIM, 1))
            p.drawLine(pt_in, pt_out)
            lbl = QPointF(c.x() + (r + 16) * math.cos(rad),
                          c.y() + (r + 16) * math.sin(rad))
            p.setPen(QPen(C_TEXTDIM))
            p.setFont(QFont("Courier New", 7))
            p.drawText(QRectF(lbl.x() - 15, lbl.y() - 8, 30, 16),
                       Qt.AlignCenter, f"{deg}°")

        p.setPen(QPen(C_ACCENT, 3, Qt.SolidLine, Qt.RoundCap))
        if self._angle >= 90:
            p.drawArc(arc_rect, 90 * 16, int((self._angle - 90) * 16))
        else:
            p.drawArc(arc_rect, int(self._angle * 16),
                      int((90 - self._angle) * 16))

        tip = self._angle_to_point(self._angle)
        p.setPen(QPen(C_ACCENT, 2))
        p.drawLine(c, tip)

        p.setBrush(QBrush(C_ACCENT))
        p.setPen(Qt.NoPen)
        p.drawEllipse(c, 6, 6)

        p.setBrush(QBrush(C_ACCENT if not self._dragging else C_ACCENT2))
        p.setPen(QPen(C_BG, 2))
        p.drawEllipse(tip, 10, 10)

        p.setPen(QPen(C_ACCENT))
        p.setFont(QFont("Courier New", 13, QFont.Bold))
        p.drawText(QRectF(c.x() - 40, c.y() + 6, 80, 22),
                   Qt.AlignCenter, f"{self._angle:.1f}°")

    def mousePressEvent(self, e):
        tip = self._angle_to_point(self._angle)
        if math.hypot(e.pos().x() - tip.x(), e.pos().y() - tip.y()) < 18:
            self._dragging = True

    def mouseMoveEvent(self, e):
        if self._dragging:
            a = self._point_to_angle(QPointF(e.pos()))
            if a is not None:
                self._angle = a
                self.update()
                self.angleChanged.emit(self._angle)

    def mouseReleaseEvent(self, e):
        self._dragging = False
        self.update()


# ═══════════════════════════════════════════════════════════════════════════
#  Widget Braccio 2D — spalla + gomito trascinabili  (polso via slider)
# ═══════════════════════════════════════════════════════════════════════════
class ArmWidget(QWidget):
    anglesChanged = pyqtSignal(list)   # [j1, j2, j3]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 360)
        self._angles    = [0.0, 0.0, 0.0]   # spalla, gomito, polso
        self._dragging  = None
        self._joints    = []
        self._magnet_on = False
        self._compute_joints()

    @property
    def angles(self):
        return list(self._angles)

    def set_angle(self, idx, val):
        self._angles[idx] = max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, val))
        self._compute_joints()
        self.update()

    def set_magnet_visual(self, on: bool):
        self._magnet_on = on
        self.update()

    def _origin(self):
        return QPointF(self.width() / 2, self.height() - 40)

    def _compute_joints(self):
        o          = self._origin()
        joints     = [o]
        cumulative = -90.0
        for length, angle in zip(SEG_LENGTHS, self._angles):
            cumulative += angle
            rad  = math.radians(cumulative)
            prev = joints[-1]
            joints.append(QPointF(prev.x() + length * math.cos(rad),
                                  prev.y() + length * math.sin(rad)))
        self._joints = joints

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        o = self._origin()

        # Griglia
        p.setPen(QPen(C_BORDER, 1))
        for x in range(0, self.width(), 30):
            p.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), 30):
            p.drawLine(0, y, self.width(), y)

        # Base
        bw = 50
        p.setBrush(QBrush(C_BORDER))
        p.setPen(Qt.NoPen)
        p.drawRect(QRectF(o.x() - bw/2, o.y(), bw, 14))
        p.setPen(QPen(C_ACCENT, 2))
        p.drawLine(QPointF(o.x() - bw/2, o.y()), QPointF(o.x() + bw/2, o.y()))

        # Segmenti
        seg_colors = [C_SEG1, C_SEG2, C_SEG3]
        for i in range(len(SEG_LENGTHS)):
            pen = QPen(seg_colors[i], 8, Qt.SolidLine, Qt.RoundCap)
            p.setPen(QPen(QColor(0, 0, 0, 60), 12, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(self._joints[i], self._joints[i + 1])
            p.setPen(pen)
            p.drawLine(self._joints[i], self._joints[i + 1])

        # Giunti
        for i, pt in enumerate(self._joints[:-1]):
            r = 10 if i == 0 else 8
            p.setBrush(QBrush(C_JOINT))
            p.setPen(QPen(C_BG, 2))
            p.drawEllipse(pt, r, r)
            if i > 0:
                p.setPen(QPen(C_TEXTDIM))
                p.setFont(QFont("Courier New", 7))
                angle_val = self._angles[i - 1]
                # Gomito (i==2): mostra anche il valore specchiato inviato
                if i == 2:
                    label = f"J{i}: {angle_val:.0f}°  (→ {-angle_val:.0f}°)"
                else:
                    label = f"J{i}: {angle_val:.0f}°"
                p.drawText(QRectF(pt.x() + 12, pt.y() - 8, 150, 14),
                           Qt.AlignLeft, label)

        # Handle trascinabili: solo spalla (joint 1) e gomito (joint 2)
        handle_colors = [C_SEG1, C_SEG2]
        for i in range(1, 3):
            if i >= len(self._joints):
                continue
            pt  = self._joints[i]
            col = handle_colors[i - 1]
            p.setBrush(QBrush(C_ACCENT2 if self._dragging == i - 1 else col))
            p.setPen(QPen(C_BG, 2))
            p.drawEllipse(pt, 11, 11)

        # End-effector / magnete
        tip      = self._joints[-1]
        grip_col = C_MAGNET if self._magnet_on else C_GRIP
        p.setBrush(QBrush(grip_col))
        p.setPen(QPen(C_BG, 2))
        p.drawEllipse(tip, 9, 9)
        # Glow magnete
        if self._magnet_on:
            glow = QColor(C_MAGNET)
            glow.setAlpha(50)
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(glow, 6))
            p.drawEllipse(tip, 14, 14)
        p.setPen(QPen(grip_col, 2))
        ang_rad = math.radians(sum(self._angles) - 90)
        for sign in [1, -1]:
            jaw = QPointF(tip.x() + 15 * math.cos(ang_rad + sign * 0.4),
                          tip.y() + 15 * math.sin(ang_rad + sign * 0.4))
            p.drawLine(tip, jaw)

        # Coordinate EE
        rel_x = tip.x() - o.x()
        rel_y = o.y() - tip.y()
        p.setPen(QPen(C_TEXTDIM))
        p.setFont(QFont("Courier New", 8))
        p.drawText(8, self.height() - 8,
                   f"EE  x:{rel_x:.0f}  y:{rel_y:.0f} px")

    def _hit_joint(self, pos):
        for i in range(1, 3):    # solo spalla e gomito
            if i >= len(self._joints):
                continue
            pt = self._joints[i]
            if math.hypot(pos.x() - pt.x(), pos.y() - pt.y()) < 18:
                return i - 1
        return None

    def mousePressEvent(self, e):
        hit = self._hit_joint(QPointF(e.pos()))
        if hit is not None:
            self._dragging = hit

    def mouseMoveEvent(self, e):
        if self._dragging is None:
            return
        prev   = self._joints[self._dragging]
        mouse  = QPointF(e.pos())
        dx, dy = mouse.x() - prev.x(), mouse.y() - prev.y()
        target = math.degrees(math.atan2(dy, dx))
        cum    = -90.0
        for k in range(self._dragging):
            cum += self._angles[k]
        relative = max(JOINT_ANGLE_MIN, min(JOINT_ANGLE_MAX, target - cum))
        self._angles[self._dragging] = relative
        self._compute_joints()
        self.update()
        self.anglesChanged.emit(self._angles)

    def mouseReleaseEvent(self, e):
        self._dragging = None
        self.update()


# ═══════════════════════════════════════════════════════════════════════════
#  Widget LED — indicatore visivo ON/OFF
# ═══════════════════════════════════════════════════════════════════════════
class LedWidget(QWidget):
    def __init__(self, color_on=C_GREEN, parent=None):
        super().__init__(parent)
        self._on       = False
        self._color_on = color_on
        self.setFixedSize(22, 22)

    def set_state(self, on: bool):
        self._on = on
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        col = self._color_on if self._on else C_BORDER
        p.setBrush(QBrush(col))
        p.setPen(QPen(col.darker(150), 1))
        p.drawEllipse(2, 2, 18, 18)
        if self._on:
            glow = QColor(col)
            glow.setAlpha(55)
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(glow, 5))
            p.drawEllipse(0, 0, 22, 22)


# ═══════════════════════════════════════════════════════════════════════════
#  Finestra Principale
# ═══════════════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teach Pendant — 4 DOF Robot Arm")
        self.setMinimumSize(1060, 640)
        self._serial = None
        self._angles = {"base": 90.0, "j1": 0.0, "j2": 0.0, "j3": 0.0}
        self._magnet = False

        self._setup_ui()
        self._apply_style()

    # ── UI ────────────────────────────────────────────────────────────────
    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 8)
        root.setSpacing(8)

        # Titolo
        title = QLabel("⚙  TEACH PENDANT  |  4 DOF Robotic Arm")
        title.setFont(QFont("Courier New", 13, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f"color: {C_ACCENT.name()}; letter-spacing: 2px;")
        root.addWidget(title)

        # ── Riga principale ───────────────────────────────────────────────
        main_row = QHBoxLayout()
        main_row.setSpacing(10)

        # ─ BASE ─
        base_panel = self._make_panel("BASE ROTATION")
        self.base_widget = BaseWidget()
        self.base_widget.angleChanged.connect(self._on_base_changed)
        base_panel.layout().addWidget(self.base_widget)
        self.base_slider = QSlider(Qt.Horizontal)
        self.base_slider.setRange(0, 180)
        self.base_slider.setValue(90)
        self.base_slider.valueChanged.connect(self._on_base_slider)
        base_panel.layout().addWidget(QLabel("Fine control:"))
        base_panel.layout().addWidget(self.base_slider)
        main_row.addWidget(base_panel, 3)

        # ─ BRACCIO ─
        arm_panel = self._make_panel("ARM — VERTICAL PLANE  (drag ● joints)")
        self.arm_widget = ArmWidget()
        self.arm_widget.anglesChanged.connect(self._on_arm_changed)
        arm_panel.layout().addWidget(self.arm_widget)
        main_row.addWidget(arm_panel, 4)

        # ─ PANNELLO DESTRA: WRIST + MAGNETE ─
        right_col = QVBoxLayout()
        right_col.setSpacing(10)

        # · WRIST ·
        wrist_panel = self._make_panel("WRIST  (J3)")
        wl = QVBoxLayout()
        wl.setSpacing(8)

        self.wrist_lbl = QLabel("0°")
        self.wrist_lbl.setFont(QFont("Courier New", 24, QFont.Bold))
        self.wrist_lbl.setAlignment(Qt.AlignCenter)
        self.wrist_lbl.setStyleSheet(f"color: {C_SEG3.name()};")
        wl.addWidget(self.wrist_lbl)

        self.wrist_slider = QSlider(Qt.Vertical)
        self.wrist_slider.setRange(JOINT_ANGLE_MIN, JOINT_ANGLE_MAX)
        self.wrist_slider.setValue(0)
        self.wrist_slider.setMinimumHeight(170)
        self.wrist_slider.valueChanged.connect(self._on_wrist_slider)
        wl.addWidget(self.wrist_slider, alignment=Qt.AlignHCenter)

        # Etichette min/centro/max
        tick_row = QHBoxLayout()
        for t in [f"{JOINT_ANGLE_MIN}°", "0°", f"{JOINT_ANGLE_MAX}°"]:
            lbl = QLabel(t)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(f"color:{C_TEXTDIM.name()}; font-size:9px;")
            tick_row.addWidget(lbl)
        wl.addLayout(tick_row)

        reset_wrist = QPushButton("⟳  CENTER")
        reset_wrist.clicked.connect(lambda: self.wrist_slider.setValue(0))
        wl.addWidget(reset_wrist)

        wrist_panel.layout().addLayout(wl)
        right_col.addWidget(wrist_panel, 3)

        # · ELETTROMAGNETE ·
        mag_panel = self._make_panel("ELECTROMAGNET")
        ml = QVBoxLayout()
        ml.setSpacing(12)

        # LED + label stato
        led_row = QHBoxLayout()
        self.mag_led = LedWidget(color_on=C_MAGNET)
        self.mag_status_lbl = QLabel("OFF")
        self.mag_status_lbl.setFont(QFont("Courier New", 16, QFont.Bold))
        self.mag_status_lbl.setStyleSheet(f"color:{C_TEXTDIM.name()};")
        led_row.addStretch()
        led_row.addWidget(self.mag_led)
        led_row.addSpacing(6)
        led_row.addWidget(self.mag_status_lbl)
        led_row.addStretch()
        ml.addLayout(led_row)

        # Pulsante toggle principale
        self.mag_btn = QPushButton("⚡  ACTIVATE")
        self.mag_btn.setCheckable(True)
        self.mag_btn.setMinimumHeight(54)
        self.mag_btn.setFont(QFont("Courier New", 11, QFont.Bold))
        self.mag_btn.toggled.connect(self._on_magnet_toggle)
        ml.addWidget(self.mag_btn)

        # Pulsante PULSE
        self.mag_pulse_btn = QPushButton("▶  PULSE  (500 ms)")
        self.mag_pulse_btn.clicked.connect(self._magnet_pulse)
        ml.addWidget(self.mag_pulse_btn)

        mag_panel.layout().addLayout(ml)
        right_col.addWidget(mag_panel, 2)
        right_col.addStretch(1)

        right_w = QWidget()
        right_w.setLayout(right_col)
        right_w.setMinimumWidth(185)
        main_row.addWidget(right_w, 2)

        root.addLayout(main_row, 1)

        # ── Bottom: angoli + seriale ──────────────────────────────────────
        bottom = QHBoxLayout()
        bottom.setSpacing(10)

        # Display angoli inviati
        angles_grp = self._make_panel("JOINT ANGLES  (values sent to Arduino)")
        ag = QGridLayout()
        ag.setSpacing(6)
        rows = [
            ("Base",                "base", C_ACCENT),
            ("J1  Shoulder",        "j1",   C_SEG1),
            ("J2  Elbow  [mirror]", "j2",   C_SEG2),
            ("J3  Wrist",           "j3",   C_SEG3),
        ]
        self._angle_labels = {}
        for ri, (name, key, col) in enumerate(rows):
            ag.addWidget(QLabel(name + ":"), ri, 0)
            val = QLabel("0.0°")
            val.setFont(QFont("Courier New", 11, QFont.Bold))
            val.setStyleSheet(f"color:{col.name()};")
            val.setAlignment(Qt.AlignRight)
            ag.addWidget(val, ri, 1)
            self._angle_labels[key] = val
        self._angle_labels["base"].setText("90.0°")
        angles_grp.layout().addLayout(ag)
        bottom.addWidget(angles_grp, 1)

        # Seriale
        serial_grp = self._make_panel("SERIAL / ARDUINO")
        sl = QVBoxLayout()

        port_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self._refresh_ports()
        refresh_btn = QPushButton("⟳")
        refresh_btn.setFixedWidth(32)
        refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(QLabel("Port:"))
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(refresh_btn)
        sl.addLayout(port_row)

        baud_row = QHBoxLayout()
        self.baud_combo = QComboBox()
        for b in ["9600", "19200", "57600", "115200"]:
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText("115200")
        baud_row.addWidget(QLabel("Baud:"))
        baud_row.addWidget(self.baud_combo, 1)
        sl.addLayout(baud_row)

        btn_row = QHBoxLayout()
        self.connect_btn = QPushButton("CONNECT")
        self.connect_btn.clicked.connect(self._toggle_serial)
        self.send_btn = QPushButton("▶  SEND")
        self.send_btn.clicked.connect(self._send_angles)
        self.send_btn.setEnabled(False)
        btn_row.addWidget(self.connect_btn)
        btn_row.addWidget(self.send_btn)
        sl.addLayout(btn_row)

        self.auto_send_btn = QPushButton("AUTO SEND: OFF")
        self.auto_send_btn.setCheckable(True)
        self.auto_send_btn.toggled.connect(self._toggle_auto_send)
        sl.addWidget(self.auto_send_btn)

        serial_grp.layout().addLayout(sl)
        bottom.addWidget(serial_grp, 1)

        root.addLayout(bottom)

        # Status bar
        self.statusBar().showMessage(
            "Ready.  Drag ● for shoulder/elbow · slider for wrist · toggle magnet.")
        self.statusBar().setStyleSheet(
            f"background:{C_PANEL.name()}; color:{C_TEXTDIM.name()};"
            " font-family:'Courier New'; font-size:10px;")

        # Timers
        self._auto_timer  = QTimer()
        self._auto_timer.setInterval(100)
        self._auto_timer.timeout.connect(self._send_angles)

        self._pulse_timer = QTimer()
        self._pulse_timer.setSingleShot(True)
        self._pulse_timer.timeout.connect(self._magnet_pulse_off)

    # ── Factory panel ─────────────────────────────────────────────────────
    def _make_panel(self, title: str) -> QGroupBox:
        gb = QGroupBox(title)
        gb.setLayout(QVBoxLayout())
        gb.layout().setContentsMargins(8, 16, 8, 8)
        gb.layout().setSpacing(6)
        return gb

    # ── Stile globale ─────────────────────────────────────────────────────
    def _apply_style(self):
        self.setStyleSheet(f"""
        QMainWindow, QWidget {{
            background-color: {C_BG.name()};
            color: {C_TEXT.name()};
            font-family: 'Courier New';
        }}
        QGroupBox {{
            border: 1px solid {C_BORDER.name()};
            border-radius: 6px;
            margin-top: 10px;
            font-size: 9px;
            font-weight: bold;
            color: {C_TEXTDIM.name()};
            letter-spacing: 1.5px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
        }}
        QPushButton {{
            background: {C_BORDER.name()};
            border: 1px solid {C_ACCENT.name()}44;
            border-radius: 4px;
            color: {C_ACCENT.name()};
            padding: 5px 12px;
            font-family: 'Courier New';
            font-size: 10px;
            font-weight: bold;
        }}
        QPushButton:hover   {{ background:{C_ACCENT.name()}22; border-color:{C_ACCENT.name()}; }}
        QPushButton:pressed {{ background:{C_ACCENT.name()}44; }}
        QPushButton:checked {{
            background:{C_MAGNET.name()}22;
            border-color:{C_MAGNET.name()};
            color:{C_MAGNET.name()};
        }}
        QPushButton:disabled {{ color:{C_TEXTDIM.name()}; border-color:{C_BORDER.name()}; }}
        QComboBox {{
            background:{C_PANEL.name()}; border:1px solid {C_BORDER.name()};
            border-radius:4px; color:{C_TEXT.name()}; padding:3px 8px;
        }}
        QSlider::groove:horizontal {{
            height:4px; background:{C_BORDER.name()}; border-radius:2px;
        }}
        QSlider::groove:vertical {{
            width:4px; background:{C_BORDER.name()}; border-radius:2px;
        }}
        QSlider::handle:horizontal, QSlider::handle:vertical {{
            background:{C_ACCENT.name()}; width:14px; height:14px;
            margin:-5px; border-radius:7px;
        }}
        QSlider::sub-page:horizontal {{ background:{C_ACCENT.name()}88; border-radius:2px; }}
        QSlider::add-page:vertical   {{ background:{C_SEG3.name()}88; border-radius:2px; }}
        QLabel {{ color:{C_TEXT.name()}; font-size:10px; }}
        """)

    # ── Callbacks BASE ────────────────────────────────────────────────────
    def _on_base_changed(self, angle):
        self._angles["base"] = angle
        self._angle_labels["base"].setText(f"{angle:.1f}°")
        self.base_slider.blockSignals(True)
        self.base_slider.setValue(int(angle))
        self.base_slider.blockSignals(False)

    def _on_base_slider(self, val):
        self.base_widget.angle   = float(val)
        self._angles["base"]     = float(val)
        self._angle_labels["base"].setText(f"{val}.0°")

    # ── Callbacks BRACCIO ─────────────────────────────────────────────────
    def _on_arm_changed(self, angles):
        self._angles["j1"] = angles[0]
        self._angles["j2"] = angles[1]   # valore visivo; verrà negato all'invio
        self._angle_labels["j1"].setText(f"{angles[0]:.1f}°")
        # Mostra entrambi: visivo e quello inviato
        self._angle_labels["j2"].setText(
            f"{angles[1]:.1f}°  →  {-angles[1]:.1f}°")

    # ── Callbacks POLSO ───────────────────────────────────────────────────
    def _on_wrist_slider(self, val):
        self._angles["j3"] = float(val)
        self.wrist_lbl.setText(f"{val}°")
        self._angle_labels["j3"].setText(f"{val}.0°")
        self.arm_widget.set_angle(2, float(val))

    # ── Callbacks MAGNETE ─────────────────────────────────────────────────
    def _on_magnet_toggle(self, checked):
        self._magnet = checked
        self.mag_led.set_state(checked)
        if checked:
            self.mag_status_lbl.setText("ON")
            self.mag_status_lbl.setStyleSheet(f"color:{C_MAGNET.name()};")
            self.mag_btn.setText("⚡  DEACTIVATE")
        else:
            self.mag_status_lbl.setText("OFF")
            self.mag_status_lbl.setStyleSheet(f"color:{C_TEXTDIM.name()};")
            self.mag_btn.setText("⚡  ACTIVATE")
        self.arm_widget.set_magnet_visual(checked)
        self._send_angles()   # invio immediato al cambio stato

    def _magnet_pulse(self):
        if not self._magnet:
            self.mag_btn.setChecked(True)
        self._pulse_timer.start(500)

    def _magnet_pulse_off(self):
        self.mag_btn.setChecked(False)

    # ── Seriale ───────────────────────────────────────────────────────────
    def _refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("(no ports found)")

    def _toggle_serial(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
            self.connect_btn.setText("CONNECT")
            self.connect_btn.setStyleSheet("")
            self.send_btn.setEnabled(False)
            self.auto_send_btn.setChecked(False)
            self.statusBar().showMessage("Disconnected.")
        else:
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            try:
                self._serial = serial.Serial(port, baud, timeout=1)
                self.connect_btn.setText("DISCONNECT")
                self.connect_btn.setStyleSheet(
                    f"background:{C_GREEN.name()}22;"
                    f"border-color:{C_GREEN.name()}; color:{C_GREEN.name()};")
                self.send_btn.setEnabled(True)
                self.statusBar().showMessage(
                    f"Connected to {port} @ {baud} baud")
            except Exception as ex:
                self.statusBar().showMessage(f"Error: {ex}")

    def _send_angles(self):
        """
        Formato: B<base> J1<j1> J2<j2_mirror> J3<j3> M<0|1>
        Esempio: B090 J1+20 J2-35 J3+10 M0
        Il gomito (J2) viene SPECCHIATO (negato) prima dell'invio.
        """
        b  = int(self._angles["base"])
        j1 = int(self._angles["j1"])
        j2 = int(-self._angles["j2"])   # ← SPECCHIATO
        j3 = int(self._angles["j3"])
        m  = 1 if self._magnet else 0
        msg = f"B{b:03d} J1{j1:+03d} J2{j2:+03d} J3{j3:+03d} M{m}\n"
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(msg.encode())
                self.statusBar().showMessage(f"Sent → {msg.strip()}")
            except Exception as ex:
                self.statusBar().showMessage(f"Send error: {ex}")
        else:
            self.statusBar().showMessage(f"[DEMO] {msg.strip()}")

    def _toggle_auto_send(self, checked):
        if checked:
            self.auto_send_btn.setText("AUTO SEND: ON")
            self._auto_timer.start()
        else:
            self.auto_send_btn.setText("AUTO SEND: OFF")
            self._auto_timer.stop()


# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
