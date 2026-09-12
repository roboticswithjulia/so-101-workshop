#!/usr/bin/env python3
"""SO-101 app (Catalan UI): zero pose button and one slider per joint.

- "Fixar la posició actual com a zero" does what `servo_positions.py --set-zero`
  does: every servo stores its current position as the middle of its range
  (2048) in EEPROM, verified by reading the position back.
- One slider per joint moves that servo as an offset (in degrees) from the zero
  pose, between the [min, max] limits of that joint. The limits come from the
  JOINTS table and can be edited in the app (type a value, press Enter): the
  slider is rescaled to them. The last column of the JOINTS table (+1 or -1)
  sets the direction of each joint; -1 inverts it (value * -1). Colze is
  inverted.

- "Desar posicions" writes the current position of every servo to a JSON file.

    python3 app/app.py                         # /dev/ttyACM0, IDs 1..6
    python3 app/app.py --port /dev/ttyUSB0 --ids 1,2,3

Install the dependencies first: pip install -r requirements.txt
"""

import argparse
import json
from datetime import datetime

try:
    import tkinter as tk
    from tkinter import filedialog, ttk
except ImportError:  # pragma: no cover - headless machines
    tk = None

import sys
from pathlib import Path

# Allow running this file directly (python3 <folder>/<file>.py) as well as with python3 -m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.servo_positions import POSITION_MIDDLE, read_positions, set_zero_positions
from src.so101_bus import (
    ARM_IDS,
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    REG_TORQUE_ENABLE,
    STEPS_PER_TURN,
    describe,
    open_bus,
    parse_ids,
)

# (servo id, Catalan name, minimum degrees, maximum degrees, direction) relative
# to the zero pose. The limits are the default [min, max] of each joint; they can
# be edited in the app. direction is +1 or -1: -1 inverts the joint (the slider
# value is multiplied by -1 before it is sent).
JOINTS = [
    (1, "Base", -75, 75, 1),
    (2, "Espatlla", -75, 60, 1),
    (3, "Colze", -25, 75, -1),
    (4, "Canell", -70, 18, 1),
    (5, "Gir del canell", -75, 75, 1),
    (6, "Pinça", -25, 45, 1),
]
NOMS_ARTICULACIONS = {servo_id: name for servo_id, name, _, _, _ in JOINTS}
JOINT_LIMITS = {servo_id: (low, high) for servo_id, _, low, high, _ in JOINTS}
JOINT_RANGES = JOINT_LIMITS  # kept as an alias
JOINT_DIRECTIONS = {servo_id: direction for servo_id, _, _, _, direction in JOINTS}
# Hard bounds for any editable limit (degrees from the zero pose).
LIMIT_MIN = -180
LIMIT_MAX = 180

STEPS_PER_DEGREE = STEPS_PER_TURN / 360.0
POSITION_MAX = 4095
# STS3215 units: speed in encoder steps per second (50 steps/s = 0.732 rpm),
# acceleration in units of 100 steps/s^2. 600 steps/s is about 9 rpm: a 90 degree
# move takes about 1.7 s. Gentle for a beginner workshop but smooth.
SAFE_SPEED = 600
SAFE_ACC = 30
# Default file name offered by "Desar posicions".
DEFAULT_POSITIONS_FILE = "positions.json"
# A slider sends its goal when the mouse button is released. Keyboard changes
# (arrow keys) are sent this long after the last change (milliseconds).
SEND_DELAY_MS = 250

TEXTS = {
    "window_title": "SO-101",
    "title": "SO-101",
    "intro": (
        "Connecta el braç ({port}, motors {first}..{last}). Amb els controls lliscants pots moure cada "
        "articulació respecte de la posició zero. Per fixar un nou zero, col·loca el braç en la posició "
        "neutra amb la pinça oberta i prem el botó."
    ),
    "button": "Fixar la posició actual com a zero",
    "connect": "Connectar",
    "zero_all": "Tornar tot a zero",
    "save": "Desar posicions",
    "save_title": "Desar les posicions actuals",
    "saved": "Posicions desades a {path} ({count} motors).",
    "save_error": "No s'han pogut desar les posicions: {detail}",
    "save_cancelled": "Desat cancel·lat.",
    "joints": "Articulacions (graus respecte del zero)",
    "entry_hint": "Escriu els graus i prem Retorn",
    "col_position": "Posició",
    "col_min": "Mín.",
    "col_max": "Màx.",
    "invalid_value": "{name}: valor no vàlid. Escriu un nombre entre {low:g} i {high:g}.",
    "invalid_limits": "{name}: límits no vàlids. Cal que mín. < màx. i que estiguin entre {low} i {high}.",
    "limits_set": "{name}: límits {low:g}..{high:g}°.",
    "ready": "A punt. Prem «Connectar».",
    "connected": "Connectat a {port}. Motors que responen: {ids}.",
    "not_connected": "No connectat. Prem «Connectar».",
    "confirm_title": "Fixar la posició actual com a zero",
    "confirm_text": (
        "La posició actual de cada motor es desarà com a posició zero (2048).\n\n"
        "Es desa a la memòria dels motors i substitueix el zero anterior.\n\n"
        "Vols continuar?"
    ),
    "yes": "Sí",
    "no": "No",
    "cancelled": "Cancel·lat. No s'ha canviat res.",
    "connecting": "Connectant amb {port} ...",
    "ok_prefix": "Correcte: ",
    "error_prefix": "Error: ",
    "no_answer": "Cap motor ha respost. Comprova l'alimentació de 7,4 V, la cadena de cables i els IDs dels motors.",
    "permission": (
        "No es pot obrir {port}: permís denegat. Afegeix el teu usuari al grup dialout "
        "(sudo usermod -aG dialout $USER) i torna a iniciar la sessió."
    ),
    "port_error": "No s'ha pogut obrir el port {port}: {detail}",
    "current": "{name} (ID {servo_id}): posició actual {position}",
    "result_ok": "{name} (ID {servo_id}): {before} -> {after} (correcte)",
    "result_fail": "{name} (ID {servo_id}): {before} -> {after} (NO aplicat)",
    "skipped": "{name} (ID {servo_id}): sense resposta, s'ha omès",
    "stored": "Posició zero desada als motors {ids}. Ara cada articulació llegeix 2048 en aquesta posició.",
    "not_applied": "No s'ha pogut aplicar el zero als motors {ids}. Comprova el firmware dels motors o utilitza l'eina de Feetech.",
    "moved": "{name}: {degrees:.0f}° (posició {position})",
    "move_error": "{name}: no s'ha pogut moure. {detail}",
    "all_zero": "Totes les articulacions tornen a la posició zero.",
    "no_tk": "Tkinter no està instal·lat. A Raspberry Pi OS / Debian executa: sudo apt install python3-tk",
    "cli_description": "Aplicació del SO-101: fixa la posició zero i mou cada articulació.",
    "cli_port": "dispositiu sèrie (per defecte: %(default)s)",
    "cli_baud": "velocitat en bauds (per defecte: %(default)s)",
    "cli_ids": "IDs dels motors, p. ex. '1-6' o '1,2,3' (per defecte: %(default)s)",
}


def nom_articulacio(servo_id):
    return NOMS_ARTICULACIONS.get(servo_id, f"Motor {servo_id}")


# -- bus helpers (no Tkinter, unit tested) ---------------------------------------


def degrees_to_position(degrees, reverse=False):
    """Absolute servo position for an offset in degrees from the zero pose (2048)."""
    sign = -1 if reverse else 1
    position = round(POSITION_MIDDLE + sign * degrees * STEPS_PER_DEGREE)
    return int(max(0, min(POSITION_MAX, position)))


def position_to_degrees(position, reverse=False):
    sign = -1 if reverse else 1
    return sign * (position - POSITION_MIDDLE) / STEPS_PER_DEGREE


def _to_float(text):
    return float(str(text).strip().replace(",", ".").replace("°", ""))


def parse_degrees(text, servo_id, limits=None):
    """Parse a typed value in degrees for one joint. Returns (degrees, error)."""
    low, high = limits or JOINT_LIMITS.get(servo_id, (-90, 90))
    try:
        degrees = _to_float(text)
    except ValueError:
        return None, TEXTS["invalid_value"].format(name=nom_articulacio(servo_id), low=low, high=high)
    if not low <= degrees <= high:
        return None, TEXTS["invalid_value"].format(name=nom_articulacio(servo_id), low=low, high=high)
    return degrees, None


def parse_limits(low_text, high_text, servo_id):
    """Parse typed [min, max] limits for one joint. Returns ((low, high), error)."""
    error = TEXTS["invalid_limits"].format(name=nom_articulacio(servo_id), low=LIMIT_MIN, high=LIMIT_MAX)
    try:
        low, high = _to_float(low_text), _to_float(high_text)
    except ValueError:
        return None, error
    if not (LIMIT_MIN <= low < high <= LIMIT_MAX):
        return None, error
    return (low, high), None


def connect_bus(port, baudrate):
    """Open the bus. Returns ((port_handler, packet_handler), None) or (None, catalan_error)."""
    try:
        return open_bus(port, baudrate), None
    except RuntimeError as exc:
        if "ermission" in str(exc) or "denegat" in str(exc):
            return None, TEXTS["permission"].format(port=port)
        return None, TEXTS["port_error"].format(port=port, detail=exc)


def prepare_joints(packet_handler, ids):
    """Hold every responding servo where it is and enable its torque.

    Returns {id: current raw position}. Commanding the current position before
    enabling the torque avoids a jump if the goal register held an old value.
    """
    readings = read_positions(packet_handler, ids)
    for servo_id, reading in readings.items():
        packet_handler.WritePosEx(servo_id, reading["position"], SAFE_SPEED, SAFE_ACC)
        packet_handler.write1ByteTxRx(servo_id, REG_TORQUE_ENABLE, 1)
    return {servo_id: reading["position"] for servo_id, reading in readings.items()}


def is_inverted(servo_id):
    return JOINT_DIRECTIONS.get(servo_id, 1) < 0


def move_joint(packet_handler, servo_id, degrees, reverse=None, limits=None):
    """Move one joint to an offset in degrees from the zero pose. Returns (position, error).

    `reverse` defaults to the joint's direction in the JOINTS table. When `limits`
    (low, high) is given the degrees are clamped to it first.
    """
    if reverse is None:
        reverse = is_inverted(servo_id)
    if limits is not None:
        degrees = max(limits[0], min(limits[1], degrees))
    position = degrees_to_position(degrees, reverse)
    result, error = packet_handler.WritePosEx(servo_id, position, SAFE_SPEED, SAFE_ACC)
    if result != COMM_SUCCESS:
        return None, describe(packet_handler, result, error)
    return position, None


def build_positions_record(readings, port, limits=None):
    """Build the JSON record for a set of servo readings."""
    limits = limits or JOINT_LIMITS
    servos = {}
    for servo_id in sorted(readings):
        reading = readings[servo_id]
        low, high = limits.get(servo_id, JOINT_LIMITS.get(servo_id, (-90, 90)))
        servos[str(servo_id)] = {
            "joint": nom_articulacio(servo_id),
            "position": reading["position"],
            "degrees": round(position_to_degrees(reading["position"], is_inverted(servo_id)), 1),
            "inverted": is_inverted(servo_id),
            "limits": [low, high],
        }
    return {
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "port": port,
        "zero_position": POSITION_MIDDLE,
        "servos": servos,
    }


def save_positions(packet_handler, ids, path, port, limits=None):
    """Read the current positions and write them as JSON. Returns (record, error)."""
    readings = read_positions(packet_handler, ids)
    if not readings:
        return None, TEXTS["no_answer"]
    record = build_positions_record(readings, port, limits)
    try:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(record, handle, indent=2, ensure_ascii=False)
            handle.write("\n")
    except OSError as exc:
        return None, TEXTS["save_error"].format(detail=exc)
    return record, None


def set_zero_on_bus(packet_handler, ids, log=print):
    """Store the current pose as zero on an open bus. Returns (results, error)."""
    readings = read_positions(packet_handler, ids)
    if not readings:
        return {}, TEXTS["no_answer"]
    for servo_id in ids:
        if servo_id in readings:
            log(TEXTS["current"].format(name=nom_articulacio(servo_id), servo_id=servo_id, position=readings[servo_id]["position"]))
    log("")
    results = set_zero_positions(packet_handler, ids, log=lambda *_: None)

    for servo_id in ids:
        name = nom_articulacio(servo_id)
        result = results.get(servo_id)
        if result is None:
            log(TEXTS["skipped"].format(name=name, servo_id=servo_id))
            continue
        after = result["after"] if result["after"] is not None else "--"
        key = "result_ok" if result["ok"] else "result_fail"
        log(TEXTS[key].format(name=name, servo_id=servo_id, before=result["before"], after=after))
    return results, None


def run_set_zero(port, baudrate, ids, log=print):
    """Open the bus, store the current pose as zero, close the bus. Returns (results, error)."""
    handlers, error = connect_bus(port, baudrate)
    if error:
        return {}, error
    port_handler, packet_handler = handlers
    try:
        return set_zero_on_bus(packet_handler, ids, log=log)
    finally:
        port_handler.closePort()


def summarize(results, error):
    if error:
        return False, error
    failed = sorted(servo_id for servo_id, result in results.items() if not result["ok"])
    if failed:
        return False, TEXTS["not_applied"].format(ids=failed)
    return True, TEXTS["stored"].format(ids=sorted(results))


# -- Tkinter UI ----------------------------------------------------------------


def ask_si_no(parent, title, text):
    """Modal confirmation dialog with Catalan buttons (Tk's askyesno only offers Yes/No)."""
    dialog = tk.Toplevel(parent)
    dialog.title(title)
    dialog.transient(parent)
    dialog.resizable(False, False)
    answer = {"value": False}

    def choose(value):
        answer["value"] = value
        dialog.destroy()

    ttk.Label(dialog, text=text, wraplength=400, justify="left", padding=(20, 20, 20, 12)).pack(fill="x")
    buttons = ttk.Frame(dialog, padding=(20, 0, 20, 20))
    buttons.pack(fill="x")
    yes_button = ttk.Button(buttons, text=TEXTS["yes"], command=lambda: choose(True), width=10)
    yes_button.pack(side="right", padx=(8, 0))
    ttk.Button(buttons, text=TEXTS["no"], command=lambda: choose(False), width=10).pack(side="right")

    dialog.protocol("WM_DELETE_WINDOW", lambda: choose(False))
    dialog.bind("<Escape>", lambda event: choose(False))
    dialog.bind("<Return>", lambda event: choose(True))

    # Centre the dialog over the parent window.
    dialog.update_idletasks()
    x = parent.winfo_rootx() + (parent.winfo_width() - dialog.winfo_width()) // 2
    y = parent.winfo_rooty() + (parent.winfo_height() - dialog.winfo_height()) // 2
    dialog.geometry(f"+{max(x, 0)}+{max(y, 0)}")

    dialog.grab_set()
    yes_button.focus_set()
    parent.wait_window(dialog)
    return answer["value"]


class RobotApp:
    def __init__(self, root, port, baudrate, ids):
        self.root = root
        self.port = port
        self.baudrate = baudrate
        self.ids = ids
        self.port_handler = None
        self.packet_handler = None
        self.responding_ids = []
        self._pending = {}
        self._dragging = set()
        self._updating_sliders = False

        root.title(TEXTS["window_title"])
        root.geometry("680x700")
        root.minsize(560, 560)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        frame = ttk.Frame(root, padding=20)
        frame.pack(fill="both", expand=True)

        ttk.Label(frame, text=TEXTS["title"], font=("Arial", 20, "bold")).pack(anchor="w")
        ttk.Label(
            frame,
            text=TEXTS["intro"].format(first=ids[0], last=ids[-1], port=port),
            wraplength=620,
            justify="left",
        ).pack(anchor="w", pady=(8, 12))

        top = ttk.Frame(frame)
        top.pack(fill="x")
        self.connect_button = ttk.Button(top, text=TEXTS["connect"], command=self.connect, width=14)
        self.connect_button.pack(side="left")
        self.zero_all_button = ttk.Button(top, text=TEXTS["zero_all"], command=self.on_zero_all, width=18)
        self.zero_all_button.pack(side="left", padx=(8, 0))
        self.save_button = ttk.Button(top, text=TEXTS["save"], command=self.on_save, width=18)
        self.save_button.pack(side="left", padx=(8, 0))

        self.button = ttk.Button(frame, text=TEXTS["button"], command=self.on_set_zero)
        self.button.pack(fill="x", ipady=12, pady=(10, 0))

        self.limits = dict(JOINT_LIMITS)
        self.slider_vars = {}
        self.scales = {}
        self.value_labels = {}
        self.entry_vars = {}
        self.limit_vars = {}
        self._build_joint_sliders(frame)

        self.status_var = tk.StringVar(value=TEXTS["ready"])
        ttk.Label(frame, textvariable=self.status_var, wraplength=620, justify="left", font=("Arial", 11, "bold")).pack(
            anchor="w", pady=(12, 6)
        )

        self.log_box = tk.Text(frame, height=7, state="disabled", wrap="word")
        self.log_box.pack(fill="both", expand=True)

    def _build_joint_sliders(self, parent):
        box = ttk.LabelFrame(parent, text=TEXTS["joints"], padding=(12, 8))
        box.pack(fill="x", pady=(12, 0))
        box.grid_columnconfigure(1, weight=1)
        header_font = ("Arial", 9, "bold")
        ttk.Label(box, text=TEXTS["col_position"], font=header_font).grid(row=0, column=3, columnspan=2, sticky="e")
        ttk.Label(box, text=TEXTS["col_min"], font=header_font).grid(row=0, column=5, sticky="e", padx=(12, 0))
        ttk.Label(box, text=TEXTS["col_max"], font=header_font).grid(row=0, column=6, sticky="e", padx=(6, 0))
        for index, (servo_id, name, low, high, _direction) in enumerate(JOINTS):
            if servo_id not in self.ids:
                continue
            row = index + 1
            ttk.Label(box, text=f"{servo_id}. {name}", width=16).grid(row=row, column=0, sticky="w", pady=4)
            var = tk.DoubleVar(value=0.0)
            scale = ttk.Scale(
                box,
                from_=low,
                to=high,
                orient="horizontal",
                variable=var,
                command=lambda value, sid=servo_id: self.on_slider(sid, float(value)),
            )
            scale.bind("<ButtonPress-1>", lambda event, sid=servo_id: self._dragging.add(sid))
            scale.bind("<ButtonRelease-1>", lambda event, sid=servo_id: self.on_slider_release(sid))
            scale.grid(row=row, column=1, sticky="ew", padx=8)
            value_label = ttk.Label(box, text="0°", width=6, anchor="e")
            value_label.grid(row=row, column=2, sticky="e")
            entry_var = tk.StringVar(value="0")
            entry = ttk.Entry(box, textvariable=entry_var, width=7, justify="right")
            entry.grid(row=row, column=3, sticky="e", padx=(12, 2))
            entry.bind("<Return>", lambda event, sid=servo_id: self.on_entry(sid))
            entry.bind("<KP_Enter>", lambda event, sid=servo_id: self.on_entry(sid))
            ttk.Label(box, text="°").grid(row=row, column=4, sticky="w")
            low_var = tk.StringVar(value=f"{low:g}")
            high_var = tk.StringVar(value=f"{high:g}")
            for column, limit_var in ((5, low_var), (6, high_var)):
                limit_entry = ttk.Entry(box, textvariable=limit_var, width=6, justify="right")
                limit_entry.grid(row=row, column=column, sticky="e", padx=(12 if column == 5 else 6, 0))
                limit_entry.bind("<Return>", lambda event, sid=servo_id: self.on_limits_entry(sid))
                limit_entry.bind("<KP_Enter>", lambda event, sid=servo_id: self.on_limits_entry(sid))
            self.slider_vars[servo_id] = var
            self.scales[servo_id] = scale
            self.value_labels[servo_id] = value_label
            self.entry_vars[servo_id] = entry_var
            self.limit_vars[servo_id] = (low_var, high_var)
        ttk.Label(box, text=TEXTS["entry_hint"], font=("Arial", 8)).grid(
            row=len(JOINTS) + 1, column=3, columnspan=4, sticky="e", pady=(2, 0)
        )

    # -- logging and status --------------------------------------------------

    def log(self, line):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", line + "\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")
        self.root.update_idletasks()

    def clear_log(self):
        self.log_box.configure(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.configure(state="disabled")

    def set_status(self, message, ok=None):
        prefix = "" if ok is None else (TEXTS["ok_prefix"] if ok else TEXTS["error_prefix"])
        self.status_var.set(prefix + message)

    # -- connection ----------------------------------------------------------

    @property
    def connected(self):
        return self.packet_handler is not None

    def connect(self):
        if self.connected:
            return True
        self.set_status(TEXTS["connecting"].format(port=self.port))
        self.root.update_idletasks()
        handlers, error = connect_bus(self.port, self.baudrate)
        if error:
            self.set_status(error, ok=False)
            return False
        port_handler, packet_handler = handlers
        positions = prepare_joints(packet_handler, self.ids)
        if not positions:
            port_handler.closePort()
            self.set_status(TEXTS["no_answer"], ok=False)
            return False
        self.port_handler, self.packet_handler = port_handler, packet_handler
        self.responding_ids = sorted(positions)
        self._show_positions(positions)
        self.set_status(TEXTS["connected"].format(port=self.port, ids=self.responding_ids), ok=True)
        return True

    def disconnect(self):
        if self.port_handler is not None:
            try:
                self.port_handler.closePort()
            except Exception:
                pass
        self.port_handler = None
        self.packet_handler = None
        self.responding_ids = []

    def on_close(self):
        self.disconnect()
        self.root.destroy()

    # -- sliders -------------------------------------------------------------

    def _set_slider(self, servo_id, degrees):
        """Update a slider without sending a move."""
        low, high = self.limits.get(servo_id, (-90, 90))
        degrees = max(low, min(high, degrees))
        self._updating_sliders = True
        try:
            self.slider_vars[servo_id].set(degrees)
            self.value_labels[servo_id].configure(text=f"{degrees:.0f}°")
            self.entry_vars[servo_id].set(f"{degrees:.0f}")
        finally:
            self._updating_sliders = False

    def _show_positions(self, positions):
        for servo_id, position in positions.items():
            if servo_id in self.slider_vars:
                self._set_slider(servo_id, position_to_degrees(position, self.reversed(servo_id)))

    def on_slider(self, servo_id, degrees):
        """Called on every slider change: update the label; send only when not dragging."""
        self.value_labels[servo_id].configure(text=f"{degrees:.0f}°")
        if self._updating_sliders or servo_id in self._dragging:
            return
        pending = self._pending.pop(servo_id, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self._pending[servo_id] = self.root.after(SEND_DELAY_MS, lambda: self._send(servo_id))

    def on_slider_release(self, servo_id):
        """Mouse released: send one goal for the final slider value."""
        self._dragging.discard(servo_id)
        pending = self._pending.pop(servo_id, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.entry_vars[servo_id].set(f"{self.slider_vars[servo_id].get():.0f}")
        self._send(servo_id)

    def on_entry(self, servo_id):
        """Enter pressed in the degrees box: move the joint to the typed value."""
        degrees, error = parse_degrees(self.entry_vars[servo_id].get(), servo_id, self.limits.get(servo_id))
        if error:
            self.set_status(error, ok=False)
            return
        self._set_slider(servo_id, degrees)
        self._send(servo_id)

    @staticmethod
    def reversed(servo_id):
        """Direction of a joint, from the last column of the JOINTS table."""
        return is_inverted(servo_id)

    def on_limits_entry(self, servo_id):
        """Enter pressed in a limit box: apply the new [min, max] and rescale the slider."""
        low_var, high_var = self.limit_vars[servo_id]
        limits, error = parse_limits(low_var.get(), high_var.get(), servo_id)
        if error:
            low, high = self.limits[servo_id]
            low_var.set(f"{low:g}")
            high_var.set(f"{high:g}")
            self.set_status(error, ok=False)
            return
        self.set_limits(servo_id, limits)
        self.set_status(TEXTS["limits_set"].format(name=nom_articulacio(servo_id), low=limits[0], high=limits[1]))

    def set_limits(self, servo_id, limits):
        """Apply [min, max] to a joint: rescale its slider and keep its value inside (no move is sent)."""
        low, high = limits
        self.limits[servo_id] = (low, high)
        low_var, high_var = self.limit_vars[servo_id]
        low_var.set(f"{low:g}")
        high_var.set(f"{high:g}")
        self.scales[servo_id].configure(from_=low, to=high)
        self._set_slider(servo_id, self.slider_vars[servo_id].get())

    def _send(self, servo_id):
        self._pending.pop(servo_id, None)
        if not self.connected:
            self.set_status(TEXTS["not_connected"], ok=False)
            return
        degrees = self.slider_vars[servo_id].get()
        position, error = move_joint(
            self.packet_handler, servo_id, degrees, reverse=self.reversed(servo_id), limits=self.limits.get(servo_id)
        )
        name = nom_articulacio(servo_id)
        if error:
            self.set_status(TEXTS["move_error"].format(name=name, detail=error), ok=False)
        else:
            self.set_status(TEXTS["moved"].format(name=name, degrees=degrees, position=position))

    def on_zero_all(self):
        for servo_id in self.slider_vars:
            self._set_slider(servo_id, 0.0)
        if not self.connected:
            self.set_status(TEXTS["not_connected"], ok=False)
            return
        for servo_id in self.responding_ids:
            if servo_id in self.slider_vars:
                # A joint whose limits exclude 0 goes to the nearest limit instead.
                move_joint(
                    self.packet_handler, servo_id, 0.0, reverse=self.reversed(servo_id), limits=self.limits.get(servo_id)
                )
        self.set_status(TEXTS["all_zero"])

    # -- save positions ------------------------------------------------------

    def on_save(self):
        """Ask for a file and write the current position of every servo to it."""
        if not self.connect():
            return
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title=TEXTS["save_title"],
            defaultextension=".json",
            initialfile=DEFAULT_POSITIONS_FILE,
            filetypes=[("JSON", "*.json"), ("Tots els fitxers", "*.*")],
        )
        if not path:
            self.set_status(TEXTS["save_cancelled"])
            return
        record, error = save_positions(self.packet_handler, self.ids, path, self.port, self.limits)
        if error:
            self.set_status(error, ok=False)
            return
        self.clear_log()
        for servo_id, entry in record["servos"].items():
            self.log(TEXTS["current"].format(name=entry["joint"], servo_id=servo_id, position=entry["position"]))
        self.set_status(TEXTS["saved"].format(path=path, count=len(record["servos"])), ok=True)

    # -- zero pose -----------------------------------------------------------

    def on_set_zero(self):
        if not self.connect():
            return
        if not ask_si_no(self.root, TEXTS["confirm_title"], TEXTS["confirm_text"]):
            self.set_status(TEXTS["cancelled"])
            return

        self.button.configure(state="disabled")
        self.clear_log()
        self.root.update_idletasks()
        try:
            results, error = set_zero_on_bus(self.packet_handler, self.ids, log=self.log)
            # The current pose is the new zero: hold it and reset the sliders.
            positions = prepare_joints(self.packet_handler, self.ids)
            self._show_positions(positions)
        finally:
            self.button.configure(state="normal")
        ok, message = summarize(results, error)
        self.set_status(message, ok=ok)


def main(argv=None):
    parser = argparse.ArgumentParser(description=TEXTS["cli_description"])
    parser.add_argument("--port", default=DEFAULT_PORT, help=TEXTS["cli_port"])
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help=TEXTS["cli_baud"])
    parser.add_argument("--ids", default=ARM_IDS, help=TEXTS["cli_ids"])
    args = parser.parse_args(argv)

    if tk is None:
        print(TEXTS["no_tk"])
        return 1

    root = tk.Tk()
    app = RobotApp(root, args.port, args.baud, parse_ids(args.ids))
    root.after(200, app.connect)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
