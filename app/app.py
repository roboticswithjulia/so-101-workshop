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

- The Programa tab is where the task is written, with ready-made helpers, and run
  with Executar / Pas a pas / Pausa / Aturar. "Afegir la posició actual" writes an
  anar_a(...) line for the pose the arm is in, and "Desar posició inici" sets where
  inici() goes. Every move is written out step by step with anar_a(), so nothing
  is hidden inside a helper.

    python3 app/app.py                         # /dev/ttyACM0, IDs 1..6
    python3 app/app.py --port /dev/ttyUSB0 --ids 1,2,3

Install the dependencies first: pip install -r requirements.txt
"""

import argparse
import base64
import io
import json
import re
import shutil
import subprocess
from collections import namedtuple
from datetime import datetime

try:
    import tkinter as tk
    from tkinter import filedialog, ttk
except ImportError:  # pragma: no cover - headless machines
    tk = None

try:  # only used to round the button corners; the app runs fine without it
    from PIL import Image, ImageDraw
except ImportError:  # pragma: no cover - optional
    Image = ImageDraw = None

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

# Poses used by the helper functions of the Programa tab. POSICIO_ZERO is where
# inici() goes; the app can replace it at run time with "Desar posició inici".
POSICIO_ZERO = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}
POSICIO_INTERMITJA = {1: 21, 2: 15, 3: 10, 4: 0, 5: 0}
PINCA_OBERTA, PINCA_TANCADA = 13, -13


# Programa d'exemple de la pestanya "Programa": només els comentaris, perquè
# cada equip escrigui el seu programa des de zero.
PROGRAMA_EXEMPLE = """# Repte 1: agafa l'objecte i deixa'l a l'altra posició.
# Escriu el teu programa a sota. Les coordenades són graus respecte de la
# posició zero.
#
# Per posar-hi una posició: mou el braç fins on el vulguis (pots marcar
# «Motors lliures» a la pestanya Control i moure'l a mà) i prem
# «Afegir la posició actual». S'escriurà la línia anar_a() aquí mateix.
#
# Funcions que pots fer servir:
#   anar_a(base, espatlla, colze, canell, gir)  -> mou el braç
#   obrir_pinca(graus) / tancar_pinca(graus)    -> mou la pinça
#   aixecar()                                   -> va a la posició intermèdia
#   inici()                                     -> va a la posició inici
#   esperar(segons)                             -> s'espera sense moure's
#
# Les articulacions que no escriguis es queden on són, i totes les funcions
# accepten segons= per canviar quant s'espera abans del pas següent.

"""

# (servo id, Catalan name, minimum degrees, maximum degrees, direction) relative
# to the zero pose. The limits are the default [min, max] of each joint; they can
# be edited in the app. direction is +1 or -1: -1 inverts the joint (the slider
# value is multiplied by -1 before it is sent).
# These defaults span the whole travel the servo can reach, so they stop nothing:
# narrow the [min, max] of a joint here, or in the app, to keep it off its
# mechanical stops.
JOINTS = [
    (1, "Base", -180, 180, 1),
    (2, "Espatlla", -180, 180, 1),
    (3, "Colze", -180, 180, -1),
    (4, "Canell", -180, 180, 1),
    (5, "Gir del canell", -180, 180, 1),
    (6, "Pinça", -180, 180, 1),
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
# acceleration in units of 100 steps/s^2. 750 steps/s is about 11 rpm: a 90
# degree move takes about 1.5 s. A higher speed and a steeper ramp draw more
# current, so if the servos start tripping their overload protection or the
# supply voltage sags under 7 V, lower these again. Below about 100 steps/s the
# servo creeps and judders instead of turning.
SAFE_SPEED = 750
SAFE_ACC = 50
# Range accepted by the speed and acceleration boxes. Below SPEED_MIN the servo
# creeps and judders instead of turning smoothly.
SPEED_MIN, SPEED_MAX = 100, 2000
ACC_MIN, ACC_MAX = 1, 150
# A bare number in a TASCA step moves only this joint ("Pinça").
GRIPPER_ID = 6
# Upper bound for a step duration, to catch a typo such as 999 seconds.
MAX_STEP_SECONDS = 60.0
# Shortest step duration the boxes accept.
SECONDS_MIN = 0.1
# One step of a task; `pose` is always {servo_id: degrees} after build_task().
TaskStep = namedtuple("TaskStep", "name pose seconds")
# Program tab: file offered by "Desar" and the ceiling that stops a runaway loop.
DEFAULT_PROGRAM_FILE = "programa.py"
MAX_PROGRAM_STEPS = 200
# Default durations used by the helper functions of the program tab (seconds).
# A step must outlast the move it starts. Time for a move of D degrees:
#   D * 4096 / 360 / SAFE_SPEED + SAFE_SPEED / (SAFE_ACC * 100)
# At the default 750 steps/s that is 0.6 s for 34 degrees and 0.8 s for 50.
# Both are editable in the app; raise them if a joint is still moving when the
# next step starts, which shows up as the arm cutting corners.
MOVE_SECONDS = 1.0
GRIPPER_SECONDS = 1.0
# Dark palette. CYAN and TEAL are the two colours of the logo, so the sliders and
# the accents match it.
BG = "#2B2B2B"          # window background, dark grey
PANEL_BG = "#333333"    # frames and label frames
FIELD_BG = "#1F1F1F"    # entry and text boxes, darker than the panels
TAB_BG = "#1A1A1A"      # unselected tabs: almost black
TEXT_BG = "#BDBDBD"     # the program editor and the log: mid grey
TEXT_FG = "#1A1A1A"     # dark text on that light grey
LOG_BG = "#3D3D3D"      # the log box: dark grey, a step lighter than BG
LOG_FG = "#E6E6E6"      # light text on it
FG = "#E6E6E6"          # normal text
MUTED_FG = "#8C8C8C"    # hints and disabled text
CYAN = "#6CDAE7"        # logo light cyan: sliders, headings, focus
TEAL = "#008B8B"        # logo dark cyan: troughs, borders, pressed buttons

# Logo shown in the top right corner. Tk reads PNG natively; the SVG next to it
# is the source, re-export with:
#   inkscape --export-type=png --export-filename=Logo_JMR.png \
#            --export-area-drawing --export-height=72 logo/Logo_JMR.svg
LOGO_PATH = Path(__file__).resolve().parents[1] / "logo" / "Logo_JMR.png"
# The Guia tab: one Markdown file per section in GUIA_DIR (read in name order),
# and the assembly videos in VIDEO_DIR, one per joint, keyed by servo id. A
# section shows the video of joint N where it says [video:N].
GUIA_DIR = Path(__file__).resolve().parents[1] / "docs" / "guia"
VIDEO_DIR = Path(__file__).resolve().parents[1] / "videos"
VIDEOS = {
    1: "Joint1_v2.mp4",
    2: "Joint2_v2.mp4",
    3: "Joint3_v2.mp4",
    4: "Joint4_v2.mp4",
    5: "Joint5_v2.mp4",
    6: "Gripper_v2.mp4",
}
# Each video has a thumbnail with the same name in GUIA_DIR / "imatges".
GUIA_IMAGE_DIR = GUIA_DIR / "imatges"
GUIA_FONT = ("Arial", 12)
# The section list of the Guia tab: bigger than the rest, to hit it easily.
GUIA_MENU_FONT = ("Arial", 12, "bold")
# Every button in the top row is this wide, so they line up.
BUTTON_WIDTH = 19
# Corner radius of the buttons, in pixels.
BUTTON_RADIUS = 8
# Buttons, joint names, panel titles and field labels are all bold.
BOLD_FONT = ("Arial", 10, "bold")
# Light grey for the text on buttons and tabs, which sit on the logo teal.
BUTTON_FG = "#D9D9D9"
# How often the live angle of each joint is re-read while connected (milliseconds).
ANGLE_REFRESH_MS = 400
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
    "button": "Fixar la posició zero",
    "connect": "Connectar",
    "zero_all": "Tornar tot a zero",
    "save_cancelled": "Desat cancel·lat.",
    "joints": "Articulacions (graus respecte del zero)",
    "entry_hint": "Escriu els graus i prem Retorn",
    "col_current": "Actual",
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
    # -- task panel --
    "task_title": "Tasca: agafar i deixar un objecte",
    "task_play": "Executar",
    "task_pause": "Pausa",
    "task_stop": "Aturar",
    "task_step_button": "Pas a pas",
    "task_step_done": "Pas {done}/{total} fet. Següent: {next}.",
    "task_step_last": "Pas {done}/{total} fet. Era l'últim: la tasca ha acabat.",
    "task_idle": "Tasca preparada: {count} passos, {seconds} s en total.",
    "task_step": "Pas {index}/{total}: {name} ({seconds} s)",
    "task_started": "Tasca iniciada.",
    "task_resumed": "Tasca represa al pas {index}/{total}.",
    "task_paused": "Tasca en pausa al pas {index}/{total}. Prem «Executar» per continuar.",
    "task_stopped": "Tasca aturada. El braç s'ha quedat on era.",
    "task_hold": "Braç aturat on era.",
    "task_finished": "Tasca acabada correctament ({total} passos).",
    "task_not_started": "No s'ha pogut iniciar. Mira els missatges de sota i corregeix el programa.",
    "task_running_busy": "Hi ha una tasca en marxa. Prem «Aturar» abans de moure les articulacions.",
    "task_empty": "No hi ha cap programa. Escriu-lo a la pestanya «Programa» i prem «Executar el programa».",
    "task_bad_step": "Pas {index}: format incorrecte. Cada pas ha de ser (\"nom\", posició, segons).",
    "task_bad_pose": (
        "Pas {index} «{name}»: la posició ha de ser un diccionari {{ID: graus}} "
        "o un sol número (només la pinça)."
    ),
    "task_bad_seconds": (
        "Pas {index} «{name}»: la durada ha de ser un nombre de segons més gran que 0 "
        "i com a màxim {high:g}."
    ),
    "task_unknown_joint": "Pas {index} «{name}»: el motor {servo_id} no existeix. Fes servir els IDs de l'1 al 6.",
    "task_missing_joint": (
        "Pas {index} «{name}»: el motor {servo_id} ({joint}) no respon. "
        "Comprova la connexió i els cables."
    ),
    "task_out_of_limits": (
        "Pas {index} «{name}»: {joint} a {degrees:g}° és fora dels límits {low:g}..{high:g}°. "
        "Canvia la posició o els límits."
    ),
    "task_move_error": "Pas {index} «{name}»: {detail} La tasca s'ha aturat.",
    # -- movement speed --
    "motion_title": "Moviment",
    "motion_speed": "Velocitat",
    "motion_acc": "Acceleració",
    "motion_set_button": "Aplicar",
    "unit_speed": "passos/s",
    "unit_acc": "×100 passos/s²",
    "unit_seconds": "s",
    "durations_move": "Espera moviment",
    "durations_gripper": "Espera pinça",
    "durations_invalid": "Les esperes han de ser nombres entre {low:g} i {high:g} segons.",
    "durations_set": "Espera de {move} s després d'un moviment i {gripper} s després de la pinça.",
    "motion_hint": (
        "Escriu els valors i prem «Aplicar» (o Retorn). Velocitat {smin}..{smax}, acceleració "
        "{amin}..{amax}: més a poc a poc vol dir menys corrent. Les esperes són el temps que cada pas "
        "del programa espera abans del següent: si el braç s'atura entre moviments baixa-les, i si "
        "encara es mou quan comença el pas següent puja-les."
    ),
    "motion_invalid": (
        "Valors no vàlids. La velocitat ha d'estar entre {smin} i {smax}, "
        "i l'acceleració entre {amin} i {amax}."
    ),
    "motion_set": "Velocitat {speed} passos/s, acceleració {acc}. Un gir de 90° triga uns {seconds} s.",
    # -- torque --
    "torque_free": "Motors lliures",
    "torque_confirm_title": "Deixar els motors lliures",
    "torque_confirm_text": (
        "Els motors es quedaran sense força i el braç caurà pel seu propi pes.\n\n"
        "AGAFA EL BRAÇ abans de continuar, i deixa la pinça sense cap objecte.\n\n"
        "Vols continuar?"
    ),
    "torque_off": "Motors lliures: pots moure el braç a mà. Prem «Copiar la posició actual» per desar-la.",
    "torque_on": "Motors amb força, aguantant la posició actual.",
    "torque_error": "{name}: no s'ha pogut canviar la força. {detail}",
    "torque_busy": "No es pot treure la força amb una tasca en marxa. Prem «Aturar» primer.",
    # -- program tab --
    "tab_control": "Control",
    "tab_program": "Programa",
    "program_title": "Repte 1: completa les coordenades",
    "program_run": "Executar el programa",
    "program_load": "Carregar",
    "program_save": "Desar",
    "program_reset": "Exemple",
    "program_insert": "Afegir la posició actual",
    "program_home": "Desar posició inici",
    "program_inserted": "Afegit: {line}",
    "program_home_saved": "Posició inici desada: {pose}. Ara inici() hi anirà.",
    "program_hint": (
        "Funcions: anar_a(base, espatlla, colze, canell, gir), obrir_pinca(graus), "
        "tancar_pinca(graus), aixecar(), inici(), esperar(segons)."
    ),
    "program_step_move": "Moure el braç",
    "program_step_open": "Obrir pinça",
    "program_step_close": "Tancar pinça",
    "program_step_lift": "Aixecar",
    "program_step_home": "Tornar a l'inici",
    "program_step_wait": "Esperar",
    "program_built": "Programa llegit: {count} passos, {seconds} s en total.",
    "program_no_steps": "El programa no mou el braç. Fes servir anar_a(), obrir_pinca() o tancar_pinca().",
    "program_syntax_error": "Error de sintaxi a la línia {line}: {detail}",
    "program_unknown_name": "No existeix aquest nom: {detail}. Mira la llista de funcions de sota.",
    "program_error": "Error en executar el programa: {detail}",
    "program_bad_coordinate": "La coordenada de {joint} ha de ser un número.",
    "program_bad_wait": "esperar() vol un nombre de segons més gran que 0.",
    "program_empty_move": "Cal donar com a mínim una coordenada, per exemple anar_a(base=0, espatlla=30, colze=10).",
    "program_too_many": "El programa demana més de {limit} passos. Comprova que no hi hagi un bucle infinit.",
    "program_saved": "Programa desat a {path}.",
    "program_loaded": "Programa carregat de {path}.",
    "program_file_error": "No s'ha pogut obrir el fitxer: {detail}",
    "program_reset_done": "S'ha tornat a posar el programa d'exemple.",
    # -- guide tab --
    "tab_guide": "Guia",
    "guide_missing": "No s'ha trobat la guia a {path}.",
    "guide_video_button": "▶ Mira el vídeo",
    "guide_copy": "Copiar al programa",
    "guide_copied": "Copiat al final del programa. Mira la pestanya «Programa».",
    "video_title": "Articulació {servo_id} – {name}",
    "video_missing": "Falta el vídeo {path}. Executa videos/baixar_videos.sh.",
    "video_opened": "Obrint el vídeo «{title}».",
    "video_no_vlc": "Obrint el vídeo «{title}» (no hi ha VLC: el nom de l'articulació no sortirà al vídeo).",
    "video_no_player": "No hi ha VLC ni cap reproductor de vídeo. Instal·la VLC: sudo apt install vlc",
    "video_error": "No s'ha pogut obrir el vídeo: {detail}",
    "no_tk": "Tkinter no està instal·lat. A Raspberry Pi OS / Debian executa: sudo apt install python3-tk",
    "cli_description": "Aplicació del SO-101: fixa la posició zero, mou cada articulació i executa la tasca.",
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


def parse_durations(move_text, gripper_text):
    """Parse the typed step durations. Returns ((move, gripper), error)."""
    error = TEXTS["durations_invalid"].format(low=SECONDS_MIN, high=MAX_STEP_SECONDS)
    try:
        move, gripper = _to_float(move_text), _to_float(gripper_text)
    except ValueError:
        return None, error
    if not all(SECONDS_MIN <= value <= MAX_STEP_SECONDS for value in (move, gripper)):
        return None, error
    return (move, gripper), None


def parse_motion(speed_text, acc_text):
    """Parse a typed speed and acceleration. Returns ((speed, acc), error)."""
    error = TEXTS["motion_invalid"].format(
        smin=SPEED_MIN, smax=SPEED_MAX, amin=ACC_MIN, amax=ACC_MAX
    )
    try:
        speed, acc = int(_to_float(speed_text)), int(_to_float(acc_text))
    except ValueError:
        return None, error
    if not (SPEED_MIN <= speed <= SPEED_MAX and ACC_MIN <= acc <= ACC_MAX):
        return None, error
    return (speed, acc), None


def connect_bus(port, baudrate):
    """Open the bus. Returns ((port_handler, packet_handler), None) or (None, catalan_error)."""
    try:
        return open_bus(port, baudrate), None
    except RuntimeError as exc:
        if "ermission" in str(exc) or "denegat" in str(exc):
            return None, TEXTS["permission"].format(port=port)
        return None, TEXTS["port_error"].format(port=port, detail=exc)


def prepare_joints(packet_handler, ids, speed=SAFE_SPEED, acc=SAFE_ACC):
    """Hold every responding servo where it is and enable its torque.

    Returns {id: current raw position}. Commanding the current position before
    enabling the torque avoids a jump if the goal register held an old value.
    """
    readings = read_positions(packet_handler, ids)
    for servo_id, reading in readings.items():
        packet_handler.WritePosEx(servo_id, reading["position"], speed, acc)
        packet_handler.write1ByteTxRx(servo_id, REG_TORQUE_ENABLE, 1)
    return {servo_id: reading["position"] for servo_id, reading in readings.items()}


def is_inverted(servo_id):
    return JOINT_DIRECTIONS.get(servo_id, 1) < 0


def move_joint(packet_handler, servo_id, degrees, reverse=None, limits=None, speed=SAFE_SPEED, acc=SAFE_ACC):
    """Move one joint to an offset in degrees from the zero pose. Returns (position, error).

    `reverse` defaults to the joint's direction in the JOINTS table. When `limits`
    (low, high) is given the degrees are clamped to it first.
    """
    if reverse is None:
        reverse = is_inverted(servo_id)
    if limits is not None:
        degrees = max(limits[0], min(limits[1], degrees))
    position = degrees_to_position(degrees, reverse)
    result, error = packet_handler.WritePosEx(servo_id, position, speed, acc)
    if result != COMM_SUCCESS:
        return None, describe(packet_handler, result, error)
    return position, None


# -- task: parsing and validation (pure, unit tested) ----------------------------


def format_seconds(seconds):
    """Catalan number for a duration: 2.5 -> '2,5', 2.0 -> '2'."""
    return f"{float(seconds):g}".replace(".", ",")


def _is_number(value):
    # bool is an int in Python; True would silently become 1 degree.
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def normalize_pose(pose):
    """Turn a TASCA pose into {servo_id: degrees}. Returns (pose, error).

    A mapping is copied with int keys and float values. A bare number means the
    gripper only, so that a step such as ("Obrir pinça", -25, 1.0) never has to
    name servo 6, and arm poses never disturb the gripper.
    """
    if _is_number(pose):
        return {GRIPPER_ID: float(pose)}, "bare"
    if not isinstance(pose, dict):
        return None, "type"
    normalized = {}
    for servo_id, degrees in pose.items():
        if not _is_number(servo_id) or isinstance(servo_id, float) or not _is_number(degrees):
            return None, "type"
        normalized[int(servo_id)] = float(degrees)
    return normalized, None


def build_task(raw_steps):
    """Normalise the team-editable TASCA list. Returns ([TaskStep], [catalan errors]).

    Checks the shape of every step only; limits are runtime state and are checked
    by validate_task. A step that fails is left out, so the steps that did build
    can still be counted and shown.
    """
    steps, errors = [], []
    for index, raw in enumerate(raw_steps, start=1):
        if isinstance(raw, (str, bytes)) or not isinstance(raw, (tuple, list)) or len(raw) != 3:
            errors.append(TEXTS["task_bad_step"].format(index=index))
            continue
        raw_name, raw_pose, raw_seconds = raw
        name = str(raw_name)
        pose, pose_error = normalize_pose(raw_pose)
        if pose is None:
            errors.append(TEXTS["task_bad_pose"].format(index=index, name=name))
            continue
        if not _is_number(raw_seconds) or not 0 < float(raw_seconds) <= MAX_STEP_SECONDS:
            errors.append(TEXTS["task_bad_seconds"].format(index=index, name=name, high=MAX_STEP_SECONDS))
            continue
        steps.append(TaskStep(name, pose, float(raw_seconds)))
    return steps, errors


def validate_task(steps, limits=None, ids=None):
    """Catalan problems with a task against the current limits. Returns [str].

    `limits` defaults to the JOINTS table; `ids` is the list of servos that
    answered, and None skips that check.
    """
    limits = limits or JOINT_LIMITS
    if not steps:
        return [TEXTS["task_empty"]]
    problems = []
    for index, step in enumerate(steps, start=1):
        for servo_id in sorted(step.pose):
            degrees = step.pose[servo_id]
            if servo_id not in JOINT_LIMITS:
                problems.append(TEXTS["task_unknown_joint"].format(index=index, name=step.name, servo_id=servo_id))
                continue
            if ids is not None and servo_id not in ids:
                problems.append(TEXTS["task_missing_joint"].format(
                    index=index, name=step.name, servo_id=servo_id, joint=nom_articulacio(servo_id)
                ))
                continue
            low, high = limits.get(servo_id, JOINT_LIMITS[servo_id])
            if not low <= degrees <= high:
                problems.append(TEXTS["task_out_of_limits"].format(
                    index=index, name=step.name, joint=nom_articulacio(servo_id),
                    degrees=degrees, low=low, high=high,
                ))
    return problems


def task_total_seconds(steps):
    return sum(step.seconds for step in steps)


def describe_step(index, total, step):
    """'Pas 3/7: Anar a l'objecte (2,5 s)' with a 1-based index."""
    return TEXTS["task_step"].format(
        index=index + 1, total=total, name=step.name, seconds=format_seconds(step.seconds)
    )


# -- program tab: the helper functions the participants call ---------------------


class TaskBuilder:
    """Collects the steps a participant's program asks for.

    Every helper appends TaskStep entries instead of touching the bus, so the
    program runs instantly and the resulting task is played by the same
    play/pause/stop machine as the TASCA block.
    """

    def __init__(self, lift=None, home=None, move_seconds=MOVE_SECONDS,
                 gripper_seconds=GRIPPER_SECONDS, max_steps=MAX_PROGRAM_STEPS):
        self.steps = []
        # Pose used to lift the arm between picking and placing.
        self.lift = dict(lift) if lift else dict(POSICIO_INTERMITJA)
        # Where inici() goes. "Desar posició inici" replaces it at run time.
        self.home = dict(home) if home else dict(POSICIO_ZERO)
        # Default wait after a move and after a gripper step; the app sets these.
        self.move_seconds = float(move_seconds)
        self.gripper_seconds = float(gripper_seconds)
        self.max_steps = max_steps

    def _add(self, name, pose, seconds):
        if seconds is None:
            seconds = self.gripper_seconds if set(pose) == {GRIPPER_ID} else self.move_seconds
        if len(self.steps) >= self.max_steps:
            raise RuntimeError(TEXTS["program_too_many"].format(limit=self.max_steps))
        self.steps.append(TaskStep(name, pose, float(seconds)))

    @staticmethod
    def _gripper_degrees(graus):
        if not _is_number(graus):
            raise TypeError(TEXTS["program_bad_coordinate"].format(joint=nom_articulacio(GRIPPER_ID)))
        return float(graus)

    @staticmethod
    def _pose(base=None, espatlla=None, colze=None, canell=None, gir=None):
        """Named Catalan arguments -> {servo_id: degrees}, skipping what is not given."""
        pose = {}
        for servo_id, value in ((1, base), (2, espatlla), (3, colze), (4, canell), (5, gir)):
            if value is not None:
                if not _is_number(value):
                    raise TypeError(TEXTS["program_bad_coordinate"].format(joint=nom_articulacio(servo_id)))
                pose[servo_id] = float(value)
        return pose

    # -- the functions the participants call --

    def anar_a(self, base=None, espatlla=None, colze=None, canell=None, gir=None, segons=None):
        pose = self._pose(base, espatlla, colze, canell, gir)
        if not pose:
            raise TypeError(TEXTS["program_empty_move"])
        self._add(TEXTS["program_step_move"], pose, segons)

    def obrir_pinca(self, graus=PINCA_OBERTA, segons=None):
        """Open the gripper. `graus` is how far, in degrees from the zero pose."""
        self._add(TEXTS["program_step_open"], {GRIPPER_ID: self._gripper_degrees(graus)}, segons)

    def tancar_pinca(self, graus=PINCA_TANCADA, segons=None):
        """Close the gripper. `graus` is how far, in degrees from the zero pose."""
        self._add(TEXTS["program_step_close"], {GRIPPER_ID: self._gripper_degrees(graus)}, segons)

    def inici(self, segons=None):
        self._add(TEXTS["program_step_home"], dict(self.home), segons)

    def esperar(self, segons=1.0):  # a pause, not a move: its own default
        if not _is_number(segons) or segons <= 0:
            raise TypeError(TEXTS["program_bad_wait"])
        # An empty pose moves nothing; the player still waits out the duration.
        self._add(TEXTS["program_step_wait"], {}, segons)

    def aixecar(self, segons=None):
        self._add(TEXTS["program_step_lift"], dict(self.lift), segons)


# Builtins a participant's program may use. Everything else, including
# __import__, open, eval and exec, is out of reach.
SAFE_BUILTINS = {
    name: getattr(__builtins__, name) if hasattr(__builtins__, name) else __builtins__[name]
    for name in (
        "abs", "bool", "dict", "enumerate", "float", "int", "len", "list",
        "max", "min", "range", "reversed", "round", "sorted", "str", "sum", "tuple", "zip",
    )
}


def program_namespace(builder, log=None):
    """The names a participant's program can use."""
    names = {
        "anar_a": builder.anar_a,
        "obrir_pinca": builder.obrir_pinca,
        "tancar_pinca": builder.tancar_pinca,
        "aixecar": builder.aixecar,
        "inici": builder.inici,
        "esperar": builder.esperar,
        # The poses are not exposed: they are keyed by servo id, so they cannot be
        # passed to the helpers, which take named joints. inici() and aixecar()
        # already go to them.
        "PINCA_OBERTA": PINCA_OBERTA,
        "PINCA_TANCADA": PINCA_TANCADA,
        "__builtins__": dict(SAFE_BUILTINS),
    }
    names["escriure"] = log if log is not None else (lambda *a: None)
    return names


def run_program(code, lift=None, home=None, move_seconds=MOVE_SECONDS,
                gripper_seconds=GRIPPER_SECONDS, log=None):
    """Run a participant's program and collect its steps. Returns (steps, error).

    The program only builds a task; nothing is sent to the servos here.
    """
    builder = TaskBuilder(lift=lift, home=home, move_seconds=move_seconds, gripper_seconds=gripper_seconds)
    try:
        compiled = compile(code, "programa", "exec")
    except SyntaxError as exc:
        return None, TEXTS["program_syntax_error"].format(line=exc.lineno or 0, detail=exc.msg)
    try:
        exec(compiled, program_namespace(builder, log=log))  # noqa: S102 - workshop code, restricted builtins
    except NameError as exc:
        return None, TEXTS["program_unknown_name"].format(detail=exc)
    except Exception as exc:  # pragma: no cover - message differs per error type
        return None, TEXTS["program_error"].format(detail=exc)
    if not builder.steps:
        return None, TEXTS["program_no_steps"]
    return builder.steps, None


# -- task: motion and capture (pure, unit tested) --------------------------------


def apply_pose(packet_handler, pose, limits=None, speed=SAFE_SPEED, acc=SAFE_ACC):
    """Send one goal per joint of a pose. Returns ({servo_id: position}, [errors]).

    Joints absent from the pose are not touched, so an arm move never disturbs
    the gripper.
    """
    positions, errors = {}, []
    for servo_id in sorted(pose):
        joint_limits = (limits or {}).get(servo_id)
        position, error = move_joint(
            packet_handler, servo_id, pose[servo_id], limits=joint_limits, speed=speed, acc=acc
        )
        if error:
            errors.append(TEXTS["move_error"].format(name=nom_articulacio(servo_id), detail=error))
            continue
        positions[servo_id] = position
    return positions, errors


def hold_here(packet_handler, ids, speed=SAFE_SPEED, acc=SAFE_ACC):
    """Freeze the arm: command every responding servo the position it reads now.

    Returns {servo_id: position}. Unlike prepare_joints this never writes the
    torque register: the torque must stay on or the arm collapses under its own
    weight and drops whatever the gripper holds.
    """
    readings = read_positions(packet_handler, ids)
    for servo_id, reading in readings.items():
        packet_handler.WritePosEx(servo_id, reading["position"], speed, acc)
    return {servo_id: reading["position"] for servo_id, reading in readings.items()}


def set_torque(packet_handler, ids, enabled, speed=SAFE_SPEED, acc=SAFE_ACC):
    """Turn the torque of every responding servo on or off.

    Enabling first commands each servo the position it currently reads, so a
    stale goal register cannot make the arm jump when the torque returns.
    Returns ({servo_id: position}, [errors]).
    """
    if enabled:
        positions = prepare_joints(packet_handler, ids, speed=speed, acc=acc)
        return positions, ([] if positions else [TEXTS["no_answer"]])

    readings = read_positions(packet_handler, ids)
    if not readings:
        return {}, [TEXTS["no_answer"]]
    errors = []
    for servo_id in sorted(readings):
        result, error = packet_handler.write1ByteTxRx(servo_id, REG_TORQUE_ENABLE, 0)
        if result != COMM_SUCCESS:
            errors.append(TEXTS["torque_error"].format(
                name=nom_articulacio(servo_id), detail=describe(packet_handler, result, error)
            ))
    return {servo_id: reading["position"] for servo_id, reading in readings.items()}, errors


def pose_literal(degrees_by_id):
    """'{1: 6, 2: 44, 3: 27, 4: 0, 5: 0}' — a ready-to-paste Python literal."""
    # int(round(...)) also kills the "-0" that formatting -0.1 would produce.
    body = ", ".join(f"{servo_id}: {int(round(degrees_by_id[servo_id]))}" for servo_id in sorted(degrees_by_id))
    return "{" + body + "}"


def anar_a_line(degrees_by_id):
    """'anar_a(base=-7, espatlla=30, colze=17, canell=0, gir=0)' for the arm joints."""
    names = [(1, "base"), (2, "espatlla"), (3, "colze"), (4, "canell"), (5, "gir")]
    args = ", ".join(
        f"{name}={int(round(degrees_by_id[servo_id]))}"
        for servo_id, name in names if servo_id in degrees_by_id
    )
    return f"anar_a({args})"


def capture_pose(packet_handler, ids):
    """Read the arm. Returns ({servo_id: degrees}, literal_without_gripper, error).

    Degrees use the same convention as the sliders and as the "degrees" field of
    the sliders, so the numbers agree with what the app shows.
    """
    readings = read_positions(packet_handler, ids)
    if not readings:
        return None, None, TEXTS["no_answer"]
    degrees_by_id = {
        servo_id: position_to_degrees(reading["position"], is_inverted(servo_id))
        for servo_id, reading in readings.items()
    }
    arm_only = {servo_id: degrees for servo_id, degrees in degrees_by_id.items() if servo_id != GRIPPER_ID}
    return degrees_by_id, pose_literal(arm_only), None


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


def _rounded_png(fill, radius=BUTTON_RADIUS, size=None, corners=None):
    """A rounded rectangle as PNG bytes, for a ttk image element.

    `corners` is (top-left, top-right, bottom-right, bottom-left); the default
    rounds all four. Tabs pass (True, True, False, False) so only the top is
    rounded and they still sit flush on the panel below.
    """
    side = 2 * radius + 2
    size = size or (side, side)
    image = Image.new("RGBA", size, (0, 0, 0, 0))
    ImageDraw.Draw(image).rounded_rectangle(
        (0, 0, size[0] - 1, size[1] - 1), radius=radius, fill=fill,
        corners=corners or (True, True, True, True),
    )
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def rounded_button_style(style, root):
    """Give TButton rounded corners. Returns the images, which must be kept alive.

    ttk has no border radius, so the button is drawn from a 9-patch image that
    Tk stretches. Without Pillow the buttons simply stay square.
    """
    if Image is None or tk is None:
        return []
    states = {"normal": TEAL, "active": CYAN, "pressed": TEAL, "disabled": PANEL_BG}
    try:
        images = {
            name: tk.PhotoImage(
                master=root, data=base64.b64encode(_rounded_png(colour)), name=f"btn_{name}"
            )
            for name, colour in states.items()
        }
        style.element_create(
            "Rounded.button", "image", images["normal"],
            ("pressed", images["pressed"]),
            ("active", images["active"]),
            ("disabled", images["disabled"]),
            border=BUTTON_RADIUS, sticky="nsew", padding=(10, 5),
        )
        style.layout("TButton", [(
            "Rounded.button", {"sticky": "nsew", "children": [(
                "Button.padding", {"sticky": "nsew", "children": [
                    ("Button.label", {"sticky": "nsew"})
                ]}
            )]}
        )])
    except Exception:  # pragma: no cover - depends on the Tk build
        return []
    return list(images.values())


def rounded_tab_style(style, root):
    """Round the top corners of the notebook tabs. Returns the images to keep."""
    if Image is None or tk is None:
        return []
    radius = BUTTON_RADIUS
    # Pillow needs a few pixels more than 2*radius when only some corners round.
    size = (2 * radius + 4, 2 * radius + 8)
    states = {"tab": TAB_BG, "tabsel": TEAL, "tabact": CYAN}
    try:
        images = {
            name: tk.PhotoImage(
                master=root, name=f"nb_{name}",
                data=base64.b64encode(
                    _rounded_png(colour, radius, size, corners=(True, True, False, False))
                ),
            )
            for name, colour in states.items()
        }
        style.element_create(
            "Rounded.tab", "image", images["tab"],
            ("selected", images["tabsel"]),
            ("active", images["tabact"]),
            border=(radius, radius, radius, 2), sticky="nsew", padding=(10, 4),
        )
        style.layout("TNotebook.Tab", [(
            "Rounded.tab", {"sticky": "nsew", "children": [(
                "Notebook.padding", {"side": "top", "sticky": "nsew", "children": [
                    ("Notebook.label", {"side": "top", "sticky": ""})
                ]}
            )]}
        )])
    except Exception:  # pragma: no cover - depends on the Tk build
        return []
    return list(images.values())


def apply_dark_theme(root):
    """Black window with the logo's cyan as the accent. Returns the ttk.Style."""
    style = ttk.Style(root)
    # Only "clam" honours most of the colour options below.
    if "clam" in style.theme_names():
        style.theme_use("clam")
    root.configure(bg=BG)

    style.configure(".", background=PANEL_BG, foreground=FG, fieldbackground=FIELD_BG,
                    bordercolor=TEAL, lightcolor=PANEL_BG, darkcolor=PANEL_BG)
    style.configure("TFrame", background=BG)
    style.configure("TLabel", background=BG, foreground=FG)
    style.configure("TCheckbutton", background=BG, foreground=FG)
    style.map("TCheckbutton",
              background=[("active", BG)],
              foreground=[("disabled", MUTED_FG)],
              indicatorcolor=[("selected", CYAN), ("!selected", FIELD_BG)])
    # "Motors lliures" is the one control that must be easy to hit in a hurry.
    style.configure("Big.TCheckbutton", background=BG, foreground=FG,
                    font=("Arial", 11, "bold"), indicatorsize=14, padding=(4, 3))
    style.map("Big.TCheckbutton",
              background=[("active", BG)],
              foreground=[("disabled", MUTED_FG)],
              indicatorcolor=[("selected", CYAN), ("!selected", FIELD_BG)])

    style.configure("TLabelframe", background=PANEL_BG, bordercolor=TEAL)
    style.configure("TLabelframe.Label", background=PANEL_BG, foreground=CYAN, font=BOLD_FONT)
    # Frames inside a label frame must not fall back to the window background.
    style.configure("Panel.TFrame", background=PANEL_BG)
    style.configure("Panel.TLabel", background=PANEL_BG, foreground=FG, font=BOLD_FONT)

    # The rounded image element paints the button. Its transparent corners are
    # flattened against this background, so it must match the container, not the
    # button colour, or the rounding is invisible.
    style.configure("TButton", background=BG, foreground=BUTTON_FG, bordercolor=TEAL,
                    focuscolor=CYAN, padding=(4, 3), font=BOLD_FONT)
    style.map("TButton",
              background=[("pressed", BG), ("active", BG), ("disabled", BG)],
              foreground=[("active", BG), ("disabled", MUTED_FG)])
    # Same, for the buttons that sit inside a label frame.
    style.configure("Panel.TButton", background=PANEL_BG, foreground=BUTTON_FG,
                    focuscolor=CYAN, padding=(4, 3), font=BOLD_FONT)
    style.map("Panel.TButton",
              background=[("pressed", PANEL_BG), ("active", PANEL_BG), ("disabled", PANEL_BG)],
              foreground=[("active", BG), ("disabled", MUTED_FG)])

    # Buttons inside the light grey guide text: the rounded corners must blend
    # with that grey, not with the window.
    style.configure("Guide.TButton", background=TEXT_BG, foreground=BUTTON_FG,
                    focuscolor=CYAN, padding=(6, 4), font=BOLD_FONT)
    style.map("Guide.TButton",
              background=[("pressed", TEXT_BG), ("active", TEXT_BG), ("disabled", TEXT_BG)],
              foreground=[("active", BG), ("disabled", MUTED_FG)])

    style.configure("TEntry", fieldbackground=FIELD_BG, foreground=FG,
                    insertcolor=CYAN, bordercolor=TEAL)
    style.map("TEntry",
              fieldbackground=[("disabled", PANEL_BG)],
              foreground=[("disabled", MUTED_FG)],
              bordercolor=[("focus", CYAN)])

    # The slider itself is CYAN, its groove TEAL, both straight from the logo.
    style.configure("Horizontal.TScale", background=CYAN, troughcolor=FIELD_BG,
                    bordercolor=TEAL, lightcolor=CYAN, darkcolor=TEAL)
    style.map("Horizontal.TScale",
              background=[("active", CYAN), ("disabled", MUTED_FG)],
              troughcolor=[("disabled", PANEL_BG)])

    style.configure("Accent.TLabel", background=PANEL_BG, foreground=CYAN)
    style.configure("Unit.TLabel", background=PANEL_BG, foreground=MUTED_FG)
    style.configure("Hint.TLabel", background=PANEL_BG, foreground=MUTED_FG)
    style.configure("Title.TLabel", background=BG, foreground=TEAL)
    style.configure("Status.TLabel", background=BG, foreground=CYAN)
    style.configure("TNotebook", background=BG, bordercolor=TEAL)
    # The tab you are on is the logo teal; the ones you can switch to are black.
    # The selected tab is the logo teal and grows, so it reads as the one you are on.
    style.configure("TNotebook.Tab", background=TAB_BG, foreground=BUTTON_FG,
                    padding=(10, 4), font=("Arial", 10))
    style.map("TNotebook.Tab",
              background=[("selected", TEAL), ("active", CYAN)],
              foreground=[("selected", BUTTON_FG), ("active", BG)],
              padding=[("selected", (12, 5))],
              font=[("selected", BOLD_FONT)])
    style.configure("TScrollbar", background=TEAL, troughcolor=FIELD_BG, bordercolor=BG,
                    arrowcolor=FG)
    return style


def style_text(widget):
    """Colours for a plain tk.Text, which ttk styles do not reach.

    The program editor holds a lot of text to read and edit, so it is light grey
    with dark text rather than dark like the rest of the window. The log box uses
    style_log() instead.
    """
    widget.configure(bg=TEXT_BG, fg=TEXT_FG, insertbackground=TEAL,
                     selectbackground=TEAL, selectforeground="#FFFFFF",
                     highlightthickness=1, highlightbackground=TEAL, highlightcolor=CYAN,
                     borderwidth=0)


def style_log(widget):
    """Colours for the log box: dark grey, one step lighter than the window."""
    widget.configure(bg=LOG_BG, fg=LOG_FG, insertbackground=CYAN,
                     selectbackground=TEAL, selectforeground="#FFFFFF",
                     highlightthickness=1, highlightbackground=TEAL, highlightcolor=CYAN,
                     borderwidth=0)


def load_logo(master=None, path=LOGO_PATH):
    """The logo as a Tk image, or None when it is missing or Tk cannot read it.

    `master` ties the image to that window's interpreter. Without it the image
    binds to the default root, which breaks as soon as a second window is made.
    """
    if tk is None or not Path(path).exists():
        return None
    try:
        return tk.PhotoImage(master=master, file=str(path))
    except Exception:  # pragma: no cover - depends on the Tk build
        return None


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


# -- guide tab (pure, unit tested) ---------------------------------------------

_GUIA_NUMBER = re.compile(r"^(\d+)\.\s+(.*)$")
_GUIA_IMAGE = re.compile(r"^!\[[^\]]*\]\(([^)]+)\)$")
_GUIA_VIDEO = re.compile(r"^\[video:(\d+)\]$")


def parse_guia(text):
    """Read one guide section, written in a small subset of Markdown.

    Returns a list of (kind, content) blocks:
      "# x" title, "## x" heading, "- x" bullet, "1. x" number (content (1, x)),
      "> x" warning, "```" ... "```" code, "![](path)" image, "[video:N]" video
      (content N, an int), and any other run of lines is one paragraph.
    Inline **bold** is left in the text; split_bold() handles it when drawing.
    """
    blocks, paragraph, code = [], [], None

    def flush():
        if paragraph:
            blocks.append(("paragraph", " ".join(paragraph)))
            paragraph.clear()

    for raw in text.splitlines():
        line = raw.strip()
        if code is not None:
            if line.startswith("```"):
                blocks.append(("code", "\n".join(code)))
                code = None
            else:
                code.append(raw.rstrip())
            continue
        if line.startswith("```"):
            flush()
            code = []
        elif not line:
            flush()
        elif line.startswith("## "):
            flush()
            blocks.append(("heading", line[3:].strip()))
        elif line.startswith("# "):
            flush()
            blocks.append(("title", line[2:].strip()))
        elif line.startswith("- "):
            flush()
            blocks.append(("bullet", line[2:].strip()))
        elif line.startswith(">"):
            flush()
            blocks.append(("warning", line[1:].strip()))
        elif _GUIA_NUMBER.match(line):
            flush()
            number, rest = _GUIA_NUMBER.match(line).groups()
            blocks.append(("number", (int(number), rest)))
        elif _GUIA_IMAGE.match(line):
            flush()
            blocks.append(("image", _GUIA_IMAGE.match(line).group(1)))
        elif _GUIA_VIDEO.match(line):
            flush()
            blocks.append(("video", int(_GUIA_VIDEO.match(line).group(1))))
        else:
            paragraph.append(line)
    if code is not None:  # an unclosed block still shows
        blocks.append(("code", "\n".join(code)))
    flush()
    return blocks


def split_bold(text):
    """'a **b** c' -> [("a ", False), ("b", True), (" c", False)], empty parts dropped."""
    return [(part, index % 2 == 1) for index, part in enumerate(text.split("**")) if part]


def load_guia(folder=GUIA_DIR):
    """[(title, blocks)] for every *.md in `folder`, in file name order.

    The title is the first "# " line of the file, or the file name without it.
    """
    sections = []
    for path in sorted(Path(folder).glob("*.md")):
        blocks = parse_guia(path.read_text(encoding="utf-8"))
        title = next((content for kind, content in blocks if kind == "title"), path.stem)
        sections.append((title, blocks))
    return sections


def video_title(servo_id):
    """What VLC shows for the video of a joint, e.g. "Articulació 1 – Base"."""
    return TEXTS["video_title"].format(servo_id=servo_id, name=nom_articulacio(servo_id))


def video_path(servo_id, folder=VIDEO_DIR):
    return Path(folder) / VIDEOS[servo_id]


def video_thumbnail(servo_id, folder=GUIA_IMAGE_DIR):
    return Path(folder) / (Path(VIDEOS[servo_id]).stem + ".png")


# Where VLC can be besides the PATH: a snap, or a flatpak.
VLC_SNAP = "/snap/bin/vlc"
VLC_FLATPAK = "org.videolan.VLC"


def find_vlc(which=None, exists=None, run=None):
    """The command that starts VLC, as a list, or None when it is not installed.

    Raspberry Pi OS ships VLC as a normal package; on Ubuntu it is often a snap
    (not always on the PATH of a desktop launcher) or a flatpak.
    """
    which = which or shutil.which
    exists = exists or (lambda path: Path(path).exists())
    run = run or subprocess.run
    vlc = which("vlc")
    if vlc:
        return [vlc]
    if exists(VLC_SNAP):
        return [VLC_SNAP]
    flatpak = which("flatpak")
    if flatpak:
        try:
            info = run([flatpak, "info", VLC_FLATPAK], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=5)
        except (OSError, subprocess.SubprocessError):
            info = None
        if info is not None and info.returncode == 0:
            return [flatpak, "run", VLC_FLATPAK]
    return None


def default_video_app(which=None, run=None):
    """The desktop's default application for .mp4 files, or "" when there is none."""
    which = which or shutil.which
    run = run or subprocess.run
    xdg_mime = which("xdg-mime")
    if not xdg_mime:
        return ""
    try:
        result = run([xdg_mime, "query", "default", "video/mp4"], capture_output=True,
                     text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return ""
    return result.stdout.strip() if result.returncode == 0 else ""


def vlc_command(vlc, path, title):
    """VLC with the joint's name on the window, in the playlist and over the video.

    `vlc` is the command that starts it, a string or a list (see find_vlc()).
    """
    vlc = [vlc] if isinstance(vlc, str) else list(vlc)
    return [
        *vlc, "--play-and-exit",
        f"--video-title={title}", f"--meta-title={title}",
        "--sub-source=marq", f"--marq-marquee={title}",
        "--marq-position=4", "--marq-size=32", "--marq-color=0x6CDAE7",
        str(path),
    ]


def play_video(path, title, popen=None, find=None, default_app=None, which=None):
    """Open a video in another application, without waiting for it.

    VLC first, so the joint's name shows; otherwise the desktop's default video
    player through xdg-open. xdg-open fails silently when there is no default
    player, so that case is checked first and reported instead.
    Returns (ok, message) for the status line.
    """
    popen = popen or subprocess.Popen
    find = find or find_vlc
    default_app = default_app or default_video_app
    which = which or shutil.which
    path = Path(path)
    if not path.exists():
        return False, TEXTS["video_missing"].format(path=path)
    vlc = find()
    if vlc:
        command, message = vlc_command(vlc, path, title), TEXTS["video_opened"]
    elif which("xdg-open") and default_app():
        command, message = [which("xdg-open"), str(path)], TEXTS["video_no_vlc"]
    else:
        return False, TEXTS["video_no_player"]
    try:
        popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    except OSError as exc:
        return False, TEXTS["video_error"].format(detail=exc)
    return True, message.format(title=title)


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
        # Task player: one after() token at a time, always in self._task_after.
        self.task, self.task_errors = [], []
        self._task_state = "stopped"
        self._task_index = 0
        self._task_after = None
        self._stepping = False  # run one step and pause, instead of the whole task
        self._angle_after = None  # timer that re-reads the live joint angles

        self.style = apply_dark_theme(root)
        # Tk drops images nothing holds, which would blank the buttons.
        self._button_images = rounded_button_style(self.style, root)
        self._tab_images = rounded_tab_style(self.style, root)
        root.title(TEXTS["window_title"])
        root.geometry("800x860")
        root.minsize(700, 680)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        frame = ttk.Frame(root, padding=20)
        frame.pack(fill="both", expand=True)

        header = ttk.Frame(frame)
        header.pack(fill="x")
        titles = ttk.Frame(header)
        titles.pack(side="left", fill="x", expand=True)
        ttk.Label(titles, text=TEXTS["title"], font=("Arial", 20, "bold"), style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            titles,
            text=TEXTS["intro"].format(first=ids[0], last=ids[-1], port=port),
            wraplength=600,
            justify="left",
        ).pack(anchor="w", pady=(8, 12))
        # Keep a reference: Tk drops an image that nothing else holds.
        self.logo_image = load_logo(root)
        if self.logo_image is not None:
            ttk.Label(header, image=self.logo_image).pack(side="right", anchor="n", padx=(12, 28))

        top = ttk.Frame(frame)
        top.pack(fill="x")
        self.connect_button = ttk.Button(top, text=TEXTS["connect"], command=self.connect, width=BUTTON_WIDTH)
        self.connect_button.pack(side="left")
        self.button = ttk.Button(top, text=TEXTS["button"], command=self.on_set_zero, width=BUTTON_WIDTH)
        self.button.pack(side="left", padx=(8, 0))
        self.zero_all_button = ttk.Button(top, text=TEXTS["zero_all"], command=self.on_zero_all, width=BUTTON_WIDTH)
        self.zero_all_button.pack(side="left", padx=(8, 0))
        self.free_var = tk.BooleanVar(value=False)
        self.free_check = ttk.Checkbutton(
            # The leading spaces are the gap between the box and the words: ttk
            # gives no option for it.
            top, text="  " + TEXTS["torque_free"], variable=self.free_var,
            command=self.on_toggle_torque, style="Big.TCheckbutton",
        )
        self.free_check.pack(side="right", padx=(16, 14))

        self.limits = dict(JOINT_LIMITS)
        self.speed, self.acc = SAFE_SPEED, SAFE_ACC
        self.home_pose = dict(POSICIO_ZERO)
        self.move_seconds, self.gripper_seconds = MOVE_SECONDS, GRIPPER_SECONDS
        self.slider_vars = {}
        self.scales = {}
        self.value_labels = {}
        self.entry_vars = {}
        self.limit_vars = {}
        self.entries = {}
        self.limit_entries = {}

        self.notebook = ttk.Notebook(frame)
        self.notebook.pack(fill="x", pady=(12, 0))
        control_tab = ttk.Frame(self.notebook, padding=(0, 10))
        program_tab = ttk.Frame(self.notebook, padding=(0, 10))
        guide_tab = ttk.Frame(self.notebook, padding=(0, 10))
        self.notebook.add(control_tab, text=TEXTS["tab_control"])
        self.notebook.add(program_tab, text=TEXTS["tab_program"])
        self.notebook.add(guide_tab, text=TEXTS["tab_guide"])
        self._build_joint_sliders(control_tab)
        self._build_motion_panel(control_tab)
        self._build_program_tab(program_tab)
        self._build_guide_tab(guide_tab)

        self.status_var = tk.StringVar(value=TEXTS["ready"])
        ttk.Label(frame, textvariable=self.status_var, wraplength=720, justify="left",
                  font=("Arial", 11, "bold"), style="Status.TLabel").pack(
            anchor="w", pady=(12, 6)
        )

        self.log_box = tk.Text(frame, height=5, state="disabled", wrap="word")
        style_log(self.log_box)
        self.log_box.pack(fill="both", expand=True)

        self._set_task_controls()
        self._update_task_label()
        self._schedule_angle_refresh()
        if self.task_errors:
            for problem in self.task_errors:
                self.log(problem)
            self.set_status(TEXTS["task_not_started"], ok=False)

    def _build_joint_sliders(self, parent):
        box = ttk.LabelFrame(parent, text=TEXTS["joints"], padding=(12, 8))
        box.pack(fill="x")
        box.grid_columnconfigure(1, weight=1)
        header_font = ("Arial", 9, "bold")
        ttk.Label(box, style="Panel.TLabel", text=TEXTS["col_current"], font=header_font).grid(row=0, column=2, sticky="e")
        ttk.Label(box, style="Panel.TLabel", text=TEXTS["col_position"], font=header_font).grid(row=0, column=3, columnspan=2, sticky="e")
        ttk.Label(box, style="Panel.TLabel", text=TEXTS["col_min"], font=header_font).grid(row=0, column=5, sticky="e", padx=(12, 0))
        ttk.Label(box, style="Panel.TLabel", text=TEXTS["col_max"], font=header_font).grid(row=0, column=6, sticky="e", padx=(6, 0))
        for index, (servo_id, name, low, high, _direction) in enumerate(JOINTS):
            if servo_id not in self.ids:
                continue
            row = index + 1
            ttk.Label(box, text=f"{servo_id}. {name}", width=16, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=4)
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
            # Column 2 shows what the servo actually reports, refreshed from the
            # arm, so it is right even while the motors are free and posed by hand.
            value_label = ttk.Label(box, text="--", width=7, anchor="e", font=("Arial", 10, "bold"),
                                    style="Accent.TLabel")
            value_label.grid(row=row, column=2, sticky="e", padx=(6, 0))
            entry_var = tk.StringVar(value="0")
            entry = ttk.Entry(box, textvariable=entry_var, width=7, justify="right")
            entry.grid(row=row, column=3, sticky="e", padx=(12, 2))
            entry.bind("<Return>", lambda event, sid=servo_id: self.on_entry(sid))
            entry.bind("<KP_Enter>", lambda event, sid=servo_id: self.on_entry(sid))
            self.entries[servo_id] = entry
            ttk.Label(box, text="°", style="Unit.TLabel").grid(row=row, column=4, sticky="w")
            low_var = tk.StringVar(value=f"{low:g}")
            high_var = tk.StringVar(value=f"{high:g}")
            limit_entries = []
            for column, limit_var in ((5, low_var), (6, high_var)):
                limit_entry = ttk.Entry(box, textvariable=limit_var, width=6, justify="right")
                limit_entry.grid(row=row, column=column, sticky="e", padx=(12 if column == 5 else 6, 0))
                limit_entry.bind("<Return>", lambda event, sid=servo_id: self.on_limits_entry(sid))
                limit_entry.bind("<KP_Enter>", lambda event, sid=servo_id: self.on_limits_entry(sid))
                limit_entries.append(limit_entry)
            self.limit_entries[servo_id] = tuple(limit_entries)
            self.slider_vars[servo_id] = var
            self.scales[servo_id] = scale
            self.value_labels[servo_id] = value_label
            self.entry_vars[servo_id] = entry_var
            self.limit_vars[servo_id] = (low_var, high_var)
        ttk.Label(box, text=TEXTS["entry_hint"], font=("Arial", 8), style="Hint.TLabel").grid(
            row=len(JOINTS) + 1, column=3, columnspan=4, sticky="e", pady=(2, 0)
        )

    def _build_motion_panel(self, parent):
        box = ttk.LabelFrame(parent, text=TEXTS["motion_title"], padding=(12, 8))
        box.pack(fill="x", pady=(12, 0))
        row = ttk.Frame(box, style="Panel.TFrame")
        row.pack(fill="x")

        self.speed_var = tk.StringVar(value=str(self.speed))
        self.acc_var = tk.StringVar(value=str(self.acc))
        ttk.Label(row, text=TEXTS["motion_speed"], style="Panel.TLabel").pack(side="left")
        self.speed_entry = ttk.Entry(row, textvariable=self.speed_var, width=7, justify="right")
        self.speed_entry.pack(side="left", padx=(6, 2))
        ttk.Label(row, text=TEXTS["unit_speed"], style="Unit.TLabel").pack(side="left", padx=(0, 16))
        ttk.Label(row, text=TEXTS["motion_acc"], style="Panel.TLabel").pack(side="left")
        self.acc_entry = ttk.Entry(row, textvariable=self.acc_var, width=7, justify="right")
        self.acc_entry.pack(side="left", padx=(6, 2))
        ttk.Label(row, text=TEXTS["unit_acc"], style="Unit.TLabel").pack(side="left")
        for entry in (self.speed_entry, self.acc_entry):
            entry.bind("<Return>", lambda event: self.on_motion_entry())
            entry.bind("<KP_Enter>", lambda event: self.on_motion_entry())
        self.motion_set_button = ttk.Button(row, text=TEXTS["motion_set_button"], command=self.on_motion_entry,
                                             width=8, style="Panel.TButton")
        self.motion_set_button.pack(side="left", padx=(16, 0))

        second = ttk.Frame(box, style="Panel.TFrame")
        second.pack(fill="x", pady=(6, 0))
        self.move_seconds_var = tk.StringVar(value=format_seconds(self.move_seconds))
        self.gripper_seconds_var = tk.StringVar(value=format_seconds(self.gripper_seconds))
        ttk.Label(second, text=TEXTS["durations_move"], style="Panel.TLabel").pack(side="left")
        self.move_seconds_entry = ttk.Entry(second, textvariable=self.move_seconds_var, width=7, justify="right")
        self.move_seconds_entry.pack(side="left", padx=(6, 2))
        ttk.Label(second, text=TEXTS["unit_seconds"], style="Unit.TLabel").pack(side="left", padx=(0, 16))
        ttk.Label(second, text=TEXTS["durations_gripper"], style="Panel.TLabel").pack(side="left")
        self.gripper_seconds_entry = ttk.Entry(second, textvariable=self.gripper_seconds_var, width=7, justify="right")
        self.gripper_seconds_entry.pack(side="left", padx=(6, 2))
        ttk.Label(second, text=TEXTS["unit_seconds"], style="Unit.TLabel").pack(side="left")
        for entry in (self.move_seconds_entry, self.gripper_seconds_entry):
            entry.bind("<Return>", lambda event: self.on_durations_entry())
            entry.bind("<KP_Enter>", lambda event: self.on_durations_entry())
        self.durations_set_button = ttk.Button(
            second, text=TEXTS["motion_set_button"], command=self.on_durations_entry,
            width=8, style="Panel.TButton",
        )
        self.durations_set_button.pack(side="left", padx=(16, 0))

        ttk.Label(
            box,
            text=TEXTS["motion_hint"].format(smin=SPEED_MIN, smax=SPEED_MAX, amin=ACC_MIN, amax=ACC_MAX),
            font=("Arial", 8),
            style="Hint.TLabel",
            wraplength=600,
            justify="left",
        ).pack(anchor="w", pady=(6, 0))

    def _build_program_tab(self, parent):
        ttk.Label(parent, text=TEXTS["program_title"], font=("Arial", 11, "bold")).pack(anchor="w")

        editor = ttk.Frame(parent)
        editor.pack(fill="both", expand=True, pady=(6, 0))
        self.code_box = tk.Text(editor, height=14, wrap="none", font=("monospace", 10), undo=True)
        scroll = ttk.Scrollbar(editor, orient="vertical", command=self.code_box.yview)
        self.code_box.configure(yscrollcommand=scroll.set)
        self.code_box.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        style_text(self.code_box)
        self.code_box.insert("1.0", PROGRAMA_EXEMPLE)

        run_row = ttk.Frame(parent)
        run_row.pack(fill="x", pady=(8, 0))
        self.program_run_button = ttk.Button(run_row, text=TEXTS["program_run"], command=self.on_run_program, width=22)
        self.program_run_button.pack(side="left")
        self.program_step_button = ttk.Button(
            run_row, text=TEXTS["task_step_button"], command=lambda: self.on_run_program(step=True), width=12
        )
        self.program_step_button.pack(side="left", padx=(8, 0))
        self.program_pause_button = ttk.Button(run_row, text=TEXTS["task_pause"], command=self.on_pause, width=10)
        self.program_pause_button.pack(side="left", padx=(8, 0))
        self.program_stop_button = ttk.Button(run_row, text=TEXTS["task_stop"], command=self.on_stop, width=10)
        self.program_stop_button.pack(side="left", padx=(8, 0))

        tool_row = ttk.Frame(parent)
        tool_row.pack(fill="x", pady=(8, 0))
        self.insert_button = ttk.Button(
            tool_row, text=TEXTS["program_insert"], command=self.on_insert_position, width=22
        )
        self.insert_button.pack(side="left")
        self.home_button = ttk.Button(tool_row, text=TEXTS["program_home"], command=self.on_save_home, width=20)
        self.home_button.pack(side="left", padx=(8, 0))
        self.program_load_button = ttk.Button(tool_row, text=TEXTS["program_load"], command=self.on_load_program, width=11)
        self.program_load_button.pack(side="right")
        self.program_save_button = ttk.Button(tool_row, text=TEXTS["program_save"], command=self.on_save_program, width=11)
        self.program_save_button.pack(side="right", padx=(0, 8))
        self.program_reset_button = ttk.Button(tool_row, text=TEXTS["program_reset"], command=self.on_reset_program, width=11)
        self.program_reset_button.pack(side="right", padx=(0, 8))

        self.task_step_var = tk.StringVar(value="")
        ttk.Label(parent, textvariable=self.task_step_var, wraplength=700, justify="left",
                  font=BOLD_FONT).pack(anchor="w", pady=(8, 0))
        ttk.Label(parent, text=TEXTS["program_hint"], font=("Arial", 8), wraplength=700, justify="left",
                  style="Hint.TLabel").pack(
            anchor="w", pady=(2, 0)
        )

    # -- guide tab -----------------------------------------------------------

    def _build_guide_tab(self, parent):
        try:
            self.guide_sections = load_guia()
        except OSError:
            self.guide_sections = []
        # Tk drops images and embedded widgets nothing holds; these are
        # replaced every time another section is shown.
        self._guide_images = []
        self._guide_widgets = []

        self.guide_list = tk.Listbox(
            parent, width=18, activestyle="none", exportselection=False,
            font=GUIA_MENU_FONT, bg=PANEL_BG, fg=FG, selectbackground=TEAL,
            selectforeground="#FFFFFF", highlightthickness=1, highlightbackground=TEAL,
            highlightcolor=CYAN, borderwidth=0,
        )
        for title, _ in self.guide_sections:
            self.guide_list.insert("end", title)
        self.guide_list.pack(side="left", fill="y")
        self.guide_list.bind("<<ListboxSelect>>", self.on_guide_select)

        reader = ttk.Frame(parent)
        reader.pack(side="left", fill="both", expand=True, padx=(10, 0))
        self.guide_text = tk.Text(reader, height=20, wrap="word", font=GUIA_FONT,
                                  padx=14, pady=10, cursor="arrow")
        scroll = ttk.Scrollbar(reader, orient="vertical", command=self.guide_text.yview)
        self.guide_text.configure(yscrollcommand=scroll.set)
        self.guide_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        style_text(self.guide_text)

        text = self.guide_text
        text.tag_configure("title", font=("Arial", 18, "bold"), foreground=TEAL, spacing3=8)
        text.tag_configure("heading", font=("Arial", 14, "bold"), foreground=TEAL,
                           spacing1=12, spacing3=4)
        text.tag_configure("bold", font=("Arial", 12, "bold"))
        text.tag_configure("paragraph", spacing3=6)
        text.tag_configure("item", lmargin1=12, lmargin2=34, spacing3=3)
        text.tag_configure("code", font=("monospace", 11), background="#D6D6D6",
                           lmargin1=18, lmargin2=18, spacing1=4)
        text.tag_configure("warning", font=("Arial", 12, "bold"), foreground="#6B1A00",
                           background="#E8C27A", lmargin1=8, lmargin2=8, spacing1=6, spacing3=6)
        text.tag_configure("widget", lmargin1=18, spacing1=4, spacing3=8)

        if self.guide_sections:
            self.guide_list.selection_set(0)
            self.show_guide_section(0)
        else:
            text.insert("end", TEXTS["guide_missing"].format(path=GUIA_DIR))
            text.configure(state="disabled")

    def on_guide_select(self, _event=None):
        selection = self.guide_list.curselection()
        if selection:
            self.show_guide_section(selection[0])

    def show_guide_section(self, index):
        text = self.guide_text
        text.configure(state="normal")
        for widget in self._guide_widgets:
            widget.destroy()
        self._guide_widgets, self._guide_images = [], []
        text.delete("1.0", "end")
        _, blocks = self.guide_sections[index]
        for kind, content in blocks:
            if kind == "title":
                self._guide_insert(content + "\n", "title")
            elif kind == "heading":
                self._guide_insert(content + "\n", "heading")
            elif kind == "paragraph":
                self._guide_insert(content + "\n", "paragraph")
            elif kind == "bullet":
                self._guide_insert("•  " + content + "\n", "item")
            elif kind == "number":
                number, rest = content
                self._guide_insert(f"{number}.  {rest}\n", "item")
            elif kind == "warning":
                self._guide_insert(content + "\n", "warning")
            elif kind == "code":
                text.insert("end", content + "\n", "code")
                self._guide_embed(ttk.Button(
                    text, text=TEXTS["guide_copy"], style="Guide.TButton",
                    command=lambda code=content: self.copy_to_program(code),
                ))
            elif kind == "image":
                image = self._guide_image(GUIA_DIR / content)
                if image is not None:
                    text.image_create("end", image=image, padx=18, pady=4)
                    text.insert("end", "\n")
            elif kind == "video" and content in VIDEOS:
                self._guide_embed(self._guide_video(content))
        text.configure(state="disabled")
        text.yview_moveto(0)

    def _guide_insert(self, line, tag):
        for part, bold in split_bold(line):
            self.guide_text.insert("end", part, (tag, "bold") if bold else (tag,))

    def _guide_embed(self, widget):
        self._guide_widgets.append(widget)
        self.guide_text.window_create("end", window=widget)
        self.guide_text.insert("end", "\n", "widget")

    def _guide_image(self, path):
        image = load_logo(self.root, path)  # any PNG, tied to this window
        if image is not None:
            self._guide_images.append(image)
        return image

    def _guide_video(self, servo_id):
        """Thumbnail and button; clicking either opens the video in VLC."""
        box = tk.Frame(self.guide_text, bg=TEXT_BG)
        play = lambda _event=None, sid=servo_id: self.open_video(sid)  # noqa: E731
        image = self._guide_image(video_thumbnail(servo_id))
        if image is not None:
            thumb = tk.Label(box, image=image, bg=TEXT_BG, cursor="hand2",
                             borderwidth=2, relief="solid", highlightthickness=0)
            thumb.pack(side="left")
            thumb.bind("<Button-1>", play)
        ttk.Button(
            box, text=f"{TEXTS['guide_video_button']}\n{video_title(servo_id)}",
            style="Guide.TButton", command=play,
        ).pack(side="left", padx=(12, 0))
        return box

    def open_video(self, servo_id):
        ok, message = play_video(video_path(servo_id), video_title(servo_id))
        self.set_status(message, ok=None if ok else False)
        return ok

    def copy_to_program(self, code):
        current = self.program_code().rstrip("\n")
        self.set_program_code((current + "\n\n" if current else "") + code.rstrip("\n") + "\n")
        self.code_box.see("end")
        self.set_status(TEXTS["guide_copied"], ok=True)

    # -- live angles ---------------------------------------------------------

    def _schedule_angle_refresh(self):
        self._angle_after = self.root.after(ANGLE_REFRESH_MS, self._refresh_angles)

    def _refresh_angles(self):
        """Show the angle each servo actually reports, then schedule the next read."""
        self._angle_after = None
        if self.connected and not self.task_playing:
            try:
                readings = read_positions(self.packet_handler, self.ids)
            except Exception:
                readings = {}
            for servo_id, label in self.value_labels.items():
                reading = readings.get(servo_id)
                if reading is None:
                    label.configure(text="--")
                    continue
                degrees = position_to_degrees(reading["position"], is_inverted(servo_id))
                label.configure(text=f"{degrees:.0f}°")
        self._schedule_angle_refresh()

    def _cancel_angle_refresh(self):
        if self._angle_after is not None:
            self.root.after_cancel(self._angle_after)
            self._angle_after = None

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
        positions = prepare_joints(packet_handler, self.ids, speed=self.speed, acc=self.acc)
        if not positions:
            port_handler.closePort()
            self.set_status(TEXTS["no_answer"], ok=False)
            return False
        self.port_handler, self.packet_handler = port_handler, packet_handler
        self.responding_ids = sorted(positions)
        self._show_positions(positions)
        self.set_status(TEXTS["connected"].format(port=self.port, ids=self.responding_ids), ok=True)
        return True

    @property
    def motors_free(self):
        return bool(self.free_var.get())

    def on_toggle_torque(self):
        """Checkbox: leave the motors free so the arm can be posed by hand."""
        wanted_free = self.motors_free
        if wanted_free and self.task_playing:
            self.free_var.set(False)
            self.set_status(TEXTS["torque_busy"], ok=False)
            return
        if not self.connect():
            self.free_var.set(not wanted_free)
            return
        if wanted_free and not ask_si_no(self.root, TEXTS["torque_confirm_title"], TEXTS["torque_confirm_text"]):
            self.free_var.set(False)
            self.set_status(TEXTS["cancelled"])
            return

        positions, errors = set_torque(
            self.packet_handler, self.responding_ids, not wanted_free, speed=self.speed, acc=self.acc
        )
        if errors:
            for problem in errors:
                self.log(problem)
            self.free_var.set(not wanted_free)  # the arm is not in the state the box claims
            self.set_status(errors[0], ok=False)
            return
        self._show_positions(positions)
        self._set_task_controls()
        if wanted_free:
            self.set_status(TEXTS["torque_off"])  # a deliberate state, not an error
        else:
            self.set_status(TEXTS["torque_on"], ok=True)

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
        # Cancel both after() chains: a pending callback would otherwise fire
        # into a destroyed widget tree.
        was_running = self._task_state != "stopped"
        self._cancel_task_timer()
        self._cancel_pending_sends()
        self._cancel_angle_refresh()
        if was_running and self.connected:
            hold_here(self.packet_handler, self.responding_ids, speed=self.speed, acc=self.acc)
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
            self.entry_vars[servo_id].set(f"{degrees:.0f}")
        finally:
            self._updating_sliders = False

    def _show_positions(self, positions):
        for servo_id, position in positions.items():
            if servo_id in self.slider_vars:
                self._set_slider(servo_id, position_to_degrees(position, self.reversed(servo_id)))

    def on_slider(self, servo_id, degrees):
        """Called on every slider change: keep the box in step; send when not dragging."""
        if self._updating_sliders or servo_id in self._dragging or self.task_playing:
            return
        pending = self._pending.pop(servo_id, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self._pending[servo_id] = self.root.after(SEND_DELAY_MS, lambda: self._send(servo_id))

    def on_slider_release(self, servo_id):
        """Mouse released: send one goal for the final slider value."""
        self._dragging.discard(servo_id)
        if self.task_playing:
            return
        pending = self._pending.pop(servo_id, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.entry_vars[servo_id].set(f"{self.slider_vars[servo_id].get():.0f}")
        self._send(servo_id)

    def on_entry(self, servo_id):
        """Enter pressed in the degrees box: move the joint to the typed value."""
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
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
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
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

    def on_durations_entry(self):
        """Enter or Aplicar in a wait box: apply the new step durations."""
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        values, error = parse_durations(self.move_seconds_var.get(), self.gripper_seconds_var.get())
        if error:
            self.move_seconds_var.set(format_seconds(self.move_seconds))
            self.gripper_seconds_var.set(format_seconds(self.gripper_seconds))
            self.set_status(error, ok=False)
            return
        self.move_seconds, self.gripper_seconds = values
        self.move_seconds_var.set(format_seconds(self.move_seconds))
        self.gripper_seconds_var.set(format_seconds(self.gripper_seconds))
        self.set_status(TEXTS["durations_set"].format(
            move=format_seconds(self.move_seconds), gripper=format_seconds(self.gripper_seconds)
        ))

    def on_motion_entry(self):
        """Enter in the speed or acceleration box: apply the new values."""
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        values, error = parse_motion(self.speed_var.get(), self.acc_var.get())
        if error:
            self.speed_var.set(str(self.speed))
            self.acc_var.set(str(self.acc))
            self.set_status(error, ok=False)
            return
        self.speed, self.acc = values
        self.speed_var.set(str(self.speed))
        self.acc_var.set(str(self.acc))
        # Time for a 90 degree move: travel plus the ramp up to speed.
        seconds = STEPS_PER_TURN / 4 / self.speed + self.speed / (self.acc * 100)
        self.set_status(TEXTS["motion_set"].format(
            speed=self.speed, acc=self.acc, seconds=format_seconds(round(seconds, 1))
        ))

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
        if self.task_playing:  # a debounce scheduled before playback started
            return
        if not self.connected:
            self.set_status(TEXTS["not_connected"], ok=False)
            return
        degrees = self.slider_vars[servo_id].get()
        position, error = move_joint(
            self.packet_handler, servo_id, degrees, reverse=self.reversed(servo_id),
            limits=self.limits.get(servo_id), speed=self.speed, acc=self.acc
        )
        name = nom_articulacio(servo_id)
        if error:
            self.set_status(TEXTS["move_error"].format(name=name, detail=error), ok=False)
        else:
            self.set_status(TEXTS["moved"].format(name=name, degrees=degrees, position=position))

    def on_zero_all(self):
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        for servo_id in self.slider_vars:
            self._set_slider(servo_id, 0.0)
        if not self.connected:
            self.set_status(TEXTS["not_connected"], ok=False)
            return
        for servo_id in self.responding_ids:
            if servo_id in self.slider_vars:
                # A joint whose limits exclude 0 goes to the nearest limit instead.
                move_joint(
                    self.packet_handler, servo_id, 0.0, reverse=self.reversed(servo_id),
                    limits=self.limits.get(servo_id), speed=self.speed, acc=self.acc
                )
        self.set_status(TEXTS["all_zero"])

    # -- task player ---------------------------------------------------------

    @property
    def task_playing(self):
        return self._task_state == "playing"

    def _cancel_task_timer(self):
        if self._task_after is not None:
            self.root.after_cancel(self._task_after)
            self._task_after = None

    def _cancel_pending_sends(self):
        """Drop slider debounces scheduled before playback started."""
        for token in list(self._pending.values()):
            self.root.after_cancel(token)
        self._pending.clear()

    def _set_task_controls(self):
        """Enable or disable every widget according to the player state."""
        busy = self._task_state != "stopped"
        state = "disabled" if busy else "normal"
        # With the torque off a goal position is stored but not obeyed, so the
        # joint controls would lie about what the arm is doing.
        joint_state = "disabled" if (busy or self.motors_free) else "normal"
        for widget in list(self.scales.values()) + list(self.entries.values()):
            widget.configure(state=joint_state)
        for low_entry, high_entry in self.limit_entries.values():
            low_entry.configure(state=joint_state)
            high_entry.configure(state=joint_state)
        for widget in (self.connect_button, self.zero_all_button,
                       self.button, self.program_run_button,
                       self.program_load_button, self.program_save_button, self.program_reset_button,
                       self.insert_button, self.home_button):
            # program_step_button is set below: it stays usable while paused.
            widget.configure(state=state)
        self.code_box.configure(state="disabled" if busy else "normal")
        self.speed_entry.configure(state=state)
        self.acc_entry.configure(state=state)
        self.motion_set_button.configure(state=state)
        self.move_seconds_entry.configure(state=state)
        self.gripper_seconds_entry.configure(state=state)
        self.durations_set_button.configure(state=state)
        play_state = "disabled" if self.task_playing else "normal"
        pause_state = "normal" if self.task_playing else "disabled"
        self.program_step_button.configure(state=play_state)
        self.program_pause_button.configure(state=pause_state)
        self.program_stop_button.configure(state="normal")
        self.free_check.configure(state="disabled" if busy else "normal")

    def _update_task_label(self):
        if self.task_errors:
            self.task_step_var.set(self.task_errors[0])
        elif not self.task:
            self.task_step_var.set("")
        elif self._task_state == "stopped" or self._task_index >= len(self.task):
            self.task_step_var.set(TEXTS["task_idle"].format(
                count=len(self.task), seconds=format_seconds(task_total_seconds(self.task))
            ))
        else:
            self.task_step_var.set(describe_step(self._task_index, len(self.task), self.task[self._task_index]))

    def on_play(self):
        """Start the task, or resume it from the step it was paused on."""
        if self.task_playing:
            return
        if not self._start_task_checks():
            return

        self._stepping = False
        resuming = self._task_state == "paused"
        if not resuming:
            self._task_index = 0
            self.clear_log()
        self._cancel_pending_sends()
        self._task_state = "playing"
        self._set_task_controls()
        if resuming:
            self.set_status(TEXTS["task_resumed"].format(index=self._task_index + 1, total=len(self.task)))
        else:
            self.set_status(TEXTS["task_started"])
        self._task_run_step()

    def _start_task_checks(self):
        """Connect, restore the torque and validate the task. True when it may run."""
        if not self.connect():
            return False
        problems = list(self.task_errors) + validate_task(self.task, self.limits, self.responding_ids)
        if problems:
            self.clear_log()
            for problem in problems:
                self.log(problem)
            self.set_status(TEXTS["task_not_started"], ok=False)
            return False
        if self.motors_free:  # a task cannot run on limp motors
            self.free_var.set(False)
            self.on_toggle_torque()
            if self.motors_free:
                return False
        return True

    def on_step(self):
        """Run one step and stop there, so the class can see what each line does."""
        if self.task_playing:
            return
        if not self._start_task_checks():
            return
        if self._task_state == "stopped" or self._task_index >= len(self.task):
            self._task_index = 0
            self.clear_log()
        self._cancel_pending_sends()
        self._task_state = "playing"
        self._stepping = True
        self._set_task_controls()
        self._task_run_step()

    def _task_run_step(self):
        self._task_after = None
        if not self.task_playing:  # a Stop landed between two callbacks
            return
        if self._task_index >= len(self.task):
            self._task_finish(TEXTS["task_finished"].format(total=len(self.task)), ok=True)
            return

        step = self.task[self._task_index]
        self._update_task_label()
        self.log(describe_step(self._task_index, len(self.task), step))
        positions, errors = apply_pose(
            self.packet_handler, step.pose, self.limits, speed=self.speed, acc=self.acc
        )
        if errors:
            self._task_abort(errors[0])
            return
        for servo_id in positions:
            if servo_id in self.slider_vars:
                self._set_slider(servo_id, step.pose[servo_id])
        self._task_after = self.root.after(max(1, int(step.seconds * 1000)), self._task_advance)

    def _task_advance(self):
        self._task_after = None
        if not self.task_playing:  # a Stop landed while this timer was pending
            return
        done = self._task_index + 1
        self._task_index = done
        if not self._stepping:
            self._task_run_step()
            return

        # Step mode: the move has finished, so pause here without freezing.
        self._stepping = False
        if self._task_index >= len(self.task):
            self._task_finish(TEXTS["task_step_last"].format(done=done, total=len(self.task)), ok=True)
            return
        self._task_state = "paused"
        self._set_task_controls()
        self._update_task_label()
        self.set_status(TEXTS["task_step_done"].format(
            done=done, total=len(self.task), next=self.task[self._task_index].name
        ))

    def _task_finish(self, message, ok):
        self._cancel_task_timer()
        self._task_state = "stopped"
        self._task_index = 0
        self._set_task_controls()
        self._update_task_label()
        self.set_status(message, ok=ok)

    def _task_abort(self, detail):
        step = self.task[self._task_index]
        message = TEXTS["task_move_error"].format(index=self._task_index + 1, name=step.name, detail=detail)
        self.log(message)
        if self.connected:
            hold_here(self.packet_handler, self.responding_ids, speed=self.speed, acc=self.acc)
        self._task_finish(message, ok=False)

    def on_pause(self):
        """Freeze the arm where it is and remember the step."""
        if not self.task_playing:
            return
        self._cancel_task_timer()
        self._task_state = "paused"
        if self.connected:
            hold_here(self.packet_handler, self.responding_ids, speed=self.speed, acc=self.acc)
        self._set_task_controls()
        self.set_status(TEXTS["task_paused"].format(index=self._task_index + 1, total=len(self.task)))

    def on_stop(self):
        """Freeze the arm and rewind to the first step. Live at all times."""
        self._cancel_task_timer()
        self._stepping = False
        was_running = self._task_state != "stopped"
        self._task_state = "stopped"
        self._task_index = 0
        if self.connected:
            hold_here(self.packet_handler, self.responding_ids, speed=self.speed, acc=self.acc)
        self._set_task_controls()
        self._update_task_label()
        self.set_status(TEXTS["task_stopped"] if was_running else TEXTS["task_hold"])

    def program_code(self):
        return self.code_box.get("1.0", "end-1c")

    def set_program_code(self, code):
        self.code_box.delete("1.0", "end")
        self.code_box.insert("1.0", code)

    def on_run_program(self, step=False):
        """Compile the participant's program into a task and run it.

        `step` runs only the first step; the Pas a pas button then advances.
        """
        if self._task_state == "paused" and step:
            self.on_step()  # continue the program already loaded
            return
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        self.clear_log()
        steps, error = run_program(
            self.program_code(), home=self.home_pose,
            move_seconds=self.move_seconds, gripper_seconds=self.gripper_seconds, log=self.log,
        )
        if error:
            self.set_status(error, ok=False)
            return
        self.task, self.task_errors = steps, []
        self._task_state = "stopped"  # a new program always starts from the beginning
        self._task_index = 0
        self.log(TEXTS["program_built"].format(
            count=len(steps), seconds=format_seconds(task_total_seconds(steps))
        ))
        self._update_task_label()
        self.on_step() if step else self.on_play()

    def on_insert_position(self):
        """Read the arm and write an anar_a(...) line into the program at the cursor."""
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        if not self.connect():
            return
        degrees_by_id, _, error = capture_pose(self.packet_handler, self.ids)
        if error:
            self.set_status(error, ok=False)
            return
        arm_only = {sid: deg for sid, deg in degrees_by_id.items() if sid != GRIPPER_ID}
        line = anar_a_line(arm_only)
        # Insert on its own line, after the line the cursor is on.
        self.code_box.insert("insert lineend", "\n" + line)
        self.code_box.see("insert")
        self.set_status(TEXTS["program_inserted"].format(line=line), ok=True)

    def on_save_home(self):
        """Take the current pose as the position that inici() goes to."""
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        if not self.connect():
            return
        degrees_by_id, _, error = capture_pose(self.packet_handler, self.ids)
        if error:
            self.set_status(error, ok=False)
            return
        self.home_pose = {
            servo_id: round(degrees, 1)
            for servo_id, degrees in degrees_by_id.items() if servo_id != GRIPPER_ID
        }
        self.set_status(TEXTS["program_home_saved"].format(pose=pose_literal(self.home_pose)), ok=True)

    def on_save_program(self):
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title=TEXTS["program_save"],
            defaultextension=".py",
            initialfile=DEFAULT_PROGRAM_FILE,
            filetypes=[("Python", "*.py"), ("Tots els fitxers", "*.*")],
        )
        if not path:
            self.set_status(TEXTS["save_cancelled"])
            return
        try:
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(self.program_code())
        except OSError as exc:
            self.set_status(TEXTS["program_file_error"].format(detail=exc), ok=False)
            return
        self.set_status(TEXTS["program_saved"].format(path=path), ok=True)

    def on_load_program(self):
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        path = filedialog.askopenfilename(
            parent=self.root,
            title=TEXTS["program_load"],
            filetypes=[("Python", "*.py"), ("Tots els fitxers", "*.*")],
        )
        if not path:
            self.set_status(TEXTS["save_cancelled"])
            return
        try:
            with open(path, encoding="utf-8") as handle:
                code = handle.read()
        except OSError as exc:
            self.set_status(TEXTS["program_file_error"].format(detail=exc), ok=False)
            return
        self.set_program_code(code)
        self.set_status(TEXTS["program_loaded"].format(path=path), ok=True)

    def on_reset_program(self):
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
        self.set_program_code(PROGRAMA_EXEMPLE)
        self.set_status(TEXTS["program_reset_done"])

    # -- save positions ------------------------------------------------------

    # -- zero pose -----------------------------------------------------------

    def on_set_zero(self):
        if self.task_playing:
            self.set_status(TEXTS["task_running_busy"], ok=False)
            return
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
            positions = prepare_joints(self.packet_handler, self.ids, speed=self.speed, acc=self.acc)
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
