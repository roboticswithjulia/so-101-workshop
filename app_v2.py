#!/usr/bin/env python3
"""SO-101 app v2 (Catalan UI): a single button that stores the current pose as the zero pose.

It does exactly what `servo_positions.py --set-zero` does, behind one big button:
open the bus servo adapter, read the position of each servo, tell every servo to
treat its current position as the middle of its range (2048) and read back to
confirm. The offset is stored in the servo EEPROM, so it survives power cycles.

    python3 app_v2.py                          # /dev/ttyACM0, IDs 1..6
    python3 app_v2.py --port /dev/ttyUSB0 --ids 1,2,3

Install the dependencies first: pip install -r requirements.txt
"""

import argparse
import sys

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:  # pragma: no cover - headless machines
    tk = None

from servo_positions import read_positions, set_zero_positions
from so101_bus import ARM_IDS, DEFAULT_BAUDRATE, DEFAULT_PORT, open_bus, parse_ids

NOMS_ARTICULACIONS = {
    1: "Base",
    2: "Espatlla",
    3: "Colze",
    4: "Canell",
    5: "Gir del canell",
    6: "Pinça",
}

TEXTS = {
    "window_title": "Posició zero del SO-101",
    "title": "Posició zero del SO-101",
    "intro": (
        "Col·loca el braç en la posició neutra amb la pinça oberta i prem el botó. "
        "Cada motor ({first}..{last}) de {port} desarà la seva posició actual com a zero (2048)."
    ),
    "button": "Fixar la posició actual com a zero",
    "ready": "A punt.",
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
    "no_tk": "Tkinter no està instal·lat. A Raspberry Pi OS / Debian executa: sudo apt install python3-tk",
    "cli_description": "Aplicació d'un sol botó: desa la posició actual com a posició zero.",
    "cli_port": "dispositiu sèrie (per defecte: %(default)s)",
    "cli_baud": "velocitat en bauds (per defecte: %(default)s)",
    "cli_ids": "IDs dels motors, p. ex. '1-6' o '1,2,3' (per defecte: %(default)s)",
}


def nom_articulacio(servo_id):
    return NOMS_ARTICULACIONS.get(servo_id, f"Motor {servo_id}")


def run_set_zero(port, baudrate, ids, log=print):
    """Open the bus, store the current pose as zero, close the bus.

    Returns (results, error): `results` is the {id: {"before", "after", "ok"}}
    dict from set_zero_positions, `error` a Catalan message when nothing could be done.
    """
    try:
        port_handler, packet_handler = open_bus(port, baudrate)
    except RuntimeError as exc:
        if "ermission" in str(exc) or "denegat" in str(exc):
            return {}, TEXTS["permission"].format(port=port)
        return {}, TEXTS["port_error"].format(port=port, detail=exc)
    try:
        readings = read_positions(packet_handler, ids)
        if not readings:
            return {}, TEXTS["no_answer"]
        for servo_id in ids:
            if servo_id in readings:
                log(TEXTS["current"].format(name=nom_articulacio(servo_id), servo_id=servo_id, position=readings[servo_id]["position"]))
        log("")
        results = set_zero_positions(packet_handler, ids, log=lambda *_: None)
    finally:
        port_handler.closePort()

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


def summarize(results, error):
    if error:
        return False, error
    failed = sorted(servo_id for servo_id, result in results.items() if not result["ok"])
    if failed:
        return False, TEXTS["not_applied"].format(ids=failed)
    return True, TEXTS["stored"].format(ids=sorted(results))


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


class ZeroApp:
    def __init__(self, root, port, baudrate, ids):
        self.root = root
        self.port = port
        self.baudrate = baudrate
        self.ids = ids

        root.title(TEXTS["window_title"])
        root.geometry("560x420")
        root.minsize(420, 320)

        frame = ttk.Frame(root, padding=20)
        frame.pack(fill="both", expand=True)

        ttk.Label(frame, text=TEXTS["title"], font=("Arial", 20, "bold")).pack(anchor="w")
        ttk.Label(
            frame,
            text=TEXTS["intro"].format(first=ids[0], last=ids[-1], port=port),
            wraplength=500,
            justify="left",
        ).pack(anchor="w", pady=(8, 16))

        self.button = ttk.Button(frame, text=TEXTS["button"], command=self.on_set_zero)
        self.button.pack(fill="x", ipady=14)

        self.status_var = tk.StringVar(value=TEXTS["ready"])
        ttk.Label(frame, textvariable=self.status_var, wraplength=500, justify="left", font=("Arial", 11, "bold")).pack(
            anchor="w", pady=(14, 6)
        )

        self.log_box = tk.Text(frame, height=9, state="disabled", wrap="word")
        self.log_box.pack(fill="both", expand=True)

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

    def on_set_zero(self):
        confirmed = ask_si_no(self.root, TEXTS["confirm_title"], TEXTS["confirm_text"])
        if not confirmed:
            self.status_var.set(TEXTS["cancelled"])
            return

        self.button.configure(state="disabled")
        self.clear_log()
        self.status_var.set(TEXTS["connecting"].format(port=self.port))
        self.root.update_idletasks()
        try:
            results, error = run_set_zero(self.port, self.baudrate, self.ids, log=self.log)
        finally:
            self.button.configure(state="normal")
        ok, message = summarize(results, error)
        self.status_var.set((TEXTS["ok_prefix"] if ok else TEXTS["error_prefix"]) + message)


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
    ZeroApp(root, args.port, args.baud, parse_ids(args.ids))
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
