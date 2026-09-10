import tkinter as tk
from tkinter import ttk

from mock_robot import MockRobot
from real_robot import RealRobotAdapter
from robot_controller import RobotController


class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LEERobot SO-101")
        self.root.geometry("900x620")
        self.root.minsize(760, 520)
        self.root.configure(bg="#edf4ff")

        self.language = "ca"
        self.mode = "demo"
        self.robot = MockRobot()
        self.controller = RobotController(self.robot)
        self.real_robot = None

        self.status_var = tk.StringVar(value="Estat: Llest per iniciar")
        self.log_var = tk.StringVar(value="Encara no s'ha executat cap acció.")
        self.mode_var = tk.StringVar(value="Mode: Demo")
        self.lang_var = tk.StringVar(value="Català")

        self._texts = {
            "ca": {
                "title": "LEERobot SO-101",
                "subtitle": "Assistent del taller",
                "welcome": "Benvingut/a",
                "demo_mode": "Iniciar taller",
                "real_mode": "Mode instructor",
                "lang": "Idioma",
                "actions": "Accions del robot",
                "home": "Anar a l'inici",
                "open": "Obrir pinça",
                "close": "Tancar pinça",
                "safe": "Moure amb seguretat",
                "stop": "Aturada d'emergència",
                "reset": "Restablir",
                "status": "Estat",
                "safety": "Seguretat",
                "safety_text": "1. Comprova l'espai. 2. Mantingues les mans allunyades. 3. Confirma que el robot està preparat abans de cada moviment. 4. Prem Aturada d'emergència si cal.",
                "task": "Tasques guiades",
                "task_intro": "Escull una tasca de manutenció",
                "pick_place": "Agafar i deixar",
                "jenga": "Jenga robòtic",
                "advanced": "Controls d'instructor",
                "mode_label": "Mode actual",
                "start_title": "Comencem",
                "start_subtitle": "Selecciona el mode de la sessió",
                "start_workshop": "Iniciar taller",
                "start_instructor": "Mode instructor",
                "start_demo": "Mode demostració",
                "exit": "Sortir",
            },
            "en": {
                "title": "LEERobot SO-101",
                "subtitle": "Workshop assistant",
                "welcome": "Welcome",
                "demo_mode": "Start workshop",
                "real_mode": "Instructor mode",
                "lang": "Language",
                "actions": "Robot actions",
                "home": "Go home",
                "open": "Open gripper",
                "close": "Close gripper",
                "safe": "Move safely",
                "stop": "Emergency stop",
                "reset": "Reset",
                "status": "Status",
                "safety": "Safety",
                "safety_text": "1. Check the area. 2. Keep hands clear. 3. Confirm the robot is ready before each move. 4. Press Emergency Stop if needed.",
                "task": "Guided tasks",
                "task_intro": "Choose a handling task",
                "pick_place": "Pick and place",
                "jenga": "Robotic Jenga",
                "advanced": "Instructor controls",
                "mode_label": "Current mode",
                "start_title": "Let’s begin",
                "start_subtitle": "Choose the session mode",
                "start_workshop": "Start workshop",
                "start_instructor": "Instructor mode",
                "start_demo": "Demo mode",
                "exit": "Exit",
            },
        }

        self._build_startup_ui()

    def _clear_root(self):
        for widget in self.root.winfo_children():
            widget.destroy()

    def _build_startup_ui(self):
        self._clear_root()
        self.root.geometry("520x350")

        frame = ttk.Frame(self.root, padding=26)
        frame.pack(fill="both", expand=True)

        ttk.Label(frame, text=self._texts[self.language]["start_title"], font=("Arial", 24, "bold")).pack(anchor="w", pady=(0, 8))
        ttk.Label(frame, text=self._texts[self.language]["start_subtitle"], font=("Arial", 11)).pack(anchor="w", pady=(0, 18))

        mode_frame = ttk.Frame(frame)
        mode_frame.pack(fill="x", pady=(0, 16))

        ttk.Button(mode_frame, text=self._texts[self.language]["start_workshop"], command=lambda: self._launch("demo"), width=24).pack(fill="x", pady=6)
        ttk.Button(mode_frame, text=self._texts[self.language]["start_instructor"], command=lambda: self._launch("real"), width=24).pack(fill="x", pady=6)
        ttk.Button(mode_frame, text=self._texts[self.language]["start_demo"], command=lambda: self._launch("demo"), width=24).pack(fill="x", pady=6)

        bottom = ttk.Frame(frame)
        bottom.pack(fill="x", pady=(20, 0))
        ttk.Button(bottom, text=self._texts[self.language]["lang"], command=self._toggle_language, width=12).pack(side="left")
        ttk.Button(bottom, text=self._texts[self.language]["exit"], command=self.root.destroy, width=12).pack(side="right")

    def _toggle_language(self):
        self.language = "en" if self.language == "ca" else "ca"
        self.lang_var.set("English" if self.language == "en" else "Català")
        self._build_startup_ui()

    def _launch(self, mode):
        self.mode = mode
        self._set_mode(mode)
        self._build_ui()

    def _build_ui(self):
        self._clear_root()
        self.root.geometry("900x620")
        self.root.minsize(760, 520)

        main = ttk.Frame(self.root, padding=16)
        main.pack(fill="both", expand=True)

        top_bar = ttk.Frame(main)
        top_bar.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 12))
        top_bar.grid_columnconfigure(0, weight=1)
        top_bar.grid_columnconfigure(1, weight=1)
        top_bar.grid_columnconfigure(2, weight=1)

        title = ttk.Label(top_bar, text=self._texts[self.language]["title"], font=("Arial", 22, "bold"))
        title.grid(row=0, column=0, sticky="w")

        mode_label = ttk.Label(top_bar, textvariable=self.mode_var, font=("Arial", 10, "bold"))
        mode_label.grid(row=0, column=1, sticky="center")

        lang_combo = ttk.Combobox(top_bar, textvariable=self.lang_var, state="readonly", width=12)
        lang_combo["values"] = ("Català", "English")
        lang_combo.grid(row=0, column=2, sticky="e")
        lang_combo.bind("<<ComboboxSelected>>", self._on_language_change)

        ttk.Label(main, text=self._texts[self.language]["subtitle"], font=("Arial", 11)).grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(0, 12)
        )

        selector_frame = ttk.LabelFrame(main, text=self._texts[self.language]["mode_label"], padding=10)
        selector_frame.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(0, 12))
        ttk.Button(selector_frame, text=self._texts[self.language]["demo_mode"], command=lambda: self._set_mode("demo")).pack(side="left", padx=(0, 8))
        ttk.Button(selector_frame, text=self._texts[self.language]["real_mode"], command=lambda: self._set_mode("real")).pack(side="left", padx=(0, 8))
        ttk.Button(selector_frame, text=self._texts[self.language]["advanced"], command=self._open_instructor_panel).pack(side="left")

        action_frame = ttk.LabelFrame(main, text=self._texts[self.language]["actions"], padding=12)
        action_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=(0, 12), pady=(0, 12))

        ttk.Button(action_frame, text=self._texts[self.language]["home"], command=self.go_home, width=18).grid(
            row=0, column=0, padx=6, pady=6, sticky="ew"
        )
        ttk.Button(action_frame, text=self._texts[self.language]["open"], command=self.open_gripper, width=18).grid(
            row=0, column=1, padx=6, pady=6, sticky="ew"
        )
        ttk.Button(action_frame, text=self._texts[self.language]["close"], command=self.close_gripper, width=18).grid(
            row=1, column=0, padx=6, pady=6, sticky="ew"
        )
        ttk.Button(action_frame, text=self._texts[self.language]["safe"], command=self.move_safe, width=18).grid(
            row=1, column=1, padx=6, pady=6, sticky="ew"
        )
        ttk.Button(action_frame, text=self._texts[self.language]["stop"], command=self.emergency_stop, width=18).grid(
            row=2, column=0, padx=6, pady=(8, 4), sticky="ew"
        )
        ttk.Button(action_frame, text=self._texts[self.language]["reset"], command=self.reset_emergency_stop, width=18).grid(
            row=2, column=1, padx=6, pady=(8, 4), sticky="ew"
        )

        for i in range(2):
            action_frame.grid_columnconfigure(i, weight=1)

        task_frame = ttk.LabelFrame(main, text=self._texts[self.language]["task"], padding=12)
        task_frame.grid(row=3, column=2, sticky="nsew")
        ttk.Label(task_frame, text=self._texts[self.language]["task_intro"], wraplength=220).pack(anchor="w", pady=(0, 8))
        ttk.Button(task_frame, text=self._texts[self.language]["pick_place"], command=self.task_pick_place, width=22).pack(fill="x", pady=4)
        ttk.Button(task_frame, text=self._texts[self.language]["jenga"], command=self.task_jenga, width=22).pack(fill="x", pady=4)

        info_frame = ttk.LabelFrame(main, text=self._texts[self.language]["status"], padding=12)
        info_frame.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(0, 12))
        ttk.Label(info_frame, textvariable=self.status_var, wraplength=760).pack(anchor="w")
        ttk.Label(info_frame, textvariable=self.log_var, wraplength=760, justify="left").pack(anchor="w", pady=(10, 0))

        safety_frame = ttk.LabelFrame(main, text=self._texts[self.language]["safety"], padding=12)
        safety_frame.grid(row=5, column=0, columnspan=3, sticky="ew")
        ttk.Label(
            safety_frame,
            text=self._texts[self.language]["safety_text"],
            wraplength=760,
            justify="left",
        ).pack(anchor="w")

        main.grid_columnconfigure(0, weight=1)
        main.grid_columnconfigure(1, weight=1)
        main.grid_columnconfigure(2, weight=1)
        main.grid_rowconfigure(4, weight=1)

    def _set_mode(self, mode):
        self.mode = mode
        if mode == "real":
            self.real_robot = RealRobotAdapter()
            try:
                self.real_robot.connect()
                self.robot = self.real_robot
                self.controller = RobotController(self.robot)
                self._update_status("Real robot connected")
                self._log("Mode: Real robot")
            except Exception as exc:
                self.real_robot = None
                self.robot = MockRobot()
                self.controller = RobotController(self.robot)
                self._update_status(f"Real robot unavailable: {exc}")
                self._log("Mode: Demo fallback")
        else:
            self.real_robot = None
            self.robot = MockRobot()
            self.controller = RobotController(self.robot)
            self._update_status("Demo mode active")
            self._log("Mode: Demo")

        self.mode_var.set(f"Mode: {'Real robot' if mode == 'real' else 'Demo'}")

    def _on_language_change(self, event):
        value = self.lang_var.get()
        self.language = "ca" if value == "Català" else "en"
        self._build_ui()

    def _update_status(self, message):
        prefix = "Estat:" if self.language == "ca" else "Status:"
        self.status_var.set(f"{prefix} {message}")

    def _log(self, message):
        self.log_var.set(message)

    def _open_instructor_panel(self):
        panel = tk.Toplevel(self.root)
        panel.title("Instructor controls")
        panel.geometry("420x220")
        panel.transient(self.root)

        ttk.Label(panel, text="Instructor access", font=("Arial", 14, "bold")).pack(padx=12, pady=(12, 8), anchor="w")
        ttk.Label(panel, text="Only for workshop staff. Use calibration and diagnostics safely.", wraplength=360).pack(padx=12, anchor="w")

        ttk.Button(panel, text="Reset emergency stop", command=self.reset_emergency_stop).pack(fill="x", padx=12, pady=(10, 6))
        ttk.Button(panel, text="Run diagnostics", command=self._diagnostics).pack(fill="x", padx=12, pady=6)
        ttk.Button(panel, text="Calibrate arm", command=self._calibration_notice).pack(fill="x", padx=12, pady=6)
        ttk.Button(panel, text="Close", command=panel.destroy).pack(fill="x", padx=12, pady=(6, 12))

    def _diagnostics(self):
        result = self.controller.reset_emergency_stop()
        self._update_status(result["message"])
        self._log("Diagnostics: emergency state reset")

    def _calibration_notice(self):
        self._update_status("Calibration should be done only by the instructor.")
        self._log("Calibration request registered")

    def go_home(self):
        result = self.controller.home()
        self._update_status(result["message"])
        self._log("Acció: Inici" if self.language == "ca" else "Action: Home")

    def open_gripper(self):
        result = self.controller.open_gripper()
        self._update_status(result["message"])
        self._log("Acció: Obrir pinça" if self.language == "ca" else "Action: Open gripper")

    def close_gripper(self):
        result = self.controller.close_gripper()
        self._update_status(result["message"])
        self._log("Acció: Tancar pinça" if self.language == "ca" else "Action: Close gripper")

    def move_safe(self):
        position = {"x": 10, "y": 10, "z": 5}
        result = self.controller.safe_move(position)
        self._update_status(result["message"])
        self._log(f"Acció: Moure a {position}" if self.language == "ca" else f"Action: Move to {position}")

    def emergency_stop(self):
        result = self.controller.emergency_stop()
        self._update_status(result["message"])
        self._log("Acció: Aturada d'emergència" if self.language == "ca" else "Action: Emergency stop")

    def reset_emergency_stop(self):
        result = self.controller.reset_emergency_stop()
        self._update_status(result["message"])
        self._log("Acció: Restablir aturada" if self.language == "ca" else "Action: Reset emergency stop")

    def task_pick_place(self):
        self._update_status("Task: pick and place is ready")
        self._log("Task: Pick and place")

    def task_jenga(self):
        self._update_status("Task: robotic Jenga is ready")
        self._log("Task: Robotic Jenga")


def main():
    root = tk.Tk()
    app = RobotApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
