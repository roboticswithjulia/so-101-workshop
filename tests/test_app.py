import ast
import json
import os
import tempfile
import unittest

import app.app as robot_app
import src.so101_bus as so101_bus
from tests.test_servo_tools import FakePacketHandler, healthy_servo


class FakePortHandler:
    def __init__(self):
        self.closed = False

    def closePort(self):
        self.closed = True


class SetZeroTests(unittest.TestCase):
    def setUp(self):
        self.original_open_bus = robot_app.open_bus

    def tearDown(self):
        robot_app.open_bus = self.original_open_bus

    def _fake_bus(self, servos):
        port_handler = FakePortHandler()
        robot_app.open_bus = lambda port, baudrate: (port_handler, FakePacketHandler(servos))
        return port_handler

    def test_run_set_zero_stores_pose_and_closes_port(self):
        port_handler = self._fake_bus({1: healthy_servo(2011), 2: healthy_servo(1500)})
        logs = []

        results, error = robot_app.run_set_zero("/dev/fake", 1000000, [1, 2, 3], log=logs.append)

        self.assertIsNone(error)
        self.assertEqual(results[1], {"before": 2011, "after": 2048, "ok": True})
        self.assertEqual(results[2]["after"], 2048)
        self.assertNotIn(3, results)
        self.assertTrue(port_handler.closed)
        self.assertTrue(any("Base (ID 1): posició actual 2011" in line for line in logs))
        self.assertTrue(any("Espatlla (ID 2): 1500 -> 2048 (correcte)" in line for line in logs))
        self.assertTrue(any("Colze (ID 3): sense resposta" in line for line in logs))
        ok, message = robot_app.summarize(results, error)
        self.assertTrue(ok)
        self.assertIn("[1, 2]", message)
        self.assertIn("Posició zero desada", message)

    def test_run_set_zero_reports_when_no_servo_answers(self):
        port_handler = self._fake_bus({})

        results, error = robot_app.run_set_zero("/dev/fake", 1000000, [1, 2], log=lambda *_: None)

        self.assertEqual(results, {})
        self.assertIn("Cap motor ha respost", error)
        self.assertTrue(port_handler.closed)
        self.assertEqual(robot_app.summarize(results, error), (False, error))

    def test_run_set_zero_reports_port_errors(self):
        def failing_open_bus(port, baudrate):
            raise RuntimeError("Could not open /dev/fake: Permission denied")

        robot_app.open_bus = failing_open_bus

        results, error = robot_app.run_set_zero("/dev/fake", 1000000, [1], log=lambda *_: None)

        self.assertEqual(results, {})
        self.assertIn("grup dialout", error)
        self.assertIn("torna a iniciar la sessió", error)

    def test_summarize_flags_servos_that_did_not_take_the_zero(self):
        results = {1: {"before": 2011, "after": 2048, "ok": True}, 2: {"before": 1500, "after": 1500, "ok": False}}

        ok, message = robot_app.summarize(results, None)

        self.assertFalse(ok)
        self.assertIn("[2]", message)
        self.assertIn("No s'ha pogut aplicar", message)


class SavePositionsTests(unittest.TestCase):
    def test_save_positions_writes_a_json_file(self):
        handler = FakePacketHandler({1: healthy_servo(2048), 3: healthy_servo(1707), 6: healthy_servo(2560)})

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "positions.json")
            record, error = robot_app.save_positions(handler, [1, 2, 3, 6], path, "/dev/ttyACM0")
            with open(path, encoding="utf-8") as handle:
                written = json.load(handle)

        self.assertIsNone(error)
        self.assertEqual(written, record)
        self.assertEqual(written["port"], "/dev/ttyACM0")
        self.assertEqual(written["zero_position"], 2048)
        self.assertEqual(sorted(written["servos"]), ["1", "3", "6"])  # servo 2 did not answer
        self.assertEqual(written["servos"]["1"], {
            "joint": "Base", "position": 2048, "degrees": 0.0, "inverted": False, "limits": [-75, 75],
        })
        # Colze is inverted: a raw position below 2048 is a positive angle.
        self.assertEqual(written["servos"]["3"]["degrees"], 30.0)
        self.assertTrue(written["servos"]["3"]["inverted"])
        self.assertEqual(written["servos"]["6"]["degrees"], 45.0)
        self.assertIn("saved_at", written)

    def test_save_positions_uses_the_limits_it_is_given(self):
        handler = FakePacketHandler({4: healthy_servo()})

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "p.json")
            record, error = robot_app.save_positions(handler, [4], path, "/dev/fake", limits={4: (-40, 40)})

        self.assertIsNone(error)
        self.assertEqual(record["servos"]["4"]["limits"], [-40, 40])

    def test_save_positions_reports_when_no_servo_answers(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "p.json")
            record, error = robot_app.save_positions(FakePacketHandler({}), [1, 2], path, "/dev/fake")
            self.assertFalse(os.path.exists(path))

        self.assertIsNone(record)
        self.assertIn("Cap motor ha respost", error)

    def test_save_positions_reports_a_file_error(self):
        handler = FakePacketHandler({1: healthy_servo()})

        record, error = robot_app.save_positions(handler, [1], "/no/such/folder/p.json", "/dev/fake")

        self.assertIsNone(record)
        self.assertIn("No s'han pogut desar", error)


class JointSliderTests(unittest.TestCase):
    def test_default_joint_limits(self):
        self.assertEqual(robot_app.JOINT_LIMITS, {
            1: (-75, 75), 2: (-75, 60), 3: (-25, 75), 4: (-70, 18), 5: (-75, 75), 6: (-25, 45),
        })

    def test_parse_limits(self):
        self.assertEqual(robot_app.parse_limits("-30", "45,5", 1), ((-30.0, 45.5), None))
        for low, high in (("abc", "10"), ("10", "10"), ("20", "10"), ("-200", "10"), ("0", "181")):
            limits, error = robot_app.parse_limits(low, high, 4)
            self.assertIsNone(limits, (low, high))
            self.assertIn("límits no vàlids", error)

    def test_move_joint_clamps_to_limits(self):
        handler = FakePacketHandler({4: healthy_servo()})

        position, error = robot_app.move_joint(handler, 4, 60, limits=(-70, 18))

        self.assertIsNone(error)
        self.assertEqual(position, robot_app.degrees_to_position(18))

    def test_only_colze_is_inverted_by_default(self):
        self.assertEqual({sid for sid in range(1, 7) if robot_app.is_inverted(sid)}, {3})
        self.assertEqual(robot_app.JOINT_DIRECTIONS[3], -1)

    def test_joint_names_are_catalan(self):
        self.assertEqual(
            [robot_app.nom_articulacio(i) for i in range(1, 7)],
            ["Base", "Espatlla", "Colze", "Canell", "Gir del canell", "Pinça"],
        )

    def test_degrees_to_position_is_relative_to_the_middle(self):
        self.assertEqual(robot_app.degrees_to_position(0), 2048)
        self.assertEqual(robot_app.degrees_to_position(90), 3072)
        self.assertEqual(robot_app.degrees_to_position(-90), 1024)

    def test_reverse_flag_multiplies_by_minus_one(self):
        self.assertEqual(robot_app.degrees_to_position(90, reverse=True), 1024)
        self.assertEqual(robot_app.degrees_to_position(-30, reverse=True), robot_app.degrees_to_position(30))
        self.assertAlmostEqual(robot_app.position_to_degrees(1024, reverse=True), 90.0)

    def test_degrees_to_position_is_clamped(self):
        self.assertEqual(robot_app.degrees_to_position(400), 4095)
        self.assertEqual(robot_app.degrees_to_position(-400), 0)

    def test_position_to_degrees_round_trips(self):
        for degrees in (-90, -12.5, 0, 33, 90):
            position = robot_app.degrees_to_position(degrees)
            self.assertAlmostEqual(robot_app.position_to_degrees(position), degrees, delta=0.1)

    def test_prepare_joints_holds_position_and_enables_torque(self):
        handler = FakePacketHandler({
            1: healthy_servo(2011, regs={so101_bus.REG_TORQUE_ENABLE: 0}),
            2: healthy_servo(1500),
        })

        positions = robot_app.prepare_joints(handler, [1, 2, 3])

        self.assertEqual(positions, {1: 2011, 2: 1500})
        self.assertEqual(handler.servos[1]["goal"], (2011, robot_app.SAFE_SPEED, robot_app.SAFE_ACC))
        self.assertEqual(handler.servos[1][so101_bus.REG_TORQUE_ENABLE], 1)
        self.assertEqual(handler.servos[2][so101_bus.REG_TORQUE_ENABLE], 1)

    def test_move_joint_sends_goal_at_safe_speed(self):
        handler = FakePacketHandler({4: healthy_servo()})

        position, error = robot_app.move_joint(handler, 4, 45)

        self.assertIsNone(error)
        self.assertEqual(position, 2560)
        self.assertEqual(handler.servos[4]["goal"], (2560, robot_app.SAFE_SPEED, robot_app.SAFE_ACC))

    def test_move_joint_uses_the_joint_direction_by_default(self):
        handler = FakePacketHandler({2: healthy_servo(), 3: healthy_servo()})

        self.assertEqual(robot_app.move_joint(handler, 2, 45), (2560, None))  # Espatlla: normal
        self.assertEqual(robot_app.move_joint(handler, 3, 45), (1536, None))  # Colze: inverted by default
        self.assertEqual(robot_app.move_joint(handler, 3, 45, reverse=False), (2560, None))

    def test_parse_degrees_accepts_numbers_within_the_joint_limits(self):
        self.assertEqual(robot_app.parse_degrees("45", 1), (45.0, None))
        self.assertEqual(robot_app.parse_degrees(" -12,5° ", 4), (-12.5, None))
        self.assertEqual(robot_app.parse_degrees("0", 6), (0.0, None))
        self.assertEqual(robot_app.parse_degrees("80", 1, limits=(-90, 90)), (80.0, None))

    def test_parse_degrees_rejects_text_and_values_outside_the_limits(self):
        for text, servo_id in (("abc", 1), ("", 2), ("-100", 2), ("46", 6), ("-76", 1), ("20", 4), ("-26", 3)):
            degrees, error = robot_app.parse_degrees(text, servo_id)
            self.assertIsNone(degrees, text)
            self.assertIn("valor no vàlid", error)
        self.assertIn("entre -70 i 18", robot_app.parse_degrees("20", 4)[1])

    def test_move_joint_reports_a_servo_that_does_not_answer(self):
        position, error = robot_app.move_joint(FakePacketHandler({}), 6, 10)

        self.assertIsNone(position)
        self.assertIn("result", error)


if __name__ == "__main__":
    unittest.main()


class TaskDefinitionTests(unittest.TestCase):
    def test_the_task_block_shipped_in_the_app_is_valid(self):
        """The TASCA block is edited by the teams, so only check that it builds and validates."""
        steps, errors = robot_app.build_task(robot_app.TASCA)

        self.assertEqual(errors, [])
        self.assertEqual(len(steps), len(robot_app.TASCA))
        self.assertEqual(robot_app.validate_task(steps), [])
        for step in steps:
            self.assertTrue(step.name)
            self.assertGreater(step.seconds, 0)
            self.assertTrue(all(isinstance(sid, int) for sid in step.pose))

    def test_a_bare_number_moves_only_the_gripper(self):
        steps, errors = robot_app.build_task([("Obrir pinça", -25, 1.0)])

        self.assertEqual(errors, [])
        self.assertEqual(steps[0].pose, {robot_app.GRIPPER_ID: -25.0})

    def test_a_dict_pose_keeps_only_its_own_joints(self):
        steps, _ = robot_app.build_task([("Base", {1: 30}, 1.0)])

        self.assertEqual(steps[0].pose, {1: 30.0})

    def test_build_task_reports_a_step_that_is_not_three_items(self):
        steps, errors = robot_app.build_task([("Bo", {1: 0}, 1.0), ("Dolent",), "text"])

        self.assertEqual(len(steps), 1)
        self.assertEqual(len(errors), 2)
        self.assertIn("Pas 2", errors[0])
        self.assertIn("format incorrecte", errors[0])

    def test_build_task_rejects_a_bad_duration(self):
        for seconds in (0, -1, "abc", 999, None):
            _, errors = robot_app.build_task([("Prova", {1: 0}, seconds)])
            self.assertEqual(len(errors), 1, seconds)
            self.assertIn("durada", errors[0])

    def test_build_task_rejects_a_pose_that_is_not_a_number_or_a_dict(self):
        for pose in ("endavant", True, None, [1, 2], {1: "molt"}, {"x": 1}):
            _, errors = robot_app.build_task([("Prova", pose, 1.0)])
            self.assertEqual(len(errors), 1, pose)
            self.assertIn("la posició ha de ser", errors[0])

    def test_validate_task_accepts_the_default_task(self):
        steps, _ = robot_app.build_task(robot_app.TASCA)

        # PINCA_OBERTA and PINCA_TANCADA sit exactly on joint 6's limits.
        self.assertEqual(robot_app.validate_task(steps), [])

    def test_validate_task_reports_a_pose_outside_the_runtime_limits(self):
        steps = [robot_app.TaskStep("Prova", {2: 70.0}, 1.0)]

        problems = robot_app.validate_task(steps, limits={2: (-75, 60)})

        self.assertEqual(len(problems), 1)
        self.assertIn("Espatlla", problems[0])
        self.assertIn("70", problems[0])
        self.assertIn("fora dels límits", problems[0])
        # The same pose passes once the limit is widened in the app.
        self.assertEqual(robot_app.validate_task(steps, limits={2: (-75, 90)}), [])

    def test_validate_task_reports_an_unknown_servo_id(self):
        problems = robot_app.validate_task([robot_app.TaskStep("Prova", {9: 0.0}, 1.0)])

        self.assertIn("el motor 9 no existeix", problems[0])

    def test_validate_task_reports_a_joint_that_does_not_answer(self):
        steps = [robot_app.TaskStep("Prova", {6: 0.0}, 1.0)]

        problems = robot_app.validate_task(steps, ids=[1, 2])

        self.assertIn("Pinça", problems[0])
        self.assertIn("no respon", problems[0])

    def test_validate_task_reports_an_empty_task(self):
        self.assertIn("buida", robot_app.validate_task([])[0])

    def test_task_total_seconds_and_formatting(self):
        steps, _ = robot_app.build_task([("Inici", {1: 0}, 2.0), ("Pinça", 0, 1.5)])

        self.assertEqual(robot_app.task_total_seconds(steps), 3.5)
        self.assertEqual(robot_app.format_seconds(2.5), "2,5")
        self.assertEqual(robot_app.format_seconds(2.0), "2")
        self.assertEqual(robot_app.describe_step(0, 2, steps[0]), "Pas 1/2: Inici (2 s)")
        self.assertEqual(robot_app.describe_step(1, 2, steps[1]), "Pas 2/2: Pinça (1,5 s)")


class TaskMotionTests(unittest.TestCase):
    def test_apply_pose_sends_one_goal_per_joint_and_honours_the_direction(self):
        handler = FakePacketHandler({servo_id: healthy_servo() for servo_id in (1, 2, 3)})

        positions, errors = robot_app.apply_pose(handler, {2: 45.0, 3: 45.0})

        self.assertEqual(errors, [])
        self.assertEqual(positions, {2: 2560, 3: 1536})  # Colze (3) is inverted

    def test_apply_pose_leaves_joints_outside_the_pose_untouched(self):
        handler = FakePacketHandler({servo_id: healthy_servo() for servo_id in range(1, 7)})

        robot_app.apply_pose(handler, {1: 10.0})

        self.assertIn("goal", handler.servos[1])
        for servo_id in (2, 3, 4, 5, 6):
            self.assertNotIn("goal", handler.servos[servo_id])

    def test_apply_pose_reports_a_servo_that_does_not_answer(self):
        positions, errors = robot_app.apply_pose(FakePacketHandler({}), {6: 0.0})

        self.assertEqual(positions, {})
        self.assertIn("Pinça", errors[0])

    def test_apply_pose_clamps_when_given_limits(self):
        handler = FakePacketHandler({4: healthy_servo()})

        positions, _ = robot_app.apply_pose(handler, {4: 60.0}, limits={4: (-70, 18)})

        self.assertEqual(positions[4], robot_app.degrees_to_position(18))

    def test_hold_here_commands_each_servo_its_own_position(self):
        handler = FakePacketHandler({1: healthy_servo(2011), 2: healthy_servo(1500)})

        positions = robot_app.hold_here(handler, [1, 2, 3])

        self.assertEqual(positions, {1: 2011, 2: 1500})
        self.assertEqual(handler.servos[1]["goal"], (2011, robot_app.SAFE_SPEED, robot_app.SAFE_ACC))
        self.assertEqual(handler.servos[2]["goal"][0], 1500)

    def test_hold_here_never_turns_the_torque_off(self):
        handler = FakePacketHandler({1: healthy_servo()})

        robot_app.hold_here(handler, [1])

        self.assertEqual(handler.servos[1][so101_bus.REG_TORQUE_ENABLE], 1)


class PoseCaptureTests(unittest.TestCase):
    def test_capture_pose_builds_a_literal_without_the_gripper(self):
        # Raw positions of config/positions_block.json.
        handler = FakePacketHandler({
            1: healthy_servo(2114), 2: healthy_servo(2544), 3: healthy_servo(1742),
            4: healthy_servo(2047), 5: healthy_servo(2048), 6: healthy_servo(2560),
        })

        degrees, literal, error = robot_app.capture_pose(handler, list(range(1, 7)))

        self.assertIsNone(error)
        self.assertEqual(literal, "{1: 6, 2: 44, 3: 27, 4: 0, 5: 0}")
        self.assertEqual(ast.literal_eval(literal), {1: 6, 2: 44, 3: 27, 4: 0, 5: 0})
        self.assertAlmostEqual(degrees[6], 45.0, places=1)  # reported separately

    def test_capture_pose_has_no_negative_zero_and_uses_the_direction(self):
        handler = FakePacketHandler({3: healthy_servo(1742), 4: healthy_servo(2047)})

        degrees, literal, _ = robot_app.capture_pose(handler, [3, 4])

        self.assertEqual(literal, "{3: 27, 4: 0}")  # not "-0"
        self.assertGreater(degrees[3], 0)  # Colze is inverted

    def test_capture_pose_reports_when_no_servo_answers(self):
        degrees, literal, error = robot_app.capture_pose(FakePacketHandler({}), [1])

        self.assertIsNone(degrees)
        self.assertIsNone(literal)
        self.assertIn("Cap motor ha respost", error)


def _has_display():
    if robot_app.tk is None:
        return False
    try:
        root = robot_app.tk.Tk()
    except Exception:
        return False
    root.destroy()
    return True


HAS_DISPLAY = _has_display()


@unittest.skipUnless(HAS_DISPLAY, "no usable display for Tk")
class TaskPlayerTests(unittest.TestCase):
    """Drive the play/pause/stop machine with recorded timers instead of real ones."""

    def setUp(self):
        self.root = robot_app.tk.Tk()
        self.root.withdraw()
        self.app = robot_app.RobotApp(self.root, "/dev/fake", 1000000, list(range(1, 7)))
        self.bus = FakePacketHandler({servo_id: healthy_servo() for servo_id in range(1, 7)})
        self.app.packet_handler = self.bus
        self.app.port_handler = object()
        self.app.responding_ids = list(range(1, 7))
        # Record timers instead of scheduling them, so the chain is stepped by hand.
        self.timers = []
        self.cancelled = []
        self.app.root.after = lambda ms, fn=None, *a: (self.timers.append((ms, fn)), f"t{len(self.timers)}")[1]
        self.app.root.after_cancel = self.cancelled.append
        # A fixture task, so these tests do not depend on the team-editable block.
        self.app.task, self.app.task_errors = robot_app.build_task([
            ("Inici", {1: 0, 2: 0}, 2.0),
            ("Obrir pinça", -20, 1.0),
            ("Anar a l'objecte", {1: 10, 2: 30}, 2.5),
        ])

    def tearDown(self):
        self.root.destroy()

    def goals(self):
        return {sid: regs["goal"][0] for sid, regs in self.bus.servos.items() if "goal" in regs}

    def clear_goals(self):
        for regs in self.bus.servos.values():
            regs.pop("goal", None)

    def log_text(self):
        return self.app.log_box.get("1.0", "end")

    def test_play_sends_the_first_pose_and_schedules_the_next_step(self):
        self.app.on_play()

        self.assertEqual(self.app._task_state, "playing")
        self.assertEqual(self.goals(), {1: 2048, 2: 2048})
        self.assertEqual(len(self.timers), 1)
        self.assertEqual(self.timers[0][0], 2000)

    def test_play_while_playing_does_not_start_a_second_chain(self):
        self.app.on_play()
        self.app.on_play()

        self.assertEqual(len(self.timers), 1)

    def test_play_refuses_when_a_pose_is_outside_the_limits(self):
        self.app.set_limits(2, (-10, 10))  # the fixture task asks for 30 degrees

        self.app.on_play()

        self.assertEqual(self.app._task_state, "stopped")
        self.assertEqual(self.goals(), {})
        self.assertEqual(self.timers, [])
        self.assertIn("fora dels límits", self.log_text())
        self.assertIn("Espatlla", self.log_text())

    def test_play_refuses_when_the_task_block_has_a_typo(self):
        self.app.task, self.app.task_errors = robot_app.build_task([("Dolent",)])

        self.app.on_play()

        self.assertEqual(self.app._task_state, "stopped")
        self.assertEqual(self.goals(), {})

    def test_the_second_step_moves_only_the_gripper(self):
        self.app.on_play()
        self.clear_goals()

        self.app._task_advance()

        self.assertEqual(self.goals(), {6: robot_app.degrees_to_position(-20)})

    def test_advancing_past_the_last_step_finishes_and_resets(self):
        self.app.on_play()
        for _ in range(len(self.app.task)):
            self.app._task_advance()

        self.assertEqual(self.app._task_state, "stopped")
        self.assertEqual(self.app._task_index, 0)
        self.assertIn("Tasca acabada", self.app.status_var.get())

    def test_pause_cancels_the_timer_and_freezes_the_arm(self):
        self.app.on_play()
        self.bus.servos[1][so101_bus.REG_PRESENT_POSITION] = 1900

        self.app.on_pause()

        self.assertEqual(self.app._task_state, "paused")
        self.assertEqual(self.cancelled, ["t1"])
        self.assertEqual(self.goals()[1], 1900)  # held where it actually is
        self.assertIn("pausa", self.app.status_var.get().lower())

    def test_play_after_pause_resumes_the_same_step(self):
        self.app.on_play()
        self.app._task_advance()
        index = self.app._task_index
        self.app.on_pause()
        self.timers.clear()

        self.app.on_play()

        self.assertEqual(self.app._task_index, index)
        self.assertEqual(self.app._task_state, "playing")
        self.assertEqual(self.timers[0][0], 1000)  # the full duration again

    def test_stop_cancels_resets_and_freezes(self):
        self.app.on_play()
        self.app._task_advance()

        self.app.on_stop()

        self.assertEqual(self.app._task_state, "stopped")
        self.assertEqual(self.app._task_index, 0)
        self.assertEqual(self.cancelled, ["t2"])
        self.assertIn("aturada", self.app.status_var.get().lower())

    def test_stop_while_stopped_still_freezes_the_arm(self):
        self.bus.servos[2][so101_bus.REG_PRESENT_POSITION] = 1800

        self.app.on_stop()

        self.assertEqual(self.goals()[2], 1800)
        self.assertIn("Braç aturat", self.app.status_var.get())

    def test_pause_while_paused_is_a_no_op(self):
        self.app.on_play()
        self.app.on_pause()
        self.cancelled.clear()

        self.app.on_pause()

        self.assertEqual(self.cancelled, [])
        self.assertEqual(self.app._task_state, "paused")

    def test_joint_controls_are_disabled_while_playing_and_paused(self):
        def states():
            return {
                str(self.app.scales[1].cget("state")),
                str(self.app.entries[1].cget("state")),
                str(self.app.limit_entries[1][0].cget("state")),
                str(self.app.save_button.cget("state")),
                str(self.app.button.cget("state")),
            }

        self.app.on_play()
        self.assertEqual(states(), {"disabled"})
        self.app.on_pause()
        self.assertEqual(states(), {"disabled"})
        self.app.on_stop()
        self.assertEqual(states(), {"normal"})

    def test_a_pending_slider_send_cannot_reach_the_bus_while_playing(self):
        self.app.on_play()
        self.bus.servos[4].pop("goal", None)

        self.app._send(4)

        self.assertNotIn("goal", self.bus.servos[4])

    def test_a_servo_that_stops_answering_aborts_the_task(self):
        self.app.on_play()
        self.timers.clear()
        del self.bus.servos[6]  # the gripper drops off the bus before its step

        self.app._task_advance()

        self.assertEqual(self.app._task_state, "stopped")
        self.assertEqual(self.timers, [])
        self.assertIn("s'ha aturat", self.app.status_var.get())

    def test_on_close_cancels_both_timer_chains(self):
        self.app.on_play()
        self.app._pending[1] = "slider-token"
        self.app.root.destroy = lambda: None
        self.app.disconnect = lambda: None

        self.app.on_close()

        self.assertIn("t1", self.cancelled)
        self.assertIn("slider-token", self.cancelled)


class ProgramTests(unittest.TestCase):
    """The agafar()/deixar() helpers and the sandbox of the program tab."""

    def names(self, steps):
        return [step.name for step in steps]

    def test_the_example_program_builds_a_valid_task(self):
        """The template is edited for each workshop, so only check that it runs."""
        steps, error = robot_app.run_program(robot_app.PROGRAMA_EXEMPLE)

        self.assertIsNone(error)
        self.assertEqual(robot_app.validate_task(steps), [])
        self.assertGreater(len(steps), 1)
        # It must open the gripper, close it, and open it again to release.
        gripper = [step.pose[robot_app.GRIPPER_ID] for step in steps if robot_app.GRIPPER_ID in step.pose]
        self.assertIn(float(robot_app.PINCA_OBERTA), gripper)
        self.assertIn(float(robot_app.PINCA_TANCADA), gripper)

    def test_agafar_opens_goes_closes_and_lifts(self):
        steps, error = robot_app.run_program("agafar(base=10, espatlla=30, colze=15)")

        self.assertIsNone(error)
        self.assertEqual(self.names(steps), ["Obrir pinça", "Anar a l'objecte", "Tancar pinça", "Aixecar"])
        self.assertEqual(steps[1].pose, {1: 10.0, 2: 30.0, 3: 15.0})
        self.assertEqual(steps[0].pose, {robot_app.GRIPPER_ID: float(robot_app.PINCA_OBERTA)})
        self.assertEqual(steps[2].pose, {robot_app.GRIPPER_ID: float(robot_app.PINCA_TANCADA)})

    def test_deixar_goes_opens_and_lifts(self):
        steps, _ = robot_app.run_program("deixar(base=40, espatlla=40, colze=20)")

        self.assertEqual(self.names(steps), ["Anar al destí", "Obrir pinça", "Aixecar"])
        self.assertEqual(steps[0].pose, {1: 40.0, 2: 40.0, 3: 20.0})

    def test_a_pose_only_moves_the_joints_the_participant_gave(self):
        steps, _ = robot_app.run_program("anar_a(base=10)")

        self.assertEqual(steps[0].pose, {1: 10.0})

    def test_canell_and_gir_are_accepted(self):
        steps, _ = robot_app.run_program("anar_a(base=1, canell=2, gir=3)")

        self.assertEqual(steps[0].pose, {1: 1.0, 4: 2.0, 5: 3.0})

    def test_the_helpers_can_be_used_in_a_loop(self):
        steps, error = robot_app.run_program("for i in range(3):\n    anar_a(base=i * 10)\n")

        self.assertIsNone(error)
        self.assertEqual([step.pose[1] for step in steps], [0.0, 10.0, 20.0])

    def test_esperar_adds_a_step_that_moves_nothing(self):
        steps, _ = robot_app.run_program("esperar(3)")

        self.assertEqual(steps[0].pose, {})
        self.assertEqual(steps[0].seconds, 3.0)

    def test_a_syntax_error_names_the_line(self):
        steps, error = robot_app.run_program("inici()\nagafar(base=1")

        self.assertIsNone(steps)
        self.assertIn("línia 2", error)

    def test_an_unknown_function_is_reported(self):
        steps, error = robot_app.run_program("agafarr(base=1)")

        self.assertIsNone(steps)
        self.assertIn("No existeix aquest nom", error)

    def test_a_coordinate_that_is_not_a_number_is_reported(self):
        _, error = robot_app.run_program("agafar(base='endavant')")

        self.assertIn("Base", error)
        self.assertIn("ha de ser un número", error)

    def test_a_move_without_coordinates_is_reported(self):
        _, error = robot_app.run_program("anar_a()")

        self.assertIn("com a mínim una coordenada", error)

    def test_a_program_that_moves_nothing_is_reported(self):
        steps, error = robot_app.run_program("x = 1 + 1")

        self.assertIsNone(steps)
        self.assertIn("no mou el braç", error)

    def test_an_infinite_loop_is_stopped(self):
        steps, error = robot_app.run_program("while True:\n    inici()\n")

        self.assertIsNone(steps)
        self.assertIn("bucle infinit", error)

    def test_imports_and_file_access_are_out_of_reach(self):
        for code in ("import os", "open('/etc/passwd')", "eval('1')", "__import__('os')"):
            steps, error = robot_app.run_program(code)
            self.assertIsNone(steps, code)
            self.assertIsNotNone(error, code)

    def test_the_lift_pose_can_be_overridden(self):
        steps, _ = robot_app.run_program("aixecar()", lift={1: 5, 2: 6})

        self.assertEqual(steps[0].pose, {1: 5, 2: 6})


class TorqueTests(unittest.TestCase):
    def test_disabling_writes_zero_to_every_responding_servo(self):
        handler = FakePacketHandler({1: healthy_servo(2011), 2: healthy_servo(1500)})

        positions, errors = robot_app.set_torque(handler, [1, 2, 3], enabled=False)

        self.assertEqual(errors, [])
        self.assertEqual(positions, {1: 2011, 2: 1500})
        self.assertEqual(handler.servos[1][so101_bus.REG_TORQUE_ENABLE], 0)
        self.assertEqual(handler.servos[2][so101_bus.REG_TORQUE_ENABLE], 0)

    def test_disabling_sends_no_goal_position(self):
        handler = FakePacketHandler({1: healthy_servo()})

        robot_app.set_torque(handler, [1], enabled=False)

        self.assertNotIn("goal", handler.servos[1])

    def test_enabling_holds_the_current_position_first(self):
        # A stale goal must not make the arm jump when the torque returns.
        handler = FakePacketHandler({1: healthy_servo(1700, regs={so101_bus.REG_TORQUE_ENABLE: 0})})

        positions, errors = robot_app.set_torque(handler, [1], enabled=True)

        self.assertEqual(errors, [])
        self.assertEqual(positions, {1: 1700})
        self.assertEqual(handler.servos[1]["goal"][0], 1700)
        self.assertEqual(handler.servos[1][so101_bus.REG_TORQUE_ENABLE], 1)

    def test_an_empty_bus_is_reported(self):
        for enabled in (True, False):
            positions, errors = robot_app.set_torque(FakePacketHandler({}), [1], enabled=enabled)
            self.assertEqual(positions, {})
            self.assertIn("Cap motor ha respost", errors[0])


class MotionParameterTests(unittest.TestCase):
    def test_parse_motion_accepts_values_inside_the_range(self):
        self.assertEqual(robot_app.parse_motion("300", "15"), ((300, 15), None))
        self.assertEqual(robot_app.parse_motion(" 250 ", "20"), ((250, 20), None))

    def test_parse_motion_rejects_text_and_values_outside_the_range(self):
        for speed, acc in (("abc", "15"), ("", "15"), ("300", "x"),
                           ("50", "15"), ("3000", "15"), ("300", "0"), ("300", "200")):
            values, error = robot_app.parse_motion(speed, acc)
            self.assertIsNone(values, (speed, acc))
            self.assertIn("no vàlids", error)

    def test_a_speed_below_the_minimum_is_refused(self):
        # Below SPEED_MIN the servo creeps and judders instead of turning.
        self.assertIsNone(robot_app.parse_motion(str(robot_app.SPEED_MIN - 1), "15")[0])
        self.assertIsNotNone(robot_app.parse_motion(str(robot_app.SPEED_MIN), "15")[0])

    def test_move_joint_uses_the_speed_and_acceleration_it_is_given(self):
        handler = FakePacketHandler({1: healthy_servo()})

        robot_app.move_joint(handler, 1, 10, speed=150, acc=8)

        self.assertEqual(handler.servos[1]["goal"][1:], (150, 8))

    def test_apply_pose_passes_the_values_through(self):
        handler = FakePacketHandler({1: healthy_servo(), 2: healthy_servo()})

        robot_app.apply_pose(handler, {1: 5.0, 2: 5.0}, speed=200, acc=12)

        for servo_id in (1, 2):
            self.assertEqual(handler.servos[servo_id]["goal"][1:], (200, 12))

    def test_hold_here_and_prepare_joints_pass_the_values_through(self):
        handler = FakePacketHandler({1: healthy_servo()})

        robot_app.hold_here(handler, [1], speed=120, acc=5)
        self.assertEqual(handler.servos[1]["goal"][1:], (120, 5))

        robot_app.prepare_joints(handler, [1], speed=400, acc=25)
        self.assertEqual(handler.servos[1]["goal"][1:], (400, 25))

    def test_the_defaults_are_inside_the_range_the_app_accepts(self):
        self.assertEqual((robot_app.SAFE_SPEED, robot_app.SAFE_ACC), (300, 15))
        self.assertGreaterEqual(robot_app.SAFE_SPEED, robot_app.SPEED_MIN)


@unittest.skipUnless(HAS_DISPLAY, "no usable display for Tk")
class TaskStepModeTests(unittest.TestCase):
    """Pas a pas: run one step and stop there."""

    def setUp(self):
        self.root = robot_app.tk.Tk()
        self.root.withdraw()
        self.app = robot_app.RobotApp(self.root, "/dev/fake", 1000000, list(range(1, 7)))
        self.bus = FakePacketHandler({servo_id: healthy_servo() for servo_id in range(1, 7)})
        self.app.packet_handler = self.bus
        self.app.port_handler = object()
        self.app.responding_ids = list(range(1, 7))
        self.timers = []
        self.app.root.after = lambda ms, fn=None, *a: (self.timers.append((ms, fn)), "t")[1]
        self.app.root.after_cancel = lambda token: None
        self.app.task, self.app.task_errors = robot_app.build_task([
            ("Un", {1: 10}, 2.0),
            ("Dos", {2: 20}, 2.0),
            ("Tres", {3: 30}, 2.0),
        ])

    def tearDown(self):
        self.root.destroy()

    def goals(self):
        return {sid: regs["goal"][0] for sid, regs in self.bus.servos.items() if "goal" in regs}

    def clear_goals(self):
        for regs in self.bus.servos.values():
            regs.pop("goal", None)

    def test_one_step_runs_and_then_pauses(self):
        self.app.on_step()
        self.assertEqual(self.app._task_state, "playing")
        self.assertEqual(self.goals(), {1: robot_app.degrees_to_position(10)})

        self.app._task_advance()

        self.assertEqual(self.app._task_state, "paused")
        self.assertEqual(self.app._task_index, 1)
        self.assertIn("Pas 1/3 fet", self.app.status_var.get())
        self.assertIn("Dos", self.app.status_var.get())

    def test_stepping_walks_the_whole_task(self):
        for expected in range(1, 4):
            self.clear_goals()
            self.app.on_step()
            self.app._task_advance()
            if expected < 3:
                self.assertEqual(self.app._task_index, expected)

        self.assertEqual(self.app._task_state, "stopped")
        self.assertEqual(self.app._task_index, 0)
        self.assertIn("l'últim", self.app.status_var.get())

    def test_play_after_stepping_runs_to_the_end(self):
        self.app.on_step()
        self.app._task_advance()

        self.app.on_play()

        self.assertFalse(self.app._stepping)
        self.assertEqual(self.app._task_index, 1)  # continues where it paused
        self.app._task_advance()
        self.assertEqual(self.app._task_state, "playing")  # did not stop after one step

    def test_stepping_after_the_end_starts_again(self):
        self.app.on_step()
        self.app._task_advance()
        self.app.on_stop()

        self.app.on_step()

        self.assertEqual(self.app._task_index, 0)

    def test_stop_leaves_step_mode(self):
        self.app.on_step()

        self.app.on_stop()

        self.assertFalse(self.app._stepping)
        self.assertEqual(self.app._task_state, "stopped")

    def test_step_is_refused_while_the_task_plays(self):
        self.app.on_play()
        self.timers.clear()

        self.app.on_step()

        self.assertFalse(self.app._stepping)
        self.assertEqual(self.timers, [])

    def test_an_advance_after_a_stop_does_not_move_the_index(self):
        self.app.on_play()
        self.app.on_stop()

        self.app._task_advance()

        self.assertEqual(self.app._task_index, 0)

    def test_the_step_button_is_usable_while_paused_and_not_while_playing(self):
        self.app.on_play()
        self.assertEqual(str(self.app.step_button.cget("state")), "disabled")
        self.app.on_pause()
        self.assertEqual(str(self.app.step_button.cget("state")), "normal")
        self.assertEqual(str(self.app.program_step_button.cget("state")), "normal")


class LoadPositionsTests(unittest.TestCase):
    def _write(self, tmp, record):
        path = os.path.join(tmp, "p.json")
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(record, handle)
        return path

    def test_a_file_written_by_save_positions_round_trips_exactly(self):
        handler = FakePacketHandler({1: healthy_servo(2114), 3: healthy_servo(1742), 6: healthy_servo(2560)})

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "p.json")
            record, _ = robot_app.save_positions(handler, [1, 3, 6], path, "/dev/fake")
            pose, error = robot_app.load_positions(path)

        self.assertIsNone(error)
        # Reloading the degrees must command the raw positions that were saved.
        for key, entry in record["servos"].items():
            servo_id = int(key)
            self.assertEqual(
                robot_app.degrees_to_position(pose[servo_id], robot_app.is_inverted(servo_id)),
                entry["position"],
                entry["joint"],
            )

    def test_the_saved_zero_pose_loads_as_the_degrees_it_holds(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(tmp, {"servos": {
                "1": {"joint": "Base", "degrees": 0.1, "position": 2049},
                "2": {"joint": "Espatlla", "degrees": 0.9, "position": 2058},
            }})
            pose, error = robot_app.load_positions(path)

        self.assertIsNone(error)
        self.assertEqual(pose, {1: 0.1, 2: 0.9})

    def test_a_missing_file_is_reported(self):
        pose, error = robot_app.load_positions("/no/such/file.json")

        self.assertIsNone(pose)
        self.assertIn("No s'ha pogut llegir", error)

    def test_a_file_that_is_not_json_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "p.json")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write("no json at all")
            pose, error = robot_app.load_positions(path)

        self.assertIsNone(pose)
        self.assertIn("No s'ha pogut llegir", error)

    def test_a_file_without_servos_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            for record in ({"servos": {}}, {"port": "/dev/fake"}, []):
                pose, error = robot_app.load_positions(self._write(tmp, record))
                self.assertIsNone(pose, record)
                self.assertIsNotNone(error)

    def test_an_entry_without_usable_degrees_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(tmp, {"servos": {"1": {"joint": "Base", "position": 2049}}})
            pose, error = robot_app.load_positions(path)

        self.assertIsNone(pose)
        self.assertIn("graus vàlids", error)
