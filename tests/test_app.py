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
