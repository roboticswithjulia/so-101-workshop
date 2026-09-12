import unittest

import app.app as zero_app
from tests.test_servo_tools import FakePacketHandler, healthy_servo


class FakePortHandler:
    def __init__(self):
        self.closed = False

    def closePort(self):
        self.closed = True


class ZeroAppTests(unittest.TestCase):
    def setUp(self):
        self.original_open_bus = zero_app.open_bus

    def tearDown(self):
        zero_app.open_bus = self.original_open_bus

    def _fake_bus(self, servos):
        port_handler = FakePortHandler()
        zero_app.open_bus = lambda port, baudrate: (port_handler, FakePacketHandler(servos))
        return port_handler

    def test_run_set_zero_stores_pose_and_closes_port(self):
        port_handler = self._fake_bus({1: healthy_servo(2011), 2: healthy_servo(1500)})
        logs = []

        results, error = zero_app.run_set_zero("/dev/fake", 1000000, [1, 2, 3], log=logs.append)

        self.assertIsNone(error)
        self.assertEqual(results[1], {"before": 2011, "after": 2048, "ok": True})
        self.assertEqual(results[2]["after"], 2048)
        self.assertNotIn(3, results)
        self.assertTrue(port_handler.closed)
        self.assertTrue(any("Base (ID 1): posició actual 2011" in line for line in logs))
        self.assertTrue(any("Espatlla (ID 2): 1500 -> 2048 (correcte)" in line for line in logs))
        self.assertTrue(any("Colze (ID 3): sense resposta" in line for line in logs))
        ok, message = zero_app.summarize(results, error)
        self.assertTrue(ok)
        self.assertIn("[1, 2]", message)
        self.assertIn("Posició zero desada", message)

    def test_run_set_zero_reports_when_no_servo_answers(self):
        port_handler = self._fake_bus({})

        results, error = zero_app.run_set_zero("/dev/fake", 1000000, [1, 2], log=lambda *_: None)

        self.assertEqual(results, {})
        self.assertIn("Cap motor ha respost", error)
        self.assertTrue(port_handler.closed)
        self.assertEqual(zero_app.summarize(results, error), (False, error))

    def test_run_set_zero_reports_port_errors(self):
        def failing_open_bus(port, baudrate):
            raise RuntimeError("Could not open /dev/fake: Permission denied")

        zero_app.open_bus = failing_open_bus

        results, error = zero_app.run_set_zero("/dev/fake", 1000000, [1], log=lambda *_: None)

        self.assertEqual(results, {})
        self.assertIn("grup dialout", error)
        self.assertIn("torna a iniciar la sessió", error)

    def test_summarize_flags_servos_that_did_not_take_the_zero(self):
        results = {1: {"before": 2011, "after": 2048, "ok": True}, 2: {"before": 1500, "after": 1500, "ok": False}}

        ok, message = zero_app.summarize(results, None)

        self.assertFalse(ok)
        self.assertIn("[2]", message)
        self.assertIn("No s'ha pogut aplicar", message)


if __name__ == "__main__":
    unittest.main()
