import unittest

import src.so101_bus as so101_bus
from src.servo_set_id import check_bus, scan, set_servo_id
from tests.test_servo_tools import FakePacketHandler, healthy_servo


class IdWritingFakeBus(FakePacketHandler):
    """A fake bus where writing the ID register actually moves the servo's id."""

    def write1ByteTxRx(self, servo_id, address, value):
        if address == so101_bus.REG_ID and servo_id in self.servos:
            self.servos[value] = self.servos.pop(servo_id)
            return 0, 0  # COMM_SUCCESS
        return super().write1ByteTxRx(servo_id, address, value)


class ScanTests(unittest.TestCase):
    def test_scan_lists_the_ids_that_answer(self):
        handler = FakePacketHandler({3: healthy_servo(), 1: healthy_servo()})

        self.assertEqual(scan(handler, range(1, 8)), [1, 3])

    def test_scan_of_an_empty_bus(self):
        self.assertEqual(scan(FakePacketHandler({}), range(1, 8)), [])


class CheckBusTests(unittest.TestCase):
    def test_a_single_servo_with_a_free_target_id_is_accepted(self):
        self.assertEqual(check_bus([1], old_id=1, new_id=3), [])

    def test_an_empty_bus_is_refused(self):
        problems = check_bus([], old_id=1, new_id=3)

        self.assertEqual(len(problems), 1)
        self.assertIn("No servo answered", problems[0])

    def test_a_missing_old_id_is_refused(self):
        problems = check_bus([2], old_id=1, new_id=3)

        self.assertIn("No servo answers to ID 1", problems[0])

    def test_a_taken_target_id_is_refused(self):
        problems = check_bus([1, 3], old_id=1, new_id=3)

        self.assertTrue(any("already taken" in problem for problem in problems))

    def test_more_than_one_servo_on_the_bus_is_refused(self):
        problems = check_bus([1, 2], old_id=1, new_id=3)

        self.assertTrue(any("ONE servo at a time" in problem for problem in problems))

    def test_the_same_old_and_new_id_is_refused(self):
        problems = check_bus([1], old_id=1, new_id=1)

        self.assertIn("are the same", problems[0])


class SetServoIdTests(unittest.TestCase):
    def test_the_servo_answers_to_the_new_id_and_not_the_old_one(self):
        handler = IdWritingFakeBus({1: healthy_servo()})

        ok, message = set_servo_id(handler, 1, 3, settle=0)

        self.assertTrue(ok, message)
        self.assertIn("now ID 3", message)
        self.assertIn("Elbow", message)  # the joint name of ID 3
        self.assertEqual(scan(handler, range(1, 8)), [3])

    def test_the_eeprom_is_unlocked_before_and_locked_after_on_the_new_id(self):
        handler = IdWritingFakeBus({1: healthy_servo()})

        set_servo_id(handler, 1, 3, settle=0)

        # The lock must reach the new id: the servo answers to it straight away.
        self.assertEqual(handler.servos[3]["lock"], 1)

    def test_a_write_that_fails_reports_it_and_relocks(self):
        handler = FakePacketHandler({})  # nothing answers, so the write fails

        ok, message = set_servo_id(handler, 1, 3, settle=0)

        self.assertFalse(ok)
        self.assertIn("Writing the ID failed", message)

    def test_a_servo_that_keeps_the_old_id_is_reported(self):
        # The write "succeeds" but the servo never changes id.
        handler = FakePacketHandler({1: healthy_servo(), 3: healthy_servo()})

        ok, message = set_servo_id(handler, 1, 3, settle=0)

        self.assertFalse(ok)
        self.assertIn("still answers", message)


if __name__ == "__main__":
    unittest.main()
