import unittest

import src.so101_bus as so101_bus
from src.servo_offsets import (
    WRAP_MARGIN,
    clear_offset,
    distance_to_wrap,
    raw_position,
    read_offsets,
)
from tests.test_servo_tools import FakePacketHandler, healthy_servo


class OffsetMathTests(unittest.TestCase):
    def test_raw_position_undoes_the_offset(self):
        self.assertEqual(raw_position(2196, 2011), 111)  # the gripper on the real arm
        self.assertEqual(raw_position(2048, 0), 2048)

    def test_raw_position_wraps_around_the_seam(self):
        self.assertEqual(raw_position(4000, 1000), 904)
        self.assertEqual(raw_position(100, -200), 3996)

    def test_distance_to_wrap_measures_both_sides_of_the_seam(self):
        self.assertEqual(distance_to_wrap(111), 111)
        self.assertEqual(distance_to_wrap(4000), 96)
        self.assertEqual(distance_to_wrap(2048), 2048)


class ReadOffsetsTests(unittest.TestCase):
    def test_a_joint_at_the_seam_is_flagged(self):
        handler = FakePacketHandler({
            6: healthy_servo(2196, regs={so101_bus.REG_OFFSET: 2011}),
            2: healthy_servo(2048, regs={so101_bus.REG_OFFSET: 0}),
        })

        report = read_offsets(handler, [2, 6])

        self.assertEqual(report[6]["offset"], 2011)
        self.assertEqual(report[6]["raw"], 111)
        self.assertLessEqual(report[6]["to_wrap"], WRAP_MARGIN)
        self.assertGreater(report[2]["to_wrap"], WRAP_MARGIN)

    def test_a_negative_offset_is_decoded(self):
        handler = FakePacketHandler({5: healthy_servo(2071, regs={so101_bus.REG_OFFSET: 0x800 | 414})})

        self.assertEqual(read_offsets(handler, [5])[5]["offset"], -414)

    def test_a_servo_that_does_not_answer_is_left_out(self):
        self.assertEqual(read_offsets(FakePacketHandler({}), [1]), {})


class ClearOffsetTests(unittest.TestCase):
    def test_the_offset_is_cleared_and_the_joint_is_held(self):
        handler = FakePacketHandler({6: healthy_servo(2196, regs={so101_bus.REG_OFFSET: 2011})})

        ok, message = clear_offset(handler, 6, settle=0)

        self.assertTrue(ok, message)
        self.assertEqual(handler.servos[6][so101_bus.REG_OFFSET], 0)
        # Held where it now reads, so it does not move when the frame shifts.
        self.assertEqual(handler.servos[6]["goal"][0], handler.servos[6][so101_bus.REG_PRESENT_POSITION])

    def test_the_torque_is_never_turned_off(self):
        handler = FakePacketHandler({6: healthy_servo(regs={so101_bus.REG_OFFSET: 500})})

        clear_offset(handler, 6, settle=0)

        self.assertEqual(handler.servos[6][so101_bus.REG_TORQUE_ENABLE], 1)

    def test_the_eeprom_is_locked_again(self):
        handler = FakePacketHandler({6: healthy_servo(regs={so101_bus.REG_OFFSET: 500})})

        clear_offset(handler, 6, settle=0)

        self.assertEqual(handler.servos[6]["lock"], 1)

    def test_a_servo_that_does_not_answer_is_reported(self):
        ok, message = clear_offset(FakePacketHandler({}), 6, settle=0)

        self.assertFalse(ok)
        self.assertIn("failed", message)


if __name__ == "__main__":
    unittest.main()
