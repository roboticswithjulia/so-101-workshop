import unittest

from real_robot import RealRobotAdapter
from servo_scan import build_packet, parse_ids, parse_status, ping_packet


class ServoScanProtocolTests(unittest.TestCase):
    def test_ping_packet_matches_feetech_documentation(self):
        # Documented ping to ID 1: FF FF 01 02 01 FB
        self.assertEqual(ping_packet(1), bytes.fromhex("ff ff 01 02 01 fb"))
        self.assertEqual(ping_packet(6), bytes.fromhex("ff ff 06 02 01 f6"))

    def test_parse_status_accepts_valid_reply(self):
        self.assertTrue(parse_status(bytes.fromhex("ff ff 01 02 00 fc"), 1))

    def test_parse_status_strips_half_duplex_echo(self):
        echo_and_reply = bytes.fromhex("ff ff 01 02 01 fb") + bytes.fromhex("ff ff 01 02 00 fc")
        self.assertTrue(parse_status(echo_and_reply, 1))

    def test_parse_status_rejects_echo_only_and_bad_checksum(self):
        self.assertFalse(parse_status(bytes.fromhex("ff ff 01 02 01 fb"), 1))
        self.assertFalse(parse_status(bytes.fromhex("ff ff 01 02 00 00"), 1))
        self.assertFalse(parse_status(bytes.fromhex("ff ff 02 02 00 fb"), 1))
        self.assertFalse(parse_status(b"", 1))

    def test_write_packet_checksum(self):
        # Write goal position 2048 (0x0800) to ID 1, register 0x2A:
        # body = 01 05 03 2A 00 08 -> sum 0x3B -> checksum 0xC4
        self.assertEqual(
            build_packet(1, 0x03, [0x2A, 0x00, 0x08]),
            bytes.fromhex("ff ff 01 05 03 2a 00 08 c4"),
        )

    def test_parse_ids(self):
        self.assertEqual(parse_ids("1-6"), [1, 2, 3, 4, 5, 6])
        self.assertEqual(parse_ids("3,1, 2"), [1, 2, 3])
        self.assertEqual(parse_ids("250-260"), [250, 251, 252, 253])


class RealRobotAdapterPacketTests(unittest.TestCase):
    def setUp(self):
        self.adapter = RealRobotAdapter()

    def test_ping_packet_matches_scanner(self):
        self.assertEqual(self.adapter._build_packet(1, 0x01, []), ping_packet(1))

    def test_goal_position_packet_uses_write_instruction_and_register(self):
        self.assertEqual(
            self.adapter._goal_position_packet(1, 2048),
            bytes.fromhex("ff ff 01 05 03 2a 00 08 c4"),
        )

    def test_goal_position_is_clamped_to_12_bits(self):
        packet = self.adapter._goal_position_packet(2, 99999)
        self.assertEqual(packet[6:8], bytes([0xFF, 0x0F]))


if __name__ == "__main__":
    unittest.main()
