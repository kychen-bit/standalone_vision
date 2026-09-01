import unittest

from jetson_recognition.protocol import crc16_ccitt, decode_frame, encode_frame


class ProtocolTests(unittest.TestCase):
    def test_result_frame_contains_crc_of_body(self):
        frame = encode_frame("VISION_RESULT", "STABLE", "COLOR", "1", "10", "20")
        body, encoded_crc = frame[1:-1].rsplit(b"*", 1)
        self.assertEqual(int(encoded_crc, 16), crc16_ccitt(body))

    def test_invalid_field_is_rejected(self):
        with self.assertRaises(ValueError):
            encode_frame("RESULT", "bad,field")

    def test_decode_round_trip(self):
        frame = encode_frame(
            "MAIX_QR", "123+123+456+231", "320.0", "240.0", "120.0", "4", "STABLE"
        )
        self.assertEqual(
            decode_frame(frame),
            ("MAIX_QR", ["123+123+456+231", "320.0", "240.0", "120.0", "4", "STABLE"]),
        )

    def test_decode_accepts_str_input(self):
        frame = encode_frame("PING", "Q001")
        self.assertEqual(decode_frame(frame.decode("ascii")), ("PING", ["Q001"]))

    def test_decode_rejects_bad_crc(self):
        self.assertIsNone(decode_frame(b"@MAIX_QR,payload*0000\n"))

    def test_decode_rejects_non_frame(self):
        self.assertIsNone(decode_frame(b"hello world\n"))
        self.assertIsNone(decode_frame(b"@NO_CRC\n"))
        self.assertIsNone(decode_frame(b""))


if __name__ == "__main__":
    unittest.main()
