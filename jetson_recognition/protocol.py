"""Checksummed ASCII output used only when serial output is enabled."""


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _field(value):
    text = str(value).strip()
    text.encode("ascii")
    if not text or any(character in text for character in ",*@\r\n"):
        raise ValueError("invalid serial field")
    return text


def encode_frame(name, *fields):
    body = ",".join([_field(name).upper()] + [_field(value) for value in fields]).encode("ascii")
    if len(body) > 176:
        raise ValueError("serial frame too long")
    return b"@" + body + ("*%04X\n" % crc16_ccitt(body)).encode("ascii")


def decode_frame(line):
    """Parse one '@NAME,F1,F2,...*CRC\n' frame.

    Returns ``(name, fields)`` with ``name`` uppercased, or ``None`` when the
    line is not a frame or its CRC does not match. Accepts ``bytes`` or
    ``str``; used by the Maix -> Jetson bridge.
    """
    if not isinstance(line, bytes):
        line = str(line).encode("ascii", "replace")
    line = line.strip()
    if not line.startswith(b"@"):
        return None
    body, separator, crc_text = line[1:].rpartition(b"*")
    if not separator or not crc_text:
        return None
    try:
        expected = int(crc_text, 16)
    except ValueError:
        return None
    if crc16_ccitt(body) != expected:
        return None
    parts = body.decode("ascii", "replace").split(",")
    if not parts or not parts[0]:
        return None
    return parts[0].upper(), parts[1:]
