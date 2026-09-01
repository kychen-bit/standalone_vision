"""Hardware-independent recognition result types (Python 3.6 compatible)."""

from collections import namedtuple


Measurement = namedtuple(
    "Measurement",
    "kind target_id x y yaw confidence unit residual pixel_x pixel_y",
)
Measurement.__new__.__defaults__ = (0.0, None, None)
