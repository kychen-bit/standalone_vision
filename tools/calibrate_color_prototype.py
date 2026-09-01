"""Compatibility notice for the removed Lab prototype calibration workflow."""


def main():
    print(
        "Lab prototype calibration is disabled in HSV_CONTOUR_V1.\n"
        "Use live --show-hsv, then edit config/jetson.json colors.*.hsv_ranges.\n"
        "Current values are temporary screen-test parameters; recalibrate with real material."
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
