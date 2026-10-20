"""Which camera board is attached, without opening the video or VISCA port.

`python3 -m fcb_base_driver.devices` from fcb_orin, as a ROS executable.
Safe to run while the driver holds both devices.
"""
import sys

from fcb_orin_driver import prefer_cv2  # noqa: F401  -- before cv2
from fcb_base_driver import devices


def main():
    status, lines = devices.describe_camera()
    print("\n".join(lines))
    return status


if __name__ == "__main__":
    sys.exit(main())
