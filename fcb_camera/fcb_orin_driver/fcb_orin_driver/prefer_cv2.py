"""Make `import cv2` load an OpenCV that can reach the Jetson's encoder.

Imported for its side effect, before cv2:   import prefer_cv2  # noqa

A pip-installed opencv-python shadows the system one (it lives in
/usr/local, which comes first on sys.path), and pip's Linux wheels are
built without GStreamer -- so the recorder lost the NVENC hardware encoder
and fell back to CPU encoding, which drops frames at 1080p60. Found on the
Orin NX `uasdtu` (Oct 2026): pip's OpenCV 5.0.0 hid Ubuntu's python3-opencv
4.6, which has GStreamer.

So when the cv2 Python would import comes from a pip wheel and a system
cv2 is installed, the system directory is put first. Decided before cv2
is imported -- two OpenCVs cannot share one process. FCB_PREFER_PIP_CV2=1
leaves the order alone.
"""
import glob
import importlib.util
import os
import sys


def _system_cv2_dirs():
    pattern = "/usr/lib/python3*/dist-packages"
    return [d for d in ["/usr/lib/python3/dist-packages"] + sorted(glob.glob(pattern))
            if glob.glob(os.path.join(d, "cv2*.so")) or os.path.isdir(os.path.join(d, "cv2"))]


def _from_pip(spec):
    if spec is None or not spec.origin:
        return False
    package_dir = os.path.dirname(os.path.dirname(spec.origin)) \
        if os.path.basename(os.path.dirname(spec.origin)) == "cv2" else os.path.dirname(spec.origin)
    return bool(glob.glob(os.path.join(package_dir, "opencv_*python*.dist-info")))


if "cv2" not in sys.modules and os.environ.get("FCB_PREFER_PIP_CV2") != "1":
    try:
        if _from_pip(importlib.util.find_spec("cv2")):
            for directory in reversed(_system_cv2_dirs()):
                if directory in sys.path:
                    sys.path.remove(directory)
                sys.path.insert(0, directory)
    except Exception:
        pass   # never stop a script from starting over this
