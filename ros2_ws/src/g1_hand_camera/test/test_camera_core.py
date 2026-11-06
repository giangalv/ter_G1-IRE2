"""Unit tests for g1_hand_camera.camera_core (no ROS, no camera needed)."""

import numpy as np
import pytest

from g1_hand_camera import camera_core as cc


# --------------------------------------------------------------------------
# bgr_to_rgb8 / encode_jpeg
# --------------------------------------------------------------------------


def test_bgr_to_rgb8_swaps_channels_and_is_contiguous():
    bgr = np.zeros((2, 3, 3), dtype=np.uint8)
    bgr[..., 0] = 10  # B
    bgr[..., 1] = 20  # G
    bgr[..., 2] = 30  # R
    rgb = cc.bgr_to_rgb8(bgr)
    assert rgb.shape == (2, 3, 3)
    assert rgb.flags["C_CONTIGUOUS"]
    assert (rgb[..., 0] == 30).all() and (rgb[..., 1] == 20).all() and (rgb[..., 2] == 10).all()


@pytest.mark.parametrize("shape", [(4, 5), (4, 5, 1)])
def test_bgr_to_rgb8_replicates_gray(shape):
    gray = np.full(shape, 77, dtype=np.uint8)
    rgb = cc.bgr_to_rgb8(gray)
    assert rgb.shape == (4, 5, 3)
    assert (rgb == 77).all()


def test_bgr_to_rgb8_rejects_non_8bit_and_bad_shapes():
    with pytest.raises(cc.CameraError):
        cc.bgr_to_rgb8(np.zeros((2, 2, 3), dtype=np.uint16))
    with pytest.raises(cc.CameraError):
        cc.bgr_to_rgb8(np.zeros((2, 2, 4), dtype=np.uint8))


@pytest.mark.skipif(not cc.HAS_CV2, reason="OpenCV not installed")
def test_encode_jpeg_roundtrip_keeps_colours():
    rgb = np.zeros((32, 48, 3), dtype=np.uint8)
    rgb[..., 0] = 220  # red image
    data = cc.encode_jpeg(rgb, quality=90)
    assert data[:2] == b"\xff\xd8"  # JPEG SOI
    decoded = cc.cv2.imdecode(np.frombuffer(data, np.uint8), cc.cv2.IMREAD_COLOR)
    assert decoded.shape == (32, 48, 3)
    b, g, r = decoded.reshape(-1, 3).mean(axis=0)
    assert r > 200 and g < 30 and b < 30  # BGR order from imdecode


@pytest.mark.skipif(not cc.HAS_CV2, reason="OpenCV not installed")
def test_encode_jpeg_clamps_quality():
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    assert cc.encode_jpeg(rgb, quality=0)
    assert cc.encode_jpeg(rgb, quality=1000)


def test_to_u8_array_is_a_byte_array_with_the_same_content():
    raw = bytes(range(256)) * 4
    arr = cc.to_u8_array(raw)
    assert arr.typecode == "B" and len(arr) == len(raw)
    assert arr.tobytes() == raw


# --------------------------------------------------------------------------
# MockFrameSource
# --------------------------------------------------------------------------


def test_mock_source_requires_open_and_valid_side():
    with pytest.raises(cc.CameraError):
        cc.MockFrameSource("middle")
    src = cc.MockFrameSource("left", 32, 24, realtime=False)
    with pytest.raises(cc.CameraError):
        src.read()
    src.open()
    assert src.is_open
    src.close()
    assert not src.is_open


def test_mock_frames_have_requested_shape_and_move():
    src = cc.MockFrameSource("right", 64, 48, realtime=False)
    src.open()
    a, b = src.read(), src.read()
    assert a.shape == (48, 64, 3) and a.dtype == np.uint8
    assert src.actual_size == (64, 48)
    assert not np.array_equal(a, b)  # the bar sweeps


def test_mock_frames_are_deterministic_and_differ_per_side():
    def first(side):
        s = cc.MockFrameSource(side, 32, 24, realtime=False)
        s.open()
        return s.read()

    assert np.array_equal(first("left"), first("left"))
    assert not np.array_equal(first("left"), first("right"))


def test_mock_frame_does_not_alias_internal_buffer():
    src = cc.MockFrameSource("left", 16, 16, realtime=False)
    src.open()
    a = src.read()
    a[:] = 0
    assert src.read().max() > 0


# --------------------------------------------------------------------------
# OpenCvFrameSource (without a device)
# --------------------------------------------------------------------------


def test_opencv_source_read_before_open_raises():
    src = cc.OpenCvFrameSource("/dev/does-not-exist")
    assert not src.is_open
    assert src.actual_size == (cc.DEFAULT_WIDTH, cc.DEFAULT_HEIGHT)
    with pytest.raises(cc.CameraError):
        src.read()


def test_opencv_source_open_missing_device_raises_and_stays_closed():
    src = cc.OpenCvFrameSource("/dev/does-not-exist")
    with pytest.raises(cc.CameraError):
        src.open()
    assert not src.is_open
    src.close()  # idempotent


class _FakeCap:
    def __init__(self, frames):
        self._frames = list(frames)
        self.released = False
        self.props = {}

    def isOpened(self):  # noqa: N802 - OpenCV API
        return True

    def set(self, prop, value):
        self.props[prop] = value

    def get(self, prop):
        return self.props.get(prop, 0)

    def read(self):
        if not self._frames:
            return False, None
        return True, self._frames.pop(0)

    def release(self):
        self.released = True


def test_opencv_source_read_failure_closes_handle():
    src = cc.OpenCvFrameSource("/dev/x")
    cap = _FakeCap([np.zeros((2, 2, 3), dtype=np.uint8)])
    src._cap = cap  # inject the fake capture
    assert src.read().shape == (2, 2, 3)
    with pytest.raises(cc.CameraError):
        src.read()
    assert cap.released and not src.is_open


# --------------------------------------------------------------------------
# ReconnectBackoff / RateLimiter
# --------------------------------------------------------------------------


def test_backoff_doubles_caps_and_resets():
    b = cc.ReconnectBackoff(2.0, 10.0)
    assert [b.next_delay() for _ in range(5)] == [2.0, 4.0, 8.0, 10.0, 10.0]
    b.reset()
    assert b.next_delay() == 2.0


def test_backoff_max_never_below_initial():
    b = cc.ReconnectBackoff(5.0, 1.0)
    assert b.next_delay() == 5.0
    assert b.next_delay() == 5.0


def test_rate_limiter_zero_passes_everything():
    r = cc.RateLimiter(0.0)
    assert all(r.should_publish(t * 0.001) for t in range(50))


def test_rate_limiter_decimates_a_fast_stream():
    r = cc.RateLimiter(10.0)
    # 30 Hz input for 3 s -> ~10 Hz out
    times = [i / 30.0 for i in range(90)]
    out = sum(r.should_publish(t) for t in times)
    assert 28 <= out <= 31


def test_rate_limiter_does_not_burst_after_a_stall():
    r = cc.RateLimiter(10.0)
    assert r.should_publish(0.0)
    assert r.should_publish(5.0)  # long gap -> one frame...
    assert not r.should_publish(5.01)  # ...not a catch-up burst
    assert r.should_publish(5.11)


# --------------------------------------------------------------------------
# Calibration
# --------------------------------------------------------------------------

_GOOD_YAML = """\
image_width: 640
image_height: 480
camera_name: left_palm
distortion_model: plumb_bob
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 501.0, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [500.0, 0.0, 320.0, 0.0, 0.0, 501.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def test_uncalibrated_placeholder_follows_rep104():
    c = cc.uncalibrated(640, 480)
    assert not c.calibrated
    assert c.k == [0.0] * 9 and c.p == [0.0] * 12
    assert c.r == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert c.d == []


@pytest.mark.skipif(not cc.HAS_YAML, reason="PyYAML not installed")
def test_load_calibration_reads_ros_yaml(tmp_path):
    f = tmp_path / "cal.yaml"
    f.write_text(_GOOD_YAML)
    c = cc.load_calibration(str(f))
    assert (c.width, c.height) == (640, 480)
    assert c.calibrated
    assert c.k[0] == 500.0 and c.k[4] == 501.0
    assert c.d == [0.1, -0.2, 0.0, 0.0, 0.0]
    assert len(c.p) == 12 and c.distortion_model == "plumb_bob"


@pytest.mark.skipif(not cc.HAS_YAML, reason="PyYAML not installed")
@pytest.mark.parametrize(
    "text",
    [
        "just a string",
        "image_width: 640\nimage_height: 480\n",  # no camera_matrix
        _GOOD_YAML.replace("[500.0, 0.0, 320.0, 0.0, 501.0, 240.0, 0.0, 0.0, 1.0]", "[1, 2, 3]"),
        "image_width: [\n",  # YAML syntax error
    ],
)
def test_load_calibration_rejects_malformed_files(tmp_path, text):
    f = tmp_path / "bad.yaml"
    f.write_text(text)
    with pytest.raises(cc.CameraError):
        cc.load_calibration(str(f))


@pytest.mark.skipif(not cc.HAS_YAML, reason="PyYAML not installed")
def test_load_calibration_missing_file():
    with pytest.raises(cc.CameraError):
        cc.load_calibration("/nonexistent/cal.yaml")


@pytest.mark.skipif(not cc.HAS_YAML, reason="PyYAML not installed")
def test_calibration_for_stream_drops_mismatched_resolution(tmp_path):
    f = tmp_path / "cal.yaml"
    f.write_text(_GOOD_YAML)
    c = cc.load_calibration(str(f))
    assert cc.calibration_for_stream(c, 640, 480) is c
    other = cc.calibration_for_stream(c, 320, 240)
    assert not other.calibrated and (other.width, other.height) == (320, 240)
    assert not cc.calibration_for_stream(None, 640, 480).calibrated


# --------------------------------------------------------------------------
# Defaults
# --------------------------------------------------------------------------


def test_default_devices_are_distinct_stable_by_path_names():
    assert set(cc.DEFAULT_DEVICES) == set(cc.SIDES)
    assert cc.DEFAULT_DEVICES["left"] != cc.DEFAULT_DEVICES["right"]
    assert all(p.startswith("/dev/v4l/by-path/") for p in cc.DEFAULT_DEVICES.values())
    assert "2.2.1" in cc.DEFAULT_DEVICES["right"] and "2.2.2" in cc.DEFAULT_DEVICES["left"]
