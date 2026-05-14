#!/usr/bin/env python3
"""
ob_depth.py — CTypes wrapper for OrbbecSDK (libOrbbecSDK.so)
Minimal DepthCamera class for depth stream capture from Orbbec Astra Pro.
"""

import ctypes
import os
import time
from ctypes import (
    c_void_p, c_char_p, c_int32, c_uint32, c_uint64,
    c_float, c_bool, c_uint8, c_uint16,
    POINTER, byref, CFUNCTYPE
)

# --- path setup ---
_PROJ_DIR = os.path.dirname(os.path.abspath(__file__))
_SDK_LIB_DIR = os.path.abspath(os.path.join(_PROJ_DIR, "OrbbecSDK", "lib", "arm64"))

if _SDK_LIB_DIR not in os.environ.get("LD_LIBRARY_PATH", ""):
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    os.environ["LD_LIBRARY_PATH"] = _SDK_LIB_DIR + (":" + existing if existing else "")

_LIB_PATH = os.path.join(_SDK_LIB_DIR, "libOrbbecSDK.so")

# --- enums ---
OB_STREAM_DEPTH = 3
OB_FORMAT_Y16 = 8

# --- error callback type ---
ob_error_cb = CFUNCTYPE(None, c_void_p, c_char_p)

# --- library loading ---
def _load_sdk():
    """Load libOrbbecSDK.so and set function signatures. Preloads dependencies first."""
    import platform
    arch = platform.machine()
    if arch not in ("aarch64", "arm64"):
        raise RuntimeError(
            f"OrbbecSDK so compativel com aarch64/arm64. "
            f"Maquina atual: {arch}. Execute no Raspberry Pi."
        )

    deps = ["libdepthengine.so", "libob_usb.so", "liblive555.so"]
    for dep in deps:
        dep_path = os.path.join(_SDK_LIB_DIR, dep)
        try:
            ctypes.CDLL(dep_path, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass  # dependency may be satisfied elsewhere

    try:
        lib = ctypes.CDLL(os.path.join(_SDK_LIB_DIR, "libOrbbecSDK.so.1.10.35"), mode=ctypes.RTLD_GLOBAL)
    except OSError as e:
        raise RuntimeError(
            f"Nao foi possivel carregar libOrbbecSDK de {_SDK_LIB_DIR}.\n"
            f"Certifique-se que o OrbbecSDK foi clonado e as .so estao em lib/arm64/\n"
            f"Erro: {e}"
        )

    # ob_create_pipeline
    lib.ob_create_pipeline.argtypes = [POINTER(c_void_p)]
    lib.ob_create_pipeline.restype = c_void_p

    # ob_delete_pipeline
    lib.ob_delete_pipeline.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_delete_pipeline.restype = None

    # ob_create_config
    lib.ob_create_config.argtypes = [POINTER(c_void_p)]
    lib.ob_create_config.restype = c_void_p

    # ob_delete_config
    lib.ob_delete_config.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_delete_config.restype = None

    # ob_config_enable_video_stream
    lib.ob_config_enable_video_stream.argtypes = [
        c_void_p, c_int32, c_int32, c_int32, c_int32, c_int32,
        POINTER(c_void_p)
    ]
    lib.ob_config_enable_video_stream.restype = None

    # ob_pipeline_start_with_config
    lib.ob_pipeline_start_with_config.argtypes = [
        c_void_p, c_void_p, POINTER(c_void_p)
    ]
    lib.ob_pipeline_start_with_config.restype = None

    # ob_pipeline_stop
    lib.ob_pipeline_stop.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_pipeline_stop.restype = None

    # ob_pipeline_wait_for_frameset
    lib.ob_pipeline_wait_for_frameset.argtypes = [
        c_void_p, c_uint32, POINTER(c_void_p)
    ]
    lib.ob_pipeline_wait_for_frameset.restype = c_void_p

    # ob_frameset_depth_frame
    lib.ob_frameset_depth_frame.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_frameset_depth_frame.restype = c_void_p

    # ob_frame_data
    lib.ob_frame_data.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_frame_data.restype = c_void_p

    # ob_frame_data_size
    lib.ob_frame_data_size.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_frame_data_size.restype = c_uint32

    # ob_video_frame_width
    lib.ob_video_frame_width.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_video_frame_width.restype = c_uint32

    # ob_video_frame_height
    lib.ob_video_frame_height.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_video_frame_height.restype = c_uint32

    # ob_depth_frame_get_value_scale
    lib.ob_depth_frame_get_value_scale.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_depth_frame_get_value_scale.restype = c_float

    # ob_delete_frame
    lib.ob_delete_frame.argtypes = [c_void_p, POINTER(c_void_p)]
    lib.ob_delete_frame.restype = None

    return lib


class DepthCamera:
    """Minimal depth-only interface for Orbbec Astra Pro."""

    def __init__(self):
        self._lib = _load_sdk()
        self._pipeline = None
        self._config = None
        self._started = False
        self._value_scale = 1.0
        self._width = 0
        self._height = 0

    def _check(self, obj, name):
        if obj is None or obj == 0:
            raise RuntimeError(f"{name} retornou NULL — câmera conectada?")

    def start(self, width=640, height=480, fps=30):
        error = c_void_p(0)
        self._pipeline = self._lib.ob_create_pipeline(byref(error))
        self._check(self._pipeline, "ob_create_pipeline")
        self._config = self._lib.ob_create_config(byref(error))
        self._check(self._config, "ob_create_config")
        self._lib.ob_config_enable_video_stream(
            self._config, OB_STREAM_DEPTH, width, height, fps,
            OB_FORMAT_Y16, byref(error)
        )
        self._lib.ob_pipeline_start_with_config(self._pipeline, self._config, byref(error))
        self._started = True
        self._width = width
        self._height = height

    def get_frame(self, timeout_ms=1000):
        import numpy as np
        if not self._started:
            return None
        error = c_void_p(0)
        frameset = self._lib.ob_pipeline_wait_for_frameset(
            self._pipeline, c_uint32(timeout_ms), byref(error)
        )
        if frameset is None or frameset == 0:
            return None
        depth = self._lib.ob_frameset_depth_frame(frameset, byref(error))
        if depth is None or depth == 0:
            self._lib.ob_delete_frame(frameset, byref(error))
            return None
        data_ptr = self._lib.ob_frame_data(depth, byref(error))
        data_size = self._lib.ob_frame_data_size(depth, byref(error))
        w = self._lib.ob_video_frame_width(depth, byref(error))
        h = self._lib.ob_video_frame_height(depth, byref(error))
        self._value_scale = self._lib.ob_depth_frame_get_value_scale(depth, byref(error))
        self._width = w
        self._height = h
        if data_ptr is None or data_size == 0:
            self._lib.ob_delete_frame(depth, byref(error))
            self._lib.ob_delete_frame(frameset, byref(error))
            return None
        arr = np.ctypeslib.as_array(
            ctypes.cast(data_ptr, ctypes.POINTER(ctypes.c_uint16)),
            shape=(h, w)
        ).copy()
        if self._value_scale and self._value_scale != 1.0:
            arr = (arr.astype(np.float32) * self._value_scale).astype(np.uint16)
        self._lib.ob_delete_frame(depth, byref(error))
        self._lib.ob_delete_frame(frameset, byref(error))
        return arr

    @property
    def value_scale(self):
        return self._value_scale

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def connected(self):
        return self._started

    def stop(self):
        if self._pipeline and self._started:
            error = c_void_p(0)
            self._lib.ob_pipeline_stop(self._pipeline, byref(error))
            self._started = False

    def close(self):
        self.stop()
        error = c_void_p(0)
        if self._config:
            self._lib.ob_delete_config(self._config, byref(error))
            self._config = None
        if self._pipeline:
            self._lib.ob_delete_pipeline(self._pipeline, byref(error))
            self._pipeline = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.close()
