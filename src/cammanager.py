# cam_manager.py
# CamManager: background capture thread with 1.5 s buffer (Y, U, V only!),
# ROI ring + angular patch, manual exposure/gain + FPS lock, preview toggle,
# analyze_patch_at_time() that converts only the patch pixels to HSV, optional
# headless debug previews saved to disk with a per-manager cap, and clean shutdown.
#
# PERFORMANCE: get_patch_mask() uses a cached ring angle lookup computed only on
# ring pixels. This avoids recomputing angles on the full HxW grid and reduces
# per call time from about 60 to 70 ms down to under 1 ms on a Pi.
"""
Overview
--------
This module provides a single class CamManager that opens a libcamera viewfinder
stream, keeps a rolling buffer of the last 1.5 seconds of frames, and exposes a
patch analysis method for a circular ROI. The analysis converts only the pixels
inside a narrow angular patch on the ring ROI from YUV to HSV for lightweight
color classification. A background thread handles capture and buffering. Live
desktop preview can be enabled, and on demand headless debug frames can be
saved to disk from analyze_patch_at_time.

Key points
- Input pixel format is YUV420 (I420 layout). The capture thread stores only
  square cropped planes Y, U, V to avoid converting the full frame to BGR.
- The ROI is a circle centered near the image center with a configurable
  radius and thickness. The patch is an angular sector on that ring.
- Angle convention: 0 degrees at the bottom of the frame, increasing clockwise.
  Left is 90, top is 180, right is 270.
- Performance: only ring pixels are used to compute angles. A cache stores
  ring indices and their angles relative to the ROI center. Subsequent calls to
  build a patch mask filter this 1D angle array and scatter into an HxW mask.
- Headless debug: analyze_patch_at_time can save at most N preview images per
  CamManager. This is off by default and controlled per call via the preview
  flag. The save location and cap are constructor parameters.

API sketch
- CamManager(preview_on=False, patch_angle_deg=210.0, patch_width_deg=2.0,
             debug_save_dir="/tmp/cammanager_debug", debug_max_previews=100)
    Starts capture in a second thread and begins buffering frames.

- analyze_patch_at_time(query_ts_ns, angle_deg, width_deg, preview=False)
    Looks up the buffered frame with timestamp closest to query_ts_ns and
    classifies the selected patch. Returns
    (label, used_ts_ns, signed_age_ms, h_mean, s_mean, v_mean).
    If preview is True and the manager has not yet reached its save cap, a PNG
    with only the patch overlay and simple text is written to disk.

- close()
    Stops capture, releases resources, and closes mmaps.

"""

import os, select, mmap, time, collections, math, threading
import numpy as np
import cv2
import libcamera as lc

# ---------- tuned knobs ----------
SIZE = (1100, 1100)            # requested (w,h); libcamera will pick nearest VF
PIXEL_FMT = "YUV420"

TARGET_FPS     = 30
FRAME_USEC     = int(1_000_000 / TARGET_FPS)
EXPOSURE_USEC  = max(FRAME_USEC - 1, 33000)   # must be <= FRAME_USEC
ANALOG_GAIN    = 14.0                         # ~ISO ≈ gain*100

# ROI (square-space)
ROI_RADIUS     = 504
ROI_THICKNESS  = 4
ROI_OFFSET_X   = -7
ROI_OFFSET_Y   = -6                           # 1 px up is negative
ROI_COLOR      = (255, 0, 255)                # BGR magenta

# Patch
PATCH_COLOR       = (255, 255, 0)             # BGR
SHOW_PATCH_CENTER = True

# Buffer (~1.5 s). Using a small safety margin above theoretical frame count.
BUFFER_SEC = 1.5
BUFFER_MAX = int(max(60, TARGET_FPS * BUFFER_SEC * 1.2))

# ---------- helpers ----------
def page_size():
    """System page size used for proper mmap alignment of libcamera planes."""
    return os.sysconf("SC_PAGESIZE")

def mmap_plane(fd: int, length: int, offset: int) -> memoryview:
    """Map a single libcamera plane into user space as a read-only memoryview.

    The return value is a slice that compensates for page alignment, so callers
    can treat it as a tight plane buffer starting at the given offset.
    """
    ps = page_size()
    page_off = offset % ps
    map_off = offset - page_off
    map_len = length + page_off
    fd_dup = os.dup(fd)
    mm = mmap.mmap(fd_dup, map_len, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ, offset=map_off)
    os.close(fd_dup)
    return memoryview(mm)[page_off:page_off + length]

def now_boottime_ns() -> int:
    """Nanoseconds from CLOCK_BOOTTIME if available, else monotonic."""
    if hasattr(time, "CLOCK_BOOTTIME"):
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    return time.monotonic_ns()

# -------- ROI and patch utilities (cached coords and masks) --------
class _MaskCache:
    """Cache for ring geometry and per ring pixel angles.

    The cache removes per call HxW angle computations. For a given geometry
    defined by (h, w, radius, thickness, dx, dy) it stores:
    - ring_bool: HxW boolean mask of the ring
    - ring_ys, ring_xs: indices of ring pixels
    - theta_ring: per ring pixel angle in degrees with 0 at bottom and positive
      clockwise
    """

    def __init__(self):
        self.last_ring_key  = None   # (h,w,radius,thickness,dx,dy)
        self.ring_bool      = None   # (H,W) bool
        self.ring_ys        = None   # (N,)
        self.ring_xs        = None   # (N,)
        self.theta_ring     = None   # (N,) float32, 0 bottom, +CW

    def _build_ring_cache(self, h: int, w: int, radius: int, thickness: int, dx: int, dy: int):
        """Build ring mask and per ring pixel angles, then store in the cache."""
        yy, xx = np.mgrid[0:h, 0:w]
        cx, cy = (w // 2 + dx), (h // 2 + dy)
        r = np.sqrt((xx - cx)**2 + (yy - cy)**2)
        ring_bool = (np.abs(r - radius) <= (thickness / 2.0))

        ys, xs = np.nonzero(ring_bool)
        # Angle only for ring pixels. atan2 returns angle from +x axis CCW.
        dxp = xs - cx
        dyp = ys - cy
        ang = np.degrees(np.arctan2(dyp, dxp))
        theta = (ang - 90.0) % 360.0  # map to 0 at bottom, positive clockwise

        # Cache fields
        self.ring_bool  = ring_bool
        self.ring_ys    = ys.astype(np.int32)
        self.ring_xs    = xs.astype(np.int32)
        self.theta_ring = theta.astype(np.float32)

    def get_ring_mask(self, h: int, w: int, radius: int, thickness: int, dx: int, dy: int):
        """Return the HxW boolean ring mask, rebuilding cache when geometry changes."""
        key = (h, w, radius, thickness, dx, dy)
        if key != self.last_ring_key:
            self._build_ring_cache(h, w, radius, thickness, dx, dy)
            self.last_ring_key = key
        return self.ring_bool

    def get_patch_mask(self, h: int, w: int, ring_mask: np.ndarray,
                       dx: int, dy: int,
                       center_deg: float, width_deg: float) -> np.ndarray:
        """Return an HxW boolean mask for a narrow angular sector on the ring.

        This reuses cached ring indices and precomputed angles, then scatters the
        1D selection back into an HxW mask. Sector inclusion handles wraparound.
        """
        # Ensure cache corresponds to the provided geometry
        _ = self.get_ring_mask(h, w, ROI_RADIUS, ROI_THICKNESS, dx, dy)

        half = width_deg / 2.0
        a0 = (center_deg - half) % 360.0
        a1 = (center_deg + half) % 360.0
        if a0 <= a1:
            on_sector = (self.theta_ring >= a0) & (self.theta_ring <= a1)
        else:
            on_sector = (self.theta_ring >= a0) | (self.theta_ring <= a1)

        mask = np.zeros((h, w), dtype=bool)
        if np.any(on_sector):
            mask[self.ring_ys[on_sector], self.ring_xs[on_sector]] = True
        return mask

# -------- YUV helpers (no full frame conversion) --------
def _y_plane_view(y_plane: memoryview, W: int, H: int):
    """Return a 2D view (H, y_stride) over the Y plane. Caller crops columns to W."""
    y_len = len(y_plane)
    y_stride = y_len // H
    y_src = np.frombuffer(y_plane, dtype=np.uint8).reshape(H, y_stride)
    return y_src, y_stride

def _uv_plane_view(uv_plane: memoryview, W: int, H: int):
    """Return a 2D view (H/2, u_stride) over a chroma plane. Caller crops to W/2."""
    uv_len = len(uv_plane)
    uv_h = H // 2
    u_stride = uv_len // uv_h
    u_src = np.frombuffer(uv_plane, dtype=np.uint8).reshape(uv_h, u_stride)
    return u_src, u_stride

def _square_crop_from_yuv_planes(W_full, H_full, x0, side,
                                 y_plane, u_plane, v_plane):
    """Build square cropped Y, U, V from viewfinder planes only by slicing.

    Returns
    - Ysq: shape (side, side)
    - Usq: shape (side/2, side/2)
    - Vsq: shape (side/2, side/2)
    """
    # Luma
    y_src, y_stride = _y_plane_view(y_plane, W_full, H_full)
    Ysq = y_src[:, x0:x0+side].copy()

    # Chroma (subsampled by 2)
    u_src, u_stride = _uv_plane_view(u_plane, W_full, H_full)
    v_src, v_stride = _uv_plane_view(v_plane, W_full, H_full)
    u_x0 = x0 // 2
    side_uv = side // 2
    Usq = u_src[:, u_x0:u_x0+side_uv].copy()
    Vsq = v_src[:, u_x0:u_x0+side_uv].copy()

    return Ysq, Usq, Vsq

def _patch_yuv_vectors(Ysq, Usq, Vsq, patch_mask):
    """Extract Y, U, V vectors for all True positions in the patch mask.

    U and V are sampled using 2x2 subsampling at indices (y//2, x//2).
    Returns three 1D arrays or (None, None, None) if the mask is empty.
    """
    ys, xs = np.nonzero(patch_mask)
    if ys.size == 0:
        return None, None, None
    Yv = Ysq[ys, xs]
    Uv = Usq[ys // 2, xs // 2]
    Vv = Vsq[ys // 2, xs // 2]
    return Yv, Uv, Vv

def _yuv_to_bgr_vec(Yv, Uv, Vv):
    """Convert YUV vectors to B, G, R vectors using BT.601 integer math."""
    C = Yv.astype(np.int32) - 16
    D = Uv.astype(np.int32) - 128
    E = Vv.astype(np.int32) - 128

    R = (298*C + 409*E + 128) >> 8
    G = (298*C - 100*D - 208*E + 128) >> 8
    B = (298*C + 516*D + 128) >> 8

    R = np.clip(R, 0, 255).astype(np.uint8)
    G = np.clip(G, 0, 255).astype(np.uint8)
    B = np.clip(B, 0, 255).astype(np.uint8)
    return B, G, R

def _classify_bgr_vectors(B, G, R):
    """Classify color from the mean HSV of BGR vectors.

    Returns a tuple (label, h_mean, s_mean, v_mean). Label is one of
    "RED", "GREEN", "OTHER", or "none" if inputs are empty.
    """
    if B is None or B.size == 0:
        return ("none", float("nan"), float("nan"), float("nan"))
    pix = np.stack([B, G, R], axis=1).reshape(-1, 1, 3)
    hsv = cv2.cvtColor(pix, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    h = hsv[:, 0].astype(np.float32)
    s = hsv[:, 1].astype(np.float32)
    v = hsv[:, 2].astype(np.float32)
    h_mean = float(np.mean(h)); s_mean = float(np.mean(s)); v_mean = float(np.mean(v))
    if s_mean < 11 or v_mean < 17:
        return ("OTHER", h_mean, s_mean, v_mean)
    if (((h_mean <= 18.0) or (h_mean >= 164.0)) and s_mean > 50 and v_mean > 15) or (((h_mean <= 25.0) or (h_mean >= 140.0)) and s_mean > 220 and v_mean > 15) or (s_mean > 239):
        return ("RED", h_mean, s_mean, v_mean)
    if (35.0 <= h_mean <= 102) and (210 > s_mean >= 11) and v_mean > 14:
        return ("GREEN", h_mean, s_mean, v_mean)
    return ("OTHER", h_mean, s_mean, v_mean)

def _square_bgr_from_yuv(Ysq, Usq, Vsq):
    """Reconstruct a square BGR image from Y, U, V planes for preview overlays."""
    H = Ysq.shape[0]
    W = H
    uv_h = H // 2
    i420 = np.concatenate([Ysq.reshape(-1), Usq.reshape(-1), Vsq.reshape(-1)])
    i420_mat = i420.reshape((H + uv_h, W))
    bgr = cv2.cvtColor(i420_mat, cv2.COLOR_YUV2BGR_I420)
    return bgr

# ==========================================================
#                      CamManager
# ==========================================================
class CamManager:
    """Camera manager with background capture, ROI patch analysis and debug saves."""

    def __init__(self,
                 preview_on: bool = False,
                 patch_angle_deg: float = 210.0,
                 patch_width_deg: float = 2.0,
                 debug_save_dir: str = "/tmp/cammanager_debug",
                 debug_max_previews: int = 100):
        """Create and start a camera manager.

        preview_on controls a desktop live preview window from the capture
        thread. For headless systems this is usually False.

        patch_angle_deg and patch_width_deg set the default live preview patch.

        debug_save_dir is where analyze_patch_at_time(preview=True) writes PNGs.
        debug_max_previews limits how many debug frames are saved per manager.
        """
        self.preview_on = preview_on
        self.patch_angle_deg = patch_angle_deg
        self.patch_width_deg = patch_width_deg

        # Debug preview saving configuration
        self.debug_save_dir = debug_save_dir
        self.debug_max_previews = int(debug_max_previews)
        self._debug_saved = 0
        try:
            os.makedirs(self.debug_save_dir, exist_ok=True)
        except Exception:
            # If creation fails the save feature is disabled silently
            self.debug_save_dir = None
            self.debug_max_previews = 0

        # Frame buffer stores tuples (sensor_ts_ns, Ysq, Usq, Vsq)
        self.frame_buffer = collections.deque(maxlen=BUFFER_MAX)
        self.ts_hist = collections.deque(maxlen=90)  # for simple FPS display
        self._buf_lock = threading.Lock()

        self._stop = threading.Event()
        self._mask_cache = _MaskCache()

        # --- libcamera setup matching the confirmed working control set ---
        self.cm = lc.CameraManager.singleton()
        self.cam = self.cm.cameras[0]
        self.cam.acquire()

        self.cfg = self.cam.generate_configuration([lc.StreamRole.Viewfinder])
        sc = self.cfg.at(0)
        sc.pixel_format = lc.PixelFormat(PIXEL_FMT)
        sc.size = lc.Size(*SIZE)
        self.cam.configure(self.cfg)

        actual = self.cfg.at(0).size
        self.W_full, self.H_full = actual.width, actual.height

        self.alloc = lc.FrameBufferAllocator(self.cam)
        self.stream = self.cfg.at(0).stream
        if self.alloc.allocate(self.stream) < 0:
            raise RuntimeError("Buffer allocation failed")

        # Map planes for all frame buffers once
        self.fb_maps = {}
        for fb in self.alloc.buffers(self.stream):
            self.fb_maps[fb] = [mmap_plane(p.fd, p.length, p.offset) for p in fb.planes]

        self.reqs = []
        for fb in self.alloc.buffers(self.stream):
            r = self.cam.create_request()
            r.add_buffer(self.stream, fb)
            r.set_control(lc.controls.FrameDurationLimits, (FRAME_USEC, FRAME_USEC))
            r.set_control(lc.controls.AeEnable, False)
            r.set_control(lc.controls.AnalogueGainMode, int(1))
            r.set_control(lc.controls.ExposureTime, int(EXPOSURE_USEC))
            r.set_control(lc.controls.AnalogueGain, float(ANALOG_GAIN))
            self.reqs.append(r)

        self.cam.start()
        for r in self.reqs:
            self.cam.queue_request(r)

        self.poller = select.poll()
        self.poller.register(self.cm.event_fd, select.POLLIN)

        # Background capture thread
        self._thread = threading.Thread(target=self._run, name="CamCaptureThread", daemon=True)
        self._thread.start()

    # ---------- public API ----------
    def close(self):
        """Signal stop and release camera resources and mmaps."""
        try:
            self._stop.set()
            if self._thread.is_alive():
                self._thread.join(timeout=2.0)
        finally:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            try:
                self.cam.stop()
            except Exception:
                pass
            try:
                self.cam.release()
            except Exception:
                pass
            # Close mapped planes
            for mmap_list in self.fb_maps.values():
                for mv in mmap_list:
                    try:
                        mm = mv.obj; mv.release()
                        if hasattr(mm, "close"): mm.close()
                    except Exception:
                        pass

    def analyze_patch_at_time(self, query_ts_ns: int,
                              angle_deg: float,
                              width_deg: float,
                              preview: bool = False):
        """
        Return (label, used_ts_ns, signed_age_ms, h_mean, s_mean, v_mean).

        If preview=True and debug saving is enabled, a patch-only debug image is saved
        to disk until the per-manager cap is reached. No GUI windows are opened here.

        Notes on timing prints:
          [perf] mask_build shows the cost of building the angular patch mask only.
          total shows the cost of the entire function call.
        """
        # t0 starts total runtime measurement for this call
        t0 = time.monotonic_ns()

        # 1) Pick the buffered frame closest in time to the requested timestamp.
        #    The buffer holds tuples of (sensor_ts_ns, Ysq, Usq, Vsq).
        #    The lock guards against the capture thread mutating the buffer while we read.
        with self._buf_lock:
            if not self.frame_buffer:
                # No frames yet. Return a neutral result with NaNs for metrics.
                return ("none", query_ts_ns, float("nan"), float("nan"), float("nan"), float("nan"))
            idx_best = min(
                range(len(self.frame_buffer)),
                key=lambda i: abs(self.frame_buffer[i][0] - query_ts_ns)
            )
            used_ts, Ysq, Usq, Vsq = self.frame_buffer[idx_best]

        # 2) Work in square-space dimensions. H == W == side of the center crop.
        H = Ysq.shape[0]
        W = H  # by construction

        # 3) Get the binary ring mask for the configured ROI geometry.
        #    This call is cache-backed and only recomputes when geometry changes.
        ring = self._mask_cache.get_ring_mask(
            H, W, ROI_RADIUS, ROI_THICKNESS, ROI_OFFSET_X, ROI_OFFSET_Y
        )

        # 4) Build the angular "patch" mask on that ring for the requested angle and width.
        #    The mask cache precomputes angles only on ring pixels and scatters them,
        #    which is why this step is fast. t1..t2 isolates just the patch construction time.
        t1 = time.monotonic_ns()
        patch = self._mask_cache.get_patch_mask(
            H, W, ring, ROI_OFFSET_X, ROI_OFFSET_Y, angle_deg, width_deg
        )
        t2 = time.monotonic_ns()

        # 5) Convert only the pixels inside the patch from YUV to BGR vectors.
        #    This avoids converting the whole frame. U and V are sampled at 2x2 subsampling.
        Yv, Uv, Vv = _patch_yuv_vectors(Ysq, Usq, Vsq, patch)

        # Signed age in milliseconds of the actual frame we used relative to the query time.
        # Positive if the used frame is newer, negative if it is older.
        age_ms = (used_ts - query_ts_ns) / 1e6

        # 6) Handle the case where the patch contains no pixels.
        #    This can happen if the ROI parameters or angle are out of bounds.
        if Yv is None:
            # Optional one-off preview save for debugging geometry, if enabled and under cap.
            if preview and self._debug_saved < self.debug_max_previews and self.debug_save_dir:
                view = _square_bgr_from_yuv(Ysq, Usq, Vsq)
                view[patch, :] = np.maximum(view[patch, :], np.array(PATCH_COLOR, dtype=np.uint8))
                self._save_debug_frame(
                    view, used_ts, angle_deg, width_deg, label="none", hsv=None, age_ms=age_ms
                )
            # Print performance numbers and return a neutral classification.
            t3 = time.monotonic_ns()
            print(f"[perf] mask_build={(t2 - t1)/1e6:.3f} ms | total={(t3 - t0)/1e6:.3f} ms")
            return ("none", used_ts, age_ms, float("nan"), float("nan"), float("nan"))

        # 7) Convert the YUV vectors to BGR vectors and compute HSV statistics.
        #    Classification returns the color label and the mean H, S, V for inspection.
        B, G, R = _yuv_to_bgr_vec(Yv, Uv, Vv)
        label, h_mean, s_mean, v_mean = _classify_bgr_vectors(B, G, R)

        # 8) Optional debug image save. This reconstructs a square BGR frame only for saving,
        #    tints the patch, draws the center line, and annotates HSV and timing. Saving stops
        #    automatically after the per-manager cap is reached.
        if preview and self._debug_saved < self.debug_max_previews and self.debug_save_dir:
            view = _square_bgr_from_yuv(Ysq, Usq, Vsq)
            # Tint patch area to make it visible in the saved image
            view[patch, :] = np.maximum(view[patch, :], np.array(PATCH_COLOR, dtype=np.uint8))
            # Draw center line for the requested angle
            cx, cy = W // 2 + ROI_OFFSET_X, H // 2 + ROI_OFFSET_Y
            theta_std = math.radians(90.0 - angle_deg)  # 0 deg at bottom, clockwise positive
            x_end = int(cx + ROI_RADIUS * math.cos(theta_std))
            y_end = int(cy + ROI_RADIUS * math.sin(theta_std))
            cv2.line(view, (cx, cy), (x_end, y_end), (0, 255, 255), 2, cv2.LINE_AA)
            # Annotate label and HSV means
            cv2.putText(view, f"Label: {label}", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(view, f"H:{h_mean:5.1f}  S:{s_mean:5.1f}  V:{v_mean:5.1f}",
                        (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(view, f"Δ to query: {age_ms:0.2f} ms", (12, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            self._save_debug_frame(
                view, used_ts, angle_deg, width_deg, label, (h_mean, s_mean, v_mean), age_ms
            )

        # 9) Print performance numbers for quick profiling and return the result.
        t3 = time.monotonic_ns()
        print(f"[perf] mask_build={(t2 - t1)/1e6:.3f} ms | total={(t3 - t0)/1e6:.3f} ms")
        return (label, used_ts, age_ms, h_mean, s_mean, v_mean)


    def _save_debug_frame(self, img_bgr: np.ndarray, ts_ns: int, angle_deg: float, width_deg: float,
                          label: str, hsv, age_ms: float):
        """Save a single PNG into debug_save_dir with a name that encodes parameters.

        Stops saving automatically after reaching debug_max_previews for this manager.
        """
        try:
            if self._debug_saved >= self.debug_max_previews or not self.debug_save_dir:
                return
            fname = f"patch_{ts_ns}_ang{angle_deg:.1f}_w{width_deg:.1f}_{label}"
            if hsv is not None:
                h, s, v = hsv
                fname += f"_H{h:.1f}_S{s:.1f}_V{v:.1f}"
            fname += f"_d{age_ms:+.1f}ms.png"
            path = os.path.join(self.debug_save_dir, fname)
            cv2.imwrite(path, img_bgr)
            self._debug_saved += 1
        except Exception:
            # Saving errors are ignored to keep analysis real time
            pass

    # ---------- background thread ------------
    def _run(self):
        """Capture loop that fills the rolling buffer and handles optional live preview."""
        try:
            while not self._stop.is_set():
                # Wait up to 1 s for camera events
                self.poller.poll(1000)

                for req in self.cm.get_ready_requests():
                    if self._stop.is_set():
                        break

                    md = req.metadata
                    ts  = md.get(lc.controls.SensorTimestamp)   # ns

                    fb = req.buffers[self.stream]
                    planes = self.fb_maps[fb]
                    y_plane, u_plane, v_plane = planes[0], planes[1], planes[2]

                    # Center square crop indices for the viewfinder frame
                    side = self.H_full
                    x0 = max(0, (self.W_full - side) // 2)

                    # Build only square Y, U, V. No full frame BGR conversion here.
                    Ysq, Usq, Vsq = _square_crop_from_yuv_planes(
                        self.W_full, self.H_full, x0, side, y_plane, u_plane, v_plane
                    )

                    # Buffer the planes with the sensor timestamp
                    if ts is not None:
                        with self._buf_lock:
                            self.frame_buffer.append((ts, Ysq, Usq, Vsq))
                            self.ts_hist.append(ts)

                    # Optional desktop preview window
                    if self.preview_on:
                        overlay = _square_bgr_from_yuv(Ysq, Usq, Vsq)

                        # Draw ring and current live patch
                        ring = self._mask_cache.get_ring_mask(side, side, ROI_RADIUS, ROI_THICKNESS,
                                                              ROI_OFFSET_X, ROI_OFFSET_Y)
                        patch_live = self._mask_cache.get_patch_mask(side, side, ring,
                                                                     ROI_OFFSET_X, ROI_OFFSET_Y,
                                                                     self.patch_angle_deg, self.patch_width_deg)
                        cx, cy = side // 2 + ROI_OFFSET_X, side // 2 + ROI_OFFSET_Y
                        cv2.circle(overlay, (cx, cy), int(ROI_RADIUS), ROI_COLOR,
                                   thickness=int(ROI_THICKNESS), lineType=cv2.LINE_AA)
                        overlay[patch_live, :] = np.maximum(
                            overlay[patch_live, :], np.array(PATCH_COLOR, dtype=np.uint8)
                        )
                        if SHOW_PATCH_CENTER:
                            theta_std = math.radians(90.0 - self.patch_angle_deg)
                            x_end = int(cx + ROI_RADIUS * math.cos(theta_std))
                            y_end = int(cy + ROI_RADIUS * math.sin(theta_std))
                            cv2.line(overlay, (cx, cy), (x_end, y_end),
                                     (0, 255, 255), 2, cv2.LINE_AA)

                        # Minimal HUD: FPS
                        fps_txt = "FPS: n/a"
                        with self._buf_lock:
                            if len(self.ts_hist) >= 2:
                                dt = (self.ts_hist[-1] - self.ts_hist[0]) / 1e9
                                if dt > 0:
                                    fps = (len(self.ts_hist) - 1) / dt
                                    fps_txt = f"FPS: {fps:5.2f}"
                        cv2.putText(overlay, fps_txt, (12, 26),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,200,0), 2)

                        cv2.imshow("CamManager preview", overlay)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self._stop.set()
                            break

                    # Requeue the request to continue streaming
                    req.reuse()
                    self.cam.queue_request(req)

        except Exception:
            # Avoid the thread dying silently in production. Logging could be added.
            pass

# ==========================================================
#                        Test Main
# ==========================================================

def main():
    """Simple harness that repeatedly analyzes a patch slightly in the past.

    The loop prints the classification and basic timing. Stop with Ctrl+C.
    """
    PREVIEW_LIVE = False

    ANGLE_DEG = 180.0
    WIDTH_DEG = 10.0

    mgr = CamManager(preview_on=PREVIEW_LIVE,
                     patch_angle_deg=ANGLE_DEG,
                     patch_width_deg=WIDTH_DEG,
                     debug_save_dir="/home/carl/tmp/cammanager_debug",
                     debug_max_previews=10)
    try:
        while True:
            # Analyze a frame about 200 ms old relative to now
            target_ts = now_boottime_ns() - int(200 * 1e6)
            label, used_ts, age_ms, h, s, v = mgr.analyze_patch_at_time(
                target_ts, ANGLE_DEG, WIDTH_DEG, preview=True
            )
            print(f"Patch -> {label:>5s} | H:{h:5.1f} S:{s:5.1f} V:{v:5.1f} | "
                  f"closest Δ={age_ms:0.2f} ms | saved={mgr._debug_saved}/{mgr.debug_max_previews}")
            time.sleep(0.06)
    except KeyboardInterrupt:
        pass
    finally:
        mgr.close()

if __name__ == "__main__":
    main()
