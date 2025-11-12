# cam_manager.py
# CamManager: background capture thread with 1.5 s buffer (Y, U, V only!),
# ROI ring + angular patch, manual exposure/gain + FPS lock, preview toggle,
# analyze_patch_at_time() that converts only the patch pixels to HSV, optional
# headless debug previews saved to disk with a per-manager cap, and clean shutdown.

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
ANALOG_GAIN    = 25.0                         # ~ISO ≈ gain*100

# ROI (square-space)
ROI_RADIUS     = 504
ROI_THICKNESS  = 4
ROI_OFFSET_X   = -13
ROI_OFFSET_Y   = 0
ROI_COLOR      = (255, 0, 255)                # BGR magenta

# Patch
PATCH_COLOR       = (255, 255, 0)             # BGR
SHOW_PATCH_CENTER = True

# Buffer (~1.5s)
BUFFER_SEC = 1.5
BUFFER_MAX = int(max(60, TARGET_FPS * BUFFER_SEC * 1.2))

# ---------- helpers ----------
def page_size():
    return os.sysconf("SC_PAGESIZE")

def mmap_plane(fd: int, length: int, offset: int) -> memoryview:
    ps = page_size()
    page_off = offset % ps
    map_off = offset - page_off
    map_len = length + page_off
    fd_dup = os.dup(fd)
    mm = mmap.mmap(fd_dup, map_len, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ, offset=map_off)
    os.close(fd_dup)
    return memoryview(mm)[page_off:page_off + length]

def now_boottime_ns() -> int:
    if hasattr(time, "CLOCK_BOOTTIME"):
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    return time.monotonic_ns()

# -------- ROI & patch utilities (cached coords/masks) --------
class _MaskCache:
    def __init__(self):
        self.last_coords = None   # ((H,W), yy, xx)
        self.last_mask_key = None
        self.last_ring_mask = None

    def get_square_coords(self, h: int, w: int):
        if (self.last_coords is None) or (self.last_coords[0] != (h, w)):
            yy, xx = np.mgrid[0:h, 0:w]
            self.last_coords = ((h, w), yy, xx)
        _, yy, xx = self.last_coords
        return yy, xx

    def get_ring_mask(self, h: int, w: int, radius: int, thickness: int, dx: int, dy: int):
        key = (h, w, radius, thickness, dx, dy)
        if key == self.last_mask_key and self.last_ring_mask is not None:
            return self.last_ring_mask
        yy, xx = self.get_square_coords(h, w)
        cx, cy = (w // 2 + dx), (h // 2 + dy)
        r = np.sqrt((xx - cx)**2 + (yy - cy)**2)
        ring_bool = (np.abs(r - radius) <= (thickness / 2.0))
        self.last_mask_key, self.last_ring_mask = key, ring_bool
        return ring_bool

    def get_patch_mask(self, h: int, w: int, ring_mask: np.ndarray,
                       dx: int, dy: int,
                       center_deg: float, width_deg: float) -> np.ndarray:
        """
        Angle convention: 0° at bottom; CLOCKWISE positive (left=90, top=180, right=270).
        """
        yy, xx = self.get_square_coords(h, w)
        cx, cy = (w // 2 + dx), (h // 2 + dy)
        dxp = xx - cx
        dyp = yy - cy
        # atan2 degrees from +x CCW
        ang = np.degrees(np.arctan2(dyp, dxp))
        # map to desired system: bottom=0, clockwise+
        theta = (ang - 90.0) % 360.0
        half = width_deg / 2.0
        a0 = (center_deg - half) % 360.0
        a1 = (center_deg + half) % 360.0
        if a0 <= a1:
            sector = (theta >= a0) & (theta <= a1)
        else:
            sector = (theta >= a0) | (theta <= a1)
        return ring_mask & sector

# -------- YUV helpers (no full-frame conversion) --------
def _y_plane_view(y_plane: memoryview, W: int, H: int):
    """Return a 2D view (H, y_stride) over Y, then caller can crop columns to W."""
    y_len = len(y_plane)
    y_stride = y_len // H
    y_src = np.frombuffer(y_plane, dtype=np.uint8).reshape(H, y_stride)
    return y_src, y_stride

def _uv_plane_view(uv_plane: memoryview, W: int, H: int):
    """Return a 2D view (H/2, u_stride) over a chroma plane, then crop to W/2."""
    uv_len = len(uv_plane)
    uv_h = H // 2
    u_stride = uv_len // uv_h
    u_src = np.frombuffer(uv_plane, dtype=np.uint8).reshape(uv_h, u_stride)
    return u_src, u_stride

def _square_crop_from_yuv_planes(W_full, H_full, x0, side,
                                 y_plane, u_plane, v_plane):
    """
    Build square-cropped Y, U, V (not BGR):
      Ysq: (side, side)
      Usq: (side/2, side/2)
      Vsq: (side/2, side/2)
    by slicing from the plane views with correct strides, avoiding full conversion.
    """
    # Y
    y_src, y_stride = _y_plane_view(y_plane, W_full, H_full)
    Ysq = y_src[:, x0:x0+side].copy()

    # UV (subsampled by 2)
    u_src, u_stride = _uv_plane_view(u_plane, W_full, H_full)
    v_src, v_stride = _uv_plane_view(v_plane, W_full, H_full)
    u_x0 = x0 // 2
    side_uv = side // 2
    Usq = u_src[:, u_x0:u_x0+side_uv].copy()
    Vsq = v_src[:, u_x0:u_x0+side_uv].copy()

    return Ysq, Usq, Vsq

def _patch_yuv_vectors(Ysq, Usq, Vsq, patch_mask):
    """
    Extract Y, U, V vectors (N,) for all True positions in patch_mask.
    U/V are sampled at (y//2, x//2).
    """
    ys, xs = np.nonzero(patch_mask)  # vectors
    if ys.size == 0:
        return None, None, None
    Yv = Ysq[ys, xs]
    Uv = Usq[ys // 2, xs // 2]
    Vv = Vsq[ys // 2, xs // 2]
    return Yv, Uv, Vv

def _yuv_to_bgr_vec(Yv, Uv, Vv):
    """
    Convert YUV vectors (BT.601) to BGR vectors (uint8).
    Y in [0..255], U/V in [0..255].
    """
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
    """
    Classify using HSV means computed from BGR vectors.
    Returns (label, h_mean, s_mean, v_mean).
    """
    if B is None or B.size == 0:
        return ("none", float("nan"), float("nan"), float("nan"))
    pix = np.stack([B, G, R], axis=1).reshape(-1, 1, 3)
    hsv = cv2.cvtColor(pix, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    h = hsv[:, 0].astype(np.float32)
    s = hsv[:, 1].astype(np.float32)
    v = hsv[:, 2].astype(np.float32)
    h_mean = float(np.mean(h)); s_mean = float(np.mean(s)); v_mean = float(np.mean(v))
    if s_mean < 40 or v_mean < 40:
        return ("other", h_mean, s_mean, v_mean)
    if 35.0 <= h_mean <= 102.0:
        return ("green", h_mean, s_mean, v_mean)
    if (h_mean <= 15.0) or (h_mean >= 165.0):
        return ("red", h_mean, s_mean, v_mean)
    return ("other", h_mean, s_mean, v_mean)

def _square_bgr_from_yuv(Ysq, Usq, Vsq):
    """
    Reconstruct a square BGR image from Ysq,Usq,Vsq for preview/overlay generation.
    """
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
    def __init__(self,
                 preview_on: bool = False,
                 patch_angle_deg: float = 210.0,
                 patch_width_deg: float = 2.0,
                 debug_save_dir: str = "/tmp/cammanager_debug",
                 debug_max_previews: int = 100):
        """
        preview_on: live window (imshow) from the capture thread (desktop use).
        debug_save_dir: where analyze_patch_at_time(preview=True) saves images.
        debug_max_previews: per-manager cap for saved debug frames (auto-disables after).
        """
        self.preview_on = preview_on
        self.patch_angle_deg = patch_angle_deg
        self.patch_width_deg = patch_width_deg

        # debug preview saving
        self.debug_save_dir = debug_save_dir
        self.debug_max_previews = int(debug_max_previews)
        self._debug_saved = 0
        try:
            os.makedirs(self.debug_save_dir, exist_ok=True)
        except Exception:
            # if creation fails, silently continue; we'll simply not save
            self.debug_save_dir = None
            self.debug_max_previews = 0

        # buffer of tuples: (sensor_ts_ns, Ysq, Usq, Vsq)
        self.frame_buffer = collections.deque(maxlen=BUFFER_MAX)
        self.ts_hist = collections.deque(maxlen=90)
        self._buf_lock = threading.Lock()

        self._stop = threading.Event()
        self._mask_cache = _MaskCache()

        # --- libcamera setup (same working controls) ---
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

        # mmap planes
        self.fb_maps = {}
        for fb in self.alloc.buffers(self.stream):
            self.fb_maps[fb] = [mmap_plane(p.fd, p.length, p.offset) for p in fb.planes]

        # requests + controls
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

        # start background capture thread
        self._thread = threading.Thread(target=self._run, name="CamCaptureThread", daemon=True)
        self._thread.start()

    # ---------- API ----------
    def close(self):
        """Signal stop and clean up."""
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
            # close mmaps
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
        If preview=True, save ONLY the analyzed patch overlay (no full HUD) to disk.
        Saving stops automatically after 'debug_max_previews' images for this manager.
        """
        with self._buf_lock:
            if not self.frame_buffer:
                return ("none", query_ts_ns, float("nan"), float("nan"), float("nan"), float("nan"))
            idx_best = min(range(len(self.frame_buffer)),
                           key=lambda i: abs(self.frame_buffer[i][0] - query_ts_ns))
            used_ts, Ysq, Usq, Vsq = self.frame_buffer[idx_best]

        H = Ysq.shape[0]; W = H
        # ring + patch mask
        ring = self._mask_cache.get_ring_mask(H, W, ROI_RADIUS, ROI_THICKNESS, ROI_OFFSET_X, ROI_OFFSET_Y)
        patch = self._mask_cache.get_patch_mask(H, W, ring, ROI_OFFSET_X, ROI_OFFSET_Y,
                                                angle_deg, width_deg)

        # Extract patch YUV vectors and classify (convert only those pixels)
        Yv, Uv, Vv = _patch_yuv_vectors(Ysq, Usq, Vsq, patch)
        age_ms = (used_ts - query_ts_ns) / 1e6
        if Yv is None:
            # Nothing to analyze in this patch
            if preview and self._debug_saved < self.debug_max_previews and self.debug_save_dir:
                # Still honor saving an empty overlay (helps debugging geometry)
                view = _square_bgr_from_yuv(Ysq, Usq, Vsq)
                view[patch, :] = np.maximum(view[patch, :], np.array(PATCH_COLOR, dtype=np.uint8))
                self._save_debug_frame(view, used_ts, angle_deg, width_deg, label="none", hsv=None, age_ms=age_ms)
            return ("none", used_ts, age_ms, float("nan"), float("nan"), float("nan"))

        B, G, R = _yuv_to_bgr_vec(Yv, Uv, Vv)
        label, h_mean, s_mean, v_mean = _classify_bgr_vectors(B, G, R)

        # Optionally save a debug image (headless-friendly)
        if preview and self._debug_saved < self.debug_max_previews and self.debug_save_dir:
            view = _square_bgr_from_yuv(Ysq, Usq, Vsq)
            # tint patch
            view[patch, :] = np.maximum(view[patch, :], np.array(PATCH_COLOR, dtype=np.uint8))
            # center line for the chosen angle
            cx, cy = W // 2 + ROI_OFFSET_X, H // 2 + ROI_OFFSET_Y
            theta_std = math.radians(90.0 - angle_deg)
            x_end = int(cx + ROI_RADIUS * math.cos(theta_std))
            y_end = int(cy + ROI_RADIUS * math.sin(theta_std))
            cv2.line(view, (cx, cy), (x_end, y_end), (0, 255, 255), 2, cv2.LINE_AA)
            # simple text with HSV + label
            cv2.putText(view, f"Label: {label}", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(view, f"H:{h_mean:5.1f}  S:{s_mean:5.1f}  V:{v_mean:5.1f}",
                        (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(view, f"Δ to query: {age_ms:0.2f} ms", (12, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            self._save_debug_frame(view, used_ts, angle_deg, width_deg, label, (h_mean, s_mean, v_mean), age_ms)

        return (label, used_ts, age_ms, h_mean, s_mean, v_mean)

    def _save_debug_frame(self, img_bgr: np.ndarray, ts_ns: int, angle_deg: float, width_deg: float,
                          label: str, hsv, age_ms: float):
        """
        Save a debug PNG into debug_save_dir with a helpful filename.
        Auto-stops saving after reaching debug_max_previews.
        """
        try:
            fname = f"patch_{ts_ns}_ang{angle_deg:.1f}_w{width_deg:.1f}_{label}"
            if hsv is not None:
                h, s, v = hsv
                fname += f"_H{h:.1f}_S{s:.1f}_V{v:.1f}"
            fname += f"_d{age_ms:+.1f}ms.png"
            path = os.path.join(self.debug_save_dir, fname)
            cv2.imwrite(path, img_bgr)
            self._debug_saved += 1
        except Exception:
            # ignore saving errors
            pass

    # ---------- background thread ------------
    def _run(self):
        try:
            while not self._stop.is_set():
                # wait up to 1s for camera events
                self.poller.poll(1000)

                for req in self.cm.get_ready_requests():
                    if self._stop.is_set():
                        break

                    md = req.metadata
                    ts  = md.get(lc.controls.SensorTimestamp)   # ns
                    # exp = md.get(lc.controls.ExposureTime)    # (unused here)

                    fb = req.buffers[self.stream]
                    planes = self.fb_maps[fb]
                    y_plane, u_plane, v_plane = planes[0], planes[1], planes[2]

                    # center square crop indices
                    side = self.H_full
                    x0 = max(0, (self.W_full - side) // 2)

                    # Build only square Y, U, V (no full-image conversion)
                    Ysq, Usq, Vsq = _square_crop_from_yuv_planes(
                        self.W_full, self.H_full, x0, side, y_plane, u_plane, v_plane
                    )

                    # buffer the planes
                    if ts is not None:
                        with self._buf_lock:
                            self.frame_buffer.append((ts, Ysq, Usq, Vsq))
                            self.ts_hist.append(ts)

                    # preview overlay (optional); reconstruct once for UI
                    if self.preview_on:
                        overlay = _square_bgr_from_yuv(Ysq, Usq, Vsq)

                        # ring + live patch
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

                        # minimal HUD: FPS
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

                    # requeue request
                    req.reuse()
                    self.cam.queue_request(req)

        except Exception:
            # Don't let the thread die silently in production; print/log if desired.
            pass

# ==========================================================
#                        Test Main
# ==========================================================
def main():
    # Desktop live preview off; we demonstrate headless patch-debug saving.
    PREVIEW_LIVE = False

    ANGLE_DEG = 180.0
    WIDTH_DEG = 10.0

    # Save at most 5 debug patch frames to /tmp/cammanager_debug
    mgr = CamManager(preview_on=PREVIEW_LIVE,
                     patch_angle_deg=ANGLE_DEG,
                     patch_width_deg=WIDTH_DEG,
                     debug_save_dir="/home/carl/tmp/cammanager_debug",
                     debug_max_previews=10)
    try:
        while True:
            # Request a slightly old frame for testing
            target_ts = now_boottime_ns() - int(200 * 1e6)  # 200 ms ago
            label, used_ts, age_ms, h, s, v = mgr.analyze_patch_at_time(
                target_ts, ANGLE_DEG, WIDTH_DEG, preview=False  # will save until cap reached
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
