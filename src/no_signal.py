import os
import random
import threading
import time

from .audio_player import MpvPlayer


class NoSignalPlayer:
  def __init__(self, on_state=None, debug=False):
    self.on_state = on_state
    self.debug = debug
    self.player = MpvPlayer(debug=debug)
    self.path = os.path.normpath(
      os.path.join(
        os.path.dirname(__file__),
        "..",
        "audio",
        "no_signal.flac"
      )
    )
    self.active = False
    self.stop_requested = False
    self.pending_offset = None
    self.fade_cancel = threading.Event()
    self.fading = False
    self.last_log = 0.0
    self.log_interval = 1.0

    self.player.on_event("file-loaded", self._on_loaded)
    self.player.on_event("end-file", self._on_end)
    self.player.on_property("playback-time", self._on_playback_time)

  def start(self):
    if not os.path.exists(self.path):
      return
    if self.active:
      return
    self.stop_requested = False
    if self.fading:
      self.fade_cancel.set()
      self.fading = False
    self.pending_offset = self._random_offset()
    self.player.set("pause", True)
    self.player.cmd("loadfile", self.path, "replace")
    self.player.set("loop_file", "inf")
    self.player.set("volume", 100)
    self.player.set("mute", False)
    self.active = True
    if self.debug:
      print(f"no_signal start offset={self.pending_offset}")
      self._log_state("start")
    if self.on_state:
      self.on_state(True)

  def stop(self):
    self.stop_requested = True
    self.pending_offset = None
    self.fade_cancel.set()
    self.player.cmd("stop")
    self.active = False
    if self.on_state:
      self.on_state(False)

  def fade_out(self, duration):
    if not self.active:
      self.stop()
      return
    if self.fading:
      return
    self.fading = True
    self.fade_cancel.clear()
    try:
      start_vol = float(self.player.get("volume") or 100.0)
    except Exception:
      start_vol = 100.0
    steps = max(1, int(duration / 0.1))
    step_time = duration / steps
    def _fade():
      for i in range(steps):
        if not self.active or self.fade_cancel.is_set():
          break
        vol = start_vol * (1.0 - (i + 1) / steps)
        if not self.player.set("volume", max(0.0, vol)):
          break
        time.sleep(step_time)
      self.stop()
      self.fading = False
    threading.Thread(target=_fade, daemon=True).start()

  def _on_loaded(self, _event):
    if not self.active:
      return
    if self.pending_offset is not None:
      self.player.cmd(
        "seek",
        float(self.pending_offset),
        "absolute",
        "exact"
      )
      self.pending_offset = None
    self.player.set("pause", False)

  def _on_end(self, _event):
    if self.stop_requested:
      self.stop_requested = False
      return
    if self.active:
      self.start()

  def _on_playback_time(self, _name, _value):
    if not self.active or not self.debug:
      return
    now = time.monotonic()
    if now - self.last_log < self.log_interval:
      return
    self.last_log = now
    self._log_state("tick")

  def _log_state(self, tag):
    ns_time = self.player.get("time-pos")
    ns_len = self.player.get("duration")
    ns_pos = self.player.get("percent-pos")
    ns_path = self.player.get("path")
    ns_idle = self.player.get("core-idle")
    ns_pause = self.player.get("pause")
    ns_mute = self.player.get("mute")
    ns_vol = self.player.get("volume")
    print(
      "no_signal "
      f"{tag} time={ns_time} len={ns_len} pos={ns_pos} "
      f"path={ns_path} idle={ns_idle} pause={ns_pause} "
      f"mute={ns_mute} vol={ns_vol}"
    )

  def _random_offset(self):
    duration = self.player.get("duration")
    if duration and duration > 1:
      max_start = max(0.0, float(duration) - 1.0)
      if max_start > 0:
        return random.uniform(0.0, max_start)
    return random.uniform(0.0, 30.0)
