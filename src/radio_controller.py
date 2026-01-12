import threading

from .audio_player import MpvPlayer
from pyradios import RadioBrowser


class RadioController:
  def __init__(self, player, on_event=None, debug=False):
    self.player = player
    self.on_event = on_event
    self.debug = debug
    self.rb = RadioBrowser()
    self.radio_active = False
    self.radio_name = None
    self.playing_confirmed = False
    self.buffering_percent = None
    self.cache_paused = False
    self.first_play = False
    self.confirm_timer = None
    self.load_timeout_timer = None
    self.retry_delay = 5.0
    self.load_timeout = 30.0
    self.retry_event = threading.Event()
    self.stop_event = threading.Event()
    self.retry_thread = None

    self.player.on_event("start-file", self._on_start)
    self.player.on_event("file-loaded", self._on_loaded)
    self.player.on_event("end-file", self._on_end)
    self.player.on_property("paused-for-cache", self._on_cache_paused)
    self.player.on_property("cache-buffering-state", self._on_cache)
    self.player.on_property("playback-time", self._on_playback_time)

  def start(self, name):
    self.stop()
    self.radio_active = True
    self.radio_name = name
    self._start_retry_thread()
    self._trigger_retry(0.0)

  def stop(self):
    self.radio_active = False
    self.radio_name = None
    self._cancel_confirm_timer()
    self._cancel_load_timeout()
    self._stop_retry_thread()
    self.player.cmd("stop")

  def _emit_event(self, label, extra=None):
    data = {"state": label}
    if extra:
      data.update(extra)
    if self.on_event:
      self.on_event(data)
    if self.debug:
      print(
        "mpv state="
        f"{label} cache={self.buffering_percent} paused_cache={self.cache_paused} "
        f"playing_confirmed={self.playing_confirmed}"
      )

  def _on_start(self, _event):
    if not self.radio_active:
      return
    self._reset_state()
    self.first_play = True
    self._emit_event("opening")

  def _on_loaded(self, _event):
    if not self.radio_active:
      return
    if self.first_play:
      self._emit_event("loaded")

  def _on_end(self, event):
    if not self.radio_active:
      return
    self._emit_event("stopped", {"reason": getattr(event, "reason", None)})
    self._trigger_retry(self.retry_delay)

  def _on_cache_paused(self, _name, value):
    if not self.radio_active:
      return
    self.cache_paused = bool(value)
    if self.cache_paused:
      self._emit_event("buffering", {"paused_for_cache": True})

  def _on_cache(self, _name, value):
    if not self.radio_active:
      return
    if value is None:
      return
    raw = float(value)
    if raw > 100.0:
      raw = raw / 100.0
    self.buffering_percent = max(0.0, min(raw, 100.0))
    self._emit_event("buffering", {"percent": self.buffering_percent})

  def _on_playback_time(self, _name, value):
    if not self.radio_active:
      return
    if value is None:
      return
    if self.cache_paused:
      return
    if self.playing_confirmed:
      return
    if self.buffering_percent is not None and self.buffering_percent < 100.0:
      return
    if value >= 0.2:
      self.playing_confirmed = True
      self._emit_event("playing")
      self.first_play = False

  def _reset_state(self):
    self.playing_confirmed = False
    self.buffering_percent = None
    self._cancel_confirm_timer()
    self._cancel_load_timeout()

  def _schedule_confirm_check(self):
    if self.confirm_timer is not None:
      return
    def _check():
      if self.radio_active and not self.playing_confirmed:
        if self._has_audio_started():
          self.playing_confirmed = True
      self.confirm_timer = None
    self.confirm_timer = threading.Timer(0.2, _check)
    self.confirm_timer.daemon = True
    self.confirm_timer.start()

  def _has_audio_started(self):
    if self.player.get("pause"):
      return False
    t = self.player.get("time-pos")
    return t is not None and t > 0.2

  def _cancel_confirm_timer(self):
    if self.confirm_timer is None:
      return
    self.confirm_timer.cancel()
    self.confirm_timer = None

  def _schedule_load_timeout(self):
    self._cancel_load_timeout()
    def _timeout():
      if self.radio_active and not self.playing_confirmed:
        self._trigger_retry(self.retry_delay)
    self.load_timeout_timer = threading.Timer(self.load_timeout, _timeout)
    self.load_timeout_timer.daemon = True
    self.load_timeout_timer.start()

  def _cancel_load_timeout(self):
    if self.load_timeout_timer is None:
      return
    self.load_timeout_timer.cancel()
    self.load_timeout_timer = None

  def _start_retry_thread(self):
    if self.retry_thread is not None and self.retry_thread.is_alive():
      return
    self.stop_event.clear()
    self.retry_thread = threading.Thread(
      target=self._retry_loop,
      daemon=True
    )
    self.retry_thread.start()

  def _stop_retry_thread(self):
    if self.retry_thread is None:
      return
    self.stop_event.set()
    self.retry_event.set()
    self.retry_thread = None

  def _trigger_retry(self, delay):
    if not self.radio_active:
      return
    if delay <= 0:
      self.retry_event.set()
      return
    def _fire():
      if self.radio_active:
        self.retry_event.set()
    t = threading.Timer(delay, _fire)
    t.daemon = True
    t.start()

  def _retry_loop(self):
    while not self.stop_event.is_set():
      self.retry_event.wait()
      if self.stop_event.is_set():
        break
      self.retry_event.clear()
      if not self.radio_active or not self.radio_name:
        continue
      self._attempt_start()

  def _attempt_start(self):
    self._reset_state()
    self.player.cmd("stop")
    try:
      stations = self.rb.search(name=self.radio_name, limit=20)
    except Exception as exc:
      self._emit_event("error", {"error": str(exc)})
      self._trigger_retry(self.retry_delay)
      return
    if not stations:
      self._trigger_retry(self.retry_delay)
      return
    if self.debug:
      self._log_radio_stations(stations)
    s = stations[0]
    url = s.get("url_resolved") or s.get("url")
    if not url:
      self._trigger_retry(self.retry_delay)
      return
    self.player.cmd("loadfile", url, "replace")
    self._schedule_load_timeout()

  def _log_radio_stations(self, stations):
    lines = []
    for i, s in enumerate(stations, start=1):
      name = s.get("name") or "?"
      br = s.get("bitrate")
      codec = s.get("codec")
      url = s.get("url_resolved") or s.get("url") or ""
      lines.append(
        f"{i:02d}. {name} bitrate={br} codec={codec} url={url}"
      )
    print("radio stations:")
    for line in lines:
      print(line)
