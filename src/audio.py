from .audio_player import MpvPlayer
from .no_signal import NoSignalPlayer
from .radio_controller import RadioController


class AudioPlayer:
  def __init__(self, on_event=None, debug_no_signal=False):
    self.on_event = on_event
    self.debug_no_signal = debug_no_signal
    self.player = MpvPlayer(debug=debug_no_signal)
    self.no_signal = NoSignalPlayer(on_state=self._on_no_signal_state, debug=debug_no_signal)
    self.radio = RadioController(self.player, on_event=self._on_radio_event, debug=debug_no_signal)
    self.radio_active = False

  def stop(self):
    self.set_radio_active(False)
    self.player.cmd("stop")
    self.no_signal.stop()

  def play_loop(self, path):
    self.set_radio_active(False)
    self.no_signal.stop()
    self.stop()
    self.player.cmd("loadfile", path, "replace")
    self.player.set("loop_file", "inf")

  def play_radio(self, name):
    self.stop()
    self.set_radio_active(True)
    self.radio.start(name)

  def set_radio_active(self, active):
    self.radio_active = active
    if not active:
      self.radio.stop()
      self.no_signal.stop()

  def _on_radio_event(self, info):
    state = info.get("state")
    if self.radio_active:
      if state in {"opening", "buffering", "stopped", "error"}:
        self.no_signal.start()
      if state == "playing":
        self.no_signal.fade_out(0.6)
    if self.on_event:
      self.on_event(info)

  def _on_no_signal_state(self, active):
    if self.debug_no_signal:
      print(f"no_signal active={active}")
