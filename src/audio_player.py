import os

try:
  import mpv
except ImportError:
  mpv = None


class MpvPlayer:
  def __init__(self, debug=False):
    if mpv is None:
      raise RuntimeError("python-mpv not available")
    ao = os.environ.get("KOZA_AO")
    audio_device = os.environ.get("KOZA_AUDIO_DEVICE")
    mpv_kwargs = {
      "log_handler": None,
      "input_default_bindings": False,
      "input_vo_keyboard": False,
      "audio_exclusive": False,
      "vo": "null"
    }
    if ao:
      mpv_kwargs["ao"] = ao
    if audio_device:
      mpv_kwargs["audio_device"] = audio_device
    self.player = mpv.MPV(**mpv_kwargs)
    self.debug = debug

  def cmd(self, *args):
    try:
      self.player.command(*args)
      return True
    except Exception as exc:
      if self.debug:
        print(f"mpv command failed: {args} err={exc}")
      return False

  def set(self, name, value):
    try:
      setattr(self.player, name, value)
      return True
    except Exception as exc:
      if self.debug:
        print(f"mpv set failed: {name}={value} err={exc}")
      return False

  def get(self, name):
    try:
      return self.player.get_property(name)
    except Exception:
      return None

  def on_event(self, event_name, handler):
    self.player.event_callback(event_name)(handler)

  def on_property(self, prop, handler):
    self.player.observe_property(prop, handler)
