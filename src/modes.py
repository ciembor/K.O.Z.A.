import yaml


class ModeConfig:
  def __init__(self, path):
    with open(path, "r", encoding="utf-8") as f:
      data = yaml.safe_load(f) or {}
    modes = data.get("modes", {})
    self.modes = {int(k): v for k, v in modes.items()}

  def get(self, mode):
    return self.modes.get(mode)


class ModeManager:
  def __init__(self, player, config):
    self.player = player
    self.config = config
    self.mode = None

  def apply_mode(self, mode):
    if mode == self.mode:
      return
    self.mode = mode

    entry = self.config.get(mode)
    if not entry:
      self.player.stop()
      return

    t = entry.get("type")
    v = entry.get("value")

    if t == "stop":
      self.player.stop()
      return
    if t == "file":
      self.player.play_loop(v)
      return
    if t == "radio":
      self.player.play_radio(v)
      return

    self.player.stop()
