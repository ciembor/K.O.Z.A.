import select
import sys
import termios
import time
import tty

from . import hardware
from .audio import AudioPlayer
from .modes import ModeConfig, ModeManager


class App:
  def __init__(self):
    if hardware.GPIO is None or hardware.SMBus is None:
      raise RuntimeError("GPIO/SMBus not available; run with --development")
    hardware.GPIO.setmode(hardware.GPIO.BOARD)

    self.light = hardware.Relay(12)
    self.amp_relay = hardware.Relay(16)
    self.rotary = hardware.EightPositionRotarySwitch(
      [29, 31, 33, 35, 37, 38, 40]
    )
    self.amp_switch = hardware.AmpSwitch(22)

    config = 0x4000 | 0x0200 | 0x0080 | 0x0003
    self.volume_sensor = hardware.ADS1115VolumeSensor(3, 0x48, 0x00, 0x01, config)
    self.volume_controller = hardware.VolumeController()

    self.player = AudioPlayer(on_event=self._on_audio_event)
    self.mode_config = ModeConfig("config/modes.yaml")
    self.mode_manager = ModeManager(self.player, self.mode_config)

    self.last_volume = None
    self.last_audio_state = None

  def _on_audio_event(self, info):
    self.last_audio_state = info
    state = info.get("state")
    if state == "buffering":
      percent = info.get("percent")
      print(f"radio buffering {percent}%")
    elif state == "error":
      print("radio error")
    else:
      print(f"radio {state}")

  def update(self):
    self.rotary.check_change()
    self.amp_switch.check_change()

    mode = self.rotary.last_position
    self.mode_manager.apply_mode(mode)

    if self.amp_switch.last_state == 0:
      self.amp_relay.off()
    else:
      self.amp_relay.on()

    if mode == 1:
      self.light.off()
    else:
      self.light.on()

    v = self.volume_sensor.read_voltage()
    if self.last_volume is None or abs(v - self.last_volume) > 0.01:
      self.last_volume = v
      self.volume_controller.apply(v)

  def run(self):
    try:
      while True:
        self.update()
        time.sleep(0.05)
    finally:
      hardware.GPIO.cleanup()
      self.player.stop()


class DevelopmentApp:
  def __init__(self):
    self.player = AudioPlayer(
      on_event=self._on_audio_event,
      debug_no_signal=True
    )
    self.mode_config = ModeConfig("config/modes.yaml")
    self.mode_manager = ModeManager(self.player, self.mode_config)
    self.last_audio_state = None

  def _on_audio_event(self, info):
    self.last_audio_state = info
    state = info.get("state")
    if state == "buffering":
      percent = info.get("percent")
      print(f"radio buffering {percent}%")
    elif state == "error":
      print("radio error")
    else:
      print(f"radio {state}")

  def run(self):
    print("Development mode: press 1-8 to switch mode, or 'q' to quit.")
    try:
      fd = sys.stdin.fileno()
      old_settings = termios.tcgetattr(fd)
      try:
        tty.setcbreak(fd)
        while True:
          rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
          if not rlist:
            continue
          ch = sys.stdin.read(1)
          if ch in {"q", "Q"}:
            break
          if ch.isdigit():
            mode = int(ch)
            if mode == 0:
              continue
            self.mode_manager.apply_mode(mode)
      finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    except (EOFError, KeyboardInterrupt):
      pass
    finally:
      self.player.stop()
