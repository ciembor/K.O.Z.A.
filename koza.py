import RPi.GPIO as GPIO
import time
import subprocess
import os
from smbus import SMBus
from pyradios import RadioBrowser
import yaml


class ADS1115VolumeSensor:
  def __init__(self, bus_id, address, reg_conv, reg_config, config_value):
    self.i2c = SMBus(bus_id)
    self.address = address
    self.reg_conv = reg_conv
    self.reg_config = reg_config
    self.i2c.write_word_data(
      self.address,
      self.reg_config,
      ((config_value >> 8) & 0xFF) | ((config_value & 0xFF) << 8)
    )

  def read_voltage(self):
    raw = self.i2c.read_word_data(self.address, self.reg_conv)
    raw = ((raw & 0xFF) << 8) | (raw >> 8)
    return raw * 4.096 / 32768.0


class VolumeController:
  def __init__(self, max_voltage=3.31):
    self.max_voltage = max_voltage
    self.last_percent = None

  def voltage_to_percent(self, v):
    p = int((v / self.max_voltage) * 100)
    if p < 0:
      p = 0
    if p > 100:
      p = 100
    return p

  def apply(self, voltage):
    percent = self.voltage_to_percent(voltage)
    if self.last_percent is None or percent != self.last_percent:
      self.last_percent = percent
      os.system(f"amixer -c 0 set PCM {percent}% >/dev/null 2>/dev/null")


class AudioPlayer:
  def __init__(self):
    self.process = None
    self.rb = RadioBrowser()

  def stop(self):
    if self.process is not None:
      self.process.kill()
      self.process = None

  def play_loop(self, path):
    self.stop()
    self.process = subprocess.Popen(
      ["ffplay", "-nodisp", "-autoexit", "-loop", "0", path],
      stdout=subprocess.DEVNULL,
      stderr=subprocess.DEVNULL
    )

  def play_radio(self, name):
    self.stop()
    stations = self.rb.search(name=name, limit=20)
    if not stations:
      return
    s = stations[0]
    url = s.get("url_resolved") or s.get("url")
    if not url:
      return
    self.process = subprocess.Popen(
      ["ffplay", "-nodisp", "-autoexit", url],
      stdout=subprocess.DEVNULL,
      stderr=subprocess.DEVNULL
    )


class Relay:
  def __init__(self, pin):
    self.pin = pin
    self.state = False
    GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.HIGH)

  def on(self):
    self.state = True
    GPIO.output(self.pin, GPIO.LOW)

  def off(self):
    self.state = False
    GPIO.output(self.pin, GPIO.HIGH)

  def toggle(self):
    self.state = not self.state
    if self.state:
      self.on()
    else:
      self.off()

  def __str__(self):
    return "on" if self.state else "off"


class EightPositionRotarySwitch:
  def __init__(self, pins, samples=3, interval=0.003):
    self.pins = pins
    self.samples = samples
    self.interval = interval
    GPIO.setup(self.pins, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    self.last_position = self.read_position()

  def _sample_bits(self):
    acc = [0] * len(self.pins)
    for _ in range(self.samples):
      for i, p in enumerate(self.pins):
        acc[i] += 1 if GPIO.input(p) == GPIO.LOW else 0
      time.sleep(self.interval)
    return [1 if a >= (self.samples // 2 + 1) else 0 for a in acc]

  def read_position(self):
    bits = self._sample_bits()
    active = sum(bits)
    if active == 0:
      return 1
    if active == 1:
      return bits.index(1) + 2
    return self.last_position

  def check_change(self):
    pos = self.read_position()
    if pos != self.last_position:
      self.last_position = pos
      print(f"mode {pos}")


class AmpSwitch:
  def __init__(self, pin):
    self.pin = pin
    GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    self.last_state = 1 - GPIO.input(self.pin)

  def check_change(self):
    raw = GPIO.input(self.pin)
    s = 1 - raw
    if s != self.last_state:
      self.last_state = s
      print(f"amp switch {s}")


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


class App:
  def __init__(self):
    GPIO.setmode(GPIO.BOARD)

    self.light = Relay(12)
    self.amp_relay = Relay(16)
    self.rotary = EightPositionRotarySwitch([29, 31, 33, 35, 37, 38, 40])
    self.amp_switch = AmpSwitch(22)

    config = 0x4000 | 0x0200 | 0x0080 | 0x0003
    self.volume_sensor = ADS1115VolumeSensor(3, 0x48, 0x00, 0x01, config)
    self.volume_controller = VolumeController()

    self.player = AudioPlayer()
    self.mode_config = ModeConfig("config/modes.yaml")
    self.mode_manager = ModeManager(self.player, self.mode_config)

    self.last_volume = None

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
      GPIO.cleanup()
      self.player.stop()


if __name__ == "__main__":
  App().run()

