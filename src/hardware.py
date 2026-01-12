import os
import time

try:
  import RPi.GPIO as GPIO
except ImportError:
  GPIO = None

try:
  from smbus import SMBus
except ImportError:
  SMBus = None


class ADS1115VolumeSensor:
  def __init__(self, bus_id, address, reg_conv, reg_config, config_value):
    if SMBus is None:
      raise RuntimeError("SMBus not available")
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


class Relay:
  def __init__(self, pin):
    if GPIO is None:
      raise RuntimeError("GPIO not available")
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
    if GPIO is None:
      raise RuntimeError("GPIO not available")
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
    if GPIO is None:
      raise RuntimeError("GPIO not available")
    self.pin = pin
    GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    self.last_state = 1 - GPIO.input(self.pin)

  def check_change(self):
    raw = GPIO.input(self.pin)
    s = 1 - raw
    if s != self.last_state:
      self.last_state = s
      print(f"amp switch {s}")
