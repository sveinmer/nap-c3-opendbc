"""NAP debug-logger for VDAS + carcontroller state per tick.

Lett-vekt JSONL-skriver kontrollert av `NAPDebugLog` Params-key. Aktivert
via param-toggle; ingen overhead når disabled (singleton no-op).

Bruk fra carcontroller:
    from opendbc.car.tesla.preap.debug_log import nap_debug_log
    nap_debug_log.write({
        't': frame * 0.01,
        'v_ego': v_ego,
        'a_target': a_target,
        ...
    })

Filen ligger på `/data/nap_debug.jsonl`. Roteres til `.jsonl.prev` ved
SIZE_CAP_BYTES (default 50 MB). Throttlet til 5 Hz (skriver hvert 10.
kall ved 50 Hz tick).

Param-state caches i 200 ticks (~2 s) for å unngå Params.get_bool på
hver tick.
"""
import json
import os
import time

LOG_PATH = "/data/nap_debug.jsonl"
LOG_PATH_PREV = "/data/nap_debug.jsonl.prev"
SIZE_CAP_BYTES = 50 * 1024 * 1024
WRITE_EVERY_N = 10        # 5 Hz at 50 Hz tick
PARAM_RECHECK_TICKS = 200  # ~2 s


class DebugLogger:
  """Singleton — instansieres én gang ved import."""

  def __init__(self):
    self._enabled = False
    self._fp = None
    self._call_count = 0
    self._last_param_check = -1
    self._size_check_count = 0

  def _check_param(self):
    """Sjekk param hver ~2 s. Trygt mot Params-import-feil
    (test-miljø uten openpilot Params)."""
    try:
      from openpilot.common.params import Params
      self._enabled = Params().get_bool("NAPDebugLog")
    except Exception:
      self._enabled = False

    if not self._enabled and self._fp is not None:
      try:
        self._fp.close()
      except Exception:
        pass
      self._fp = None

  def _open_lazy(self):
    if self._fp is not None:
      return
    try:
      # Rotér hvis eksisterende fil over cap
      if os.path.exists(LOG_PATH):
        if os.path.getsize(LOG_PATH) > SIZE_CAP_BYTES:
          try:
            os.replace(LOG_PATH, LOG_PATH_PREV)
          except OSError:
            pass
      self._fp = open(LOG_PATH, 'a', buffering=1)  # line-buffered
      # Marker boot-grense slik at post-drive grep kan splitte sessions
      self._fp.write(json.dumps({'t': time.time(), 'event': 'session_start'}) + '\n')
    except OSError:
      self._enabled = False
      self._fp = None

  def write(self, record: dict):
    """Skriv én JSON-linje hvis enabled. Throttlet til WRITE_EVERY_N kall."""
    self._call_count += 1

    # Param-recheck ~2 s
    if self._call_count - self._last_param_check >= PARAM_RECHECK_TICKS:
      self._check_param()
      self._last_param_check = self._call_count

    if not self._enabled:
      return

    if self._call_count % WRITE_EVERY_N != 0:
      return

    if self._fp is None:
      self._open_lazy()
      if self._fp is None:
        return

    # Size-cap periodic check (cheap, hver 100 writes)
    self._size_check_count += 1
    if self._size_check_count % 100 == 0:
      try:
        if os.fstat(self._fp.fileno()).st_size > SIZE_CAP_BYTES:
          self._fp.close()
          os.replace(LOG_PATH, LOG_PATH_PREV)
          self._fp = open(LOG_PATH, 'a', buffering=1)
      except OSError:
        pass

    try:
      self._fp.write(json.dumps(record, default=float) + '\n')
    except (OSError, TypeError, ValueError):
      pass  # never let logging crash the controller

  def close(self):
    if self._fp is not None:
      try:
        self._fp.close()
      except Exception:
        pass
      self._fp = None


# Singleton — én per prosess
nap_debug_log = DebugLogger()
