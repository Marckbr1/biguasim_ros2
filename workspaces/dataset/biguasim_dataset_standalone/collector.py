from __future__ import annotations

# collector.py
from pathlib import Path

import numpy as np


def _same_value(a, b) -> bool:
    """True if `a` and `b` are the same sensor reading (used for the
    real-Hz dedup below -- see Collector.save)."""
    try:
        return bool(np.array_equal(a, b))
    except Exception:
        return a == b


class Collector:
    def __init__(self, output_dir:str, dedupe_repeated_samples:bool = True):
        self.output_dir = Path(output_dir)
        self.writers = []
        # BUG FOUND EMPIRICALLY (not in the original audit, which assumed
        # BiguaSim only puts a sensor's key in `state` on ticks where it
        # actually produced new data -- section E/K: "state is a dict...
        # only the keys of the sensors that 'fired' on that tick appear"). Verified against the real engine (biguasim v1.0.0,
        # this session, see docs/synchronization.md): a sub-Hz sensor's key
        # is present in `state` on EVERY tick, but its value is only
        # recomputed every `ticks_per_sec/Hz` ticks -- in between it's the
        # same array repeated. Without this dedup, a sensor declared at
        # Hz=5 with ticks_per_sec=30 was written 6x more often than its
        # real rate (an ImagingSonar run wrote a full 394x768x4-channel
        # frame on every single tick, ~6x the disk it needed). Comparing by
        # VALUE (not by a computed tick-modulo, since the phase offset
        # turned out not to be 0 -- see docs/synchronization.md) is what
        # actually fixes this, for any sensor/Hz combination.
        self.dedupe_repeated_samples = dedupe_repeated_samples
        self._last_value = {}

    def register_writer(self, writer):
        self.writers.append(writer)

    def save(self,
             frame_id:int,
             tick:int,
             timestamp:float,
             state:dict
    ):
        """Save the state data using all registered writers.

        timestamp MUST be simulation time (state['t'] = num_ticks /
        ticks_per_sec), never wall-clock time -- see
        docs/synchronization.md. Each writer is skipped for this call if
        its sensor is absent from `state`, and (see
        `dedupe_repeated_samples` above) also skipped if BiguaSim handed
        back byte-for-byte the same reading it returned on a previous
        tick -- that repetition is how a sub-Hz sensor's real rate shows
        up in `state`, not a sparse key.
        """

        for writer in self.writers:
            if writer.sensor_cfg.name not in state:
                continue

            data = state[writer.sensor_cfg.name]

            if writer.writer_cfg.field is not None:
                data = data[writer.writer_cfg.field]

            if self.dedupe_repeated_samples and getattr(writer.writer_cfg, "dedupe", True):
                key = id(writer)
                last = self._last_value.get(key)
                if last is not None and _same_value(last, data):
                    continue
                self._last_value[key] = np.copy(data) if isinstance(data, np.ndarray) else data

            writer.write(
                frame_id,
                tick,
                timestamp,
                data
            )

    def close(self):
        """Close all writers."""
        for writer in self.writers:
            if hasattr(writer, 'close'):
                writer.close()