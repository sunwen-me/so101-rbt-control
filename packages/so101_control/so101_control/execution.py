"""Absolute-deadline trajectory publication through a supplied arm client."""
import math
import time


class JointDispatch:
    """Iterate published joint targets with the existing runner timing.

    target_at(index) may solve IK lazily (relative mode) or select a prepared
    interpolation point (absolute mode). No model or audit dependency.
    """
    def __init__(self, target_at, count, period_s, *, client=None,
                 stop_requested=None, clock=time, absolute=False):
        self.target_at, self.count, self.period_s = target_at, count, period_s
        self.client, self.clock, self.absolute = client, clock, absolute
        self.stop_requested = stop_requested or (lambda: False)
        self.skipped_samples = 0
        self.intervals_ms = []
        self.elapsed_s = 0.0

    def __iter__(self):
        clock, client = self.clock, self.client
        send = client is not None and client.send
        start = clock.monotonic()
        index = 0
        previous_publish = None
        while index < self.count:
            if self.stop_requested():
                break
            tick_start = clock.monotonic()
            if send and self.absolute:
                due = max(index, min(self.count - 1, math.floor((clock.monotonic() - start) / self.period_s)))
                self.skipped_samples += due - index
                index = due
                clock.sleep(max(0.0, start + index * self.period_s - clock.monotonic()))
                if self.stop_requested():
                    break
                due = max(index, min(self.count - 1, math.floor((clock.monotonic() - start) / self.period_s)))
                self.skipped_samples += due - index
                index = due
            target = self.target_at(index)
            if self.stop_requested():
                break
            row = dict(index=index, target=target, published=False, sent=False)
            if send:
                published_ns = clock.monotonic_ns() if self.absolute else None
                publication = client.send_target(target)
                row.update(published=True, command_sequence=publication["command_sequence"],
                           driver_status=publication["status"])
                if self.absolute:
                    interval = None if previous_publish is None else (published_ns - previous_publish) / 1e6
                    if interval is not None:
                        self.intervals_ms.append(interval)
                    previous_publish = published_ns
                    row.update(publish_call_ns=published_ns, publish_interval_ms=interval,
                               lateness_ms=max(0.0, published_ns / 1e9 - (start + index * self.period_s)) * 1000)
                    clock.sleep(max(0.0, start + (index + 1) * self.period_s - clock.monotonic()))
                else:
                    clock.sleep(max(0.0, self.period_s - (clock.monotonic() - tick_start)))
                measured = client.snapshot()
                row.update(measured=measured["arms"], measured_state_sequence=measured["sequence"],
                           measured_read_intervals=measured["read_intervals"],
                           observed_driver_command_sequence=measured.get("last_command_sequence"),
                           observed_driver_status=measured.get("last_command_status"))
                if not self.absolute:
                    elapsed_ms = (clock.monotonic() - tick_start) * 1000
                    row.update(tick_elapsed_ms=elapsed_ms,
                               tick_overrun_ms=max(0.0, elapsed_ms - self.period_s * 1000))
            yield row
            index += 1
        self.elapsed_s = clock.monotonic() - start

def execute_trajectory(client, trajectory, *, stop_requested=None, clock=time):
    """Publish native samples; client owns permission and driver connectivity.

    No torque, calibration, serial opening, model, logger or acknowledgement gate.
    Overdue intermediate samples are skipped instead of sent as a burst.
    """
    stop_requested = stop_requested or (lambda: False)
    if not client.send:
        raise PermissionError("client is read-only")
    start = clock.monotonic()
    index = sent = skipped = 0
    last_publication = None
    while index < len(trajectory):
        if stop_requested():
            break
        deadline = start + (index + 1) * trajectory.period_s
        clock.sleep(max(0.0, deadline - clock.monotonic()))
        if stop_requested():
            break
        due = max(index, min(len(trajectory) - 1,
                  math.floor((clock.monotonic() - start) / trajectory.period_s + 1e-9) - 1))
        skipped += due - index
        index = due
        last_publication = client.send_target(trajectory.samples[index])
        sent += 1
        index += 1
    return dict(published_count=sent, skipped_samples=skipped,
                completed=index == len(trajectory), elapsed_s=clock.monotonic() - start,
                last_publication=last_publication, confirmation="publication_only")
