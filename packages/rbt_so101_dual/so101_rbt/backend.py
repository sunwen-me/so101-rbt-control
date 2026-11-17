"""Minimal Feetech extraction; see ../NOTICE.md for LeRobot attribution."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Protocol

from .calibration import ArmCalibration, MOTOR_NAMES, decode_sign_magnitude, encode_sign_magnitude


class ArmBackend(Protocol):
    def read_state(self) -> tuple[float, ...]: ...
    def prepare_target(self, target: tuple[float, ...]) -> object: ...
    def send_prepared(self, prepared: object) -> None: ...
    def close(self) -> None: ...


@dataclass
class MockBackend:
    arm: str
    state: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def read_state(self) -> tuple[float, ...]:
        return self.state

    def prepare_target(self, target: tuple[float, ...]) -> tuple[float, ...]:
        if len(target) != 6 or not all(math.isfinite(v) for v in target):
            raise ValueError("invalid mock target")
        return tuple(target)

    def send_prepared(self, prepared: tuple[float, ...]) -> None:
        self.state = prepared

    def close(self) -> None:
        pass


class FeetechSTS3215Backend:
    """LeRobot-style reads (two retries); never retry uncertain goal writes."""

    MODEL_NUMBER = 777
    BAUDRATE = 1_000_000
    DEFAULT_WRITE_TIMEOUT_S = None
    DEFAULT_READ_RETRIES = 2
    REGISTERS = {
        "Model_Number": (3, 2), "Return_Delay_Time": (7, 1),
        "Min_Position_Limit": (9, 2), "Max_Position_Limit": (11, 2),
        "Phase": (18, 1), "Homing_Offset": (31, 2), "Operating_Mode": (33, 1),
        "Torque_Enable": (40, 1), "Goal_Position": (42, 2), "Present_Position": (56, 2),
    }

    def __init__(self, port: str, calibration: ArmCalibration, *, allow_goal_writes: bool = False,
                 write_timeout_s: float | None = DEFAULT_WRITE_TIMEOUT_S,
                 num_read_retries: int = DEFAULT_READ_RETRIES) -> None:
        if write_timeout_s is not None and (
                isinstance(write_timeout_s, bool) or not isinstance(write_timeout_s, (int, float))
                or not math.isfinite(write_timeout_s) or write_timeout_s <= 0):
            raise ValueError("serial write_timeout_s must be None or positive and finite")
        if type(num_read_retries) is not int or num_read_retries < 0:
            raise ValueError("num_read_retries must be a nonnegative integer")
        import scservo_sdk as scs
        import serial

        self._scs = scs
        self.port = port
        self.calibration = calibration
        self.allow_goal_writes = allow_goal_writes
        self.write_timeout_s = write_timeout_s
        self.num_read_retries = num_read_retries
        self.port_handler = scs.PortHandler(port)
        self.packet_handler = scs.PacketHandler(0)
        self._closed = False
        self.configuration: dict[str, dict[str, int]] = {}
        try:
            ph = self.port_handler
            ph.baudrate = self.BAUDRATE
            # Match the original SDK open: no exclusive lock or added write deadline.
            ph.ser = serial.Serial(port=port, baudrate=self.BAUDRATE, bytesize=serial.EIGHTBITS,
                                   timeout=0, write_timeout=self.write_timeout_s)
            ph.is_open = True
            ph.tx_time_per_byte = 1000.0 / self.BAUDRATE * 10.0
            ph.ser.reset_input_buffer()
            ph.getCurrentTime = lambda: time.monotonic_ns() / 1_000_000
            def packet_timeout(packet_length: int) -> None:
                ph.packet_start_time = ph.getCurrentTime()
                ph.packet_timeout = ph.tx_time_per_byte * (packet_length + 3.0) + 50.0
            ph.setPacketTimeout = packet_timeout
            self._verify_inventory_and_calibration()
        except BaseException:
            self.close()
            raise

    def _read_register(self, motor_id: int, register: str) -> int:
        address, width = self.REGISTERS[register]
        read = self.packet_handler.read2ByteTxRx if width == 2 else self.packet_handler.read1ByteTxRx
        value, comm, error = read(self.port_handler, motor_id, address)
        if comm != self._scs.COMM_SUCCESS or error:
            raise ConnectionError(f"{self.port}: register {register}, motor {motor_id}, comm={comm}, error={error}")
        return decode_sign_magnitude(value, 11) if register == "Homing_Offset" else int(value)

    def _verify_inventory_and_calibration(self) -> None:
        for name in MOTOR_NAMES:
            cal = self.calibration.motors[name]
            registers = {"Model_Number": self._read_register(cal.motor_id, "Model_Number")}
            if registers["Model_Number"] != self.MODEL_NUMBER:
                raise RuntimeError(f"{self.port}/{name}: unexpected motor model {registers['Model_Number']}")
            self.configuration[name] = registers

    def read_raw(self) -> dict[str, int]:
        reader = self._scs.GroupSyncRead(self.port_handler, self.packet_handler, 56, 2)
        try:
            for name in MOTOR_NAMES:
                if not reader.addParam(self.calibration.motors[name].motor_id):
                    raise RuntimeError("failed to add synchronized read parameter")
            for attempt in range(1 + self.num_read_retries):
                comm = reader.txRxPacket()
                missing = [self.calibration.motors[name].motor_id for name in MOTOR_NAMES
                           if comm == self._scs.COMM_SUCCESS and not reader.isAvailable(
                               self.calibration.motors[name].motor_id, 56, 2)]
                if comm == self._scs.COMM_SUCCESS and not missing:
                    return {name: decode_sign_magnitude(reader.getData(
                        self.calibration.motors[name].motor_id, 56, 2), 15) for name in MOTOR_NAMES}
                if attempt == self.num_read_retries:
                    raise ConnectionError(f"{self.port}: synchronized read failed after {attempt+1} attempts, "
                                          f"comm={comm}, missing={missing}")
        finally:
            reader.clearParam()

    def read_state(self) -> tuple[float, ...]:
        return self.calibration.normalize_raw(self.read_raw())

    def prepare_target(self, target: tuple[float, ...]) -> tuple[tuple[int, int, int], ...]:
        if not self.allow_goal_writes:
            raise PermissionError("read-only backend: goal writes disabled")
        if len(target) != 6 or not all(math.isfinite(v) for v in target):
            raise ValueError("invalid target")
        raw = self.calibration.unnormalize(target)
        out = []
        for name in MOTOR_NAMES:
            cal = self.calibration.motors[name]
            # Match LeRobot DEGREES mode: calibration supplies the zero/midpoint,
            # not an additional host-side rejection of Goal_Position targets.
            # Existing servo Min/Max_Position_Limit registers remain unchanged.
            encoded = encode_sign_magnitude(raw[name], 15)
            out.append((cal.motor_id, encoded & 0xFF, encoded >> 8))
        return tuple(out)

    def send_prepared(self, prepared: tuple[tuple[int, int, int], ...]) -> None:
        if not self.allow_goal_writes:
            raise PermissionError("read-only backend: goal writes disabled")
        if len(prepared) != 6 or tuple(v[0] for v in prepared) != tuple(range(1, 7)):
            raise ValueError("invalid prepared command")
        writer = self._scs.GroupSyncWrite(self.port_handler, self.packet_handler, 42, 2)
        try:
            for motor_id, low, high in prepared:
                if not 0 <= low <= 255 or not 0 <= high <= 255:
                    raise ValueError("invalid prepared servo goal")
                if not writer.addParam(motor_id, [low, high]):
                    raise RuntimeError("failed to add synchronized write parameter")
            comm = writer.txPacket()
            if comm != self._scs.COMM_SUCCESS:
                raise ConnectionError(f"{self.port}: goal transmission failed/uncertain, comm={comm}")
        finally:
            writer.clearParam()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.port_handler.ser is not None:
            # Disconnect never sends torque-off; preserve the current physical hold.
            self.port_handler.closePort()
