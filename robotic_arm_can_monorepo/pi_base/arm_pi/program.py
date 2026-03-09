"""
G-code program parser and executor for the 6-DOF arm.

Format:
    G1 J<joint_id> A<angle_deg> [S<speed_mdeg_s>]
    DWELL <milliseconds>
    ; comment lines

Example:
    G1 J1 A90 S60000
    G1 J2 A45
    DWELL 500
    G1 J2 A-45
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from typing import Callable, List, Optional


# Per-joint limits (degrees)
JOINT_LIMITS = {
    1: (None, None),      # unlimited
    2: (-90.0, 90.0),     # -90 to +90
    3: (None, None),
    4: (None, None),
    5: (None, None),
    6: (None, None),
}


@dataclass
class Instruction:
    """One instruction in a program."""
    type: str  # "move", "dwell"
    joint_id: Optional[int] = None
    angle: Optional[float] = None
    speed: Optional[int] = None  # mdeg/s, 0 = default
    dwell_ms: Optional[int] = None


class GCodeProgram:
    """Parse and execute a G-code program."""

    def __init__(self, text: str):
        self.instructions: List[Instruction] = []
        self.parse(text)

    def parse(self, text: str) -> None:
        """Parse G-code text into instructions."""
        for line_num, line in enumerate(text.split("\n"), 1):
            line = line.strip()

            # Skip empty lines and comments
            if not line or line.startswith(";"):
                continue

            # Parse G1 (move)
            if line.upper().startswith("G1"):
                m = re.match(
                    r"G1\s+J(\d+)\s+A([\d.\-]+)(?:\s+S(\d+))?",
                    line,
                    re.IGNORECASE
                )
                if not m:
                    raise ValueError(f"Line {line_num}: invalid G1 format: {line}")

                jid = int(m.group(1))
                angle = float(m.group(2))
                speed = int(m.group(3)) if m.group(3) else 0

                # Check limits
                lo, hi = JOINT_LIMITS.get(jid, (None, None))
                if lo is not None and angle < lo:
                    raise ValueError(
                        f"Line {line_num}: Joint {jid} angle {angle}° below limit {lo}°"
                    )
                if hi is not None and angle > hi:
                    raise ValueError(
                        f"Line {line_num}: Joint {jid} angle {angle}° above limit {hi}°"
                    )

                self.instructions.append(
                    Instruction(type="move", joint_id=jid, angle=angle, speed=speed)
                )

            # Parse DWELL
            elif line.upper().startswith("DWELL"):
                m = re.match(r"DWELL\s+(\d+)", line, re.IGNORECASE)
                if not m:
                    raise ValueError(f"Line {line_num}: invalid DWELL format: {line}")
                ms = int(m.group(1))
                self.instructions.append(Instruction(type="dwell", dwell_ms=ms))

            else:
                raise ValueError(f"Line {line_num}: unknown command: {line}")

    def __len__(self) -> int:
        return len(self.instructions)

    def __getitem__(self, idx: int) -> Instruction:
        return self.instructions[idx]


class ProgramRunner:
    """Execute a program with callbacks for moves and status."""

    def __init__(
        self,
        program: GCodeProgram,
        on_move: Optional[Callable[[int, float, int], None]] = None,
        on_status: Optional[Callable[[str], None]] = None,
    ):
        self.program = program
        self.on_move = on_move
        self.on_status = on_status
        self.current_step = 0
        self.running = False
        self.paused = False

    def _log(self, msg: str) -> None:
        if self.on_status:
            self.on_status(msg)

    def run(self) -> None:
        """Execute the program (blocking)."""
        self.running = True
        self.paused = False
        self.current_step = 0

        try:
            for idx, instr in enumerate(self.program.instructions):
                if not self.running:
                    self._log("Program stopped")
                    break

                while self.paused and self.running:
                    time.sleep(0.05)

                self.current_step = idx

                if instr.type == "move":
                    self._log(
                        f"[{idx+1}/{len(self.program)}] Move J{instr.joint_id} "
                        f"to {instr.angle}° @ {instr.speed} mdeg/s"
                    )
                    if self.on_move:
                        self.on_move(instr.joint_id, instr.angle, instr.speed)
                    # Small delay to let the move start
                    time.sleep(0.1)

                elif instr.type == "dwell":
                    self._log(f"[{idx+1}/{len(self.program)}] Dwell {instr.dwell_ms} ms")
                    start = time.time()
                    while (time.time() - start) * 1000 < instr.dwell_ms and self.running:
                        if not self.paused:
                            time.sleep(0.01)
                        else:
                            time.sleep(0.05)

            self._log("Program complete")
        finally:
            self.running = False

    def pause(self) -> None:
        self.paused = True
        self._log("Program paused")

    def resume(self) -> None:
        self.paused = False
        self._log("Program resumed")

    def stop(self) -> None:
        self.running = False
        self._log("Program stopped")
