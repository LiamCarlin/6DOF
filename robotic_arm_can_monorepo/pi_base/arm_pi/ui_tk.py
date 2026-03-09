"""
Tkinter GUI for controlling the 6-DOF robotic arm over CAN.

Each connected joint gets its own column with independent angle entry,
enable/disable/stop/zero/send buttons, and live telemetry.
Offline joints are greyed out.  Scales to 6 joints automatically.

Run:  python -m arm_pi.ui_tk
"""

from __future__ import annotations

import time
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass, field
from typing import Dict

from . import protocol as proto
from .can_bus import CANBus
from .node_client import ArmBusManager, JointNodeClient, NodeState, NUM_JOINTS

# How many joints to show in the UI (increase when you add more)
ACTIVE_JOINTS = [1, 2]


@dataclass
class JointWidgets:
    """All Tk widgets belonging to one joint column."""
    node_id: int
    # inputs
    angle_var: tk.StringVar = field(default_factory=lambda: tk.StringVar(value="0.0"))
    speed_var: tk.StringVar = field(default_factory=lambda: tk.StringVar(value="0"))
    # status labels
    lbl_status: ttk.Label = None  # type: ignore[assignment]
    lbl_angle: ttk.Label = None   # type: ignore[assignment]
    lbl_turns: ttk.Label = None   # type: ignore[assignment]
    lbl_target: ttk.Label = None  # type: ignore[assignment]
    lbl_flags: ttk.Label = None   # type: ignore[assignment]
    lbl_fault: ttk.Label = None   # type: ignore[assignment]
    lbl_uptime: ttk.Label = None  # type: ignore[assignment]


class ArmUI:
    """Multi-joint control panel — one column per joint."""

    POLL_MS = 50

    def __init__(self) -> None:
        self.bus = CANBus()
        self.manager: ArmBusManager | None = None
        self._connected = False
        self.joints: Dict[int, JointWidgets] = {}

        self.root = tk.Tk()
        self.root.title("6-DOF Arm — CAN Control Panel")
        self.root.resizable(False, False)

        self._build_ui()
        self._connect()
        self.root.after(self.POLL_MS, self._poll)

    # ==================================================================
    # UI construction
    # ==================================================================

    def _build_ui(self) -> None:
        pad = dict(padx=6, pady=3)

        # ---- Top bar: connection + global buttons -----------------------
        frm_top = ttk.Frame(self.root)
        frm_top.grid(row=0, column=0, columnspan=len(ACTIVE_JOINTS), sticky="ew", **pad)

        self.lbl_conn = ttk.Label(frm_top, text="Disconnected", foreground="red")
        self.lbl_conn.pack(side="left", **pad)

        ttk.Button(frm_top, text="Reconnect", command=self._connect).pack(side="left", **pad)

        ttk.Separator(frm_top, orient="vertical").pack(side="left", fill="y", padx=8)

        ttk.Button(frm_top, text="ENABLE ALL",  command=self._cmd_enable_all).pack(side="left", padx=3)
        ttk.Button(frm_top, text="DISABLE ALL", command=self._cmd_disable_all).pack(side="left", padx=3)
        ttk.Button(frm_top, text="STOP ALL",    command=self._cmd_stop_all).pack(side="left", padx=3)
        ttk.Button(frm_top, text="ZERO ALL",    command=self._cmd_zero_all).pack(side="left", padx=3)

        # ---- One column per joint ---------------------------------------
        for idx, jid in enumerate(ACTIVE_JOINTS):
            jw = JointWidgets(node_id=jid)
            self.joints[jid] = jw

            col_frame = ttk.LabelFrame(self.root, text=f"Joint {jid}")
            col_frame.grid(row=1, column=idx, sticky="nsew", **pad)

            r = 0  # row counter inside column

            # -- Status indicator --
            jw.lbl_status = ttk.Label(col_frame, text="OFFLINE", foreground="red", font=("", 11, "bold"))
            jw.lbl_status.grid(row=r, column=0, columnspan=2, **pad); r += 1

            ttk.Separator(col_frame, orient="horizontal").grid(row=r, column=0, columnspan=2, sticky="ew", pady=4); r += 1

            # -- Angle entry --
            ttk.Label(col_frame, text="Target (°):").grid(row=r, column=0, sticky="e", **pad)
            ent = ttk.Entry(col_frame, textvariable=jw.angle_var, width=10, font=("", 11))
            ent.grid(row=r, column=1, sticky="w", **pad)
            ent.bind("<Return>", lambda _e, j=jid: self._cmd_send_pos(j))
            r += 1

            # -- Quick angle buttons --
            frm_q = ttk.Frame(col_frame)
            frm_q.grid(row=r, column=0, columnspan=2, **pad); r += 1
            for angle in [-360, -90, 0, 90, 360]:
                ttk.Button(
                    frm_q, text=f"{angle}°", width=5,
                    command=lambda a=angle, j=jid: self._quick_angle(j, a),
                ).pack(side="left", padx=1)

            # -- Speed entry --
            ttk.Label(col_frame, text="Speed (mdeg/s):").grid(row=r, column=0, sticky="e", **pad)
            ttk.Entry(col_frame, textvariable=jw.speed_var, width=8).grid(row=r, column=1, sticky="w", **pad)
            r += 1

            # -- Action buttons --
            frm_btn = ttk.Frame(col_frame)
            frm_btn.grid(row=r, column=0, columnspan=2, **pad); r += 1
            for txt, cmd in [
                ("ENABLE",   lambda j=jid: self._cmd_enable(j)),
                ("DISABLE",  lambda j=jid: self._cmd_disable(j)),
                ("SEND POS", lambda j=jid: self._cmd_send_pos(j)),
            ]:
                ttk.Button(frm_btn, text=txt, command=cmd).pack(side="left", padx=2)

            frm_btn2 = ttk.Frame(col_frame)
            frm_btn2.grid(row=r, column=0, columnspan=2, **pad); r += 1
            for txt, cmd in [
                ("SET ZERO", lambda j=jid: self._cmd_set_zero(j)),
                ("STOP",     lambda j=jid: self._cmd_stop(j)),
            ]:
                ttk.Button(frm_btn2, text=txt, command=cmd).pack(side="left", padx=2)

            ttk.Separator(col_frame, orient="horizontal").grid(row=r, column=0, columnspan=2, sticky="ew", pady=4); r += 1

            # -- Telemetry readouts --
            def _add_lbl(parent, row, label_text):
                ttk.Label(parent, text=label_text).grid(row=row, column=0, sticky="e", **pad)
                lbl = ttk.Label(parent, text="—", width=22, anchor="w")
                lbl.grid(row=row, column=1, sticky="w", **pad)
                return lbl

            jw.lbl_angle  = _add_lbl(col_frame, r, "Angle:");  r += 1
            jw.lbl_turns  = _add_lbl(col_frame, r, "Turns:");  r += 1
            jw.lbl_target = _add_lbl(col_frame, r, "State:");  r += 1
            jw.lbl_flags  = _add_lbl(col_frame, r, "Flags:");  r += 1
            jw.lbl_fault  = _add_lbl(col_frame, r, "Fault:");  r += 1
            jw.lbl_uptime = _add_lbl(col_frame, r, "Uptime:"); r += 1

    # ==================================================================
    # Quick helpers
    # ==================================================================

    def _quick_angle(self, jid: int, deg: float) -> None:
        self.joints[jid].angle_var.set(str(deg))

    def _get_client(self, jid: int) -> JointNodeClient | None:
        if self.manager is None:
            return None
        return self.manager.nodes.get(jid)

    # ==================================================================
    # CAN connection
    # ==================================================================

    def _connect(self) -> None:
        if self.manager:
            self.manager.stop_threads()
        try:
            self.bus.close()
        except Exception:
            pass

        try:
            self.bus.open()
            self.manager = ArmBusManager(self.bus)
            self.manager.start(keepalive_nodes=ACTIVE_JOINTS)
            self._connected = True
            self.lbl_conn.config(text="CAN0 Connected", foreground="green")
        except Exception as exc:
            self._connected = False
            self.lbl_conn.config(text=f"Error: {exc}", foreground="red")

    # ==================================================================
    # Per-joint commands
    # ==================================================================

    def _cmd_enable(self, jid: int) -> None:
        c = self._get_client(jid)
        if c:
            c.enable()

    def _cmd_disable(self, jid: int) -> None:
        c = self._get_client(jid)
        if c:
            c.disable()

    def _cmd_set_zero(self, jid: int) -> None:
        c = self._get_client(jid)
        if c:
            c.set_zero()

    def _cmd_stop(self, jid: int) -> None:
        c = self._get_client(jid)
        if c:
            c.stop()

    def _cmd_send_pos(self, jid: int) -> None:
        c = self._get_client(jid)
        if not c:
            return
        jw = self.joints[jid]
        try:
            deg = float(jw.angle_var.get())
        except ValueError:
            return
        try:
            speed = int(jw.speed_var.get())
        except ValueError:
            speed = 0
        c.set_pos(deg, speed)

    # Batch commands
    def _cmd_enable_all(self) -> None:
        for jid in ACTIVE_JOINTS:
            self._cmd_enable(jid)

    def _cmd_disable_all(self) -> None:
        for jid in ACTIVE_JOINTS:
            self._cmd_disable(jid)

    def _cmd_stop_all(self) -> None:
        for jid in ACTIVE_JOINTS:
            self._cmd_stop(jid)

    def _cmd_zero_all(self) -> None:
        for jid in ACTIVE_JOINTS:
            self._cmd_set_zero(jid)

    # ==================================================================
    # Periodic UI refresh
    # ==================================================================

    def _poll(self) -> None:
        if self.manager:
            for jid, jw in self.joints.items():
                client = self._get_client(jid)
                if client is None:
                    continue
                state = client.state

                # Online / offline
                if state.online:
                    jw.lbl_status.config(text="● ONLINE", foreground="green")
                else:
                    jw.lbl_status.config(text="● OFFLINE", foreground="red")

                # Telemetry
                t = state.telemetry
                if t is not None:
                    angle_deg = t.current_angle_deg
                    turns = angle_deg / 360.0
                    jw.lbl_angle.config(text=f"{angle_deg:.2f}°")
                    jw.lbl_turns.config(text=f"{turns:.3f}")
                    jw.lbl_flags.config(text=proto.format_status_flags(t.status_flags))
                    jw.lbl_fault.config(text=t.fault_name)

                    if t.at_target:
                        jw.lbl_target.config(text="AT TARGET ✓", foreground="green")
                    elif t.moving:
                        jw.lbl_target.config(text="MOVING…", foreground="orange")
                    else:
                        jw.lbl_target.config(text="IDLE", foreground="black")
                else:
                    jw.lbl_angle.config(text="—")
                    jw.lbl_turns.config(text="—")
                    jw.lbl_target.config(text="—", foreground="black")

                # Heartbeat
                hb = state.heartbeat
                if hb is not None:
                    jw.lbl_uptime.config(text=f"{hb.uptime_ms / 1000:.1f} s")

        self.root.after(self.POLL_MS, self._poll)

    # ==================================================================
    # Main loop
    # ==================================================================

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            if self.manager:
                self.manager.stop_threads()
            self.bus.close()


def main() -> None:
    app = ArmUI()
    app.run()


if __name__ == "__main__":
    main()
