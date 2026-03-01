"""
Tkinter GUI for controlling Joint 1 of the robotic arm over CAN.

Run:  python -m arm_pi.ui_tk
"""

from __future__ import annotations

import threading
import time
import tkinter as tk
from tkinter import ttk

from . import protocol as proto
from .can_bus import CANBus
from .node_client import JointNodeClient, NodeState


class ArmUI:
    """Single-joint control panel."""

    NODE_ID = 1
    POLL_MS = 50  # UI refresh interval

    def __init__(self) -> None:
        # ---- CAN + node client ------------------------------------------
        self.bus = CANBus()
        self.client: JointNodeClient | None = None
        self._connected = False

        # ---- Tk root ----------------------------------------------------
        self.root = tk.Tk()
        self.root.title("6-DOF Arm — Joint 1 Control")
        self.root.resizable(False, False)

        self._build_ui()

        # Try to connect on start
        self._connect()

        # Periodic UI refresh
        self.root.after(self.POLL_MS, self._poll)

    # ==================================================================
    # UI construction
    # ==================================================================

    def _build_ui(self) -> None:
        pad = dict(padx=6, pady=3)

        # ---- Connection frame -------------------------------------------
        frm_conn = ttk.LabelFrame(self.root, text="Connection")
        frm_conn.grid(row=0, column=0, sticky="ew", **pad)

        self.lbl_conn = ttk.Label(frm_conn, text="Disconnected", foreground="red")
        self.lbl_conn.grid(row=0, column=0, **pad)

        self.lbl_node = ttk.Label(frm_conn, text="Node: OFFLINE", foreground="gray")
        self.lbl_node.grid(row=0, column=1, **pad)

        btn_reconn = ttk.Button(frm_conn, text="Reconnect", command=self._connect)
        btn_reconn.grid(row=0, column=2, **pad)

        # ---- Command frame ----------------------------------------------
        frm_cmd = ttk.LabelFrame(self.root, text="Commands")
        frm_cmd.grid(row=1, column=0, sticky="ew", **pad)

        # Target angle slider
        ttk.Label(frm_cmd, text="Target angle (°):").grid(row=0, column=0, **pad)
        self.target_var = tk.DoubleVar(value=0.0)
        self.slider = ttk.Scale(
            frm_cmd, from_=-180, to=180, orient="horizontal",
            variable=self.target_var, length=300,
        )
        self.slider.grid(row=0, column=1, columnspan=2, **pad)

        # Numeric entry
        self.entry_var = tk.StringVar(value="0.0")
        self.entry = ttk.Entry(frm_cmd, textvariable=self.entry_var, width=10)
        self.entry.grid(row=0, column=3, **pad)
        self.entry.bind("<Return>", self._entry_to_slider)

        # Sync slider → entry
        self.target_var.trace_add("write", self._slider_to_entry)

        # Speed
        ttk.Label(frm_cmd, text="Speed (mdeg/s):").grid(row=1, column=0, **pad)
        self.speed_var = tk.IntVar(value=0)
        self.speed_entry = ttk.Entry(frm_cmd, textvariable=self.speed_var, width=10)
        self.speed_entry.grid(row=1, column=1, **pad)
        ttk.Label(frm_cmd, text="(0 = default)").grid(row=1, column=2, **pad)

        # Buttons
        frm_btn = ttk.Frame(frm_cmd)
        frm_btn.grid(row=2, column=0, columnspan=4, pady=6)

        for text, cmd in [
            ("ENABLE",  self._cmd_enable),
            ("DISABLE", self._cmd_disable),
            ("SET ZERO", self._cmd_set_zero),
            ("STOP",    self._cmd_stop),
            ("SEND POSITION", self._cmd_send_pos),
        ]:
            ttk.Button(frm_btn, text=text, command=cmd).pack(side="left", padx=4)

        # ---- Telemetry frame --------------------------------------------
        frm_tel = ttk.LabelFrame(self.root, text="Telemetry")
        frm_tel.grid(row=2, column=0, sticky="ew", **pad)

        labels = [
            ("Current angle (°):", "lbl_angle"),
            ("Status flags:", "lbl_flags"),
            ("Fault:", "lbl_fault"),
            ("Uptime (s):", "lbl_uptime"),
            ("Last RX:", "lbl_rx"),
        ]
        for i, (text, attr) in enumerate(labels):
            ttk.Label(frm_tel, text=text).grid(row=i, column=0, sticky="e", **pad)
            lbl = ttk.Label(frm_tel, text="—", width=50, anchor="w")
            lbl.grid(row=i, column=1, sticky="w", **pad)
            setattr(self, attr, lbl)

    # ==================================================================
    # CAN connection
    # ==================================================================

    def _connect(self) -> None:
        # Tear down old
        if self.client:
            self.client.stop_threads()
        try:
            self.bus.close()
        except Exception:
            pass

        try:
            self.bus.open()
            self.client = JointNodeClient(self.bus, node_id=self.NODE_ID)
            self.client.start(keepalive=True)
            self._connected = True
            self.lbl_conn.config(text="CAN0 Connected", foreground="green")
        except Exception as exc:
            self._connected = False
            self.lbl_conn.config(text=f"Error: {exc}", foreground="red")

    # ==================================================================
    # Commands
    # ==================================================================

    def _cmd_enable(self) -> None:
        if self.client:
            self.client.enable()

    def _cmd_disable(self) -> None:
        if self.client:
            self.client.disable()

    def _cmd_set_zero(self) -> None:
        if self.client:
            self.client.set_zero()

    def _cmd_stop(self) -> None:
        if self.client:
            self.client.stop()

    def _cmd_send_pos(self) -> None:
        if not self.client:
            return
        try:
            deg = float(self.entry_var.get())
        except ValueError:
            deg = self.target_var.get()
        speed = self.speed_var.get()
        self.client.set_pos(deg, speed)

    # ==================================================================
    # Slider ↔ entry sync
    # ==================================================================

    def _slider_to_entry(self, *_args) -> None:
        self.entry_var.set(f"{self.target_var.get():.1f}")

    def _entry_to_slider(self, _event=None) -> None:
        try:
            val = float(self.entry_var.get())
            val = max(-180.0, min(180.0, val))
            self.target_var.set(val)
        except ValueError:
            pass

    # ==================================================================
    # Periodic UI refresh
    # ==================================================================

    def _poll(self) -> None:
        if self.client:
            state = self.client.state

            # Node online indicator
            if state.online:
                self.lbl_node.config(text="Node: ONLINE", foreground="green")
            else:
                self.lbl_node.config(text="Node: OFFLINE", foreground="red")

            # Telemetry
            t = state.telemetry
            if t is not None:
                self.lbl_angle.config(text=f"{t.current_angle_deg:.2f}°  ({t.current_angle_mdeg} mdeg)")
                self.lbl_flags.config(text=proto.format_status_flags(t.status_flags))
                self.lbl_fault.config(text=t.fault_name)
                self.lbl_rx.config(
                    text=time.strftime("%H:%M:%S", time.localtime(state.last_telemetry_time))
                )

            # Heartbeat
            hb = state.heartbeat
            if hb is not None:
                self.lbl_uptime.config(text=f"{hb.uptime_ms / 1000:.1f}")

        self.root.after(self.POLL_MS, self._poll)

    # ==================================================================
    # Main loop
    # ==================================================================

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            if self.client:
                self.client.stop_threads()
            self.bus.close()


def main() -> None:
    app = ArmUI()
    app.run()


if __name__ == "__main__":
    main()
