#!/usr/bin/env python3
"""Minimal Tk teleop for the AGV fleet.

Exists because ros2's own teleop_twist_keyboard is unusable against this
stack. Two reasons, both fatal on their own:

  1. Its getKey() is a *blocking* `sys.stdin.read(1)` with no timeout, so it
     publishes exactly ONE message per keypress and then blocks. Paired with
     diff_drive_controller's cmd_vel_timeout the robot twitches for one
     timeout period per key and stops - it looks completely dead.
  2. It needs `-p stamped:=true` to emit TwistStamped (diff_drive_controller
     4.x accepts nothing else), and getting that wrong fails *silently*: the
     publisher connects by topic name and no message is ever delivered.

This node instead publishes continuously at `publish_rate` (default 20 Hz)
for as long as a key is held, and publishes explicit zeros the moment it is
released - so the controller's timeout is never what stops the robot during
normal driving, and a held key means sustained motion.
"""

import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    import tkinter as tk
except ImportError as exc:  # pragma: no cover - depends on python3-tk
    raise SystemExit(
        "tkinter is missing. Install it with `apt-get install -y python3-tk` "
        "(declared as a python3-tk exec_depend in package.xml, so "
        "`make rosdeps` covers it too)."
    ) from exc


# Held-key -> (linear scale, angular scale). Arrow keys and WASD both, plus
# the numpad-style diagonals so a turn-while-driving is one keypress.
BINDINGS = {
    "Up": (1.0, 0.0),
    "Down": (-1.0, 0.0),
    "Left": (0.0, 1.0),
    "Right": (0.0, -1.0),
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    "q": (1.0, 1.0),
    "e": (1.0, -1.0),
    "z": (-1.0, 1.0),
    "c": (-1.0, -1.0),
}

BG = "#14161a"
FG = "#e6e8eb"
DIM = "#7d858f"
ACCENT = "#4c9aff"
STOP = "#ff5d5d"
PANEL = "#1c1f26"


class TeleopUi(Node):
    def __init__(self) -> None:
        super().__init__("teleop_ui")

        self.declare_parameter("cmd_vel_topic", "cmd_vel_nav")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 1.5)
        # X11 key auto-repeat arrives as repeated KeyPress/KeyRelease PAIRS,
        # not one press and a late release. Acting on every KeyRelease would
        # chop a held key into a stutter of stop/go, so a release only takes
        # effect if no new press for the same key lands within this window.
        self.declare_parameter("key_release_grace", 0.06)

        topic = self.get_parameter("cmd_vel_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate = float(self.get_parameter("publish_rate").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.grace_ms = int(float(self.get_parameter("key_release_grace").value) * 1000)

        # RELIABLE is not optional here. nav2's velocity_smoother subscribes
        # to cmd_vel_nav as RELIABLE, and a BEST_EFFORT publisher is an
        # incompatible-QoS match: the connection is refused and rclpy logs
        # "requesting incompatible QoS ... No messages will be sent to it"
        # while the topic still shows a publisher and a subscriber. (Found
        # exactly that way - the first cut of this file used BEST_EFFORT on
        # the theory that stale setpoints are worse than dropped ones.)
        # RELIABLE also matches diff_drive_controller's own BEST_EFFORT
        # cmd_vel subscription, so cmd_vel_topic:=cmd_vel works too.
        self.pub = self.create_publisher(
            TwistStamped, topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.resolved_topic = self.pub.topic_name

        self.linear_scale = 0.0
        self.angular_scale = 0.0
        self.speed_frac = 0.4
        self.turn_frac = 0.4
        self.estopped = False
        self.sent = 0
        self._held: set[str] = set()
        self._pending_release: dict[str, str] = {}

        self.get_logger().info(f"teleop publishing TwistStamped on {self.resolved_topic} at {self.rate:.0f} Hz")

    # ── command generation ──────────────────────────────────────────────────
    def current_cmd(self) -> tuple[float, float]:
        if self.estopped:
            return 0.0, 0.0
        return (
            self.linear_scale * self.speed_frac * self.max_linear,
            self.angular_scale * self.turn_frac * self.max_angular,
        )

    def publish(self) -> tuple[float, float]:
        vx, wz = self.current_cmd()
        # A SIGTERM tears the rclpy context down under the Tk loop, so both
        # the timer tick and the stop-on-exit below can land after it is
        # already invalid.
        if not rclpy.ok():
            return vx, wz
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = vx
        msg.twist.angular.z = wz
        self.pub.publish(msg)
        self.sent += 1
        return vx, wz


def build_ui(node: TeleopUi) -> tk.Tk:
    root = tk.Tk()
    root.title("AGV Teleop")
    root.configure(bg=BG)
    root.resizable(False, False)

    def label(parent, text, *, size=10, color=FG, weight="normal", **kw):
        return tk.Label(parent, text=text, bg=kw.pop("bg", BG), fg=color, font=("DejaVu Sans", size, weight), **kw)

    label(root, node.resolved_topic, size=9, color=ACCENT).pack(pady=(12, 0))
    label(root, "hold a key — release stops", size=8, color=DIM).pack()

    readout = label(root, "0.00 m/s   0.00 rad/s", size=17, weight="bold")
    readout.pack(pady=(8, 10))

    # ── D-pad ───────────────────────────────────────────────────────────────
    pad = tk.Frame(root, bg=BG)
    pad.pack(padx=18)
    cells = [
        ("q", "↖", 0, 0), ("Up", "↑", 0, 1), ("e", "↗", 0, 2),
        ("Left", "←", 1, 0), (None, "■", 1, 1), ("Right", "→", 1, 2),
        ("z", "↙", 2, 0), ("Down", "↓", 2, 1), ("c", "↘", 2, 2),
    ]
    buttons: dict[str, tk.Label] = {}
    for key, glyph, r, c in cells:
        cell = tk.Label(pad, text=glyph, bg=PANEL, fg=FG if key else DIM,
                        font=("DejaVu Sans", 15), width=3, height=1)
        cell.grid(row=r, column=c, padx=3, pady=3)
        if key:
            buttons[key] = cell
            # Click-and-hold drives too, for anyone without a keyboard focus.
            cell.bind("<ButtonPress-1>", lambda _e, k=key: press(k))
            cell.bind("<ButtonRelease-1>", lambda _e, k=key: release(k))
        else:
            cell.bind("<ButtonPress-1>", lambda _e: stop_all())

    # ── speed sliders ───────────────────────────────────────────────────────
    sliders = tk.Frame(root, bg=BG)
    sliders.pack(pady=(12, 4), padx=18, fill="x")

    lin_val = label(sliders, "", size=9, color=DIM)
    ang_val = label(sliders, "", size=9, color=DIM)

    def slider(row, initial, on_change):
        s = tk.Scale(sliders, from_=5, to=100, orient="horizontal", showvalue=False,
                     bg=BG, fg=FG, troughcolor=PANEL, highlightthickness=0, bd=0,
                     activebackground=ACCENT, sliderrelief="flat", length=150,
                     command=on_change)
        s.set(int(initial * 100))
        s.grid(row=row, column=1, sticky="ew")
        return s

    def set_speed(v):
        node.speed_frac = int(v) / 100.0
        lin_val.config(text=f"{node.speed_frac * node.max_linear:.2f} m/s")

    def set_turn(v):
        node.turn_frac = int(v) / 100.0
        ang_val.config(text=f"{node.turn_frac * node.max_angular:.2f} rad/s")

    label(sliders, "speed", size=9, color=DIM).grid(row=0, column=0, sticky="w", padx=(0, 8))
    slider(0, node.speed_frac, set_speed)
    lin_val.grid(row=0, column=2, sticky="e", padx=(8, 0))

    label(sliders, "turn", size=9, color=DIM).grid(row=1, column=0, sticky="w", padx=(0, 8))
    slider(1, node.turn_frac, set_turn)
    ang_val.grid(row=1, column=2, sticky="e", padx=(8, 0))
    set_speed(node.speed_frac * 100)
    set_turn(node.turn_frac * 100)

    # ── e-stop ──────────────────────────────────────────────────────────────
    estop = tk.Label(root, text="E-STOP   (space)", bg=PANEL, fg=STOP,
                     font=("DejaVu Sans", 10, "bold"), pady=7)
    estop.pack(fill="x", padx=18, pady=(10, 4))

    def toggle_estop(*_):
        node.estopped = not node.estopped
        node.linear_scale = node.angular_scale = 0.0
        node._held.clear()
        estop.config(bg=STOP if node.estopped else PANEL, fg=BG if node.estopped else STOP,
                     text="E-STOPPED — space to clear" if node.estopped else "E-STOP   (space)")

    estop.bind("<ButtonPress-1>", toggle_estop)

    hint = label(root, "arrows / wasd + qezc diagonals", size=8, color=DIM)
    hint.pack(pady=(0, 12))

    # ── key handling ────────────────────────────────────────────────────────
    def recompute():
        lin = ang = 0.0
        for k in node._held:
            dl, da = BINDINGS[k]
            lin += dl
            ang += da
        node.linear_scale = max(-1.0, min(1.0, lin))
        node.angular_scale = max(-1.0, min(1.0, ang))
        for k, cell in buttons.items():
            cell.config(bg=ACCENT if k in node._held else PANEL,
                        fg=BG if k in node._held else FG)

    def press(key):
        if key not in BINDINGS:
            return
        pending = node._pending_release.pop(key, None)
        if pending is not None:
            root.after_cancel(pending)
        node._held.add(key)
        recompute()

    def release(key):
        if key not in BINDINGS or key in node._pending_release:
            return

        def commit():
            node._pending_release.pop(key, None)
            node._held.discard(key)
            recompute()

        node._pending_release[key] = root.after(node.grace_ms, commit)

    def stop_all():
        for pending in node._pending_release.values():
            root.after_cancel(pending)
        node._pending_release.clear()
        node._held.clear()
        recompute()

    def on_key(event, down):
        if event.keysym == "space":
            if down:
                toggle_estop()
            return
        key = event.keysym if event.keysym in BINDINGS else event.keysym.lower()
        (press if down else release)(key)

    root.bind("<KeyPress>", lambda e: on_key(e, True))
    root.bind("<KeyRelease>", lambda e: on_key(e, False))
    # Losing focus mid-drive must not latch the last command.
    root.bind("<FocusOut>", lambda _e: stop_all())
    root.focus_force()

    # ── publish loop ────────────────────────────────────────────────────────
    period_ms = max(1, int(1000.0 / node.rate))

    def tick():
        if not rclpy.ok():
            root.destroy()
            return
        vx, wz = node.publish()
        readout.config(
            text=f"{vx:+.2f} m/s   {wz:+.2f} rad/s",
            fg=STOP if node.estopped else (ACCENT if abs(vx) + abs(wz) > 1e-9 else FG),
        )
        root.after(period_ms, tick)

    root.after(period_ms, tick)
    return root


def main() -> None:
    rclpy.init()
    node = TeleopUi()

    # rclpy needs its own thread: Tk owns the main thread's event loop, and
    # /clock has to keep being serviced for use_sim_time timestamps.
    def spin() -> None:
        try:
            rclpy.spin(node)
        except (rclpy.executors.ExternalShutdownException, rclpy._rclpy_pybind11.RCLError):
            # Normal on SIGTERM/Ctrl-C: the context is torn down while this
            # thread is inside wait_set. Nothing to recover, and the
            # traceback is pure noise in the launching terminal.
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()

    root = build_ui(node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave the robot stopped, whatever the exit path. If the
        # context is already gone these are no-ops and diff_drive_controller's
        # cmd_vel_timeout is what halts the robot - which is the case that
        # timeout exists for.
        node.estopped = True
        for _ in range(3):
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
