#!/usr/bin/env python3
"""
bag_to_map_gui.py  –  Tkinter GUI for the bag-to-map Docker tool.

Builds and runs:
    docker run --rm -v "<mount_dir>:/app/data" bag-to-map \
        /app/data/<input_bag> /app/data/<output_path> [options]

Usage:
    python3 bag_to_map_gui.py
"""

import os
import platform
import queue
import shlex
import subprocess
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

DOCKER_IMAGE = "bag-to-map"

DEFAULTS = {
    "pc_topic":         "/dlio/odom_node/pointcloud/deskewed",
    "odom_topic":       "/dlio/odom_node/odom",
    "octree_res":       "0.1",
    "grid_res":         "0.05",
    "slope_deg":        "15.0",
    "normal_radius":    "0.2",
    "z_min":            "0.1",
    "z_max":            "2.0",
    "downsample":       "0.05",
    "workers":          "4",
    "min_cluster_size": "20",
    "closing_iters":    "1",
}

# Platform-aware fonts
_SYS = platform.system()
UI_FONT   = "Segoe UI"    if _SYS == "Windows" else ("SF Pro Text" if _SYS == "Darwin" else "DejaVu Sans")
MONO_FONT = "Consolas"    if _SYS == "Windows" else ("Menlo"       if _SYS == "Darwin" else "DejaVu Sans Mono")

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def to_docker_mount_path(local_path: str) -> str:
    """Convert a local path to a Docker-compatible volume mount path string."""
    p = Path(local_path).resolve()
    if _SYS == "Windows":
        # C:\Users\foo  →  /c/Users/foo  (Git Bash / MSYS style)
        drive = p.drive.rstrip(":").lower()
        rest  = p.as_posix()[2:]          # strip "C:"
        return f"/{drive}{rest}"
    return p.as_posix()


def path_relative_to(child: str, parent: str) -> str | None:
    """Return posix relative path of child w.r.t. parent, or None."""
    try:
        return Path(child).resolve().relative_to(Path(parent).resolve()).as_posix()
    except ValueError:
        return None


def format_command_display(cmd: list) -> str:
    """Format a command list as a readable multi-line shell string."""
    if not cmd:
        return ""
    lines = []
    i = 0
    while i < len(cmd):
        token = cmd[i]
        # Group -v flag with its value on one line
        if token in ("-v",) and i + 1 < len(cmd):
            lines.append(f"  {token} {shlex.quote(cmd[i+1])}")
            i += 2
        elif token.startswith("--") or (i >= 3 and not token.startswith("/")):
            # option flags and their values
            if i + 1 < len(cmd) and not cmd[i+1].startswith("--"):
                lines.append(f"  {token} {cmd[i+1]}")
                i += 2
            else:
                lines.append(f"  {token}")
                i += 1
        else:
            lines.append(f"  {shlex.quote(token)}")
            i += 1
    # First token (docker) on its own line
    result = shlex.quote(cmd[0]) + " \\\n" + " \\\n".join(lines[1:])
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Main Application
# ─────────────────────────────────────────────────────────────────────────────

class BagToMapGUI(tk.Tk):

    # ── Colour palette (dark theme) ───────────────────────────────────────
    C = {
        "bg":         "#1a1a2e",
        "panel":      "#16213e",
        "surface":    "#0f3460",
        "accent":     "#4f98a3",
        "accent2":    "#6daa45",
        "fg":         "#cdccca",
        "fg_muted":   "#797876",
        "fg_faint":   "#4a4948",
        "entry_bg":   "#1e1e32",
        "border":     "#2a2a45",
        "btn_run":    "#437a22",
        "btn_stp":    "#a12c7b",
        "btn_copy":   "#01696f",
        "err":        "#a13544",
        "console_bg": "#0d0d1a",
        "console_fg": "#a0d8a8",
        "cmd_fg":     "#6daa45",
    }

    def __init__(self):
        super().__init__()
        self.title("ROS 2 Bag → Nav2 Map  ·  bag-to-map GUI")
        self.configure(bg=self.C["bg"])
        self.minsize(760, 680)
        self.resizable(True, True)

        self._proc: subprocess.Popen | None = None
        self._log_q: queue.Queue = queue.Queue()

        self._apply_styles()
        self._build_ui()
        self._update_command()
        self._poll_log_queue()

    # ── Style setup ───────────────────────────────────────────────────────

    def _apply_styles(self):
        C = self.C
        s = ttk.Style(self)
        s.theme_use("clam")

        s.configure("TFrame",        background=C["bg"])
        s.configure("Card.TFrame",   background=C["panel"],
                                     relief="flat")

        s.configure("TLabel",        background=C["bg"], foreground=C["fg"],
                                     font=(UI_FONT, 10))
        s.configure("Head.TLabel",   background=C["bg"], foreground=C["accent"],
                                     font=(UI_FONT, 8, "bold"))
        s.configure("Muted.TLabel",  background=C["bg"], foreground=C["fg_muted"],
                                     font=(UI_FONT, 8))
        s.configure("Title.TLabel",  background=C["bg"], foreground=C["accent"],
                                     font=(UI_FONT, 14, "bold"))
        s.configure("Sub.TLabel",    background=C["bg"], foreground=C["fg_muted"],
                                     font=(UI_FONT, 9))
        s.configure("Hint.TLabel",   background=C["panel"], foreground=C["fg_muted"],
                                     font=(UI_FONT, 8))
        s.configure("OK.TLabel",     background=C["bg"], foreground=C["accent2"],
                                     font=(UI_FONT, 8))

        s.configure("TEntry",
                    fieldbackground=C["entry_bg"],
                    foreground=C["fg"],
                    bordercolor=C["border"],
                    insertcolor=C["fg"],
                    font=(UI_FONT, 10))
        s.map("TEntry",
              bordercolor=[("focus", C["accent"])])

        s.configure("TButton",       background=C["panel"], foreground=C["fg"],
                                     bordercolor=C["border"], font=(UI_FONT, 9),
                                     padding=(8, 4))
        s.map("TButton",
              background=[("active", C["border"]), ("disabled", C["panel"])],
              foreground=[("disabled", C["fg_faint"])])

        s.configure("Run.TButton",   background=C["btn_run"], foreground="#dff2d0",
                                     font=(UI_FONT, 10, "bold"), padding=(14, 6))
        s.map("Run.TButton",
              background=[("active", "#2e5c10"), ("disabled", C["panel"])],
              foreground=[("disabled", C["fg_faint"])])

        s.configure("Stop.TButton",  background=C["btn_stp"], foreground="#f5d5ed",
                                     font=(UI_FONT, 10, "bold"), padding=(14, 6))
        s.map("Stop.TButton",
              background=[("active", "#7d1e5e"), ("disabled", C["panel"])],
              foreground=[("disabled", C["fg_faint"])])

        s.configure("Copy.TButton",  background=C["btn_copy"], foreground="#d5f0f0",
                                     font=(UI_FONT, 9), padding=(8, 3))
        s.map("Copy.TButton",
              background=[("active", "#0c4e54")])

        s.configure("Clear.TButton", background=C["panel"], foreground=C["fg_muted"],
                                     font=(UI_FONT, 9), padding=(8, 3))

        s.configure("TNotebook",     background=C["bg"],
                                     bordercolor=C["border"], tabmargins=[0, 0, 0, 0])
        s.configure("TNotebook.Tab", background=C["panel"], foreground=C["fg_muted"],
                                     padding=[12, 5], font=(UI_FONT, 9))
        s.map("TNotebook.Tab",
              background=[("selected", C["bg"])],
              foreground=[("selected", C["accent"])],
              expand=[("selected", [1, 1, 1, 0])])

        s.configure("TSeparator",    background=C["border"])

    # ── Root UI ───────────────────────────────────────────────────────────

    def _build_ui(self):
        C = self.C
        root = ttk.Frame(self, style="TFrame", padding=14)
        root.pack(fill="both", expand=True)

        # Title bar
        title_row = ttk.Frame(root, style="TFrame")
        title_row.pack(fill="x", pady=(0, 2))
        ttk.Label(title_row,
                  text="🗺  ROS 2 Bag  →  Nav2 Occupancy Map",
                  style="Title.TLabel").pack(side="left")
        ttk.Label(title_row,
                  text=f"  image: {DOCKER_IMAGE}",
                  foreground=C["fg_faint"], background=C["bg"],
                  font=(UI_FONT, 9)).pack(side="left", padx=10)

        ttk.Label(root,
                  text="Configure parameters below, then click ▶ Run to launch the Docker container.",
                  style="Sub.TLabel").pack(anchor="w", pady=(0, 10))

        # ── Parameter tabs ────────────────────────────────────────────────
        nb = ttk.Notebook(root)
        nb.pack(fill="x", pady=(0, 10))

        tab_paths  = ttk.Frame(nb, style="TFrame", padding=(10, 8))
        tab_topics = ttk.Frame(nb, style="TFrame", padding=(10, 8))
        tab_res    = ttk.Frame(nb, style="TFrame", padding=(10, 8))
        tab_perf   = ttk.Frame(nb, style="TFrame", padding=(10, 8))

        nb.add(tab_paths,  text="  📁  Paths  ")
        nb.add(tab_topics, text="  📡  Topics  ")
        nb.add(tab_res,    text="  🔬  Resolution & Geometry  ")
        nb.add(tab_perf,   text="  ⚙  Performance & Denoising  ")

        self._build_paths_tab(tab_paths)
        self._build_topics_tab(tab_topics)
        self._build_res_tab(tab_res)
        self._build_perf_tab(tab_perf)

        # ── Generated command ─────────────────────────────────────────────
        sep = ttk.Separator(root, orient="horizontal")
        sep.pack(fill="x", pady=(2, 8))

        cmd_hdr = ttk.Frame(root, style="TFrame")
        cmd_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(cmd_hdr, text="GENERATED COMMAND",
                  style="Head.TLabel").pack(side="left")
        self._copy_btn = ttk.Button(cmd_hdr, text="⎘  Copy Command",
                                    style="Copy.TButton",
                                    command=self._copy_command)
        self._copy_btn.pack(side="right")

        self._cmd_text = tk.Text(
            root, height=4,
            bg=C["entry_bg"], fg=C["cmd_fg"],
            insertbackground=C["fg"],
            font=(MONO_FONT, 9),
            relief="flat", bd=0,
            state="disabled", wrap="none",
            highlightbackground=C["border"],
            highlightthickness=1,
            padx=8, pady=6,
        )
        self._cmd_text.pack(fill="x")

        # Horizontal scrollbar for command
        cmd_xsb = ttk.Scrollbar(root, orient="horizontal",
                                 command=self._cmd_text.xview)
        self._cmd_text.configure(xscrollcommand=cmd_xsb.set)
        cmd_xsb.pack(fill="x", pady=(0, 6))

        # ── Run / Stop controls ───────────────────────────────────────────
        ctrl_row = ttk.Frame(root, style="TFrame")
        ctrl_row.pack(fill="x", pady=(2, 8))

        self._status_var = tk.StringVar(value="Ready.")
        ttk.Label(ctrl_row, textvariable=self._status_var,
                  style="Muted.TLabel").pack(side="left")

        self._stop_btn = ttk.Button(ctrl_row, text="■  Stop",
                                    style="Stop.TButton",
                                    command=self._stop,
                                    state="disabled")
        self._stop_btn.pack(side="right", padx=(6, 0))

        self._run_btn = ttk.Button(ctrl_row, text="▶  Run",
                                   style="Run.TButton",
                                   command=self._run)
        self._run_btn.pack(side="right")

        # ── Output console ────────────────────────────────────────────────
        con_hdr = ttk.Frame(root, style="TFrame")
        con_hdr.pack(fill="x", pady=(0, 4))
        ttk.Label(con_hdr, text="OUTPUT", style="Head.TLabel").pack(side="left")
        ttk.Button(con_hdr, text="Clear", style="Clear.TButton",
                   command=self._clear_console).pack(side="right")

        self._console = scrolledtext.ScrolledText(
            root,
            height=12,
            bg=C["console_bg"], fg=C["console_fg"],
            insertbackground=C["fg"],
            font=(MONO_FONT, 9),
            relief="flat", bd=0,
            state="disabled",
            wrap="char",
            highlightbackground=C["border"],
            highlightthickness=1,
            padx=8, pady=6,
        )
        self._console.pack(fill="both", expand=True)

    # ── Tab: Paths ────────────────────────────────────────────────────────

    def _build_paths_tab(self, tab):
        C = self.C

        ttk.Label(tab,
            text=(
                "Choose the local folder to mount into the container, "
                "then select/enter the bag and output paths relative to it."
            ),
            foreground=C["fg_muted"], background=C["bg"],
            font=(UI_FONT, 8), wraplength=640
        ).pack(anchor="w", pady=(0, 8))

        self._mount_var  = tk.StringVar()
        self._input_var  = tk.StringVar()
        self._output_var = tk.StringVar()

        for v in (self._mount_var, self._input_var, self._output_var):
            v.trace_add("write", lambda *_: self._update_command())

        def field_row(parent, label, var, hint, browse_cmd=None,
                      required=False):
            r = ttk.Frame(parent, style="TFrame")
            r.pack(fill="x", pady=4)
            req_mark = " *" if required else "  "
            lbl_text = label + req_mark
            ttk.Label(r, text=lbl_text, width=16,
                      foreground=(C["accent"] if required else C["fg"]),
                      background=C["bg"],
                      font=(UI_FONT, 10)).pack(side="left")
            e = ttk.Entry(r, textvariable=var)
            e.pack(side="left", fill="x", expand=True, padx=(0, 6))
            if browse_cmd:
                ttk.Button(r, text="Browse…",
                           command=browse_cmd, width=8).pack(side="left")
            ttk.Label(r, text=hint, style="Muted.TLabel").pack(
                side="left", padx=(8, 0))
            return e

        field_row(tab, "Mount Dir",  self._mount_var,
                  "Local folder → /app/data",
                  self._browse_mount, required=True)
        field_row(tab, "Input Bag",  self._input_var,
                  "Relative path inside Mount Dir",
                  self._browse_bag, required=True)
        field_row(tab, "Output Path", self._output_var,
                  "e.g. my_map  (no extension)", required=True)

        ttk.Label(tab,
            text="★  All three fields are required.",
            foreground=C["accent2"], background=C["bg"],
            font=(UI_FONT, 8)
        ).pack(anchor="w", pady=(6, 0))

        # Live path preview
        preview_frame = tk.Frame(tab, bg=C["entry_bg"],
                                 highlightbackground=C["border"],
                                 highlightthickness=1)
        preview_frame.pack(fill="x", pady=(10, 0), ipady=6, ipadx=10)

        tk.Label(preview_frame, text="Container paths preview:",
                 fg=C["fg_muted"], bg=C["entry_bg"],
                 font=(UI_FONT, 8, "bold")).pack(anchor="w")

        self._preview_in_lbl  = tk.Label(preview_frame, text="  input  → —",
                                          fg=C["accent"], bg=C["entry_bg"],
                                          font=(MONO_FONT, 9))
        self._preview_in_lbl.pack(anchor="w")

        self._preview_out_lbl = tk.Label(preview_frame, text="  output → —",
                                          fg=C["accent2"], bg=C["entry_bg"],
                                          font=(MONO_FONT, 9))
        self._preview_out_lbl.pack(anchor="w")

        # Update preview on var change
        for v in (self._mount_var, self._input_var, self._output_var):
            v.trace_add("write", lambda *_: self._update_path_preview())

    def _update_path_preview(self):
        inp = self._input_var.get().strip().strip("/")
        out = self._output_var.get().strip().strip("/")
        in_txt  = f"  input  →  /app/data/{inp}"  if inp else "  input  →  —"
        out_txt = f"  output →  /app/data/{out}"  if out else "  output →  —"
        self._preview_in_lbl.config(text=in_txt)
        self._preview_out_lbl.config(text=out_txt)

    # ── Tab: Topics ───────────────────────────────────────────────────────

    def _build_topics_tab(self, tab):
        C = self.C

        self._pc_topic_var   = tk.StringVar(value=DEFAULTS["pc_topic"])
        self._odom_topic_var = tk.StringVar(value=DEFAULTS["odom_topic"])

        for v in (self._pc_topic_var, self._odom_topic_var):
            v.trace_add("write", lambda *_: self._update_command())

        rows = [
            ("Point Cloud Topic", self._pc_topic_var,   DEFAULTS["pc_topic"],
             "--pc_topic",   "PointCloud2 topic from the bag."),
            ("Odometry Topic",    self._odom_topic_var, DEFAULTS["odom_topic"],
             "--odom_topic", "nav_msgs/Odometry topic for sensor pose."),
        ]

        for label, var, default, flag, hint in rows:
            r = ttk.Frame(tab, style="TFrame")
            r.pack(fill="x", pady=6)
            ttk.Label(r, text=label, width=20).pack(side="left")
            ttk.Entry(r, textvariable=var, width=50).pack(side="left", padx=(0, 8))
            ttk.Button(r, text="Reset",
                       command=lambda v=var, d=default: v.set(d)
                       ).pack(side="left")
            info = ttk.Frame(tab, style="TFrame")
            info.pack(fill="x", pady=(0, 4))
            ttk.Label(info, text=f"  {flag}  ·  {hint}  ·  default: {default}",
                      style="Muted.TLabel").pack(anchor="w", padx=(160, 0))

        ttk.Label(tab,
            text=(
                "Tip:  run  ros2 bag info data/YOUR_BAG  inside the container\n"
                "      to discover topic names in your bag file."
            ),
            foreground=C["fg_muted"], background=C["bg"],
            font=(MONO_FONT, 8), justify="left"
        ).pack(anchor="w", pady=(12, 0))

    # ── Tab: Resolution & Geometry ────────────────────────────────────────

    def _build_res_tab(self, tab):
        C = self.C

        params = [
            ("octree_res",   "Octree Res (m)",    "0.1",  "Resolution of the intermediate 3D OcTree."),
            ("grid_res",     "Grid Res (m)",       "0.05", "Resolution of the final 2D occupancy grid."),
            ("slope_deg",    "Slope (°)",          "15.0", "Max slope in degrees to classify a point as ground."),
            ("normal_radius","Normal Radius (m)",  "0.2",  "Radius (m) used for surface-normal estimation."),
            ("z_min",        "Z Min (m)",          "0.1",  "Min height above local ground to check for obstacles."),
            ("z_max",        "Z Max (m)",          "2.0",  "Max height above local ground to check for obstacles."),
            ("downsample",   "Downsample (m)",     "0.05", "Voxel size (m) for downsampling. Set 0 to disable."),
        ]
        self._res_vars = {}
        self._build_param_grid(tab, params, self._res_vars)

    # ── Tab: Performance & Denoising ──────────────────────────────────────

    def _build_perf_tab(self, tab):
        C = self.C

        params = [
            ("workers",          "Workers",           "4",  "Parallel worker threads for grid generation."),
            ("min_cluster_size", "Min Cluster Size",  "20", "Min occupied cells for a cluster to be kept. 0 = disable."),
            ("closing_iters",    "Closing Iters",     "1",  "Morphological closing passes before labeling. 0 = skip."),
        ]
        self._perf_vars = {}
        self._build_param_grid(tab, params, self._perf_vars)

    def _build_param_grid(self, tab, params, var_store):
        C = self.C
        for key, label, default, hint in params:
            var = tk.StringVar(value=DEFAULTS.get(key, default))
            var.trace_add("write", lambda *_: self._update_command())
            var_store[key] = var

            row = ttk.Frame(tab, style="TFrame")
            row.pack(fill="x", pady=4)

            ttk.Label(row, text=label, width=20).pack(side="left")
            ttk.Entry(row, textvariable=var, width=12).pack(side="left", padx=(0, 8))
            ttk.Button(row, text="↺",
                       command=lambda v=var, d=default: v.set(d),
                       width=2).pack(side="left", padx=(0, 10))
            ttk.Label(row, text=hint, style="Muted.TLabel").pack(side="left")

        # Reset all button
        def reset_all():
            for key, var in var_store.items():
                var.set(DEFAULTS.get(key, ""))
        ttk.Button(tab, text="↺  Reset All to Defaults",
                   command=reset_all).pack(anchor="e", pady=(10, 0))

    # ── Browse helpers ────────────────────────────────────────────────────

    def _browse_mount(self):
        d = filedialog.askdirectory(title="Select Mount Directory (local data folder)")
        if d:
            self._mount_var.set(d)

    def _browse_bag(self):
        mount = self._mount_var.get().strip()
        init  = mount if mount and Path(mount).is_dir() else os.path.expanduser("~")
        d = filedialog.askdirectory(
            title="Select Input Bag Directory", initialdir=init)
        if not d:
            return
        if mount:
            rel = path_relative_to(d, mount)
            if rel:
                self._input_var.set(rel)
                return
        # If outside mount dir, warn user
        messagebox.showwarning(
            "Outside Mount Directory",
            f"The selected folder is not inside the Mount Directory.\n\n"
            f"Mount Dir:  {mount}\n"
            f"Selected:   {d}\n\n"
            "Please set the Mount Directory first, then select a bag "
            "folder that lives inside it."
        )

    # ── Command builder ───────────────────────────────────────────────────

    def _build_cmd_list(self) -> list[str] | None:
        """Build the docker run command as a flat list of strings."""
        mount = self._mount_var.get().strip()
        inp   = self._input_var.get().strip().strip("/")
        out   = self._output_var.get().strip().strip("/")

        if not mount or not inp or not out:
            return None

        volume        = f"{to_docker_mount_path(mount)}:/app/data"
        container_in  = f"/app/data/{inp}"
        container_out = f"/app/data/{out}"

        cmd = ["docker", "run", "--rm",
               "-v", volume,
               DOCKER_IMAGE,
               container_in,
               container_out]

        # Topics (only if changed from default)
        topic_map = {
            "--pc_topic":   (self._pc_topic_var.get().strip(),   "pc_topic"),
            "--odom_topic": (self._odom_topic_var.get().strip(),  "odom_topic"),
        }
        for flag, (val, key) in topic_map.items():
            if val and val != DEFAULTS[key]:
                cmd += [flag, val]

        # Numeric params (only if changed from default)
        all_param_vars = {**self._res_vars, **self._perf_vars}
        for key, var in all_param_vars.items():
            val = var.get().strip()
            if val and val != DEFAULTS[key]:
                cmd += [f"--{key}", val]

        return cmd

    def _update_command(self, *_):
        cmd = self._build_cmd_list()
        if cmd:
            display = format_command_display(cmd)
        else:
            display = "# Fill in Mount Dir, Input Bag, and Output Path to generate command."

        self._cmd_text.config(state="normal")
        self._cmd_text.delete("1.0", "end")
        self._cmd_text.insert("1.0", display)
        self._cmd_text.config(state="disabled")

        if hasattr(self, "_preview_in_lbl"):
            self._update_path_preview()

    # ── Copy ──────────────────────────────────────────────────────────────

    def _copy_command(self):
        cmd = self._build_cmd_list()
        if not cmd:
            messagebox.showwarning("Nothing to Copy",
                                   "Please fill in all required fields first.")
            return
        self.clipboard_clear()
        self.clipboard_append(shlex.join(cmd))
        self._copy_btn.config(text="✓  Copied!")
        self.after(1800, lambda: self._copy_btn.config(text="⎘  Copy Command"))

    # ── Run / Stop ────────────────────────────────────────────────────────

    def _run(self):
        cmd = self._build_cmd_list()
        if not cmd:
            messagebox.showerror("Missing Fields",
                                 "Please fill in:\n  • Mount Dir\n  • Input Bag\n  • Output Path")
            return

        mount = self._mount_var.get().strip()
        if not Path(mount).is_dir():
            messagebox.showerror("Invalid Mount Dir",
                                 f"Directory does not exist:\n{mount}")
            return

        self._clear_console()
        self._log("$ " + shlex.join(cmd) + "\n")
        self._log("─" * 70 + "\n\n")

        self._run_btn.config(state="disabled")
        self._stop_btn.config(state="normal")
        self._status_var.set("⏳  Running…")

        threading.Thread(target=self._worker, args=(cmd,), daemon=True).start()

    def _worker(self, cmd: list[str]):
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            for line in self._proc.stdout:
                self._log_q.put(line)
            self._proc.wait()
            rc = self._proc.returncode
            self._log_q.put("\n" + "─" * 70 + "\n")
            if rc == 0:
                self._log_q.put("✅  Finished successfully.\n")
            else:
                self._log_q.put(f"❌  Process exited with code {rc}.\n")
        except FileNotFoundError:
            self._log_q.put(
                "❌  'docker' not found.\n"
                "    Make sure Docker Desktop is running and 'docker' is in PATH.\n"
            )
        except Exception as exc:
            self._log_q.put(f"❌  Unexpected error: {exc}\n")
        finally:
            self._log_q.put(None)    # sentinel → re-enable Run button

    def _stop(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            self._status_var.set("Stopped by user.")
            self._log("\n⚠  Process terminated by user.\n")

    # ── Console helpers ───────────────────────────────────────────────────

    def _log(self, text: str):
        self._console.config(state="normal")
        self._console.insert("end", text)
        self._console.see("end")
        self._console.config(state="disabled")

    def _clear_console(self):
        self._console.config(state="normal")
        self._console.delete("1.0", "end")
        self._console.config(state="disabled")

    def _poll_log_queue(self):
        try:
            while True:
                msg = self._log_q.get_nowait()
                if msg is None:
                    self._run_btn.config(state="normal")
                    self._stop_btn.config(state="disabled")
                    self._status_var.set("Done.")
                    break
                self._log(msg)
        except queue.Empty:
            pass
        self.after(40, self._poll_log_queue)


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = BagToMapGUI()
    app.mainloop()
