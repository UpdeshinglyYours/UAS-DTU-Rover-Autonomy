#!/usr/bin/env python3
"""
UAS-DTU Rover Autonomy - Node Resource Monitor (top-style)
Monitors CPU % and Memory (MB / %) used by each active ROS 2 / Autonomy node.
The display stays pinned at the top of the terminal without scrolling.
"""

import os
import sys
import time
import argparse
import signal
import shutil
import psutil

# ANSI Colors & Terminal Controls
C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_DIM = "\033[2m"
C_CYAN = "\033[36m"
C_GREEN = "\033[32m"
C_YELLOW = "\033[33m"
C_RED = "\033[31m"
C_BLUE = "\033[34m"
C_MAGENTA = "\033[35m"
C_BG_BLUE = "\033[44m"
C_WHITE = "\033[37m"

TERM_HOME = "\033[H"
TERM_CLEAR_DOWN = "\033[J"
TERM_CLEAR_LINE = "\033[K"
TERM_HIDE_CURSOR = "\033[?25l"
TERM_SHOW_CURSOR = "\033[?25h"


def cleanup(*args):
    """Restore terminal settings upon exit."""
    sys.stdout.write(TERM_SHOW_CURSOR + C_RESET + "\n")
    sys.stdout.flush()
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)


def make_bar(percent, width=15):
    """Render a visual ASCII bar for CPU/Memory."""
    filled = int(round(width * (percent / 100.0)))
    filled = max(0, min(width, filled))
    empty = width - filled

    if percent > 80:
        color = C_RED
    elif percent > 50:
        color = C_YELLOW
    else:
        color = C_GREEN

    return f"{color}[{'|' * filled}{'.' * empty}]{C_RESET}"


def is_ros_process(proc, info, include_all=False):
    """Determine whether a process is an active ROS / Autonomy stack node."""
    pid = proc.pid
    if pid == os.getpid():
        return False

    name = info.get('name') or ''
    cmdline = info.get('cmdline') or []
    if not cmdline:
        return False

    # Exclude self and monitor scripts
    if 'node_monitor' in name or any('node_monitor' in arg for arg in cmdline):
        return False

    # Exclude system shells, editors, compilers, and development tools
    excluded_names = (
        'bash', 'sh', 'zsh', 'dash', 'code', 'chrome', 'gedit', 'nano', 'vim',
        'grep', 'colcon', 'cmake', 'make', 'gcc', 'g++', 'agy', 'node', 'git',
        'systemd', 'dbus-daemon', 'snap', 'snapd', 'crashpad'
    )
    if any(name.startswith(x) for x in excluded_names):
        return False

    exe = ''
    try:
        exe = proc.exe()
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        pass

    if any(x in exe for x in ['/snap/', '/usr/share/code', 'antigravity']):
        return False

    if include_all:
        return True

    # 1. Official ROS executable path
    if '/opt/ros/' in exe or '/install/' in exe:
        return True

    # 2. Contains ROS argument flags or remap
    if any(arg in ('--ros-args', '-r', '--remap') or '__node:=' in arg for arg in cmdline):
        return True

    # 3. Known autonomy workspace scripts
    if any(arg.endswith('.py') for arg in cmdline):
        for arg in cmdline:
            if arg.endswith('.py') and any(
                ws in arg for ws in [
                    'uas_nav', 'DARPA_ws', 'Mavros_ws', 'Blickfeld_ws',
                    'lirovo', 'casualty_dispatcher', 'omega', 'odom_to_tf',
                    'map_expander', 'converter', 'depth_to_pointcloud'
                ]
            ):
                return True

    # 4. Known ROS nodes
    known_nodes = (
        'controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
        'lifecycle_manager', 'pointcloud_to_laserscan_node', 'static_transform_publisher',
        'mavros_node', 'blickfeld_driver_node', 'rviz2', 'robot_state_publisher',
        'component_container', 'component_container_isolated', 'ekf_node',
        'slam_toolbox', 'async_slam_toolbox_node', 'sync_slam_toolbox_node',
        'sf45b', 'lightwarelidar'
    )
    if any(name == kn or name.startswith(kn) for kn in known_nodes):
        return True

    return False


def extract_node_name(cmdline, proc_name):
    """Extract a clean, human-readable ROS node name from process command line."""
    if not cmdline:
        return proc_name

    # 1. Check for explicit ROS 2 node remap: __node:=<name>
    for i, arg in enumerate(cmdline):
        if '__node:=' in arg:
            return arg.split('__node:=')[-1]
        if arg in ('-r', '--remap') and i + 1 < len(cmdline) and '__node:=' in cmdline[i + 1]:
            return cmdline[i + 1].split('__node:=')[-1]

    # 2. Check for python scripts (e.g. casualty_dispatcher.py, new_omega_fixed5.py)
    for arg in cmdline:
        if arg.endswith('.py'):
            base = os.path.basename(arg)
            # Give friendly names for key rover controllers
            if 'omega' in base:
                return f"{base} (omega_ctrl)"
            if 'casualty' in base:
                return f"{base} (dispatcher)"
            return base

    # 3. Handle static_transform_publisher frame inspection
    base_exe = os.path.basename(cmdline[0])
    if 'static_transform_publisher' in base_exe or proc_name == 'static_transform_publisher':
        frames = [a for a in cmdline[1:] if not a.startswith('-') and not a.startswith('{')]
        if len(frames) >= 2:
            return f"static_tf ({frames[-2]}->{frames[-1]})"
        return "static_transform_publisher"

    # 4. Handle launch file parent process
    if base_exe == 'ros2' and len(cmdline) >= 3 and cmdline[1] == 'launch':
        launch_file = os.path.basename(cmdline[2])
        return f"launch ({launch_file})"

    # 5. Handle MAVROS, Blickfeld, and SF45/B
    if 'mavros_node' in base_exe:
        return "mavros (apm)"
    if 'blickfeld_driver_node' in base_exe:
        return "bf_lidar (blickfeld)"
    if 'sf45b' in base_exe or proc_name == 'sf45b':
        return "sf45b (lightware)"

    # Default to base executable
    return base_exe.replace('_node', '')


def format_uptime(create_time):
    """Format uptime as HH:MM:SS or MM:SS."""
    secs = int(time.time() - create_time)
    if secs < 0:
        secs = 0
    m, s = divmod(secs, 60)
    h, m = divmod(m, 60)
    if h > 0:
        return f"{h:02d}:{m:02d}:{s:02d}"
    return f"{m:02d}:{s:02d}"


def get_cpu_color(val):
    if val >= 50.0:
        return C_RED
    if val >= 15.0:
        return C_YELLOW
    return C_GREEN


def get_mem_color(mb):
    if mb >= 1000.0:
        return C_RED
    if mb >= 400.0:
        return C_YELLOW
    return C_CYAN


def main():
    parser = argparse.ArgumentParser(
        description="Continuously monitor CPU & Memory for ROS 2 autonomy nodes (pinned to top)."
    )
    parser.add_argument(
        "-i", "--interval", type=float, default=1.0,
        help="Refresh interval in seconds (default: 1.0s)"
    )
    parser.add_argument(
        "-s", "--sort", choices=["cpu", "mem", "name", "pid"], default="cpu",
        help="Sort column (default: cpu)"
    )
    parser.add_argument(
        "-a", "--all", action="store_true",
        help="Monitor all processes instead of filtering for ROS nodes"
    )
    args = parser.parse_args()

    # Track process objects across ticks so cpu_percent calculates properly
    tracked_procs = {}

    # Initialize terminal
    sys.stdout.write("\033[2J" + TERM_HIDE_CURSOR)
    sys.stdout.flush()

    try:
        while True:
            # 1. System-wide stats
            total_cpu = psutil.cpu_percent(interval=None)
            mem_info = psutil.virtual_memory()
            total_ram_gb = mem_info.total / (1024 ** 3)
            used_ram_gb = mem_info.used / (1024 ** 3)
            ram_percent = mem_info.percent
            num_cpus = psutil.cpu_count(logical=True)

            # 2. Discover active processes
            active_pids = set()
            node_entries = []

            for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'create_time']):
                try:
                    pid = proc.pid
                    info = proc.info
                    if not is_ros_process(proc, info, include_all=args.all):
                        continue

                    active_pids.add(pid)
                    if pid not in tracked_procs:
                        tracked_procs[pid] = proc
                        # Prime cpu counter
                        proc.cpu_percent(interval=None)

                    p = tracked_procs[pid]
                    cpu = p.cpu_percent(interval=None)
                    mem_rss = p.memory_info().rss / (1024 * 1024)  # MB
                    mem_pct = p.memory_percent()
                    threads = p.num_threads()
                    node_name = extract_node_name(info.get('cmdline'), info.get('name'))
                    uptime = format_uptime(info.get('create_time', time.time()))

                    cmd_str = " ".join(info.get('cmdline', []))
                    if len(cmd_str) > 60:
                        cmd_str = cmd_str[:57] + "..."

                    node_entries.append({
                        'pid': pid,
                        'name': node_name,
                        'cpu': cpu,
                        'mem_mb': mem_rss,
                        'mem_pct': mem_pct,
                        'threads': threads,
                        'uptime': uptime,
                        'cmd': cmd_str
                    })
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass

            # Prune dead processes from cache
            tracked_procs = {pid: p for pid, p in tracked_procs.items() if pid in active_pids}

            # 3. Sort entries
            if args.sort == "cpu":
                node_entries.sort(key=lambda x: x['cpu'], reverse=True)
            elif args.sort == "mem":
                node_entries.sort(key=lambda x: x['mem_mb'], reverse=True)
            elif args.sort == "name":
                node_entries.sort(key=lambda x: x['name'].lower())
            elif args.sort == "pid":
                node_entries.sort(key=lambda x: x['pid'])

            # 4. Aggregate autonomy stack metrics
            autonomy_cpu = sum(x['cpu'] for x in node_entries)
            autonomy_mem_mb = sum(x['mem_mb'] for x in node_entries)
            node_count = len(node_entries)

            # 5. Build display buffer (PINNED AT TOP)
            cols, rows = shutil.get_terminal_size((80, 24))
            buf = []
            buf.append(TERM_HOME)  # Jump cursor to (1,1)

            # Title Header
            now_str = time.strftime("%Y-%m-%d %H:%M:%S")
            title = f" UAS-DTU ROVER AUTONOMY - NODE RESOURCE MONITOR "
            header_line = f"{C_BOLD}{C_BG_BLUE}{C_WHITE}{title.center(cols)}{C_RESET}{TERM_CLEAR_LINE}\n"
            buf.append(header_line)

            # System summary cards
            cpu_bar = make_bar(total_cpu, width=14)
            ram_bar = make_bar(ram_percent, width=14)
            buf.append(
                f" {C_BOLD}Total CPU:{C_RESET} {cpu_bar} {total_cpu:5.1f}% ({num_cpus} cores) "
                f"| {C_BOLD}Total RAM:{C_RESET} {ram_bar} {used_ram_gb:4.1f}/{total_ram_gb:4.1f} GB ({ram_percent:4.1f}%)"
                f"{TERM_CLEAR_LINE}\n"
            )
            buf.append(
                f" {C_BOLD}Autonomy Stack:{C_RESET} {C_CYAN}{node_count}{C_RESET} active nodes "
                f"| Stack CPU: {get_cpu_color(autonomy_cpu)}{autonomy_cpu:5.1f}%{C_RESET} "
                f"| Stack RAM: {get_mem_color(autonomy_mem_mb)}{autonomy_mem_mb:6.1f} MB{C_RESET} "
                f"| Time: {C_DIM}{now_str}{C_RESET}"
                f"{TERM_CLEAR_LINE}\n"
            )
            buf.append(f"{C_DIM}{'─' * cols}{C_RESET}{TERM_CLEAR_LINE}\n")

            # Table Header
            show_details = cols >= 100
            name_width = 30 if show_details else 26

            tbl_hdr = (
                f"{C_BOLD}{'NODE / PROCESS NAME':<{name_width}} "
                f"{'PID':>7} "
                f"{'CPU %':>8} "
                f"{'RAM (MB)':>10} "
                f"{'RAM %':>7} "
                f"{'THREADS':>8} "
                f"{'UPTIME':>9}"
            )
            if show_details:
                tbl_hdr += f"  {'COMMAND / LAUNCH DETAILS'}"
            tbl_hdr += f"{C_RESET}{TERM_CLEAR_LINE}\n"
            buf.append(tbl_hdr)
            buf.append(f"{C_DIM}{'─' * cols}{C_RESET}{TERM_CLEAR_LINE}\n")

            # Table Rows
            max_rows = max(3, rows - 8)
            visible_entries = node_entries[:max_rows]

            if not visible_entries:
                buf.append(
                    f"{C_YELLOW}  Waiting for ROS 2 autonomy nodes to start... (No active nodes detected){C_RESET}{TERM_CLEAR_LINE}\n"
                )
                buf.append(
                    f"{C_DIM}  (Launch lirovo.launch.py, mavros, blickfeld, or new_omega_fixed5 to see stats here){C_RESET}{TERM_CLEAR_LINE}\n"
                )
            else:
                for entry in visible_entries:
                    cpu_c = get_cpu_color(entry['cpu'])
                    mem_c = get_mem_color(entry['mem_mb'])
                    name_disp = entry['name']
                    if len(name_disp) > name_width:
                        name_disp = name_disp[:name_width - 1] + "…"

                    row_str = (
                        f"{C_BOLD}{name_disp:<{name_width}}{C_RESET} "
                        f"{entry['pid']:>7} "
                        f"{cpu_c}{entry['cpu']:>7.1f}%{C_RESET} "
                        f"{mem_c}{entry['mem_mb']:>9.1f}M{C_RESET} "
                        f"{entry['mem_pct']:>6.1f}% "
                        f"{entry['threads']:>8} "
                        f"{C_DIM}{entry['uptime']:>9}{C_RESET}"
                    )
                    if show_details:
                        row_str += f"  {C_DIM}{entry['cmd']}{C_RESET}"
                    row_str += f"{TERM_CLEAR_LINE}\n"
                    buf.append(row_str)

            # Footer / Hotkeys
            buf.append(f"{C_DIM}{'─' * cols}{C_RESET}{TERM_CLEAR_LINE}\n")
            buf.append(
                f"{C_DIM} [Ctrl+C] Exit  |  Interval: {args.interval}s  |  Sort: {args.sort.upper()}  |  Pinned to top{C_RESET}"
                f"{TERM_CLEAR_LINE}\n"
            )

            # Erase any remaining lines from previous renders
            buf.append(TERM_CLEAR_DOWN)

            # Push atomic frame to stdout
            sys.stdout.write("".join(buf))
            sys.stdout.flush()

            time.sleep(args.interval)

    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()
