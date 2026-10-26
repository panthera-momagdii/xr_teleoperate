import time
import numpy as np  # [RT5] session stats percentiles
import argparse
from multiprocessing import Value, Array, Lock
import threading
# [panthera] RT4. Must run before `import logging_mp`: its listener is the first child
# this process forks. Every multiprocessing child (logging_mp's listener, televuer's
# vuer server, the hand controllers' control loops) is then started with
# PR_SET_PDEATHSIG=SIGKILL, so a launcher killed with SIGKILL -- which runs no finally,
# no atexit, no handler -- cannot leave a child publishing hand commands or holding 8012.
import os as _os, sys as _sys
_sys.path.append(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
from tools._procs import install_parent_death_signal
install_parent_death_signal()
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_29_Arm_Internal_Dex1_Controller, G1_23_ArmController, H1_2_ArmController, H1_ArmController, H2_ArmController, R1_A5_ArmController, R1_A7_ArmController
from teleop.robot_control import hand_config
from teleop.robot_control import wrist_offset
from teleop.robot_control import start_pose as start_pose_mod
from teleop.robot_control import link_guard as link_guard_mod
from teleop.robot_control import torque_cap as torque_cap_mod
from teleop.robot_control import start_gate as start_gate_mod
from teleop.robot_control import ik_worker as ik_worker_mod
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK, R1_A5_ArmIK, R1_A7_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# [panthera] Exit codes. 0 clean, 1 an exception got out, 2 argparse (its own default).
# Everything above 2 is a pre-flight refusal that a script can act on without parsing
# log text. Keep these stable -- RUNBOOK_PARTB.md documents them.
EXIT_NO_EE_STATE = 3     # an --ee was given and its state topic never arrived
EXIT_PORT_BUSY   = 4     # the XR port (8012) is already held by another process
EXIT_HANGUP      = 5     # [RT4] SIGHUP: the operator's terminal went away; orderly stop as [q]
EXIT_TERMINATED  = 6     # [RT4] SIGTERM: asked to stop; orderly stop as [q]
EXIT_START_GATE  = 7     # [RT6f] an arm joint within XR_GATE_LIMIT_MARGIN of a limit at start


def notice(text):
    """Operator-facing text, printed verbatim to stderr.

    [panthera] logging_mp renders through rich, which word-wraps every message into a
    narrow column, breaks long tokens across lines -- DDS topic names and file paths
    come out as "rt/dex1/left/stat e" and "/home/.../cert.pe m" -- and injects the
    source-location gutter INTO the first line of the text. That is acceptable for
    tracing and useless for a refusal an operator has to act on at 2 a.m. on a robot
    day, so anything that names a topic, a path or a PID goes out here unwrapped as
    well as to the log.
    """
    # [RT4] After the operator's terminal dies, stderr can raise EIO. A log line must
    # never be what stops the shutdown from reaching go-home and the child reaping.
    try:
        print(text, file=sys.stderr, flush=True)
    except (OSError, ValueError):
        pass


# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
REENGAGE       = False  # [RT4] [r] pressed while already tracking: ask the link guard to re-engage
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

_ARM_REF = [None]   # [RT7] the arm controller, for "a stop during the lowering = release at once"


def _release_if_lowering(why):
    """[RT7] Owner, 23 Oct: any stop during the lowering releases at once (zero torque)."""
    c = _ARM_REF[0]
    if c is not None and getattr(c, "lowering", False):
        c.release_now(f"{why} during the lowering")
        notice(f"[lower] {why} during the lowering: RELEASED AT ONCE (zero torque)")
        return True
    return False


def on_press(key):
    global STOP, START, RECORD_TOGGLE, REENGAGE
    # [RT5b] One plain line per key, keyboard or IPC: in RT5 an operator's [r] and [q]
    # never reached the launcher and nothing showed it.
    notice(f"[key] {key} received")
    if key == 'q' and _release_if_lowering("[q]"):
        return
    if key == 'r':
        # [RT4] The first [r] starts tracking. Every later [r] is the deliberate
        # re-engage the link guard waits for after a HOLD; it does nothing in FOLLOW.
        if START:
            REENGAGE = True
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'H2', 'R1_A5', 'R1_A7'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex1_internal', 'dex3', 'dex5', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    # network parameters
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    args = parser.parse_args()
    logger_mp.debug(f"args: {args}")

    # [RT5b] GIL switch interval. The IK runs in the main thread; the 250 Hz arm publisher
    # and the DDS readers are Python threads in the same process. At Python's default
    # 5 ms switch interval the IK waits for the GIL: inside the launcher IK p50 was
    # 10 ms and p99 14..21 ms, against 3..4 ms in ik_smoke. Measured with a busy thread:
    # 5 ms -> p50 5.46 / p99 10.05 ms; 0.5 ms -> p50 3.46 / p99 4.63 ms. XR_GIL_SWITCH_S.
    # BUT inside the full launcher on an arm-following fake it did not help (0.5 ms:
    # p50 6.04 / p99 11.22; 5 ms: p50 4.70 / p99 9.66), so the default is Python's own;
    # set XR_GIL_SWITCH_S to experiment.
    try:
        _gil_s = float(os.environ.get("XR_GIL_SWITCH_S", "0"))
        if _gil_s > 0:
            sys.setswitchinterval(_gil_s)
    except ValueError:
        parser.error(f"XR_GIL_SWITCH_S={os.environ.get('XR_GIL_SWITCH_S')!r} is not a number")

    # [panthera] logging_mp forks a NON-DAEMON listener on the first getLogger() and
    # reaps it only from an atexit hook. Ctrl-C and `kill` do not run atexit reliably
    # here, and the orphan inherits the LISTENING SOCKET on 8012 -- so a launcher that
    # is killed leaves a process squatting on the port, and the NEXT launcher refuses
    # to start with exit 4. Four such orphans accumulated while this gate was being
    # written. install_reaper() covers atexit + SIGINT + SIGTERM; nothing can cover
    # SIGKILL, which is why the exit-4 message names the PID to kill.
    # [panthera] RT4: what happens when the operator's terminal dies.
    #
    # The SSH session that carries the [r]/[s]/[q] keys crosses the WAN. When it dies,
    # the kernel sends SIGHUP to the launcher's session. Python's default for SIGHUP is
    # to die on the spot: no finally, so no go-home and no reaping, and the arm thread
    # simply stops publishing mid-command. Defined behaviour instead:
    #
    #   SIGHUP  -> the same orderly stop as [q]: leave the main loop, go home at
    #              XR_ARM_VEL_LIMIT, reap every child, exit 5 (EXIT_HANGUP).
    #   SIGTERM -> the same, exit 6 (EXIT_TERMINATED).
    #   SIGINT  -> the same, exit 0. Not left to KeyboardInterrupt: when the signal lands
    #              inside CasADi's IPOPT solve, CasADi catches the KeyboardInterrupt
    #              itself ("KeyboardInterruptException") and the main loop never sees it.
    #              G7 caught that: 2 of 5 SIGINTs left the launcher running.
    #
    # On SIGHUP stdout/stderr are first moved to logs/launcher_hup_<pid>.log: writes to
    # a hung-up terminal raise EIO, and the shutdown must not trip over its own logging.
    # Run the launcher inside tmux on PC2 and an SSH drop never reaches it at all; the
    # link guard then holds the arms if the headset also drops, and the operator
    # re-attaches and presses [r] or [q].
    #
    # Installed BEFORE install_reaper(), which chains to it: for SIGINT and SIGTERM the
    # reaper kills the children first and then calls this, instead of raising
    # KeyboardInterrupt or os._exit(143).
    import signal as _signal
    _signal_stop = {"sig": None}

    def _detach_from_terminal():
        try:
            _log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
            _fd = os.open(os.path.join(_log_dir, f"launcher_hup_{os.getpid()}.log"),
                          os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        except OSError:
            _fd = os.open(os.devnull, os.O_WRONLY)
        for _target in (1, 2):
            try:
                os.dup2(_fd, _target)
            except OSError:
                pass
        try:
            os.dup2(os.open(os.devnull, os.O_RDONLY), 0)
        except OSError:
            pass

    def _orderly_stop(signum, frame):
        global START, STOP
        _release_if_lowering(f"signal {_signal.Signals(signum).name}")
        first = _signal_stop["sig"] is None
        if first:
            _signal_stop["sig"] = signum
            if signum == _signal.SIGHUP:
                _detach_from_terminal()
        START = False
        STOP = True
        if first:
            _code = {_signal.SIGHUP: EXIT_HANGUP, _signal.SIGTERM: EXIT_TERMINATED}.get(signum, 0)
            notice(f"[signal] {_signal.Signals(signum).name}: orderly stop, same as [q] "
                   f"(go home, reap children, exit {_code})")

    for _sig in (_signal.SIGHUP, _signal.SIGTERM, _signal.SIGINT):
        try:
            _signal.signal(_sig, _orderly_stop)
        except (ValueError, OSError):
            pass

    from tools._procs import install_reaper, reap_child_processes
    install_reaper(log=logger_mp)

    if args.ee == "dex1_internal" and args.motion:
        parser.error("--ee dex1_internal does not currently support --motion.")

    # [panthera] Checked here, before the try block: ChannelFactoryInitialize and
    # MotionSwitcher().Enter_Debug_Mode() both run inside it, and Enter_Debug_Mode
    # releases the robot's motion control. Nothing that touches the robot may happen
    # on the way to reporting a bad argument combination.
    if args.ee == "dex5" and args.sim:
        parser.error("--ee dex5 has no simulation target (unitree_sim_isaaclab ships "
                     "Dex3 only); use --ee dex3 with --sim, or drop --sim")

    # [panthera] The hand model follows from --ee. Choosing an end effector IS choosing a
    # hand, and requiring a matching env var alongside it is a way to end up commanding
    # one hand's topics with another fitted. HAND_MODEL remains available as an explicit
    # override for bench work, but if it disagrees with --ee that is an operator mistake,
    # not a preference -- so it is an error here, before any DDS init and long before
    # Enter_Debug_Mode, rather than a surprise at the first command.
    ee_model_error = hand_config.select_model_for_ee(args.ee)
    if ee_model_error:
        parser.error(ee_model_error)

    # [panthera] ---- static pre-flight, before ANY of this touches the robot ----------
    #
    # Both checks below are properties of this host, not of the robot, so they run
    # before the try block: before ChannelFactoryInitialize, before Enter_Debug_Mode,
    # and before the image server is contacted. A plain sys.exit() here cannot be
    # swallowed by the finally block's own exit().

    # The certificate televuer will serve. One line, always printed. A regenerated cert
    # invalidates the Quest's stored exception, and on 2026-09-09 that cost a session:
    # nothing named the file or its fingerprint, so the first symptom was a headset that
    # would not connect. See tools/cert_info.py and logs/overnight/cert_evidence/.
    try:
        from tools.cert_info import oneline as _cert_oneline
        _cert_line = _cert_oneline()
        notice(_cert_line)
        logger_mp.info(_cert_line)
    except Exception as exc:                      # never block a session on a log line
        logger_mp.warning(f"[cert] could not describe the certificate: {exc}")

    # The XR port. Vuer binds it inside its own aiohttp startup thread, and the launcher
    # never learns that the bind failed: on 2026-09-09 quest_link_check.py was still
    # holding 8012, the launcher ran on with no XR data, and [r] drove the arms to
    # televuer's fallback pose instead of following the operator. Refuse instead.
    from tools import port_guard
    try:
        from vuer import Vuer as _Vuer
        _xr_port = int(os.environ.get("XR_VUER_PORT", getattr(_Vuer, "port", 8012)))
        _vuer_free_port = getattr(_Vuer, "free_port", None)
    except Exception:
        _xr_port = int(os.environ.get("XR_VUER_PORT", "8012"))
        _vuer_free_port = None
    if _vuer_free_port:
        # Vuer has been told to pick any free port, so a busy 8012 is not fatal --
        # but the headset URL will not be the one in the runbook.
        logger_mp.warning(f"[xr] Vuer.free_port={_vuer_free_port} is set; skipping the "
                          f"port {_xr_port} check. The headset URL will NOT be :{_xr_port}.")
    elif not port_guard.port_is_free(_xr_port):
        _busy = port_guard.describe_busy(_xr_port)
        notice(f"[xr] REFUSING TO START (exit {EXIT_PORT_BUSY})\n{_busy}\n"
               f"    vuer needs {_xr_port} and cannot have it. Running on would give a\n"
               f"    dead XR path, and [r] would move the arms to a default pose with\n"
               f"    nothing to follow.")
        logger_mp.error(_busy)
        sys.exit(EXIT_PORT_BUSY)
    else:
        notice(f"[xr] port {_xr_port} is free")
        logger_mp.info(f"[xr] port {_xr_port} is free")

    # [panthera] Make XR_VUER_PORT move the BIND, not just the check -- checking one
    # port and serving another would be worse than having no knob at all. televuer is
    # an upstream submodule and constructs Vuer() with no port argument, so the port is
    # vuer's params_proto class attribute; setting it here is equivalent to passing
    # port= and needs no submodule change. Left alone at the default, so the ordinary
    # path constructs exactly the Vuer it always did.
    if _xr_port != 8012:
        try:
            from vuer import Vuer as _VuerCls
            _VuerCls.port = _xr_port
            logger_mp.info(f"[xr] vuer will serve on {_xr_port} (XR_VUER_PORT)")
        except Exception as exc:
            logger_mp.error(f"[xr] could not move vuer to port {_xr_port}: {exc}")
            sys.exit(EXIT_PORT_BUSY)

    # [panthera] The operator-to-robot wrist mapping (G3). Resolved HERE, before any DDS
    # init, so a typo in XR_WRIST_Z_SCALE stops the session while the robot still holds
    # itself up rather than at the first frame after [r]. All four knobs default to the
    # identity, and at the identity apply() is skipped entirely -- byte-for-byte the
    # behaviour of the unmodified code.
    try:
        wrist_map = wrist_offset.from_env()
    except ValueError as exc:
        parser.error(str(exc))
    notice(wrist_map.describe())
    logger_mp.info(wrist_map.describe())

    # [panthera] Read here rather than beside set_arm_velocity_limit(): it is pure env,
    # and the start-pose sequencer below needs it before any DDS exists.
    arm_velocity_limit = float(os.environ.get("XR_ARM_VEL_LIMIT", "30.0"))

    # [panthera] The fixed start pose (G4). A pose FILE is a static input like the port
    # and the wrist knobs, so it is parsed and refused HERE -- before ChannelFactory-
    # Initialize and long before Enter_Debug_Mode. A typo in the YAML must not cost a
    # go-home with the arms released. The pose is re-checked against the real URDF joint
    # limits once the IK model exists; this pass catches everything that does not need
    # the model (missing joints, extra joints, nan, unreadable file).
    try:
        start_seq = start_pose_mod.from_env(velocity_limit=arm_velocity_limit)
    except start_pose_mod.StartPoseError as exc:
        parser.error(str(exc))
    notice(start_seq.describe())
    logger_mp.info(start_seq.describe())

    # [RT6b] Exit lowering knobs, parsed and refused before any DDS like the others.
    #   XR_LOWER_SPEED  the most any arm joint may move on the way down (rad/s, default
    #                   0.3, never above XR_ARM_VEL_LIMIT). [RT6d] The plan is min-jerk
    #                   peaking at 60 % of it, and a governor slows the plan above 70 %
    #                   measured (RT6b planned AT this speed and measured 0.47 rad/s).
    #   XR_LOWER_PROFILE minjerk (default) or linear (the RT6b plan, for comparison)
    #   XR_RELEASE_T    gain ramp-down after the rest pose is reached (s, default 1.5)
    #   XR_REST_POSE    optional yaml (start-pose format); default = the arm pose
    #                   measured when the arm controller starts (the hanging pose)
    try:
        lower_speed = min(float(os.environ.get("XR_LOWER_SPEED", "0.3")), arm_velocity_limit)
        release_t = float(os.environ.get("XR_RELEASE_T", "1.5"))
        if not (lower_speed > 0 and release_t >= 0):
            raise ValueError("XR_LOWER_SPEED must be > 0 and XR_RELEASE_T >= 0")
        lower_profile = os.environ.get("XR_LOWER_PROFILE", "minjerk").strip().lower()
        if lower_profile not in ("minjerk", "linear"):
            raise ValueError(f"XR_LOWER_PROFILE={lower_profile!r}: use minjerk or linear")
        rest_pose = (start_pose_mod.load_pose(os.environ["XR_REST_POSE"])
                     if os.environ.get("XR_REST_POSE") else None)
    except (ValueError, start_pose_mod.StartPoseError) as exc:
        parser.error(f"[lower] {exc}")
    def _kd_text():
        _ka = float(os.environ.get('XR_LOWER_KD', '') or G1_29_ArmController.LOWER_ARM_KD)
        _kw = float(os.environ.get('XR_LOWER_WRIST_KD', '') or G1_29_ArmController.LOWER_WRIST_KD)
        if not _ka and not _kw:
            return "kd as while following (no gain raised)"
        return (f"kd {_ka:g} on shoulders and elbows" if _ka else "shoulders/elbows kd as following") + \
               (f", {_kw:g} on wrists" if _kw else ", wrists kd as following")
    _lw_line = (f"[lower] at exit: arms lowered to the "
                f"{'XR_REST_POSE yaml' if rest_pose is not None else 'pose measured at start'} "
                + (f"at {lower_speed:g} rad/s (linear, RT6b plan)" if lower_profile == "linear" else
                   f"with no joint above {lower_speed:g} rad/s (min-jerk plan peaking at "
                   f"{0.6 * lower_speed:.2f}, wrists at "
                   f"{min(float(os.environ.get('XR_LOWER_WRIST_SPEED', '0.15') or 0.15), lower_speed):.2f}, "
                   f"slowed above {0.7 * lower_speed:.2f} measured; [RT6h] command rate limited to "
                   f"{min(float(os.environ.get('XR_LOWER_CMD_VEL', '') or G1_29_ArmController.LOWER_CMD_VEL), lower_speed):g} rad/s, "
                   + _kd_text()
                   + ")")
                + f", gravity feed-forward, then gains released over {release_t:g} s")
    notice(_lw_line)
    logger_mp.info(_lw_line)

    # [RT6d] Shoulder pitch torque cap while following (XR_SHOULDER_TAU_CAP, Nm, default
    # 4, 0 = off). Parsed here, before any DDS; built once the IK model exists.
    try:
        tau_cap_limit = torque_cap_mod.limit_from_env()
    except torque_cap_mod.TorqueCapError as exc:
        parser.error(str(exc))

    # [RT6f] Arm start gate knobs (XR_GATE_LIMIT_MARGIN, XR_GATE_MOVED_TOL, XR_LAST_POSE_FILE,
    # XR_GATE_WAIT_S). The gate itself runs right after DDS init, before Enter_Debug_Mode.
    try:
        gate_margin, gate_moved_tol, last_pose_path = start_gate_mod.settings_from_env()
        gate_wait_s = float(os.environ.get("XR_GATE_WAIT_S", "5"))
    except ValueError as exc:
        parser.error(f"[gate] {exc}")

    # [panthera] RT4 link-loss guard. Env only (XR_STALE_S, XR_REENGAGE_T), so it is
    # parsed and refused here, before any DDS, like the other knobs.
    try:
        link_guard = link_guard_mod.from_env(velocity_limit=arm_velocity_limit, notify=notice)
    except link_guard_mod.LinkGuardError as exc:
        parser.error(str(exc))
    _lg_line = (f"[link] guard: HOLD after {link_guard.stale_s:g} s without a new XR pose; "
                f"re-engage with [r], blend {link_guard.reengage_t:g} s at "
                f"{arm_velocity_limit:g} rad/s max")
    notice(_lg_line)
    logger_mp.info(_lg_line)

    # [panthera] Defined before the try so the finally block can distinguish "the arm
    # controller was never built" from "it was built and we are shutting down". Without
    # this a pre-flight refusal ends in a spurious "Failed to ctrl_dual_arm_go_home:
    # name 'arm_ctrl' is not defined", which reads like a second, unrelated fault.
    arm_ctrl = None
    tau_cap = None          # [RT6d] built with the G1_29 IK; other arms run uncapped
    _ik_lo = _ik_hi = None  # [RT6g] IK joint-margin bounds (G1_29 only)
    had_exception = False        # [panthera] see the except/finally at the bottom
    # [panthera] An explicit override for the exit status. SystemExit raised inside the
    # try block reaches `finally`, whose own exit() would otherwise replace the code
    # with 0/1 and throw away which pre-flight refused.
    exit_code = None
    # [RT6h] The IK worker process is started here, before DDS is initialized: nothing but
    # this thread runs yet, so there is nothing to fork past. Attached to the launcher's
    # own G1_29_ArmIK once that exists (XR_IK_WORKER=0: in-process, as before).
    ik_worker = None
    if args.arm == "G1_29" and ik_worker_mod.enabled_from_env():
        ik_worker = ik_worker_mod.IKWorker(log=notice)

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # [panthera] Hand pre-flight, deliberately the FIRST thing after DDS init.
        #
        # Ordering matters more than it looks. Enter_Debug_Mode() below releases the
        # robot's own motion control; from that moment a refusal costs a go-home with
        # the arms limp and leaves debug mode active. Dex5_1_Controller is built much
        # later still, so its (correct) fail-closed refusal would land in `finally`.
        # Checking here means a wrong or silent hand stops the session while the robot
        # is still holding itself up, and before the image server is even contacted.
        #
        # The controller keeps its own identical check: this is defence in depth, not a
        # replacement. preflight() closes its subscribers before returning.
        # dex5 is the parked lane; inspire_ftp is the hand actually fitted. Both refuse
        # here rather than after the release.
        #
        # [panthera] Every OTHER --ee family gets a bounded check too, from the same
        # place. Their controllers all wait for a state topic in an unbounded loop
        # (hand_config.EE_STATE_TOPICS lists them with line numbers), so before this
        # `--ee dex1` on a robot with no Dex1 hung forever printing "Waiting to
        # subscribe dds..." at 100 Hz. Now it refuses in XR_HAND_WAIT_S and exits 3.
        #
        # --ee stays OPTIONAL: with no --ee nothing here runs, and arms-only
        # teleoperation is unaffected.
        if args.ee is not None:
            _ee_wait_s = float(os.environ.get("XR_HAND_WAIT_S", "10"))
            try:
                if args.ee in ("dex5", "inspire_ftp"):
                    hand_config.preflight(timeout_s=_ee_wait_s, log=logger_mp)
                else:
                    hand_config.ee_state_preflight(args.ee, timeout_s=_ee_wait_s,
                                                   log=logger_mp)
            except RuntimeError as exc:
                notice(f"[ee] REFUSING TO START (exit {EXIT_NO_EE_STATE})\n"
                       f"    {exc}")
                logger_mp.error(str(exc))
                exit_code = EXIT_NO_EE_STATE
                raise SystemExit(EXIT_NO_EE_STATE)

        # [RT6f] Arm start gate, before Enter_Debug_Mode and before the arm controller
        # exists: a refusal here commands nothing, and the pose is never recorded as the
        # rest pose. See teleop/robot_control/start_gate.py.
        if args.arm == "G1_29":
            _g_lo, _g_hi = start_gate_mod.urdf_arm_limits()
            _g_q = start_gate_mod.read_arm_q(timeout_s=gate_wait_s)
            if _g_q is None:
                notice(f"[gate] no rt/lowstate within {gate_wait_s:g} s: arms not checked "
                       f"(the arm controller waits for it next)")
            else:
                _g_refuse, _g_notes = start_gate_mod.check(
                    _g_q, _g_lo, _g_hi, gate_margin, gate_moved_tol, last_pose_path,
                    q_start=start_seq.pose if start_seq.enabled else None,
                    rest_pose_set=rest_pose is not None)
                for _g_line in _g_notes:
                    notice(_g_line)
                    logger_mp.info(_g_line)
                if _g_refuse:
                    notice(_g_refuse)
                    logger_mp.error(_g_refuse)
                    exit_code = EXIT_START_GATE
                    raise SystemExit(EXIT_START_GATE)
            # Defence in depth: the controller refuses to record a rest pose near a limit --
            # only when that measured pose IS the rest pose. With XR_REST_POSE the yaml pose
            # is the rest pose, and it must itself be clear of the limits.
            # [RT6e] The velocity limit applies from the controller's first command.
            G1_29_ArmController.INIT_VEL_LIMIT = arm_velocity_limit
            if rest_pose is None:
                G1_29_ArmController.REST_GUARD = (_g_lo, _g_hi, gate_margin)
            else:
                _g_rv = start_gate_mod.limit_violations(rest_pose, _g_lo, _g_hi, gate_margin)
                if _g_rv:
                    _g_txt = start_gate_mod.describe_violations(_g_rv, gate_margin).replace(
                        "arm joint(s)", "XR_REST_POSE joint(s)")
                    notice(_g_txt)
                    exit_code = EXIT_START_GATE
                    raise SystemExit(EXIT_START_GATE)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            # [RT5c] Keep the terminal settings, so a listener that never stops cannot
            # leave the operator's terminal in raw mode (restored in finally).
            try:
                import termios as _termios
                _tty_saved = _termios.tcgetattr(sys.stdin.fileno()) if sys.stdin.isatty() else None
            except Exception:
                _tty_saved = None
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

        # [RT5b] Headset video knobs (XR_DISPLAY_FPS, XR_JPEG_QUALITY, XR_DISPLAY_SCALE) and
        # the closed-websocket guard. Must be installed before TeleVuerWrapper exists.
        from teleop.utils import televuer_stream
        _tvs_line = televuer_stream.install()
        notice(_tvs_line)
        logger_mp.info(_tvs_line)
        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand", 
                                     binocular=camera_config['head_camera']['binocular'],
                                     img_shape=camera_config['head_camera']['image_shape'],
                                     # maybe should decrease fps for better performance?
                                     # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                     # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                     display_mode=args.display_mode,
                                     zmq=camera_config['head_camera']['enable_zmq'],
                                     webrtc=camera_config['head_camera']['enable_webrtc'],
                                     webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                     arm_reference_mode="head_yaw"
                                     )
        
        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        xr_motion_data_ready = Value('b', False, lock=True)        # [input] whether XR hand/controller motion data has arrived

        if args.ee == "dex1_internal":
            if args.arm != "G1_29":
                raise ValueError("dex1_internal is only supported with --arm G1_29.")
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            tau_cap = torque_cap_mod.ShoulderTorqueCap(
                tau_cap_limit, torque_cap_mod.gravity_fn_for(arm_ik.reduced_robot.model),
                frequency=args.frequency,
                # [RT6h] the slide and the catch-up stay inside the IK's joint bounds
                lower=getattr(arm_ik, "ik_lower", None), upper=getattr(arm_ik, "ik_upper", None))
            notice(tau_cap.describe())
            logger_mp.info(tau_cap.describe())
            # [RT6g] Joint margin after the IK: the IK's own bounds are XR_IK_JOINT_MARGIN
            # inside the URDF limits; this clip guarantees it (IPOPT tolerance, the IK's
            # failure path that returns the measured pose).
            _ik_lo, _ik_hi = getattr(arm_ik, "ik_lower", None), getattr(arm_ik, "ik_upper", None)
            _ik_gfn = torque_cap_mod.gravity_fn_for(arm_ik.reduced_robot.model)
            _mg_line = (f"[ik] every arm joint kept {getattr(arm_ik, 'joint_margin', 0.0):g} rad inside its "
                        f"limit after the IK (XR_IK_JOINT_MARGIN)")
            notice(_mg_line)
            logger_mp.info(_mg_line)
            # [RT6h] The IK in a child process: CasADi holds the GIL for the whole solve
            # and starved the arm controller's 250 Hz publish thread (145 to 190 Hz on the
            # robot in RT6g). arm_ik keeps every other attribute in-process; a failed
            # worker falls back to the in-process solve (XR_IK_WORKER=0 forces that).
            if ik_worker is not None:
                arm_ik = ik_worker.attach(arm_ik)
                notice(arm_ik.describe())
                logger_mp.info(arm_ik.describe())
            if args.ee == "dex1_internal":
                arm_ctrl = G1_29_Arm_Internal_Dex1_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array,
                                                              dual_gripper_action_array, motion_mode=args.motion, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
            else:
                try:
                    arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
                    _ARM_REF[0] = arm_ctrl
                    if os.environ.get("XR_RELEASE_FILE"):
                        notice(f"[lower] release at once (zero torque) on chatter: watching "
                               f"{os.path.basename(os.environ['XR_RELEASE_FILE'])}; a second stop during the "
                               f"lowering releases too")
                except start_gate_mod.StartGateError as exc:
                    notice(str(exc))
                    logger_mp.error(str(exc))
                    exit_code = EXIT_START_GATE
                    raise SystemExit(EXIT_START_GATE)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "H2":
            arm_ik = H2_ArmIK()
            arm_ctrl = H2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "R1_A5":
            arm_ik = R1_A5_ArmIK()
            arm_ctrl = R1_A5_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "R1_A7":
            arm_ik = R1_A7_ArmIK()
            arm_ctrl = R1_A7_ArmController(motion_mode=args.motion, simulation_mode=args.sim)

        # [panthera] One place for every --arm branch. Upstream fixes the limit at 30.0 rad/s
        # in the constructor with no way to lower it; hardware sessions run XR_ARM_VEL_LIMIT=5
        # until we trust the mapping. Default is unchanged, so behaviour without the env var
        # is exactly upstream's.
        # arm_velocity_limit was read before the try (the start-pose sequencer needs it).
        if hasattr(arm_ctrl, "set_arm_velocity_limit"):
            arm_ctrl.set_arm_velocity_limit(arm_velocity_limit)
            logger_mp.info(f"[arm] velocity limit set to {arm_velocity_limit} rad/s "
                           f"(XR_ARM_VEL_LIMIT, default 30.0)")
        else:
            logger_mp.warning(f"[arm] {type(arm_ctrl).__name__} has no set_arm_velocity_limit; "
                              f"XR_ARM_VEL_LIMIT={arm_velocity_limit} NOT applied")

        # [panthera] Now that the IK model exists, re-check the start pose against the
        # REAL URDF joint limits. The early parse could not do this without building the
        # model twice. A pose outside the limits would be clipped by the controller into
        # something nobody chose, so it is refused instead.
        if start_seq.enabled:
            try:
                _model = arm_ik.reduced_robot.model
                start_pose_mod.load_pose(
                    os.environ["XR_START_POSE"],
                    joint_limits=(_model.lowerPositionLimit, _model.upperPositionLimit))
                logger_mp.info("[start-pose] inside every URDF joint limit")
            except start_pose_mod.StartPoseError as exc:
                notice(f"[start-pose] REFUSING TO START\n    {exc}")
                logger_mp.error(str(exc))
                exit_code = 2
                raise SystemExit(2)
            except Exception as exc:
                # Never block a session because the limit CHECK itself broke.
                logger_mp.warning(f"[start-pose] could not re-check joint limits: {exc}")

        # end-effector
        if args.ee in ("dex3", "dex5", "inspire_ftp", "inspire_dfx") and args.input_mode == "controller":
            raise ValueError(f"{args.ee} does not support controller input mode.")
        elif args.ee == "dex5":
            from teleop.robot_control.robot_hand_unitree import Dex5_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 40, lock = False)   # [output] current left, right hand state(40) data.
            dual_hand_action_array = Array('d', 40, lock = False)  # [output] current left, right hand action(40) data.
            hand_ctrl = Dex5_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock,
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "hand":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_hand
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_hand(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                                dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "controller":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_ctrl
            left_gripper_trigger_in = Value('d', 10.0, lock=True)  # [input]
            left_gripper_squeeze_in = Value('d', 0.0, lock=True)   # [input]
            right_gripper_trigger_in = Value('d', 10.0, lock=True) # [input]
            right_gripper_squeeze_in = Value('d', 0.0, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_ctrl(left_gripper_trigger_in, left_gripper_squeeze_in, right_gripper_trigger_in, right_gripper_squeeze_in,
                                                dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        else:
            pass
        
        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            # [panthera] Record WHAT produced the episode. --record is independent of
            # --ee, and arms-only recording is a supported, now routine mode -- but with
            # an all-empty joint_names block an arms-only episode on disk was
            # indistinguishable from one where the hand block silently failed to record.
            # ee_present is the flag that tells them apart; the joint names make the
            # arrays self-describing.
            _ee_dof = {"dex5": 20, "dex3": 7, "dex1": 1, "dex1_internal": 1,
                       "inspire_ftp": 6, "inspire_dfx": 6, "brainco": 6}.get(args.ee, 0)
            _ee_names = {
                "inspire_ftp": list(hand_config.DOF_NAMES),
                "inspire_dfx": list(hand_config.DOF_NAMES),
            }.get(args.ee, [f"{args.ee}_{i}" for i in range(_ee_dof)] if args.ee else [])
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless,
                                     source = {
                                         "arm": args.arm,
                                         "ee": args.ee,
                                         "input_mode": args.input_mode,
                                         "sim": bool(args.sim),
                                         "end_effector": {
                                             "present": args.ee is not None,
                                             "name": args.ee,
                                             "dof_per_side": _ee_dof,
                                         },
                                     },
                                     joint_names = {
                                         "left_arm":  list(start_pose_mod.ARM_JOINT_NAMES[:7]),
                                         "right_arm": list(start_pose_mod.ARM_JOINT_NAMES[7:]),
                                         "left_ee":   _ee_names,
                                         "right_ee":  _ee_names,
                                         "body":      [],
                                     })

        # [panthera] The key prompt goes through notice() as well as the log. rich wraps
        # the logged copy and injects its source-location gutter mid-sentence, so
        # "Press [r] to start syncing..." arrives split across three lines with
        # "teleop_hand_and_arm.py:NNN" in the middle of it. This is the one banner an
        # operator must not miss, and the one line a script waits for.
        _banner = [
            "----------------------------------------------------------------",
            "🟢  Press [r] to start syncing the robot with your movements.",
            ("🟡  Press [s] to START or SAVE recording (toggle cycle)." if args.record
             else "🔵  Recording is DISABLED (run with --record to enable)."),
            "🔴  Press [q] to stop and exit the program.",
            "⚠️  IMPORTANT: Please keep your distance and stay safe.",
        ]
        notice("\n".join(_banner))
        # [RT5b] Where the operator's keys must go.
        try:
            if os.environ.get("TMUX"):
                import subprocess as _sp
                # [RT5c] stdin=DEVNULL: a tmux client that inherits the tty clears
                # O_NONBLOCK on it, which made sshkeyboard's read blocking and hung the
                # exit after [q] (both RT5b rechecks).
                _sess = _sp.run(["tmux", "display-message", "-p", "#S"], capture_output=True,
                                text=True, timeout=3, stdin=_sp.DEVNULL).stdout.strip() or "?"
                notice(f"attach check: tmux ls must show {_sess} (attached)")
            else:
                notice("attach check: NOT inside tmux -- run the launcher in tmux on PC2")
        except Exception:
            pass
        for _line in _banner:
            logger_mp.info(_line)
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            link_guard.observe(time.monotonic(), link_guard_mod.wrist_marker(tv_wrapper.tvuer))
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)

        # [RT6h] The arm controller's 250 Hz publish thread waits for the GIL whenever the
        # main thread runs Python (the torque cap's search, televuer, stats). A thread
        # holding the GIL is asked to drop it after the switch interval: 5 ms by default,
        # more than a control step. 1 ms (XR_GIL_SWITCH_MS) bounds that wait. Set here, for
        # the loop only: start-up (DDS, the controllers) runs at the default.
        try:
            sys.setswitchinterval(max(0.0002, float(os.environ.get("XR_GIL_SWITCH_MS", "1") or 1) / 1000.0))
        except ValueError:
            pass
        notice("--------------------- start Tracking -------------------------")
        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")

        # [panthera] Arm the start-pose sequence from where the arms ACTUALLY ARE at the
        # moment [r] was pressed, not from a reading taken earlier.
        # [RT4] Fresh stream -> FOLLOW. Otherwise HOLD where the arms ARE, and wait for a
        # live stream and a second [r].
        if not STOP:
            REENGAGE = False
            link_guard.start(time.monotonic(), arm_ctrl.get_current_dual_arm_q())
            if tau_cap is not None:
                tau_cap.reset(arm_ctrl.get_current_dual_arm_q())
        if start_seq.enabled and not STOP:
            start_seq.start(time.monotonic(), arm_ctrl.get_current_dual_arm_q())
            notice(f"[start-pose] approaching over {start_seq.start_t:g}s, "
                   f"then blending to the operator over {start_seq.blend_t:g}s")

        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        # [RT5] Session stats, to logs/launcher_stats_<pid>.csv every XR_STATS_S seconds
        # (default 5, 0 = off). File only, so the operator's pane stays readable: loop
        # rate, how many loops saw a NEW wrist pose, IK time, new head frames rendered to
        # the headset, and the guard state. Never allowed to break the loop.
        try:
            _st_period = float(os.environ.get("XR_STATS_S", "5") or 0)
        except ValueError:
            _st_period = 0.0
        _st = {"t0": time.monotonic(), "loops": 0, "fresh": 0, "ik": [], "img_new": 0,
               "img_fps": 0.0, "last_jpg": None, "margin_clips": 0}
        _st_fh = None
        if _st_period > 0:
            try:
                _st_fh = open(os.path.join(parent_dir, "logs",
                                           f"launcher_stats_{os.getpid()}.csv"), "a", buffering=1)
                _st_fh.write("t_mono,loop_hz,fresh_per_s,ik_p50_ms,ik_p99_ms,ik_max_ms,"
                             "img_new_per_s,img_client_fps,guard,"
                             "sp_ff_pred_max_L_nm,sp_ff_pred_max_R_nm,sp_ff_cmd_max_L_nm,"
                             "sp_ff_cmd_max_R_nm,cap_frames_L,cap_frames_R,"
                             "ctrl_steps,clip_steps,clip_steps_L,clip_steps_R,margin_clips\n")
            except OSError:
                _st_fh = None

        _cap_on, _cap_free = [False, False], [0, 0]    # [RT6d] per arm, for the [cap] notices
        # main loop. robot start to follow VR user's motion
        _tlog = {"fh": None}
        if os.environ.get("XR_TARGET_LOG"):
            try:
                _tlog["fh"] = open(os.environ["XR_TARGET_LOG"], "a", buffering=1)
                _tlog["fh"].write("t_mono,guard," + ",".join(f"tgt_{i}" for i in range(15, 29)) + ","
                                  + ",".join(f"meas_{i}" for i in range(15, 29)) + "\n")
            except OSError:
                _tlog["fh"] = None
        while not STOP:
            start_time = time.time()
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
                    _jpg = getattr(head_img, "jpg", None)
                    if _jpg is not None and _jpg is not _st["last_jpg"]:
                        _st["img_new"] += 1
                        _st["last_jpg"] = _jpg
                        _st["img_fps"] = float(getattr(head_img, "fps", 0.0) or 0.0)
            if camera_config['left_wrist_camera']['enable_zmq']:
                if args.record:
                    left_wrist_img = img_client.get_left_wrist_frame()
            if camera_config['right_wrist_camera']['enable_zmq']:
                if args.record:
                    right_wrist_img = img_client.get_right_wrist_frame()

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            # get xr's tele data
            tele_data = tv_wrapper.get_tele_data()
            # [RT4] Freshness of the wrist data, then any [r] the operator gave.
            _t_lg = time.monotonic()
            if link_guard.observe(_t_lg, link_guard_mod.wrist_marker(tv_wrapper.tvuer)):
                _st["fresh"] += 1
            if REENGAGE:
                REENGAGE = False
                link_guard.request_reengage(_t_lg)
            if link_guard.holding:
                pass    # [RT4] HOLD: the hand targets are not updated either; the hands hold.
            elif args.ee in ("dex3", "dex5", "inspire_ftp", "inspire_dfx", "brainco")  and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "brainco" and args.input_mode == "controller":
                with left_gripper_trigger_in.get_lock():
                    left_gripper_trigger_in.value = tele_data.left_ctrl_triggerValue
                with left_gripper_squeeze_in.get_lock():
                    left_gripper_squeeze_in.value = tele_data.left_ctrl_squeezeValue
                with right_gripper_trigger_in.get_lock():
                    right_gripper_trigger_in.value = tele_data.right_ctrl_triggerValue
                with right_gripper_squeeze_in.get_lock():
                    right_gripper_squeeze_in.value = tele_data.right_ctrl_squeezeValue
            elif args.ee in ("dex1", "dex1_internal") and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee in ("dex1", "dex1_internal") and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass
            with xr_motion_data_ready.get_lock():
                xr_motion_data_ready.value = tele_data.motion_data_ready
            
            # high level control
            if args.input_mode == "controller" and args.motion:
                # quit teleoperate
                if tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    loco_wrapper.Damp()
                # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                                  -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                                  -tele_data.right_ctrl_thumbstickValue[0]* 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            # [panthera] Apply the wrist mapping between televuer and the IK. It belongs
            # inside transform_IPunitree_Brobot_world_arm_to_head_then_waist(), but
            # televuer is an upstream submodule and a change there cannot ship in a
            # patch -- so it runs here on the same quantity instead. See
            # teleop/robot_control/wrist_offset.py and docs/xr_frames.md.
            # At the default knobs this returns the input object unchanged.
            left_wrist_target, right_wrist_target = wrist_map.apply_pair(
                tele_data.left_wrist_pose, tele_data.right_wrist_pose)
            sol_q, sol_tauff  = arm_ik.solve_ik(left_wrist_target, right_wrist_target, current_lr_arm_q, current_lr_arm_dq)
            if _ik_lo is not None:
                _q_in = np.clip(sol_q, _ik_lo, _ik_hi)
                if not np.array_equal(_q_in, sol_q):
                    sol_q, sol_tauff = _q_in, _ik_gfn(_q_in)
                    _st["margin_clips"] += 1
            # [RT6d] Shoulder pitch torque cap: past the limit, the reach stops where the
            # limit is met on the way to the IK solution. Pass-through (same objects) below it.
            if tau_cap is not None:
                sol_q, sol_tauff = tau_cap.step(sol_q, sol_tauff, current_lr_arm_q)
                # Notices with 1 s of hysteresis, so an operator at the edge is not a flood;
                # none while the link guard holds (the arms are not following then anyway).
                for _a, _side in enumerate(("left", "right") if not link_guard.holding else ()):
                    if tau_cap.capped[_a]:
                        _cap_free[_a] = 0
                        if not _cap_on[_a]:
                            _cap_on[_a] = True
                            notice(f"[cap] {_side} shoulder pitch at the {tau_cap.limit:g} Nm limit: sliding along it")
                    elif _cap_on[_a]:
                        _cap_free[_a] += 1
                        if _cap_free[_a] >= args.frequency:
                            _cap_on[_a] = False
                            notice(f"[cap] {_side} shoulder pitch back under the limit: following")
            time_ik_end = time.time()
            _st["ik"].append(time_ik_end - time_ik_start)
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            # [panthera] Start-pose sequence (G4). Pass-through unless XR_START_POSE is
            # set. sol_tauff is the gravity feed-forward computed FOR sol_q, so it is
            # zeroed whenever the commanded q is not sol_q -- a torque computed for a
            # pose the arm is not in is worse than no feed-forward at all.
            cmd_q = start_seq.step(time.monotonic(), current_lr_arm_q, sol_q,
                                   dt=1.0 / args.frequency)
            cmd_tauff = sol_tauff if cmd_q is sol_q else sol_tauff * 0.0
            # [RT4] Link guard last: FOLLOW passes through; HOLD freezes; BLEND re-engages.
            cmd_q, cmd_tauff = link_guard.step(time.monotonic(), cmd_q, cmd_tauff,
                                               dt=1.0 / args.frequency)
            arm_ctrl.ctrl_dual_arm(cmd_q, cmd_tauff)
            # [RT7] Per loop: the IK target given to the controller (before its speed limiter),
            # the raw IK solution and the measured arm q, to XR_TARGET_LOG (CSV). Logging only.
            if _tlog["fh"] is not None:
                try:
                    _tlog["fh"].write(f"{time.monotonic():.3f},{link_guard.state},"
                                      + ",".join(f"{v:.4f}" for v in cmd_q) + ","
                                      + ",".join(f"{v:.4f}" for v in current_lr_arm_q) + "\n")
                except Exception:
                    _tlog["fh"] = None

            # record data
            if args.record:
                READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                # dex hand or gripper
                if args.ee == "dex5" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:20]
                        right_ee_state = dual_hand_state_array[-20:]
                        left_hand_action = dual_hand_action_array[:20]
                        right_hand_action = dual_hand_action_array[-20:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee in ("dex1", "dex1_internal") and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee in ("dex1", "dex1_internal") and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                elif (args.ee == "brainco" and args.input_mode == "controller"):
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                # arm state and action (split into left/right halves by the arm's own DOF, so it works for any variant: H1/G1_23/R1_A5 = 4/5 per arm, G1_29/R1_A7 = 7)
                half = len(current_lr_arm_q) // 2
                left_arm_state,  right_arm_state  = current_lr_arm_q[:half], current_lr_arm_q[half:]
                left_arm_action, right_arm_action = sol_q[:half], sol_q[half:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}

                    # [panthera] `img is not None` was not enough: ImageClient returns a
                    # frame OBJECT whose .bgr can itself be None (no image server, or a
                    # dropped frame). That put an empty array into colors, cv2.imwrite
                    # raised, and EpisodeWriter.process_queue dropped THE WHOLE ITEM --
                    # joint data included. Recording offline produced an episode with
                    # "data": [] and a cheerful "Episode saved successfully".
                    # Measured: 149 items recorded, 149 items lost.
                    def _usable(frame):
                        return frame is not None and getattr(frame, "bgr", None) is not None

                    if camera_config['head_camera']['binocular']:
                        if _usable(head_img):
                            colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if _usable(left_wrist_img):
                                colors[f"color_{2}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if _usable(right_wrist_img):
                                colors[f"color_{3}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if _usable(head_img):
                            colors[f"color_{0}"] = head_img.bgr
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if _usable(left_wrist_img):
                                colors[f"color_{1}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if _usable(right_wrist_img):
                                colors[f"color_{2}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            _st["loops"] += 1
            if _st_fh is not None and time.monotonic() - _st["t0"] >= _st_period:
                try:
                    _dt = time.monotonic() - _st["t0"]
                    _ik = np.asarray(_st["ik"]) * 1000.0 if _st["ik"] else np.zeros(1)
                    _st_fh.write(f"{time.monotonic():.2f},{_st['loops'] / _dt:.1f},"
                                 f"{_st['fresh'] / _dt:.1f},{np.percentile(_ik, 50):.2f},"
                                 f"{np.percentile(_ik, 99):.2f},{_ik.max():.2f},"
                                 f"{_st['img_new'] / _dt:.1f},{_st['img_fps']:.1f},"
                                 f"{link_guard.state},"
                                 + (",".join(f"{v:.2f}" if isinstance(v, float) else str(v)
                                             for v in tau_cap.read_stats())
                                    if tau_cap is not None else ",,,,,")
                                 + "," + (",".join(str(v) for v in arm_ctrl.read_clip_stats())
                                          if hasattr(arm_ctrl, "read_clip_stats") else ",,,")
                                 + f",{_st['margin_clips']}\n")
                except Exception:
                    pass
                _st.update(t0=time.monotonic(), loops=0, fresh=0, ik=[], img_new=0, margin_clips=0)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
        # [panthera] Upstream logs the traceback and then exits 0 from `finally`, so a
        # startup that never got off the ground is indistinguishable from a clean run
        # for anything scripting this. Ctrl-C and the normal `q` path stay 0.
        had_exception = True
    finally:
        try:
            # [panthera] Only if it was ever built -- see the arm_ctrl = None above.
            if arm_ctrl is not None:
                # [RT6b] Lower and release instead of go-home (q = 0). In RT6 go-home could
                # not reach q = 0 against gravity, and the arms fell as soon as the
                # process stopped commanding them.
                if getattr(arm_ctrl, "released", None):
                    # [RT7] Released at once earlier (chatter or a stop during the lowering):
                    # zero torque stays, no lowering, nothing else commanded.
                    notice(f"[lower] no lowering: RELEASED AT ONCE (zero torque) earlier: {arm_ctrl.released}")
                elif hasattr(arm_ctrl, "lower_and_release") and not args.motion:
                    _gfn = None
                    try:
                        import pinocchio as _pin
                        _gm = arm_ik.reduced_robot.model
                        _gd = _gm.createData()
                        _z = np.zeros(_gm.nv)
                        _gfn = lambda q: _pin.rnea(_gm, _gd, np.asarray(q, dtype=float), _z, _z)
                    except Exception as e:
                        logger_mp.error(f"[lower] no gravity model ({e}); lowering without feed-forward")
                    notice("[lower] lowering the arms to the rest pose, then releasing")
                    _ok, _secs = arm_ctrl.lower_and_release(q_rest=rest_pose, gravity_fn=_gfn,
                                                            speed=lower_speed, release_t=release_t)
                    if getattr(arm_ctrl, "released", None):
                        notice(f"[lower] RELEASED AT ONCE (zero torque) after {_secs:.1f} s: {arm_ctrl.released}")
                    else:
                        notice(f"[lower] {'done' if _ok else 'TIMED OUT'} in {_secs:.1f} s")
                    # [RT6f] Remember where the arms were released, for the next start's
                    # "arms were moved" check.
                    if _ok and last_pose_path:
                        try:
                            start_gate_mod.save_last(last_pose_path, arm_ctrl.get_current_dual_arm_q(),
                                                     note=f"lower_and_release ok in {_secs:.1f} s")
                            notice(f"[gate] released pose saved ({os.path.basename(last_pose_path)})")
                        except Exception as e:
                            logger_mp.error(f"[gate] could not save the released pose: {e}")
                else:
                    arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
        try:
            if ik_worker is not None:
                ik_worker.close()
        except Exception as e:
            logger_mp.error(f"[ik] worker close: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                # [RT5c] Bounded. sshkeyboard only sees the stop flag on its next loop
                # turn, which needs its stdin read to return. If anything has made that
                # read blocking (RT5b: a child process that inherited the tty cleared
                # O_NONBLOCK), the turn never comes and an unbounded join() hangs the
                # exit after go-home until someone kills the session. The thread is a
                # daemon, so leaving it behind cannot keep the process alive.
                try:
                    _kb_join_s = float(os.environ.get("XR_KEYBOARD_JOIN_S", "1.0"))
                except ValueError:
                    _kb_join_s = 1.0
                listen_keyboard_thread.join(timeout=_kb_join_s)
                if listen_keyboard_thread.is_alive():
                    notice(f"[key] keyboard listener did not stop within {_kb_join_s:g} s; "
                           f"continuing the exit (non-fatal)")
                    try:
                        if _tty_saved is not None:
                            _termios.tcsetattr(sys.stdin.fileno(), _termios.TCSANOW, _tty_saved)
                    except Exception:
                        pass
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if not args.motion:
                pass
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to exit debug mode: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        try:
            reap_child_processes(log=logger_mp)
        except Exception as e:
            logger_mp.error(f"Failed to reap child processes: {e}")

        logger_mp.info("✅ Finally, exiting program.")
        # [panthera] exit_code wins when a pre-flight refused, so the caller learns
        # WHICH check failed (3 = no ee state, 4 = port busy) instead of a flat 1.
        if exit_code is None and _signal_stop["sig"] in (_signal.SIGHUP, _signal.SIGTERM):
            exit_code = EXIT_HANGUP if _signal_stop["sig"] == _signal.SIGHUP else EXIT_TERMINATED
        _final_code = exit_code if exit_code is not None else (1 if had_exception else 0)
        # [RT5c] The builtin exit() first calls sys.stdin.close(), which waits for the
        # stdin lock -- held forever by a keyboard thread stuck in its read. That was the
        # second half of the RT5b hang after [q]. Children are already reaped above, so
        # in that case leave by os._exit() after flushing.
        try:
            _kb_stuck = (not args.ipc) and listen_keyboard_thread.is_alive()
        except Exception:
            _kb_stuck = False
        if _kb_stuck:
            for _stream in (sys.stdout, sys.stderr):
                try:
                    _stream.flush()
                except Exception:
                    pass
            os._exit(_final_code)
        exit(_final_code)
