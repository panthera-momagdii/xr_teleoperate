import time
import argparse
from multiprocessing import Value, Array, Lock
import threading
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
    print(text, file=sys.stderr, flush=True)


# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
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

def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
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

    # [panthera] logging_mp forks a NON-DAEMON listener on the first getLogger() and
    # reaps it only from an atexit hook. Ctrl-C and `kill` do not run atexit reliably
    # here, and the orphan inherits the LISTENING SOCKET on 8012 -- so a launcher that
    # is killed leaves a process squatting on the port, and the NEXT launcher refuses
    # to start with exit 4. Four such orphans accumulated while this gate was being
    # written. install_reaper() covers atexit + SIGINT + SIGTERM; nothing can cover
    # SIGKILL, which is why the exit-4 message names the PID to kill.
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

    # [panthera] Defined before the try so the finally block can distinguish "the arm
    # controller was never built" from "it was built and we are shutting down". Without
    # this a pre-flight refusal ends in a spurious "Failed to ctrl_dual_arm_go_home:
    # name 'arm_ctrl' is not defined", which reads like a second, unrelated fault.
    arm_ctrl = None
    had_exception = False        # [panthera] see the except/finally at the bottom
    # [panthera] An explicit override for the exit status. SystemExit raised inside the
    # try block reaches `finally`, whose own exit() would otherwise replace the code
    # with 0/1 and throw away which pre-flight refused.
    exit_code = None

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

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

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
            if args.ee == "dex1_internal":
                arm_ctrl = G1_29_Arm_Internal_Dex1_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array,
                                                              dual_gripper_action_array, motion_mode=args.motion, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
            else:
                arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
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
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

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
        for _line in _banner:
            logger_mp.info(_line)
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)

        notice("--------------------- start Tracking -------------------------")
        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")

        # [panthera] Arm the start-pose sequence from where the arms ACTUALLY ARE at the
        # moment [r] was pressed, not from a reading taken earlier.
        if start_seq.enabled and not STOP:
            start_seq.start(time.monotonic(), arm_ctrl.get_current_dual_arm_q())
            notice(f"[start-pose] approaching over {start_seq.start_t:g}s, "
                   f"then blending to the operator over {start_seq.blend_t:g}s")

        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
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
            if args.ee in ("dex3", "dex5", "inspire_ftp", "inspire_dfx", "brainco")  and args.input_mode == "hand":
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
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            # [panthera] Start-pose sequence (G4). Pass-through unless XR_START_POSE is
            # set. sol_tauff is the gravity feed-forward computed FOR sol_q, so it is
            # zeroed whenever the commanded q is not sol_q -- a torque computed for a
            # pose the arm is not in is worse than no feed-forward at all.
            cmd_q = start_seq.step(time.monotonic(), current_lr_arm_q, sol_q,
                                   dt=1.0 / args.frequency)
            cmd_tauff = sol_tauff if cmd_q is sol_q else sol_tauff * 0.0
            arm_ctrl.ctrl_dual_arm(cmd_q, cmd_tauff)

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
                    if camera_config['head_camera']['binocular']:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{2}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{3}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{1}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
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
                arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
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
        exit(exit_code if exit_code is not None else (1 if had_exception else 0))
