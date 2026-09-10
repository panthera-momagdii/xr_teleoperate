from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

Inspire_Num_Motors = 6
kTopicInspireDFXCommand = "rt/inspire/cmd"
kTopicInspireDFXState = "rt/inspire/state"

class Inspire_Controller_DFX:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False, xr_motion_data_ready_in = None):
        logger_mp.info("Initialize Inspire_Controller_DFX...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireDFXCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireDFXState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and 
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller_DFX] Waiting to subscribe dds...")
        logger_mp.info("[Inspire_Controller_DFX] Subscribe dds ok.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, xr_motion_data_ready_in))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_DFX OK!")

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = left_q_target[idx]         
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = right_q_target[idx] 

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")
    
    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None, xr_motion_data_ready_in = None):
        self.running = True

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()
                if xr_motion_data_ready_in is not None:
                    with xr_motion_data_ready_in.get_lock():
                        xr_motion_data_ready = xr_motion_data_ready_in.value
                else:
                    xr_motion_data_ready = True

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if xr_motion_data_ready:
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3
                    # We normalize them using (max - value) / range
                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))    
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_DFX has been closed.")



# [panthera] Topics and the expected DOF count come from hand_config, so the launcher's
# pre-flight and this controller cannot disagree about which hand is fitted or where it
# talks. The constant NAMES are kept so the rest of the upstream file is untouched.
from teleop.robot_control import hand_config

kTopicInspireFTPLeftCommand   = hand_config.TOPIC_LEFT_CMD
kTopicInspireFTPRightCommand  = hand_config.TOPIC_RIGHT_CMD
kTopicInspireFTPLeftState  = hand_config.TOPIC_LEFT_STATE
kTopicInspireFTPRightState = hand_config.TOPIC_RIGHT_STATE

class Inspire_Controller_FTP:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False, xr_motion_data_ready_in = None):
        logger_mp.info("Initialize Inspire_Controller_FTP...")
        from inspire_sdkpy import inspire_dds  # lazy import
        import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
        self.inspire_hand_default = inspire_hand_default
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # Initialize hand command publishers
        self.LeftHandCmd_publisher = ChannelPublisher(kTopicInspireFTPLeftCommand, inspire_dds.inspire_hand_ctrl)
        self.LeftHandCmd_publisher.Init()
        self.RightHandCmd_publisher = ChannelPublisher(kTopicInspireFTPRightCommand, inspire_dds.inspire_hand_ctrl)
        self.RightHandCmd_publisher.Init()

        # Shared Arrays for hand states ([0,1] normalized values)
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # [panthera] Readiness and fault state, filled by the callbacks below.
        self._state_seen = {}
        self._logged_first = set()
        self._subscribe_error = None

        # [panthera] Callback subscribers, not the polling thread upstream used. A bare
        # ChannelSubscriber.Read() maps to cyclonedds take_one(), which BLOCKS FOREVER on
        # a silent topic -- so upstream's loop would hang on the left hand and never even
        # look at the right one. Init(handler) has neither problem.
        self.LeftHandState_subscriber = ChannelSubscriber(kTopicInspireFTPLeftState, inspire_dds.inspire_hand_state)
        self.LeftHandState_subscriber.Init(self._on_state("left"))
        self.RightHandState_subscriber = ChannelSubscriber(kTopicInspireFTPRightState, inspire_dds.inspire_hand_state)
        self.RightHandState_subscriber.Init(self._on_state("right"))

        # [panthera] Fail closed. Upstream waited 5 s, logged "Proceeding anyway" and
        # carried on, and was satisfied by EITHER side (`or`). Three problems:
        #
        #  1. Proceeding without a hand means the first command goes to a hand whose
        #     position is unknown -- the controller's idea of "current" is all zeros.
        #  2. `any()` cannot tell "no data" from "a hand reporting all zeros", and on an
        #     RH56 all-zeros is a REAL pose: angle 0 is fully bent. A closed hand would
        #     have looked like a missing one forever.
        #  3. One side is not enough. Teleoperating with one hand silently dead is worse
        #     than not starting.
        #
        # Readiness is now "a valid first state arrived on each side", on a monotonic
        # deadline, and the wait raises instead of shrugging.
        logger_mp.info(hand_config.describe())
        deadline = time.monotonic() + hand_config.STATE_TIMEOUT_S
        last_warning = 0.0
        while True:
            if self._subscribe_error is not None:
                raise self._subscribe_error
            if self._state_seen.get("left") and self._state_seen.get("right"):
                break
            if time.monotonic() >= deadline:
                raise hand_config.state_timeout_error(
                    "[Inspire_Controller_FTP]", hand_config.STATE_TIMEOUT_S,
                    self._state_seen)
            if time.monotonic() - last_warning >= 1.0:
                last_warning = time.monotonic()
                logger_mp.warning(
                    f"[Inspire_Controller_FTP] waiting for hand state "
                    f"(L: {bool(self._state_seen.get('left'))}, "
                    f"R: {bool(self._state_seen.get('right'))})... "
                    f"({deadline - time.monotonic():.0f}s left)")
            time.sleep(0.01)
        logger_mp.info("[Inspire_Controller_FTP] Subscribe dds ok.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array, self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, xr_motion_data_ready_in))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_FTP OK!\n")

    # [panthera] _on_state used to sit in Inspire_Controller_DFX, above. It was written
    # for THIS class -- its log lines say "[Inspire_Controller_FTP]" and it uses
    # self._logged_first / _state_seen / _subscribe_error, which only this class
    # initialises -- so it was dead where it was and missing where it was needed, and
    # `--ee inspire_ftp` died at construction with
    #     AttributeError: 'Inspire_Controller_FTP' object has no attribute '_on_state'
    # The Sep 9 session ran arms-only, so nothing hit it. inspire_ftp is the hand
    # ACTUALLY FITTED to this robot, so it would have been the first thing to fail on
    # the next visit. Found by driving the real controller against
    # tools/fake_inspire_state.py in tools/overnight/test_g5_inspire_slew.py.
    def _on_state(self, side):
        """[panthera] One callback per side: validate, record, and surface faults.

        Anything raised in here happens on a DDS callback thread, where it would be
        swallowed. It is stored instead and re-raised by the wait loop, so a dead
        subscriber cannot look like "still waiting".
        """
        array = self.left_hand_state_array if side == "left" else self.right_hand_state_array

        def handler(msg):
            try:
                n_dof, _ = hand_config.read_hand_counts(msg)
                if side not in self._logged_first:
                    self._logged_first.add(side)
                    # The whole first message, once. On the next visit this is the first
                    # real look at these hands -- err/status/temperature included, which
                    # upstream reads and throws away.
                    logger_mp.info(f"[Inspire_Controller_FTP] {side} hand first state: "
                                   f"{hand_config.describe_state(msg)}")
                hand_config.check_motor_count(side, n_dof)
                hand_config.check_hand_health(side, msg)
                with array.get_lock():
                    for i in range(Inspire_Num_Motors):
                        array[i] = msg.angle_act[i] / 1000.0
                self._state_seen[side] = True
            except BaseException as exc:
                if self._subscribe_error is None:
                    self._subscribe_error = exc

        return handler

    def _subscribe_hand_state(self):
        logger_mp.info("[Inspire_Controller_FTP] Subscribe thread started.")
        while True:
            # Left Hand
            left_state_msg = self.LeftHandState_subscriber.Read()
            if left_state_msg is not None:
                if hasattr(left_state_msg, 'angle_act') and len(left_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.left_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.left_hand_state_array[i] = left_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received left_state_msg but attributes are missing or incorrect. Type: {type(left_state_msg)}, Content: {str(left_state_msg)[:100]}")
            # Right Hand
            right_state_msg = self.RightHandState_subscriber.Read()
            if right_state_msg is not None:
                if hasattr(right_state_msg, 'angle_act') and len(right_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.right_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.right_hand_state_array[i] = right_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received right_state_msg but attributes are missing or incorrect. Type: {type(right_state_msg)}, Content: {str(right_state_msg)[:100]}")

            time.sleep(0.002)

    def _send_hand_command(self, left_angle_cmd_scaled, right_angle_cmd_scaled):
        """
        Send scaled angle commands [0-1000] to both hands.
        """
        # Left Hand Command
        left_cmd_msg = self.inspire_hand_default.get_inspire_hand_ctrl()
        left_cmd_msg.angle_set = left_angle_cmd_scaled
        left_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.LeftHandCmd_publisher.Write(left_cmd_msg)

        # Right Hand Command
        right_cmd_msg = self.inspire_hand_default.get_inspire_hand_ctrl()
        right_cmd_msg.angle_set = right_angle_cmd_scaled
        right_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.RightHandCmd_publisher.Write(right_cmd_msg)

        # 临时打开前 N 次的 log
        if not hasattr(self, "_debug_count"):
            self._debug_count = 0
        if self._debug_count < 50:
            logger_mp.info(f"[Inspire_Controller_FTP] Publish cmd L={left_angle_cmd_scaled} R={right_angle_cmd_scaled} ")
            self._debug_count += 1


    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None, xr_motion_data_ready_in = None):
        logger_mp.info("[Inspire_Controller_FTP] Control process started.")
        self.running = True

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        # [panthera] FIRST-COMMAND RAMP. left_q_target above is 1.0 = FULLY OPEN, and it
        # was published every cycle from the moment this process started -- before any
        # XR data had arrived. So the very first command told the hand to snap to the
        # open pose from wherever it actually was, and the first frame of real operator
        # data snapped it again to the operator's pose. Both are full-travel steps.
        #
        # The slew instead starts from where the hand ACTUALLY IS. __init__ has already
        # waited for a valid first state on both sides, so these arrays are populated;
        # they hold angle_act/1000, i.e. normalised 0..1, so x1000 puts them back in the
        # hand's own 0..1000 command units.
        #
        # Ported from Dex5_1_Controller.control_process (robot_hand_unitree.py:254-269).
        left_last_cmd = np.clip(
            np.array(left_hand_state_array[:], dtype=float) * 1000.0,
            hand_config.INSPIRE_UNITS_MIN, hand_config.INSPIRE_UNITS_MAX)
        right_last_cmd = np.clip(
            np.array(right_hand_state_array[:], dtype=float) * 1000.0,
            hand_config.INSPIRE_UNITS_MIN, hand_config.INSPIRE_UNITS_MAX)
        logger_mp.info(
            f"[Inspire_Controller_FTP] slew starts from the measured state: "
            f"left={np.round(left_last_cmd, 1).tolist()}, "
            f"right={np.round(right_last_cmd, 1).tolist()}; "
            f"max {hand_config.inspire_max_step(self.fps):.1f} units/cycle "
            f"({hand_config.INSPIRE_SLEW_UNITS_PER_S:g} units/s at {self.fps:g} Hz, "
            f"env XR_HAND_SLEW), full travel in "
            f"{hand_config.INSPIRE_UNITS_MAX / hand_config.INSPIRE_SLEW_UNITS_PER_S:.2f}s")

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()
                if xr_motion_data_ready_in is not None:
                    with xr_motion_data_ready_in.get_lock():
                        xr_motion_data_ready = xr_motion_data_ready_in.value
                else:
                    xr_motion_data_ready = True

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if xr_motion_data_ready:
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # [panthera] Slew-limit toward the retargeted pose, then clamp to the
                # hand's 0..1000 range. Was an unlimited `int(np.clip(val*1000, 0, 1000))`
                # straight from the retargeter, so any jump in the operator's hand pose
                # -- including the very first frame, and including a single bad
                # retargeting frame -- went to the fingers at full speed.
                left_last_cmd = hand_config.limit_inspire_command(
                    np.asarray(left_q_target, dtype=float) * 1000.0,
                    left_last_cmd, self.fps)
                right_last_cmd = hand_config.limit_inspire_command(
                    np.asarray(right_q_target, dtype=float) * 1000.0,
                    right_last_cmd, self.fps)
                scaled_left_cmd = [int(round(v)) for v in left_last_cmd]
                scaled_right_cmd = [int(round(v)) for v in right_last_cmd]

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self._send_hand_command(scaled_left_cmd, scaled_right_cmd)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_FTP has been closed.")

# Update hand state, according to the official documentation:
# 1. https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
# 2. https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand
# the state sequence is as shown in the table below
# ┌──────┬───────┬──────┬────────┬────────┬────────────┬────────────────┬───────┬──────┬────────┬────────┬────────────┬────────────────┐
# │ Id   │   0   │  1   │   2    │   3    │     4      │       5        │   6   │  7   │   8    │   9    │    10      │       11       │
# ├──────┼───────┼──────┼────────┼────────┼────────────┼────────────────┼───────┼──────┼────────┼────────┼────────────┼────────────────┤
# │      │                    Right Hand                                │                   Left Hand                                  │
# │Joint │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │
# └──────┴───────┴──────┴────────┴────────┴────────────┴────────────────┴───────┴──────┴────────┴────────┴────────────┴────────────────┘
class Inspire_Right_Hand_JointIndex(IntEnum):
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex(IntEnum):
    kLeftHandPinky = 6
    kLeftHandRing = 7
    kLeftHandMiddle = 8
    kLeftHandIndex = 9
    kLeftHandThumbBend = 10
    kLeftHandThumbRotation = 11
