import enum
import logging
import math
import time
from datetime import datetime, timezone
import traceback
from copy import deepcopy
import copy

import numpy as np
import tqdm
from std_msgs.msg import UInt16, Empty
from rm_msgs.msg import CartePos, ArmState,GetArmState_Command ,MoveJ,JointPos
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import JointTrajectoryControllerState  # 引入 JointControllerState 消息类型
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose as RosPose
import rospy
import actionlib
import tf

from threading import Lock
import threading
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
import lerobot.common.robot_devices.motors.utils as motor_util
MODEL_RESOLUTION = {
    "scs_series": 4096,
    "sts3215": 4096,
}
class DeviceNotConnectedError(Exception):
    def __init__(self, msg):
        self.msg = msg
        super().__init__(f"{self.msg}, device not connected")
def capture_timestamp_utc():
    return datetime.now(timezone.utc)

class RobotDeviceAlreadyConnectedError(Exception):
    """Exception raised when the robot device is already connected."""

    def __init__(
        self,
        message="This robot device is already connected. Try not calling `robot_device.connect()` twice.",
    ):
        self.message = message
        super().__init__(self.message)
def convert_degrees_to_steps(degrees, model):
    """ 将度数范围转换为表示电机旋转的步数范围。假设电机通过-180度位置到+180度达到完整旋转。 """
    resolutions = MODEL_RESOLUTION[model] 
    steps = degrees / 180 * (np.array(resolutions) / 2)+np.array(resolutions)/2
    steps = steps.astype(int)
    return steps
def step2degree(steps, model):
    """ 将步数范围转换为表示电机旋转的度数范围。假设电机通过-180度位置到+180度达到完整旋转。 """
    resolutions = MODEL_RESOLUTION[model] 
    degrees = steps / (np.array(resolutions)/2) * 180-180
    degrees = degrees.astype(int)
    return degrees

def get_group_sync_key(data_name, motor_names):
    group_key = f"{data_name}_" + "_".join(motor_names)
    return group_key

def get_log_name(var_name, fn_name, data_name, motor_names):
    motor_names = "test"
    group_key = get_group_sync_key(data_name, motor_names)
    log_name = f"{var_name}_{fn_name}_{group_key}"
    return log_name

class RealmenMotorBus:
    """
    The FeetechMotorsBus class allows to efficiently read and write to the attached motors. It relies on
    the python feetech sdk to communicate with the motors. For more info, see the [feetech SDK Documentation](https://emanual.robotis.com/docs/en/software/feetech/feetech_sdk/sample_code/python_read_write_protocol_2_0/#python-read-write-protocol-20).

    A FeetechMotorsBus instance requires a port (e.g. `FeetechMotorsBus(port="/dev/tty.usbmodem575E0031751"`)).
    To find the port, you can run our utility script:
    ```bash
    python lerobot/scripts/find_motors_bus_port.py
    >>> Finding all available ports for the MotorsBus.
    >>> ['/dev/tty.usbmodem575E0032081', '/dev/tty.usbmodem575E0031751']
    >>> Remove the usb cable from your FeetechMotorsBus and press Enter when done.
    >>> The port of this FeetechMotorsBus is /dev/tty.usbmodem575E0031751.
    >>> Reconnect the usb cable.
    ```

    Example of usage for 1 motor connected to the bus:
    ```python
    motor_name = "gripper"
    motor_index = 6
    motor_model = "sts3215"

    motors_bus = FeetechMotorsBus(
        port="/dev/tty.usbmodem575E0031751",
        motors={motor_name: (motor_index, motor_model)},
    )
    motors_bus.connect()

    position = motors_bus.read("Present_Position")

    # move from a few motor steps as an example
    few_steps = 30

    # when done, consider disconnecting
    motors_bus.disconnect()
    ```
    """
    def __init__(
        self,
        port: str,
        motors: dict[str, tuple[int, str]],
        is_sim=False
        ):
        self.is_sim = is_sim
        self.port = port
        self.motors = motors
        self.calibration = None
        self.is_connected = False
        self.group_readers = {}
        self.group_writers = {}
        self.logs = {}
        self.read_state={
            "Present_Position": None,
            "Present_Speed": None,
            "Present_Load":  None,
        }
        self.write_state={
            "Goal_Position": None,
            "Goal_Speed": None,
            "Torque_Enable": None,
        }
        # TODO
        self.open_gripper_angle =  0.175
        self.close_gripper_angle = 2.775
        self.home_offset = np.array([-1.57,0.0,0.0,1.65,0.0,1.57,0.0,self.open_gripper_angle])
        self.max_w=3.14
        
        
        if motor_util.node_initialize==False:
            rospy.init_node("RealmenMotorBus",anonymous=True)
            motor_util.node_initialize=True
        self.control_interval = 1/30.0  # Control interval (s)
        self.control_rate = rospy.Rate(1 / self.control_interval)  # 200Hz
        self.control_arm_rate = rospy.Rate(200.0)  # 200Hz
        self.state_cbk = False
        self.is_write = False
        self.joint_actual_state = JointTrajectoryPoint()
        self.joint_actual_pos = self.joint_actual_state.positions
        self.robot_error = False
        self.gripper_motor_ids,self.gripper_model = self.motors["gripper"]
        self.gripper_motors_bus = FeetechMotorsBus(
            port=self.port,
            motors={"gripper": [self.gripper_motor_ids,self.gripper_model]},
        )
        self.pre_name = "/arm2"
        if self.is_sim == False:
            # Publishers and Subscribers
        
            self.joint_publisher = rospy.Publisher(
                self.pre_name +"/rm_driver/MoveJ_Cmd", MoveJ, queue_size=1
            )
            self.joint_fd_publisher = rospy.Publisher(
                self.pre_name +"/rm_driver/JointPos", JointPos, queue_size=1
            )
            self.pose_publisher = rospy.Publisher(
                self.pre_name +"/rm_driver/MoveP_Fd_Cmd", CartePos, queue_size=1
            )
            self.pose_subscriber = rospy.Subscriber(
                self.pre_name +"/rm_driver/Pose_State", RosPose, self.handlePoseCallback
            )
            
            self.joint_pos_subscriber = rospy.Subscriber(
                self.pre_name +"/joint_states", JointState, self.handleJointCallback
            )
            # self.state_subscriber = rospy.Subscriber(
            #     '/arm2/rm_driver/Arm_Current_State',
            #     ArmState,
            #     self.handleStateCallback
            # )
            self.error_subscriber = rospy.Subscriber(
                self.pre_name +"/rm_driver/ArmError", UInt16, self.errrCodeCallback
            )

            self.error_clear_pub = rospy.Publisher(
                self.pre_name +"/rm_driver/Clear_System_Err", Empty, queue_size=1
            )
        else:
            # 创建一个action client连接到机械臂的动作接口
            self.client = actionlib.SimpleActionClient(
                "/arm/arm_joint_controller/follow_joint_trajectory",
                FollowJointTrajectoryAction,
            )
            self.client.wait_for_server()  # rospy.Duration(2)
            self.listener = tf.TransformListener()
            rospy.Subscriber(
                "/arm/arm_joint_controller/state",
                JointTrajectoryControllerState,
                self.stateCallback,
            )
        
            
                
        self.q_prev = np.array(np.zeros(len(self.motor_names)))
        self.run_thread = threading.Thread(target=self._run) # daemon=True,daemon=True
        # self.run_thread.start()
    def __del__(self):
        if self.run_thread is not None and self.run_thread.is_alive():
            self.run_thread.join()
        self.disconnect()      
           
            
    def _run(self):
        
        # self.run_thread.start()
        while not rospy.is_shutdown():
            try:
                # check
                if self.robot_error:
                    print("robot error occured")
                    exit(0)
                
                
                if not self.state_cbk or self.read_state["Present_Position"]==None:
                    continue
                # print(self.read_state["Present_Position"])
                if self.is_sim == False:
                   
                        #
                    # joint_fd_cmd = JointPos()
                    # joint_fd_cmd.joint=[q for q in self.write_state["Goal_Position"]]
                    # print("joint_fd_cmd.joint: ",joint_fd_cmd.joint)
                    # self.joint_fd_publisher.publish(self.joint_pos)
                    pass
                else:
                    trajectory_points = []
                    total_time =0.0
                    actions=0
                    if self.write_state["Goal_Position"]!=None:
                        actions = len(self.write_state["Goal_Position"])
                        for i in range(actions):
                            
                            total_time += self.control_interval
                            trajectory_points.append(
                                {
                                    "positions": self.write_state["Goal_Position"][i][:-1],  # 弧度
                                    "time_from_start": total_time,  # 移动到该点所需的时间（秒）
                                }
                            )
                        # print(trajectory_points)
                        if len(trajectory_points) !=0:
                            self.setJointPositions(trajectory_points) 

                    
                    
                self.control_arm_rate.sleep()
            except Exception as e:
                print(e)              
    def setJointPositions(self, trajectory_points):  # req
        # 创建目标关节姿态
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.motor_names[:-1] # 机械臂关节

        # 创建JointTrajectoryPoint，设置目标位置
        hearder = Header()
        hearder.stamp = rospy.Time.now()
        hearder.frame_id = "base_link"
        point = FollowJointTrajectoryGoal()
        total_time =0.0
        for point in trajectory_points:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = point["positions"]
            trajectory_point.velocities = point.get(
                "velocities", [0.0] * len(goal.trajectory.joint_names)
            )  # 可选，设置速度
            trajectory_point.accelerations = point.get(
                "accelerations", [0.0] * len(goal.trajectory.joint_names)
            )  # 可选，设置加速度
            
            #时间递增
            
            trajectory_point.time_from_start = rospy.Duration(point["time_from_start"])
            goal.trajectory.points.append(trajectory_point)  # 将点添加到轨迹中
        # 发送目标到机器人
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(5.0))
        print(" server send goal")        
        
    def connect(self):
        # TODO
        self.gripper_motors_bus.connect()
        while not rospy.is_shutdown():
            read_gripper = self.gripper_motors_bus.read("Present_Position")
            gripper_q = step2degree(read_gripper,self.gripper_model)*math.pi/180.0
            self.control_rate.sleep()
            if not self.state_cbk:
                continue
            else:
                break
        
        self.run_thread.start()
        joint_pos = np.array(self.joint_actual_pos)
        self.q_prev = np.concatenate((joint_pos,gripper_q),axis=0)
        self.reset_position(self.q_prev)
        self.is_connected = True
    def reset_position(self,cur_joint_pos):
        
        
        dq_max = np.max(np.abs(self.home_offset-cur_joint_pos)) # .max() # RM_75最大角速度为180度/秒，3.14 rad/s
        interp_num = max(1,(dq_max/self.control_interval)/(self.max_w/2)) #插值点数 
        q_interped = np.stack([cur_joint_pos,self.home_offset],axis=0)
        q_interped = self.joint_poses_interp(q_interped,ratio=interp_num)
        step  =  convert_degrees_to_steps(self.home_offset[-1],self.gripper_model) 
                # time_s = time.time()
         
        self.control_rate.sleep()
        if self.is_sim == False:
            self.jointPosCtrl(q_interped)
            
        else:
            self.jointTrajCtrl(q_interped)
        print("reset position")
            
      
    def jointTrajCtrl(self,traj):
        trajectory_points = []
        total_time =0.0
        actions=0
        for q in traj:
            total_time += self.control_interval
            trajectory_points.append(
                {
                    "positions": q[:-1],  # 弧度
                    "time_from_start": total_time,  # 移动到该点所需的时间（秒）
                }
            )
        if len(trajectory_points) !=0:
            self.setJointPositions(trajectory_points)
    def jointPosCtrl(self,q_interped):
        joint_fd_cmd = JointPos()
        arm_q = q_interped[:-1]
        for q in arm_q:
            joint_fd_cmd.joint=q.tolist()
            self.joint_fd_publisher.publish(joint_fd_cmd)
            times=time.time()
            while not rospy.is_shutdown():
                if(self.joint_fd_publisher.get_num_connections()>0):
                    timee=time.time()
                    duration = timee-times
                    time.sleep((duration)/2)
                    # print(f"{duration=}")
                    break
                else:
                    print("joint_fd_publisher not subscribed")
            self.control_arm_rate.sleep()
            
        
    def jointTrajCtrl(self,traj):
        trajectory_points = []
        total_time =0.0
        actions=0
        for q in traj:
            total_time += self.control_interval
            trajectory_points.append(
                {
                    "positions": q[:-1],  # 弧度
                    "time_from_start": total_time,  # 移动到该点所需的时间（秒）
                }
            )
        if len(trajectory_points) !=0:
            self.setJointPositions(trajectory_points)
    def reconnect(self):
        self.is_connected = True
    @property
    def motor_names(self): 
        return list(self.motors.keys())
    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    def read(self,data_name,motor_names: str | list[str] | None = None): 
        if not self.is_connected:
            raise DeviceNotConnectedError(f"FeetechMotorsBus({self.port}) is not connected. You need to run `motors_bus.connect()`.")
        start_time = time.perf_counter()
        if motor_names is None:
            motor_names = self.motor_names

        if isinstance(motor_names, str):
            motor_names = [motor_names]
        self.read_state[data_name] = []
        if data_name not in self.read_state.keys():
            raise ValueError(f"Invalid data name: {data_name}")
        else:
            #TODO optimize gripper
            if data_name == "Present_Position":
                read_gripper = self.gripper_motors_bus.read("Present_Position")
                gripper_q = step2degree(read_gripper,self.gripper_model)*math.pi/180.0
                #错误 列表为可变对象 self.read_state[data_name] = copy.copy(self.joint_actual_pos)
               
                self.read_state[data_name] = copy.copy(self.joint_actual_pos) # 或者使用 deepcopy 或者切片[:]
                
                # print("before self.read_state ={}".format(self.read_state))
                # assert gripper_q.size == 1
                self.read_state[data_name].append(gripper_q.item())
        
        values = np.array(self.read_state[data_name]) 
        # print("self.read_state ={}".format(self.read_state))

        # log the number of seconds it took to read the data from the motors
        delta_ts_name = get_log_name(
            "delta_timestamp_s", "read", data_name, motor_names
        )
        self.logs[delta_ts_name] = time.perf_counter() - start_time

        # log the utc time at which the data was received
        ts_utc_name = get_log_name("timestamp_utc", "read", data_name, motor_names)
        self.logs[ts_utc_name] = capture_timestamp_utc()            
        return values
    def readGripper(self,data_name,motor_names: str | list[str] | None = None):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"FeetechMotorsBus({self.port}) is not connected. You need to run `motors_bus.connect()`.")
        start_time = time.perf_counter()
        if motor_names is None:
            motor_names = self.motor_names
        if data_name == "Present_Position":
            read_gripper = self.gripper_motors_bus.read("Present_Position")
            gripper_q = step2degree(read_gripper,self.gripper_model)*math.pi/180.0
        
        if isinstance(motor_names, str):
            motor_names = [motor_names]
        
        # log the number of seconds it took to read the data from the motors
        delta_ts_name = get_log_name(
            "delta_timestamp_s", "read", data_name, motor_names
        )
        self.logs[delta_ts_name] = time.perf_counter() - start_time

        # log the utc time at which the data was received
        ts_utc_name = get_log_name("timestamp_utc", "read", data_name, motor_names)
        self.logs[ts_utc_name] = capture_timestamp_utc()     
        return np.array([gripper_q.item()])
    
    def joint_poses_interp(self, poses, ratio=1.0):
        #0 [j1,j2,j3,j4,j5,j6,gripper]
        #1 [j1,j2,j3,j4,j5,j6,gripper]
        #2  [j1,j2,j3,j4,j5,j6,gripper]
        #steps =3 , joints = 7
        steps = poses.shape[0]
        joints =  poses.shape[1] #
        steps_new = math.ceil(steps*ratio) # round up
        xp = np.linspace(0, steps, steps) # 0,3,3
        x = np.linspace(0, steps, steps_new) # 0,3,3

        poses_new = None
        for j in range(joints):
            fp = poses[:, j] # 选择第j个列
            # print(f"{fp=}")
            y = np.interp(x, xp, fp)
            if j==0:
                poses_new = y.reshape(steps_new,1)
            else :
                poses_new = np.hstack((poses_new, y.reshape(steps_new,1)))
            # print(poses_new)
        return poses_new
    
    def write(self,data_name,values: int | float | np.ndarray, motor_names: str | list[str] | None = None): 
        if not self.is_connected:
            raise DeviceNotConnectedError(f"RealmenMotorsBus({self.port}) is not connected. You need to run `motors_bus.connect()`.")
        start_time = time.perf_counter()
        if motor_names is None:
            motor_names = self.motor_names

        if isinstance(motor_names, str):
            motor_names = [motor_names]
        
        # if isinstance(values, (int, float, np.integer)):
        #     values = [int(values)] * len(motor_names)        
        q_target = values
        
        self.write_state[data_name] = []
        if data_name == "Goal_Position":
            dq_max = np.max(np.abs((q_target-self.q_prev)))  # RM_75最大角速度为180度/秒，3.14 rad/s
            interp_num = max(1,(dq_max/self.control_interval)/(self.max_w/2)) #插值点数 
            q_interped = np.stack([self.q_prev,q_target],axis=0)
            q_interped = self.joint_poses_interp(q_interped,ratio=interp_num)
            
             # reset
            for q in q_interped:
                self.write_state[data_name].append(q)
            self.q_prev = q_target
        if data_name == "Torque_Enable":
                self.write_state[data_name].append(values) 
        # ## act
        actions=0    
        
        if self.write_state["Goal_Position"]!=None:         
            actions = len(self.write_state["Goal_Position"]) 
            if actions >= 1:
                angle = self.write_state["Goal_Position"][-1][-1]*180/math.pi
                # print(f"follow gripper_q = {angle}")
                step  =  convert_degrees_to_steps(angle,self.gripper_model) 
                # time_s = time.time()
                
                if self.is_sim == False:
                    joint_fd_cmd = JointPos()
                    for i in range(actions):
                        joint_fd_cmd.joint=[q for q in self.write_state["Goal_Position"][i][:-1]]
                        self.joint_fd_publisher.publish(joint_fd_cmd)
                        self.control_arm_rate.sleep()
                
            # print("write time = {}".format(time.time()-time_s))
        elif self.write_state["Torque_Enable"]:
            self.gripper_motors_bus.write("Torque_Enable",self.write_state["Torque_Enable"])
            
        else :
            rospy.loginfo_throttle(3,"no action")       
        self.is_write = True
        
        # print("self.write_state ={}".format(self.write_state))
        
        
        # log the number of seconds it took to write the data to the motors
        delta_ts_name = get_log_name(
            "delta_timestamp_s", "write", data_name, motor_names
        )
        self.logs[delta_ts_name] = time.perf_counter() - start_time

        # TODO(rcadene): should we log the time before sending the write command?
        # log the utc time when the write has been completed
        ts_utc_name = get_log_name("timestamp_utc", "write", data_name, motor_names)
        self.logs[ts_utc_name] = capture_timestamp_utc()
    def writeGripper(self,data_name,values: int | float | np.ndarray, motor_names: str | list[str] | None = None): 
        if not self.is_connected:
            raise DeviceNotConnectedError(f"RealmenMotorsBus({self.port}) is not connected. You need to run `motors_bus.connect()`.")
        start_time = time.perf_counter()
        if motor_names is None:
            motor_names = self.motor_names

        if isinstance(motor_names, str):
            motor_names = [motor_names]
        
        self.write_state[data_name] = [values]

        if data_name=="Goal_Position":
            angle = self.write_state["Goal_Position"][-1]*180/math.pi
            # print(f"follow gripper_q = {angle}")
            step  =  convert_degrees_to_steps(angle,self.gripper_model) 
            # time_s = time.time()
        # log the number of seconds it took to write the data to the motors
        delta_ts_name = get_log_name(
            "delta_timestamp_s", "write", data_name, motor_names
        )
        self.logs[delta_ts_name] = time.perf_counter() - start_time

        # TODO(rcadene): should we log the time before sending the write command?
        # log the utc time when the write has been completed
        ts_utc_name = get_log_name("timestamp_utc", "write", data_name, motor_names)
        self.logs[ts_utc_name] = capture_timestamp_utc()    
        
    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"FeetechMotorsBus({self.port}) is not connected.")
        
        self.packet_handler = None
        self.group_readers = {}
        self.group_writers = {}
        self.is_connected = False
        self.state_cbk = False
    def __del__(self):
        self.run_thread.join()
        if getattr(self, "is_connected", False):
            self.disconnect()
    def stateCallback(self, msg):
        self.joint_actual_state = msg.actual
        rospy.loginfo_throttle(3,"Enter stateCallback")
        self.joint_actual_pos = [
            joint_pos for joint_pos in self.joint_actual_state.positions
        ]
        self.state_cbk = True
         
    
    def handlePoseCallback(self, msg):
        """Thread-safe robot pose update"""
        # print("pose callback called")
        
        self.current_pose = msg
        self.is_pose_initialized = True

    def handleStateCallback(self, msg):
        """Thread-safe robot state update"""
        
        if msg.arm_err or msg.sys_err:
            rospy.logwarn(
                f"Robot error - Arm: {msg.arm_err}, System: {msg.sys_err}"
            )
            self.robot_error = True
        else:
            self.robot_error = False

    def errrCodeCallback(self, msg):
        if msg.data != 0:
            self.robot_error = True
            print("error occured")
            msg = Empty()
            self.error_clear_pub.publish(msg)
            self.robot_error = False        
    def handleJointCallback(self, msg):
        rospy.loginfo_once("Enter handleJointCallback")
        self.joint_actual_pos =[
            joint_pos for joint_pos in msg.position
        ] 
        self.state_cbk = True        