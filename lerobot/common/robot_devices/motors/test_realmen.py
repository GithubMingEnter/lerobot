from lerobot.common.robot_devices.motors.realmen import RealmenMotorBus,step2degree
from lerobot.common.robot_devices.motors.realmenF import FollowRealmenMotorBus

import argparse 

import numpy as np
import math
from lerobot.common.robot_devices.motors.feetech import TorqueMode
import time
from rm_msgs.msg import CartePos, ArmState,GetArmState_Command ,MoveJ,JointPos
def test_motor():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",type=str, default="/dev/ttyUSB1",help="port name")
    parser.add_argument("--brand", type=str, required=False, help="Motor brand (e.g. dynamixel,feetech)")
    parser.add_argument("--model", type=str, required=False, help="Motor model (e.g. xl330-m077,sts3215)")
    parser.add_argument("--ID", type=int, required=False, help="Desired ID of the current motor (e.g. 1,2,3)")
    args = parser.parse_args()
    print(step2degree(1931, "sts3215"))
    print(step2degree(2055, "sts3215"))
    
    try:
        real_men = RealmenMotorBus(
            port="/dev/ttyUSB0",
            motors={"joint1": [1, "rm75"],
                    "joint2": [2, "rm75"],
                    "joint3": [3, "rm75"],
                    "joint4": [4, "rm75"],
                    "joint5": [5, "rm75"],
                    "joint6": [6, "rm75"],
                    "joint7": [7, "rm75"],
                    "gripper": [8, "sts3215"]
                    }
        )
        real_menF = FollowRealmenMotorBus(
            port=args.port,
            motors={"joint1": [1, "rm75"],
                    "joint2": [2, "rm75"],
                    "joint3": [3, "rm75"],
                    "joint4": [4, "rm75"],
                    "joint5": [5, "rm75"],
                    "joint6": [6, "rm75"],
                    "joint7": [7, "rm75"],
                    "gripper": [1, "sts3215"]
                    }
        )
        real_men.connect()
        real_menF.connect()
        mode = 1
        if mode ==0:
            if real_men.state_cbk:
                read_value = real_men.read("Present_Position","rm75")
                print("read{}".format(read_value))
            write_data = np.zeros(len(real_men.motor_names))
            write_data[-1] = -10/180*math.pi
            # real_men.write("Goal_Position", write_data)
            real_men.write("Torque_Enable", TorqueMode.DISABLED.value)
            for i in range(10):
                read_value = real_men.read("Present_Position","rm75")
                print("read{}".format(read_value))
        elif mode ==1:
            real_menF.read("Present_Position","rm75")
            write_data = np.zeros(len(real_men.motor_names))
            write_data[-1] = -10/180*math.pi
            for i in range(10):

                real_menF.write("Goal_Position", write_data)
                write_data[-1]=i /10*math.pi
                time.sleep(1)
    except Exception as e:
        print(e)
def cal_angle(steps):
    print(step2degree(steps, "sts3215")/180*math.pi)
# import rospy
# from std_msgs.msg import String

# def publisher_node():
#     rospy.init_node('my_publisher_node')
#     pub = rospy.Publisher('my_topic', String, queue_size=10)
#     rate = rospy.Rate(1)  # 1Hz

#     while not rospy.is_shutdown():
#         msg = String()
#         msg.data = "Hello, ROS!"
#         pub.publish(msg)

#         start_time = rospy.get_time()
#         while not rospy.is_shutdown():
#             num_connections = pub.get_num_connections()
#             rospy.loginfo(f"Number of subscribers connected: {num_connections}")
#             if num_connections > 0 or (rospy.get_time() - start_time) > 5:  # Wait for connection or timeout after 5 seconds
#                 break

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publisher_node()
#     except rospy.ROSInterruptException:
#         pass
def test_continue_motion():
    real_men = RealmenMotorBus(
            port="/dev/ttyUSB0",
            motors={"joint1": [1, "rm75"],
                    "joint2": [2, "rm75"],
                    "joint3": [3, "rm75"],
                    "joint4": [4, "rm75"],
                    "joint5": [5, "rm75"],
                    "joint6": [6, "rm75"],
                    "joint7": [7, "rm75"],
                    "gripper": [8, "sts3215"]
                    }
        )
    real_menf = FollowRealmenMotorBus(
            port="/dev/ttyUSB0",
            motors={"joint1": [1, "rm75"],
                    "joint2": [2, "rm75"],
                    "joint3": [3, "rm75"],
                    "joint4": [4, "rm75"],
                    "joint5": [5, "rm75"],
                    "joint6": [6, "rm75"],
                    "joint7": [7, "rm75"],
                    "gripper": [8, "sts3215"]
                    }
        )
    real_men.connect()
    q1 = np.array(real_men.joint_actual_pos)
    q2 = real_men.home_offset[:-1]
    dq_max = np.max(np.abs(q2-q1))# RM_75最大角速度为180度/秒，3.14 rad/s
    interp_num = max(1,(dq_max/real_men.control_interval)/(real_men.max_w/2)) #插值点数 
    q_interped = np.stack([q1,q2],axis=0)
    q_interped = real_men.joint_poses_interp(q_interped,ratio=interp_num)
    joint_fd_cmd = JointPos()
    real_men.jointPosCtrl(q_interped)
    

if __name__ == "__main__":
    # cal_angle(1850)
    # cal_angle(1378)
    # cal_angle(1022)
    cal_angle(3070)
    test_continue_motion()
            
            
        # real_men.run()
        
        
    


