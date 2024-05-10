import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from irobot_create_msgs.msg import IrIntensityVector as ir
from nav_msgs.msg import Odometry 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import numpy as np
from sensor_msgs.msg import Imu
import math


i = 0
error = [0]
integral = [0]
time_step = .001
m1 = 1
m2 = 1
k1 = 0
k2 = 10
c1 = 0
c2 = 20

x1 = [0.0]
x2 = [5.0]
x1_dot = [0.0]
x2_dot = [0.0]
dt = 0.001

class move(Node):
    def __init__(self):
        super().__init__("Moving")
        self.move1 = self.create_publisher(Twist, "/robot_0/cmd_vel",1)
        self.move2 = self.create_publisher(TwistWithCovariance, "/robot_3/cmd_vel",1)
        self.timer = self.create_timer(.1, self.move_func)
        qos_profil = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.initial_position1 = np.array([[0]])
        self.initial_position2 = np.array([[0]])
        self.ir_sub1 = self.create_subscription(ir, "/robot_0/ir_intensity", self.ir_sense1, qos_profile = qos_profil)
        self.ir_sub2 = self.create_subscription(ir, "/robot_3/ir_intensity", self.ir_sense2, qos_profile = qos_profil)
        self.ir_status1 = [0,0,0,0,0,0,0]
        self.ir_status2 = [0,0,0,0,0,0,0]
        self.imu_status1 = np.array([[0,0]])
        self.imu_sub1 = self.create_subscription(Odometry, "/robot_0/odom", self.imu_sense1, qos_profile = qos_profil)
        self.imu_status2 = np.array([[0,0]])
        self.imu_sub2 = self.create_subscription(Odometry, "/robot_3/odom", self.imu_sense2, qos_profile = qos_profil)
    
    def imu_sense1(self, imu_msg:Odometry):
        global i
        #self.get_logger().info("Getting Odometry")
        self.imu_status1 = np.array([
            [math.floor(1*imu_msg.pose.pose.position._x), math.floor(1*imu_msg.pose.pose.position._y)]]) 
        if i == 0:
            self.initial_position1[0] = self.imu_status1[0,0]
            i = i + 1
    def imu_sense2(self, imu_msg:Odometry):
        global i
        #self.get_logger().info("Getting Odometry")
        self.imu_status2 = np.array([
            [math.floor(1*imu_msg.pose.pose.position._x), math.floor(1*imu_msg.pose.pose.position._y)]]) 
        if i == 0:
            self.initial_position2[0] = self.imu_status2[0,0]
            i = i + 1

    def ir_sense1(self,ir_msg:ir):
        #self.get_logger().info("Value = ")
        self.ir_status1[0] = ir_msg.readings[0].value
        self.ir_status1[1] = ir_msg.readings[1].value
        self.ir_status1[2] = ir_msg.readings[2].value
        self.ir_status1[3] = ir_msg.readings[3].value
        self.ir_status1[4] = ir_msg.readings[4].value
        self.ir_status1[5] = ir_msg.readings[5].value
        self.ir_status1[6] = ir_msg.readings[6].value
    
    def ir_sense2(self,ir_msg:ir):
        #self.get_logger().info("Value = ")
        self.ir_status2[0] = ir_msg.readings[0].value
        self.ir_status2[1] = ir_msg.readings[1].value
        self.ir_status2[2] = ir_msg.readings[2].value
        self.ir_status2[3] = ir_msg.readings[3].value
        self.ir_status2[4] = ir_msg.readings[4].value
        self.ir_status2[5] = ir_msg.readings[5].value
        self.ir_status2[6] = ir_msg.readings[6].value
   
    def move_func(self):
        self.get_logger().info("Moving the robots!")
        global x1
        global x2
        msg1 = Twist()
        msg2 = TwistWithCovariance()
        mass_spring(m1,m2,k1,k2,c1,c2)
        msg1.linear.x = .1#x1_dot[-1]
        msg2.linear.x = -.1#x2_dot[-1]
        print("Speed of 1 = ", x1_dot[-1])
        print("Speed of 2 = ", x2_dot[-1])
        self.move1.publish(msg1)
        self.move2.publish(msg2)

ki = 0.05
kp = .027
kd = 0.001
def actuation(e):
    error.append(e)
    integral.append(error[-1]*time_step + integral[-1])
    derivative = (error[-1] - error[-2]) / time_step if len(error) > 1 else 0
    control = kp * error[-1] + ki * integral[-1] + kd * derivative
    #print(-control)
    return (control)

def mass_spring(m1, m2, k1, k2,c1,c2):
    print("Solving the mass spring system....")
    state = np.array([
        [x1[-1]],
        [x2[-1]],
        [x1_dot[-1]],
        [x2_dot[-1]]
    ])

    A = np.array([
        [0,0,1,0],
        [0,0,0,1],
        [(-k1-k2)/m1,k2/m2,(-c1-c2)/m1,c2/m1],
        [k2/m2,-k2/m2,c2/m2,-c2/m2]
    ])

    y = A@state
    x1.append(x1[-1] + dt*y.item(0))
    x2.append(x2[-1] + dt*y.item(1))
    x1_dot.append(x1_dot[-1] + dt*y.item(2))
    x2_dot.append(x2_dot[-1] + dt*y.item(3))

def main(args = None):
    rclpy.init(args = args)
    node = move()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == 'main':
    main()
