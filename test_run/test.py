import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, TelemetryStatus, VehicleStatus, VehicleLocalPosition, OffboardControlMode
import time

class TestFlight(Node):
    def __init__(self):
        super().__init__('autonomous_flight_controller')
        self.get_logger().info("flight controller started")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.position_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.offboard_control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
  
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_position_cb, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos_profile)


        self.timer = self.create_timer(0.1, self.timer_callback)

        self.vehicle_pos = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.counter = 0



    def vehicle_position_cb(self, vehicle_pos):
        self.vehicle_pos = vehicle_pos

    
    def vehicle_status_cb(self, vehicle_status):
        self.vehicle_status = vehicle_status
    
    def engage_offboard(self):
        offboard = VehicleCommand()
        offboard.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard.param1 = 1.0
        offboard.param2 = 6.0
        self.command_publisher.publish(offboard)
        self.get_logger().info("Switching to offboard mode")
        # time.sleep(5)


    def publish_hearbeat(self):
        msg= OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_pub.publish(msg)
        # self.get_logger().info("Offboard control mode enabled")
    
    def arming(self):
        arm_command = VehicleCommand()
        arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_command.param1 = 1.0
        arm_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_publisher.publish(arm_command)
        self.get_logger().info("Arming...")
            
    def takeoff(self):
        takeoff_command = VehicleCommand()
        takeoff_command.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        takeoff_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_publisher.publish(takeoff_command)
        self.get_logger().info("Taking off...")
    
    def land(self):
        land_command = VehicleCommand()
        land_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        land_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_publisher.publish(land_command)
        self.get_logger().info("Landing...")


    def go_to_position(self, x, y, z):
        position_command = TrajectorySetpoint()
        position_command.position = [x, y, z]  # Set position array
        position_command.yaw = 1.57
        position_command.velocity = [1.0, 1.0, 1.0]
        position_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.position_publisher.publish(position_command)
        # print(f"Moving to position: {x}, {y}, {z}")

    def timer_callback(self):
        # self.get_logger().info(f"Counter: {self.counter}")
        self.publish_hearbeat()
        if self.counter == 0:
            self.engage_offboard()
            self.arming()

        if self.counter == 75:
            self.get_logger().info(f"moving to position: {0.0, 5.0, -5.0}")
            self.go_to_position(0.0, 0.0, -5.0)

        elif self.counter == 150:
            self.get_logger().info(f"moving to position: {5.0, 5.0, -5.0}")
            self.go_to_position(0.0, 5.0, -5.0)
        
        elif self.counter == 225:
            self.get_logger().info(f"moving to position: {5.0, 0.0, -5.0}")
            self.go_to_position(5.0, 5.0, -5.0)
        
        elif self.counter == 300:
            self.get_logger().info(f"moving to position: {0.0, 0.0, -5.0}")
            self.go_to_position(5.0, 0.0, -5.0)

        elif self.counter == 375:
            self.get_logger().info("Landing...")
            self.land()
            exit()

        self.counter += 1

        

def main(args=None):
    rclpy.init(args=args)
    drone_controller = TestFlight()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
