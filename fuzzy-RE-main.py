import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, ReliabilityPolicy

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front1': 0,
    'front2': 0,
    'fleft': 0,
    'bright': 0,
    'left': 0
}
twstmsg_ = None


# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if (twstmsg_ != None):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        #LIDAR readings are anti-clockwise
        'front1': find_nearest(msg.ranges[0:5]),
        'front2': find_nearest(msg.ranges[355:360]),
        'right': find_nearest(msg.ranges[265:275]),
        'fright': find_nearest(msg.ranges[310:320]),
        'bright': find_nearest(msg.ranges[220:230]),
        'fleft': find_nearest(msg.ranges[40:50]),
        'left': find_nearest(msg.ranges[85:95])

    }
    # Creating a FuzzyLogicController object to handle movement logic
    movement = FuzzyLogicController_RE()
    twstmsg_ = movement.movement()


# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=1), 1)  # Changed this from 10 to 1, Hence, it returns a default of 1 if no valid readings exist

# Fuzzy controller class for right-edge robot navigation
class FuzzyLogicController_RE:
    # Define the fuzzy sets for speed and direction
    def __init__(self):
        # Fuzzy sets for linear speed adjustments (low, medium, high)
        self.low = [0, 0.05, 0.1]
        self.med = [0.1, 0.15, 0.2]
        self.high = [0.2, 0.25, 0.3]
        # Fuzzy sets for angular direction adjustments (left, straight, right)
        self.go_right = [0.1, 0.2, 0.3]
        self.go_straight = [-0.1, 0, 0.1]
        self.go_left = [-0.3, -0.2, -0.1]

    # Membership function to determine the degree of "closeness" to edge
    def membership_calc(self, x):
        """Fuzzy categorization: near, medium, far
        Includes falling edge and rising edge calculations for 'near and medium' and 'medium and far' respectively
        where falling edge = (c-x)/(c-b); rising edge = (x-a)/(b-a)"""
        if 0 <= x <= 0.6:
            return 1, 0, 0
        elif 0.6 <= x <= 0.65:
            return (0.65 - x) / (0.65 - 0.6), (x - 0.6) / (0.65 - 0.6), 0
        elif 0.65 <= x <= 0.7:
            return 0, (0.7 - x) / (0.7 - 0.65), (x - 0.65) / (0.7 - 0.65)
        elif 0.7 <= x <= 1:
            return 0, 0, 1
        else:
            return 0, 0, 0

    # Define fuzzy rules and defuzzification logic in one function
    def rule_base_defuzzy(self, RFS, RBS):
        # Calling membership values for both front-right and back-right sensors
        RFS_near, RFS_medium, RFS_far = self.membership_calc(RFS)
        RBS_near, RBS_medium, RBS_far = self.membership_calc(RBS)

        # Debugging: Print the memberships for the input sensor values
        mynode_.get_logger().info(f"RFS Memberships: Near={RFS_near:.2f}, Medium={RFS_medium:.2f}, Far={RFS_far:.2f}")
        mynode_.get_logger().info(f"RBS Memberships: Near={RBS_near:.2f}, Medium={RBS_medium:.2f}, Far={RBS_far:.2f}")

        # Dictionary of firing strengths for each rule
        firing_strengths = {
            "RFS_near_RBS_near": min(RFS_near, RBS_near),
            "RFS_near_RBS_medium": min(RFS_near, RBS_medium),
            "RFS_near_RBS_far": min(RFS_near, RBS_far),
            "RFS_medium_RBS_near": min(RFS_medium, RBS_near),
            "RFS_medium_RBS_medium": min(RFS_medium, RBS_medium),
            "RFS_medium_RBS_far": min(RFS_medium, RBS_far),
            "RFS_far_RBS_near": min(RFS_far, RBS_near),
            "RFS_far_RBS_medium": min(RFS_far, RBS_medium),
            "RFS_far_RBS_far": min(RFS_far, RBS_far)
        }
        #initializing speed and direction variables as defuzzification formula numerators
        speed = 0
        direction = 0
        total_FS = 0  # Sum of all firing strengths for normalization - This will be denominator

        # Iterate through all rules to calculate the weighted contribution to speed and direction
        for rule, FS in firing_strengths.items():
            if FS > 0:  #considering only rules with non_zero firing strength
                mynode_.get_logger().info(f"Rule {rule} fired with strength {FS:.2f}")

                # Check which rules apply and update speed and direction accordingly
                if rule == "RFS_near_RBS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "RFS_near_RBS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "RFS_near_RBS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "RFS_medium_RBS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "RFS_medium_RBS_medium":
                    speed += FS * self.med[1]
                    direction += FS * self.go_straight[1]
                elif rule == "RFS_medium_RBS_far":
                    speed += FS * self.med[1]
                    direction += FS * self.go_left[1]
                elif rule == "RFS_far_RBS_near":
                    speed += FS * self.med[1]
                    direction += FS * self.go_right[1]
                elif rule == "RFS_far_RBS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "RFS_far_RBS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
 
                # Accumulating total firing strength for normalization
                total_FS += FS

        speed_output = speed / total_FS
        direction_output = direction / total_FS
        # return final speed and direction values
        return speed_output, direction_output

    # Generating a movement command based on sensor readings
    def movement(self):
        RFS = regions_['fright']
        RBS = regions_['bright']

        # Debug: Log the raw sensor readings for monitoring
        mynode_.get_logger().info(f"RFS: {RFS:.2f}, RBS: {RBS:.2f}")

        #Calling the rule_base/defuzzification function to variables
        linear_speed, angular_speed = self.rule_base_defuzzy(RFS, RBS)

        # Debug: Log the final calculated speed and direction
        mynode_.get_logger().info(f" Linear Speed: {linear_speed:.2f}, Angular Speed: {angular_speed:.2f}")
        print()
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed * -1 # Inverting angular speed if needed due to ROSbot faults
        return msg


# used to stop the robot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
