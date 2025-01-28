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


# Main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if twstmsg_ is not None:
        pub_.publish(twstmsg_)


# Callback function for laser scan data
def clbk_laser(msg):
    global regions_, twstmsg_

    regions_ = {
        'front1': find_nearest(msg.ranges[0:15]),
        'front2': find_nearest(msg.ranges[345:360]),
        'right': find_nearest(msg.ranges[265:275]),
        'fright': find_nearest(msg.ranges[310:320]),
        'bright': find_nearest(msg.ranges[220:230]),
        'fleft': find_nearest(msg.ranges[40:50]),
        'left': find_nearest(msg.ranges[85:95])
    }
    # Use fuzzy logic controller to determine movement
    movement = FuzzyLogicController_OA()
    twstmsg_ = movement.movement()


# Find nearest point
def find_nearest(lst):
    f_list = filter(lambda item: item > 0.0, lst)  # exclude zeros
    return min(min(f_list, default=1), 1) # Changed this from 10 to 1, Hence, it returns a default of 1 if no valid readings exist


# Fuzzy controller class for obstacle avoidance robot navigation
class FuzzyLogicController_OA:
    # Fuzzy sets for speed and direction adjustments
    def __init__(self):
        # Fuzzy sets for linear speed adjustments (low, medium, high)
        self.low = [0, 0.05, 0.1]
        self.med = [0.1, 0.15, 0.2]
        self.high = [0.2, 0.25, 0.3]
        # Fuzzy sets for angular direction adjustments (left, straight, right)
        self.go_right = [0.3, 0.4, 0.5]
        self.go_straight = [-0.3, 0, 0.3]
        self.go_left = [-0.5, -0.4, -0.3]
    
    '''Membership function to determine the degree of "closeness" to obstacle
        for the Front left and Front right sensors (near, medium, far)
        Includes falling edge and rising edge calculations for 'near and medium' and 'medium and far' respectively
        where falling edge = (c-x)/(c-b); rising edge = (x-a)/(b-a)'''
    def membership_calc1(self, x):
        if 0 <= x <= 0.4:
            return 1, 0, 0
        elif 0.4 <= x <= 0.6:
            return (0.6 - x) / (0.6 - 0.4), (x - 0.4) / (0.6 - 0.4), 0
        elif 0.6 <= x <= 0.7:
            return 0, (0.7 - x) / (0.7 - 0.6), (x - 0.6) / (0.7 - 0.6)
        elif 0.7 <= x <= 1:
            return 0, 0, 1
        else:
            return 0, 0, 0

    '''Membership function to determine the degree of "closeness" to obstacle
        for the Front middle sensor (near, medium, far)
        Includes falling edge and rising edge calculations for 'near and medium' and 'medium and far' respectively
        where falling edge = (c-x)/(c-b); rising edge = (x-a)/(b-a)'''
    def membership_calc2(self, x):
        if 0 <= x <= 0.4:
            return 1, 0, 0
        elif 0.4 <= x <= 0.75:
            return (0.75 - x) / (0.75 - 0.4), (x - 0.4) / (0.75 - 0.4), 0
        elif 0.75 <= x <= 0.85:
            return 0, (0.85 - x) / (0.85 - 0.75), (x - 0.75) / (0.85 - 0.75)
        elif 0.85 <= x <= 1:
            return 0, 0, 1
        else:
            return 0, 0, 0

    # defines fuzzy logic rules and defuzzification in one function
    def rule_base_defuzzy(self, FLS, FMS, FRS):
        # Calling membership values for each sensor reading
        FLS_near, FLS_medium, FLS_far = self.membership_calc1(FLS)
        FRS_near, FRS_medium, FRS_far = self.membership_calc1(FRS)
        FMS_near, FMS_medium, FMS_far = self.membership_calc2(FMS)

        # Debugging: Prints the memberships for the input sensor values
        mynode_.get_logger().info(f"FLS Memberships: Near={FLS_near:.2f}, Medium={FLS_medium:.2f}, Far={FLS_far:.2f}")
        mynode_.get_logger().info(f"FMS Memberships: Near={FMS_near:.2f}, Medium={FMS_medium:.2f}, Far={FMS_far:.2f}")
        mynode_.get_logger().info(f"FRS Memberships: Near={FRS_near:.2f}, Medium={FRS_medium:.2f}, Far={FRS_far:.2f}")

        # Dictionary of firing strengths for each rule
        firing_strengths = {
            'FLS_near_FMS_near_FRS_near': min(FLS_near, FMS_near, FRS_near),
            'FLS_near_FMS_near_FRS_medium': min(FLS_near, FMS_near, FRS_medium),
            'FLS_near_FMS_near_FRS_far': min(FLS_near, FMS_near, FRS_far),
            'FLS_near_FMS_medium_FRS_near': min(FLS_near, FMS_medium, FRS_near),
            'FLS_near_FMS_medium_FRS_medium': min(FLS_near, FMS_medium, FRS_medium),
            'FLS_near_FMS_medium_FRS_far': min(FLS_near, FMS_medium, FRS_far),
            'FLS_near_FMS_far_FRS_near': min(FLS_near, FMS_far, FRS_near),
            'FLS_near_FMS_far_FRS_medium': min(FLS_near, FMS_far, FRS_medium),
            'FLS_near_FMS_far_FRS_far': min(FLS_near, FMS_far, FRS_far),
            'FLS_medium_FMS_near_FRS_near': min(FLS_medium, FMS_near, FRS_near),
            'FLS_medium_FMS_near_FRS_medium': min(FLS_medium, FMS_near, FRS_medium),
            'FLS_medium_FMS_near_FRS_far': min(FLS_medium, FMS_near, FRS_far),
            'FLS_medium_FMS_medium_FRS_near': min(FLS_medium, FMS_medium, FRS_near),
            'FLS_medium_FMS_medium_FRS_medium': min(FLS_medium, FMS_medium, FRS_medium),
            'FLS_medium_FMS_medium_FRS_far': min(FLS_medium, FMS_medium, FRS_far),
            'FLS_medium_FMS_far_FRS_near': min(FLS_medium, FMS_far, FRS_near),
            'FLS_medium_FMS_far_FRS_medium': min(FLS_medium, FMS_far, FRS_medium),
            'FLS_medium_FMS_far_FRS_far': min(FLS_medium, FMS_far, FRS_far),
            'FLS_far_FMS_near_FRS_near': min(FLS_far, FMS_near, FRS_near),
            'FLS_far_FMS_near_FRS_medium': min(FLS_far, FMS_near, FRS_medium),
            'FLS_far_FMS_near_FRS_far': min(FLS_far, FMS_near, FRS_far),
            'FLS_far_FMS_medium_FRS_near': min(FLS_far, FMS_medium, FRS_near),
            'FLS_far_FMS_medium_FRS_medium': min(FLS_far, FMS_medium, FRS_medium),
            'FLS_far_FMS_medium_FRS_far': min(FLS_far, FMS_medium, FRS_far),
            'FLS_far_FMS_far_FRS_near': min(FLS_far, FMS_far, FRS_near),
            'FLS_far_FMS_far_FRS_medium': min(FLS_far, FMS_far, FRS_medium),
            'FLS_far_FMS_far_FRS_far': min(FLS_far, FMS_far, FRS_far)
        }
        #initializing speed and direction variables as defuzzification formula numerators
        speed = 0
        direction = 0
        total_FS = 0  # Sum of all firing strengths for normalization - This will be denominator

        # Iterate over each rule to compute contributions to speed and direction
        for rule, FS in firing_strengths.items():
            if FS > 0:  # considering only rules with non-zero firing strength
                mynode_.get_logger().info(f"Rule {rule} fired with strength {FS:.2f}")

                # Check which rules apply and update speed and direction accordingly
                if rule == "FLS_near_FMS_near_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_near_FMS_near_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "FLS_near_FMS_near_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "FLS_near_FMS_medium_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_near_FMS_medium_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "FLS_near_FMS_medium_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "FLS_near_FMS_far_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_near_FMS_far_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "FLS_near_FMS_far_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]

                elif rule == "FLS_medium_FMS_near_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_medium_FMS_near_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_medium_FMS_near_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "FLS_medium_FMS_medium_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_medium_FMS_medium_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_medium_FMS_medium_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                elif rule == "FLS_medium_FMS_far_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_medium_FMS_far_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_medium_FMS_far_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_right[1]
                
                elif rule == "FLS_far_FMS_near_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_far_FMS_near_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_far_FMS_near_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_far_FMS_medium_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_far_FMS_medium_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_far_FMS_medium_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_far_FMS_far_FRS_near":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]
                elif rule == "FLS_far_FMS_far_FRS_medium":
                    speed += FS * self.low[1]
                    direction += FS * self.go_left[1]

                elif rule == "FLS_far_FMS_far_FRS_far":
                    speed += FS * self.low[1]
                    direction += FS * self.go_straight[1]

                # Accumulating total firing strength for normalization
                total_FS += FS

        speed_output = speed / total_FS
        direction_output = direction / total_FS
        # return final speed and direction values
        return speed_output, direction_output

    # Generates a movement command based on sensor readings
    def movement(self):
        FLS = regions_['fleft']
        FMS = min(regions_['front1'], regions_['front2'])  # Use of minimum distance between both sensor range readings as FMS
        FRS = regions_['fright']

        # Debug: Log the raw sensor readings for monitoring
        mynode_.get_logger().info(f"FLS: {FLS:.2f}, FMS: {FMS:.2f}, FRS: {FRS:.2f}")

        #Calling the rule_base/defuzzification function to variables
        linear_speed, angular_speed = self.rule_base_defuzzy(FLS, FMS, FRS)

        # Debug: Log the final calculated speed and direction
        mynode_.get_logger().info(f"Linear Speed: {linear_speed:.2f}, Angular Speed: {angular_speed:.2f}")
        print()
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed * -1  # Inverting angular speed if needed due to ROSbot faults
        return msg


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

    # Define QoS profile
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # Publisher for twist velocity messages
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # Subscribe to laser topic
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except Exception as e:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
