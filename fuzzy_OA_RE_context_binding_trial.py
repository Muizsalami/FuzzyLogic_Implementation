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
    twstmsg_ = movement()
# if not work, increase ranges of front left and front right

# Find nearest point
def find_nearest(lst):
    f_list = filter(lambda item: item > 0.0, lst)  # exclude zeros
    return min(min(f_list, default=1), 1)

# Class1: Fuzzy controller class for right-edge robot navigation
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
        '''Fuzzy categorization: near, medium, far
        Includes falling edge and rising edge calculations for 'near and medium' and 'medium and far' respectively
        where falling edge = (c-x)/(c-b); rising edge = (x-a)/(b-a)'''
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
    def rule_base_defuzzy_RE(self, RFS, RBS):
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


# Class2: Fuzzy controller class for obstacle avoidance robot navigation
class FuzzyLogicController_OA:
    # Fuzzy sets for speed and direction adjustments
    def __init__(self):
        # Fuzzy sets for linear speed adjustments (low, medium, high)
        self.low = [0, 0.05, 0.1]
        self.med = [0.1, 0.15, 0.2]
        self.high = [0.2, 0.25, 0.3]
        # Fuzzy sets for angular direction adjustments (left, straight, right)
        self.go_right = [0.1, 0.2, 0.3]
        self.go_straight = [-0.1, 0, 0.1]
        self.go_left = [-0.3, -0.2, -0.1]
    
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
    def rule_base_defuzzy_OA(self, FLS, FMS, FRS):
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

'''New membership function for both controller
where if... return a membership of 1 for OA, 0 for RE
      elif... vice-versa'''
def new_membership_calc_OA(x):
    if 0 <= x <= 0.4:
        d1_near = 1
    elif 0.4 <= x < 0.6:
        d1_near = (0.6 -x)/(0.6 - 0.4)
    else:
        d1_near = 0
    return d1_near
def new_membership_calc_RE(x):
    if 0.6 <= x <= 0.8:
        d2_near = 1
    elif 0.8 <= x <= 1:
        d2_near = (0.8 -x)/(0.8 - 0.6)
    else:
        d2_near = 0
    return d2_near

# Generates a movement command based on context binding
def movement():
    FLS = regions_['fleft']
    FMS = min(regions_['front1'], regions_['front2'])  # Use of minimum distance between both sensor range readings as FMS
    FRS = regions_['fright']
    RFS = regions_['fright']
    RBS = regions_['bright']

    # Debug: Log the raw sensor readings for monitoring
    mynode_.get_logger().info(f"RFS: {RFS:.2f}, RBS: {RBS:.2f}, FLS: {FLS:.2f}, FMS: {FMS:.2f}, FRS: {FRS:.2f}")

    '''membership values will be called for the minimum distances of OA and RE
    AND a condtion is written to overwrite situations of zero memberships for both fuzzy implementations'''
    d1 = min(FLS, FMS, FRS)
    d2 = min(RFS, RBS)
    print(d1)
    print(d2)
    md1 = new_membership_calc_OA(d1)
    md2 = new_membership_calc_RE(d2)
    if md1 == 0 and md2 == 0:
        md2 = 1
    print(md1)
    print(md2)

    # Creating instances of both fuzzy controllers
    re = FuzzyLogicController_RE()
    oa = FuzzyLogicController_OA()
    linear_RE, angular_RE = re.rule_base_defuzzy_RE(RFS, RBS)
    linear_OA, angular_OA = oa.rule_base_defuzzy_OA(FLS, FMS, FRS)

    # Debug: Log the final calculated speed and direction for both controllers
    mynode_.get_logger().info(f"Linear Speed: {linear_RE:.2f}, Angular Speed: {angular_RE:.2f}")
    mynode_.get_logger().info(f"Linear Speed: {linear_OA:.2f}, Angular Speed: {angular_OA:.2f}")

    msg = Twist()
    msg.linear.x = (md1 * linear_OA + md2 * linear_RE) / (md1 + md2)
    msg.angular.x = ((md1 * angular_OA + md2 * angular_RE) / (md1 + md2)) * -1 #might invert angular speed, will find out tomorrow
    mynode_.get_logger().info(f"msg_speed: {msg.linear.x:.2f}, msg_direction: {msg.angular.x:.2f}")
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
    timer_period = 0.3  # seconds
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
