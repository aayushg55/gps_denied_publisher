import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random
import time
import argparse
from rclpy.clock import Clock

class GPSDeniedPublisher(Node):
    def __init__(self):
        super().__init__('gps_denied_publisher')
        self.publisher_ = self.create_publisher(Bool, '/gps_is_denied', 1)
        clock = Clock(clock_type=rclpy.clock.ClockType.ROS_TIME)
        self.timer_ = self.create_timer(0.05, self.timer_callback, clock=clock)  # 20 Hz
        self.logger = self.get_logger()

        self.declare_parameter('run_mode', 0)  # Declare the 'mode' parameter with a default value of 1
        self.mode = self.get_parameter('run_mode').value  # Get the value of the 'mode' parameter; 
        # 0 = random gps dropouts; 1+ = never denied
        self.logger.info("gps denied pub is using mode {}".format(self.mode))
        self.start_time = time.time()
        self.callback_count = 0
        self.denial_start_time = None
        self.denial_duration = None

    def timer_callback(self):
        msg = Bool()
        
        if self.mode == 1:
            msg.data = False
        else:
            current_time = time.time() - self.start_time
            self.callback_count += 1
            
            if self.callback_count % 200 == 0:  # Check every 10 seconds (200 callbacks)
                if self.denial_start_time is None:
                    if random.random() < 0.5:  # 50% chance of creating a denied zone
                        self.denial_start_time = current_time
                        self.denial_duration = random.randint(2, 5)
                        self.logger.info("Starting gps denied zone from {0:.2f} for length {1:.2f}s".format(self.denial_start_time, self.denial_duration))
            
            if self.denial_start_time is not None:
                if current_time - self.denial_start_time <= self.denial_duration:
                    msg.data = True
                else:
                    msg.data = False    # finished denied zone
                    self.denial_start_time = None
                    self.denial_duration = None
                    self.logger.info("Finished gps denied zone at {0:.2f}".format(current_time))
            else:
                msg.data = False
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    # parser = argparse.ArgumentParser(description='GPS Denied Publisher')
    # parser.add_argument('--mode', '-m', type=int, default=1, help='Select mode: 1 for never denied, 2 for randomly denied')
    # args = parser.parse_args()
    
    # mode = args.mode

    gps_denied_publisher = GPSDeniedPublisher()
    rclpy.spin(gps_denied_publisher)
    gps_denied_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()