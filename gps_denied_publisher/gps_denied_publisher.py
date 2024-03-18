import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random
import time
import argparse

class GPSDeniedPublisher(Node):
    def __init__(self, mode):
        super().__init__('gps_denied_publisher')
        self.publisher_ = self.create_publisher(Bool, '/gps_is_denied', 1)
        self.timer_ = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.mode = mode
        self.start_time = time.time()
        self.callback_count = 0
        self.denial_start_time = None
        self.denial_duration = None

    def timer_callback(self):
        msg = Bool()
        
        if self.mode == 1:
            msg.data = False
        else:
            current_time = time.time()
            self.callback_count += 1
            
            if self.callback_count % 200 == 0:  # Check every 10 seconds (200 callbacks)
                if self.denial_start_time is None:
                    if random.random() < 0.5:  # 50% chance of creating a denied zone
                        self.denial_start_time = current_time
                        self.denial_duration = random.randint(2, 5)
            
            if self.denial_start_time is not None:
                if current_time - self.denial_start_time <= self.denial_duration:
                    msg.data = True
                else:
                    msg.data = False    # finished denied zone
                    self.denial_start_time = None
                    self.denial_duration = None
            else:
                msg.data = False
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='GPS Denied Publisher')
    parser.add_argument('--mode', type=int, default=1, help='Select mode: 1 for never denied, 2 for randomly denied')
    args = parser.parse_args()
    
    mode = args.mode
    
    if mode not in [1, 2]:
        print("Invalid mode selected. Defaulting to mode 1.")
        mode = 1
    
    gps_denied_publisher = GPSDeniedPublisher(mode)
    rclpy.spin(gps_denied_publisher)
    gps_denied_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()