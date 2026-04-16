import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from odd_package_interfaces.msg import RPM


class ODDDemo(Node):
    def __init__(self):
        super().__init__('simple_search_demo')
        self.rpm_publisher = self.create_publisher(RPM, '/odd/rpm_command', 10)
        self.query_publisher = self.create_publisher(String, '/nanoowl/input_query', 10)
        self.depth_image_subscription = self.create_subscription(Image, '/odd_arm_cv/selected_depth_image',
                                                                      self.update_depth_image, 10)
        self.detection_subscription = self.create_subscription(Detection2DArray, '/nanoowl/output_detections',
                                                               self.update_detections, 10)
        self.tick_timer = self.create_timer(1 / 30, self.tick)

        self.state = 'search'
        self.rpms = [0, 0]
        self.rpms_ramped = [0, 0]
        self.detection = None
        self.detection_center = None
        self.depth = 10000
        self.error = 1000

    def update_depth_image(self, msg):
        if self.detection:
            depth_array = np.ndarray(shape=(msg.height, msg.width), dtype=np.uint16, buffer=msg.data)
            self.depth = 0.001*depth_array[round(self.detection_center[1]), round(self.detection_center[0]*1.325)] # naively remap one resolution to the other
            #print(self.depth*0.001)
            #print(self.detection.results[0].hypothesis.score)

    def update_detections(self, msg):
        if len(msg.detections) > 1:
            self.detection = msg.detections[1]
            self.detection_center = [self.detection.bbox.center.position.x, self.detection.bbox.center.position.y]
        else:
            self.detection = None

    def tick(self):
        self.query_publisher.publish(String(data="[a blue backpack]"))
        
        self.rpms = [0.0, 0.0]
        
        if self.state == 'search':
            self.tick_search()
        elif self.state == 'approach':
            self.tick_approach()
        elif self.state == 'done':
            self.rpm_publisher.publish(RPM(theta_dot_left=0.0, theta_dot_right=0.0))
            rclpy.try_shutdown()
        
        self.rpms[0] = min(max(self.rpms[0], -50), 50)
        self.rpms[1] = min(max(self.rpms[1], -50), 50)
        
        print(self.rpms)
        print(self.state)
        print(self.depth)
        
        self.rpm_publisher.publish(RPM(theta_dot_left=float(self.rpms[0]), theta_dot_right=float(self.rpms[1])))

    def tick_search(self):
        if self.detection:
            self.tick_align()
            if abs(self.error) < 20:
                self.state = 'approach'
        else:
            self.rpms = [20, -20]
            
    def tick_align(self):
        self.error = self.detection_center[0] - 320
        self.rpms = [self.rpms[0] + self.error*0.2, self.rpms[1] - self.error*0.2] # P control
        
    
    def tick_approach(self):
        self.rpms = [40, 40]
        if self.detection:
            self.tick_align()
            if self.depth < 0.1 and self.depth != 0:
                self.state = 'done'
        #else:
        #    self.state = 'search'


def main(args=None):
    rclpy.init(args=args)
    node = ODDDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == '__main__':
    main()
