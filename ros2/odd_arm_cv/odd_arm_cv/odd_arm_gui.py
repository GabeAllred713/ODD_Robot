import rclpy
import customtkinter as ctk
import PIL
from rosbags.image import message_to_cvimage
# from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from topic_tools_interfaces.srv import MuxSelect


class ComputerVisionGUI(ctk.CTk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.geometry("600x600")
        self.title("ODD ARM COMPUTER VISION")
        # self.cv_bridge = CvBridge()

        self.color_image = ctk.CTkImage(size=(400, 400))
        self.color_image_label = ctk.CTkLabel(self, image=self.color_image, text="color")
        self.color_image_label.grid(row=0, column=0, padx=10, pady=10)

        self.depth_image = ctk.CTkImage(size=(400, 400))
        self.depth_image_label = ctk.CTkLabel(self, image=self.depth_image, text="depth")
        self.depth_image_label.grid(row=1, column=0, padx=10, pady=10)

        self.odd_camera_enabled = ctk.BooleanVar(self)
        self.odd_camera_switch = ctk.CTkSwitch(self, text="thing", command=self.camera_switched,
                                               variable=self.odd_camera_enabled)
        self.odd_camera_switch.grid(row=2, column=0, padx=10, pady=10)

        self.color_image_subscription = self.node.create_subscription(Image, '/nanoowl/output_image',
                                                                      self.update_color_image, 10)
        self.depth_image_subscription = self.node.create_subscription(Image, '/odd_arm_cv/selected_depth_image',
                                                                      self.update_depth_image, 10)

        self.color_mux_client = self.node.create_client(MuxSelect, '/nanoowl/input_image_mux/select')
        self.depth_mux_client = self.node.create_client(MuxSelect, '/odd_arm_cv/depth_image_mux/select')

    def update_color_image(self, msg):
        opencv_image = message_to_cvimage(msg, 'rgb8')
        # opencv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.color_image.configure(light_image=PIL.Image.fromarray(opencv_image))

    def update_depth_image(self, msg):
        opencv_image = message_to_cvimage(msg, 'rgb8')
        # opencv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.depth_image.configure(light_image=PIL.PImage.fromarray(opencv_image))

    def camera_switched(self):
        if self.odd_camera_enabled.get():
            self.color_mux_client.call_async(MuxSelect("/odd/D435/color/image_rect_raw"))
            self.depth_mux_client.call_async(MuxSelect("/odd/D435/depth/image_rect_raw"))
        else:
            self.color_mux_client.call_async(MuxSelect("/arm/D405/color/image_rect_raw"))
            self.depth_mux_client.call_async(MuxSelect("/arm/D405/depth/image_rect_raw"))


def main(args=None):
    rclpy.init(args=args)
    app = ComputerVisionGUI(rclpy.create_node("odd_arm_cv"))
    try:
        while app.running:
            rclpy.spin_once(app.node, timeout_sec=0.01)
            app.update()
    except KeyboardInterrupt:
        pass
    finally:
        app.destroy()
        app.quit()
        app.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
