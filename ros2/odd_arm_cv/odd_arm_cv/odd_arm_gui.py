import rclpy
import customtkinter as ctk
import PIL
#from rosbags.image import message_to_cvimage
#from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from topic_tools_interfaces.srv import MuxSelect


# Following function was written by Gemini
def ros2_image_to_pil(ros_image_msg):
    """
    Converts a ROS2 sensor_msgs/Image to a PIL Image using only Pillow.

    Args:
        ros_image_msg: The uncompressed ROS2 image message.

    Returns:
        PIL.Image: The converted Pillow image.
    """
    # Map ROS2 encodings to PIL (Image Mode, Raw Mode)
    encoding_mapping = {
        'mono8': ('L', 'L'),
        '8UC1': ('L', 'L'),
        '16UC1': ('I;16', 'I;16'),
        'rgb8': ('RGB', 'RGB'),
        'rgba8': ('RGBA', 'RGBA'),
        'bgr8': ('RGB', 'BGR'),
        'bgra8': ('RGBA', 'BGRA')
    }

    if ros_image_msg.encoding not in encoding_mapping:
        raise ValueError(f"Unsupported encoding: {ros_image_msg.encoding}")

    pil_mode, raw_mode = encoding_mapping[ros_image_msg.encoding]

    # Image dimensions
    size = (ros_image_msg.width, ros_image_msg.height)

    # ROS2 message data is typically an array.array or sequence; convert to strict bytes
    byte_data = bytes(ros_image_msg.data)

    # Construct and return the PIL Image
    return PIL.Image.frombytes(pil_mode, size, byte_data, 'raw', raw_mode)


class ComputerVisionGUI(ctk.CTk):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.geometry("430x900")
        self.running = True
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.title("ODD ARM COMPUTER VISION")
        # self.cv_bridge = CvBridge()

        self.color_image = ctk.CTkImage(size=(400, 400), light_image=PIL.Image.new(mode="RGB", size=(300, 300), color="black"))
        self.color_image_label = ctk.CTkLabel(self, image=self.color_image, text="")
        self.color_image_label.grid(row=0, column=0, padx=15, pady=10)

        self.depth_image = ctk.CTkImage(size=(400, 400), light_image=PIL.Image.new(mode="RGB", size=(300, 300), color="black"))
        self.depth_image_label = ctk.CTkLabel(self, image=self.depth_image, text="")
        self.depth_image_label.grid(row=1, column=0, padx=15, pady=10)

        self.odd_camera_enabled = ctk.BooleanVar(self)
        self.odd_camera_enabled.set(True)
        self.odd_camera_switch = ctk.CTkSwitch(self, text="ARM/ODD Camera", command=self.camera_switched,
                                               variable=self.odd_camera_enabled)
        self.odd_camera_switch.grid(row=2, column=0, padx=10, pady=10)

        self.color_image_subscription = self.node.create_subscription(Image, '/nanoowl/output_image',
                                                                      self.update_color_image, 10)
        self.depth_image_subscription = self.node.create_subscription(Image, '/odd_arm_cv/selected_depth_image',
                                                                      self.update_depth_image, 10)

        self.color_mux_client = self.node.create_client(MuxSelect, '/nanoowl/input_image_mux/select')
        self.depth_mux_client = self.node.create_client(MuxSelect, '/odd_arm_cv/depth_image_mux/select')

    def update_color_image(self, msg):
        self.color_image.configure(light_image=ros2_image_to_pil(msg))

    def update_depth_image(self, msg):
        self.depth_image.configure(light_image=ros2_image_to_pil(msg))

    def camera_switched(self):
        if self.odd_camera_enabled.get():
            odd_color = MuxSelect.Request()
            odd_color.topic = "/odd/D435/color/image_rect_raw"

            odd_depth = MuxSelect.Request()
            odd_depth.topic = "/odd/D435/depth/image_rect_raw"

            self.color_mux_client.call_async(odd_color)
            self.depth_mux_client.call_async(odd_depth)
        else:
            arm_color = MuxSelect.Request()
            arm_color.topic="/arm/D405/color/image_rect_raw"

            arm_depth = MuxSelect.Request()
            arm_depth.topic = "/arm/D405/depth/image_rect_raw"

            self.color_mux_client.call_async(arm_color)
            self.depth_mux_client.call_async(arm_depth)

    def on_close(self):
        self.running = False


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
