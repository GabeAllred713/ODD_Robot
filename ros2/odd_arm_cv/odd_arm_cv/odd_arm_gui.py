import rclpy
import customtkinter as ctk
import PIL
import os
import numpy as np
#from rosbags.image import message_to_cvimage
#from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from topic_tools_interfaces.srv import MuxSelect


def ros_2_pil(msg, depth_limit=1000):
    if msg.encoding == "bgr8":
        return PIL.Image.frombytes("RGB", (msg.width, msg.height), bytes(msg.data), 'raw', "BGR")
    elif msg.encoding == "16UC1":
        depth_array = np.ndarray(shape=(msg.height, msg.width), dtype=np.uint16, buffer=msg.data)
        depth = ((depth_array.astype(np.float64) / depth_limit).clip(0, 1) * 255).astype(np.uint8)
        return PIL.Image.fromarray(depth)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")


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
        self.odd_camera_enabled.set("odd_control" in self.node.get_node_names() or os.path.exists("/home/odd/"))
        self.odd_camera_switch = ctk.CTkSwitch(self, text="ARM/ODD Camera", command=self.camera_switched,
                                               variable=self.odd_camera_enabled)
        self.odd_camera_switch.grid(row=2, column=0, padx=10, pady=10)

        self.color_image_subscription = self.node.create_subscription(Image, '/nanoowl/output_image',
                                                                      self.update_color_image, 10)
        self.depth_image_subscription = self.node.create_subscription(Image, '/odd_arm_cv/selected_depth_image',
                                                                      self.update_depth_image, 10)

        self.color_mux_client = self.node.create_client(MuxSelect, '/nanoowl/input_image_mux/select')
        self.depth_mux_client = self.node.create_client(MuxSelect, '/odd_arm_cv/depth_image_mux/select')

        self.camera_switched()

    def update_color_image(self, msg):
        self.color_image.configure(light_image=ros_2_pil(msg))

    def update_depth_image(self, msg):
        self.depth_image.configure(light_image=ros_2_pil(msg, 10000 if self.odd_camera_enabled.get() else 600)) # ARM range is about 60cm, measured working with camera UI

    def camera_switched(self):
        if self.odd_camera_enabled.get():
            odd_color = MuxSelect.Request()
            odd_color.topic = "/odd/D435/color/image_rect_raw"

            odd_depth = MuxSelect.Request()
            odd_depth.topic = "/odd/D435/aligned_depth_to_color/image_raw"

            self.color_mux_client.call_async(odd_color)
            self.depth_mux_client.call_async(odd_depth)
        else:
            arm_color = MuxSelect.Request()
            arm_color.topic="/arm/D405/color/image_rect_raw"

            arm_depth = MuxSelect.Request()
            arm_depth.topic = "/arm/D405/aligned_depth_to_color/image_raw"

            self.color_mux_client.call_async(arm_color)
            self.depth_mux_client.call_async(arm_depth)

    def on_close(self):
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    app = ComputerVisionGUI(rclpy.create_node("odd_arm_cv_gui"))
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
