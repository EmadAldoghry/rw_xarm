#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_srvs.srv import SetBool

class ImageSegmenterNode(Node):
    def __init__(self):
        super().__init__('image_segmenter_node')
        
        # --- FIX: All parameters are DECLARED first ---
        self.declare_parameter('input_image_topic', 'camera/image')
        self.declare_parameter('output_mask_topic', '/target_mask')
        self.declare_parameter('img_w', 1920)
        self.declare_parameter('img_h', 1200)
        self.declare_parameter('min_contour_area', 100)
        
        # --- Now we can GET the values ---
        self.img_w_param = self.get_parameter('img_w').value
        self.img_h_param = self.get_parameter('img_h').value
        
        # Declare parameters that depend on other parameters
        default_roi_x_start = int(self.img_w_param * 0 / 100.0)
        default_roi_y_start = int(self.img_h_param * 19 / 100.0)
        default_roi_x_end = int(self.img_w_param * 100 / 100.0)
        default_roi_y_end = int(self.img_h_param * 76 / 100.0)

        self.declare_parameter('roi_x_start', default_roi_x_start)
        self.declare_parameter('roi_y_start', default_roi_y_start)
        self.declare_parameter('roi_x_end', default_roi_x_end)
        self.declare_parameter('roi_y_end', default_roi_y_end)
        
        for name, default in [('black_h_min', 0), ('black_s_min', 0), ('black_v_min', 0),
                              ('black_h_max', 180), ('black_s_max', 255), ('black_v_max', 0)]:
            self.declare_parameter(name, default)

        # Get the rest of the parameter values
        self.min_contour_area_ = self.get_parameter('min_contour_area').value
        self.roi_x_start = self.get_parameter('roi_x_start').value
        self.roi_y_start = self.get_parameter('roi_y_start').value
        self.roi_x_end = self.get_parameter('roi_x_end').value
        self.roi_y_end = self.get_parameter('roi_y_end').value
        self.h_min, self.s_min, self.v_min = self.get_parameter('black_h_min').value, self.get_parameter('black_s_min').value, self.get_parameter('black_v_min').value
        self.h_max, self.s_max, self.v_max = self.get_parameter('black_h_max').value, self.get_parameter('black_s_max').value, self.get_parameter('black_v_max').value

        # Activation state and service
        self.is_active = False
        self.activation_service_ = self.create_service(SetBool, '~/activate_pipeline', self.handle_activation_request)

        # ROS Communications
        self.bridge_ = CvBridge()
        qos_sensor = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.image_sub_ = self.create_subscription(
            Image, self.get_parameter('input_image_topic').value, self.image_callback, qos_sensor
        )
        self.mask_pub_ = self.create_publisher(
            Image, self.get_parameter('output_mask_topic').value, 10
        )
        self.get_logger().info('Image Segmenter Node (with contour filtering) started. WAITING FOR ACTIVATION.')

    def handle_activation_request(self, request: SetBool.Request, response: SetBool.Response):
        self.is_active = request.data
        self.get_logger().info(f"Image segmentation pipeline {'ACTIVATED' if self.is_active else 'DEACTIVATED'}.")
        response.success = True
        response.message = "Activation state set."
        return response

    def image_callback(self, msg: Image):
        if not self.is_active:
            empty_mask = np.zeros((self.img_h_param, self.img_w_param), dtype=np.uint8)
            mask_msg = self.bridge_.cv2_to_imgmsg(empty_mask, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_pub_.publish(mask_msg)
            return

        try:
            cv_image = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return
        
        roi_content = cv_image[self.roi_y_start:self.roi_y_end, self.roi_x_start:self.roi_x_end]
        hsv_roi = cv2.cvtColor(roi_content, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([self.h_min, self.s_min, self.v_min])
        upper_hsv = np.array([self.h_max, self.s_max, self.v_max])
        segmentation_mask_in_roi = cv2.inRange(hsv_roi, lower_hsv, upper_hsv)
        
        contours, _ = cv2.findContours(segmentation_mask_in_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area_]
        
        final_mask_in_roi = np.zeros_like(segmentation_mask_in_roi)
        if valid_contours:
            largest_contour = max(valid_contours, key=cv2.contourArea)
            cv2.drawContours(final_mask_in_roi, [largest_contour], -1, (255), thickness=cv2.FILLED)
        
        full_mask = np.zeros((self.img_h_param, self.img_w_param), dtype=np.uint8)
        full_mask[self.roi_y_start:self.roi_y_end, self.roi_x_start:self.roi_x_end] = final_mask_in_roi
        
        mask_msg = self.bridge_.cv2_to_imgmsg(full_mask, encoding="mono8")
        mask_msg.header = msg.header
        self.mask_pub_.publish(mask_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSegmenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()