import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Subscribers for RGB and Depth images
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.rgb_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Variables to store the latest frames
        self.latest_rgb_frame = None
        self.latest_depth_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety

        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def rgb_callback(self, msg):
        """Callback function to receive and store the latest RGB frame."""
        try:
            with self.frame_lock:
                self.latest_rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error converting RGB image: {e}")

    def depth_callback(self, msg):
        """Callback function to receive and store the latest depth frame."""
        try:
            with self.frame_lock:
                # Use 32FC1 for single-channel float depth images
                self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().warn(f"Error converting depth image: {e}")

    def display_image(self):
        """Main loop to process and display the latest frame."""
        while rclpy.ok():
            with self.frame_lock:
                rgb_frame = self.latest_rgb_frame
                depth_frame = self.latest_depth_frame

            if rgb_frame is not None and depth_frame is not None:
                # Get the dimensions of the RGB frame
                height, width = rgb_frame.shape[:2]

                # Process the current image
                mask, contour, crosshair = self.process_image(rgb_frame, depth_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(rgb_frame, [mask, contour, crosshair])

                # Calculate the desired height of the display window
                small_image_height = 240  # Increased height for small images
                padding = 40  # Increased padding
                display_height = height + small_image_height + padding

                # Create and resize the window dynamically
                cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

                # Show the latest frame
                cv2.imshow("frame", result)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, rgb_img, depth_img):
        """Image processing task to detect and track the nearest black object."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        rows, cols = rgb_img.shape[:2]
        center_x = cols // 2
        center_y = rows // 2

        # Threshold to find black objects (RGB)
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([0, 0, 0], dtype=np.uint8) # Adjust upper bound if needed
        blackMask = cv2.inRange(rgb_img, lower_black, upper_black)

        stackedMask = np.dstack((blackMask, blackMask, blackMask))
        contourMask = stackedMask.copy()
        crosshairMask = rgb_img.copy() # Use RGB image for crosshair overlay

        # Find contours
        contours, _ = cv2.findContours(blackMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours on the contourMask
        cv2.drawContours(contourMask, contours, -1, (0, 255, 0), 2)  # Draw in green color

        nearest_point = None
        min_distance_to_center = float('inf')
        min_object_distance = float('inf') # To store the depth of the nearest point

        # Find the nearest point on the nearest black object
        if len(contours) > 0:
            for contour in contours:
                for point in contour:
                    px, py = point[0]
                    distance_to_center = np.sqrt((px)**2 + (py)**2)

                    # Get distance from depth image
                    if 0 <= py < depth_img.shape[0] and 0 <= px < depth_img.shape[1]:
                        object_distance = depth_img[py, px]
                        if not np.isnan(object_distance) and object_distance < min_object_distance:
                            min_object_distance = object_distance
                            if distance_to_center < min_distance_to_center:
                                min_distance_to_center = distance_to_center
                                nearest_point = (px, py)

            # Process the nearest point
            if nearest_point is not None:
                nearest_x, nearest_y = nearest_point
                cv2.circle(crosshairMask, (nearest_x, nearest_y), 5, (0, 255, 0), -1) # Mark the nearest point

                # Draw crosshair and display distance
                cv2.line(crosshairMask, (nearest_x, 0), (nearest_x, rows), (0, 0, 255), 2)
                cv2.line(crosshairMask, (0, nearest_y), (cols, nearest_y), (0, 0, 255), 2)
                cv2.line(crosshairMask, (center_x, 0), (center_x, rows), (255, 0, 0), 2)
                cv2.putText(crosshairMask, f"{min_object_distance:.2f} m", (nearest_x + 10, nearest_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Control logic to move towards the nearest point
                if abs(center_x - nearest_x) > 20:
                    msg.linear.x = 0.0
                    if center_x > nearest_x:
                        msg.angular.z = 0.2
                    else:
                        msg.angular.z = -0.2
                else:
                    msg.linear.x = 0.2
                    msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish cmd_vel
        self.publisher.publish(msg)

        return blackMask, contourMask, crosshairMask

    def add_small_pictures(self, large_image, small_images):
        """Adds small images horizontally with spacing and offset from top."""
        large_height, large_width, _ = large_image.shape
        num_small = len(small_images)
        if num_small == 0:
            return large_image

        # Parameters for layout
        small_size = min(150, large_width // (num_small * 2))  # Reduce size to accommodate spacing
        spacing = small_size // 2  # Space between images
        top_margin = 20  # Distance from top of the main image

        combined_image = large_image.copy()

        # Calculate total width needed including spacing
        total_width_needed = (small_size * num_small) + (spacing * (num_small - 1))

        # Calculate starting x position to center the group of small images
        start_x_offset = (large_width - total_width_needed) // 2

        for i, img in enumerate(small_images):
            # Resize the small image to be square
            resized_img = cv2.resize(img, (small_size, small_size))

            if len(resized_img.shape) == 2:  # Convert grayscale to BGR
                resized_img = cv2.cvtColor(resized_img, cv2.COLOR_GRAY2BGR)

            # Calculate position with spacing
            start_x = start_x_offset + (i * (small_size + spacing))
            end_x = start_x + small_size

            # Ensure the slice indices are within bounds
            if 0 <= start_x < large_width and 0 < end_x <= large_width:
                # Add a border around each small image
                border_color = (255, 255, 255)  # White border
                border_size = 2
                resized_img = cv2.copyMakeBorder(
                    resized_img,
                    border_size, border_size, border_size, border_size,
                    cv2.BORDER_CONSTANT,
                    value=border_color
                )

                # Place the image with border
                combined_image[
                    top_margin:top_margin + small_size + (2 * border_size),
                    start_x - border_size:end_x + border_size
                ] = resized_img

        return combined_image

    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()

def main(args=None):
    print("OpenCV version: %s" % cv2.__version__)
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        node.display_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()