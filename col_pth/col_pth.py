import rclpy
from .rm520n_rsrp_colorizer import Rm520nRsrpColorizer
from .dummy_colorizer import DummyColorizer
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import math
import time

class ColoredPath(Node):
    def __init__(self):
        super().__init__('colored_path')
        self.get_logger().info(f'Starting colored path node')
        self.get_logger().info(f'++++++++++++++++++++++++++++++++++++++++++++++')
        self.get_logger().info(f'https://github.com/FabianJohnThl/colored-path-ros2-humble')
        self.get_logger().info(f'Developed: jpschreiter, FabianJohnTHL')
        self.get_logger().info(f'License: https://github.com/FabianJohnThl/colored-path-ros2-humble/blob/main/LICENSE')
        self.get_logger().info(f'Cite as: Publicatin pending')
        self.get_logger().info(f'++++++++++++++++++++++++++++++++++++++++++++++')

        self.last_marker = None
        self.pub_marker = None
        self.m_id = 0
        self.markers = MarkerArray()

        # Parameters
        self.declare_parameter('colorizer', 'RM520N')                       # string to select a source for colorizing: {'RM520N', 'DUMMY'}
        self.declare_parameter('movement_min_distance', 1.0)                # Minimum distance to be moved on path before drawing/querying new actual value for colorizing

        self.movement_min_distance = self.get_parameter('movement_min_distance').value
        self.path_in = 'path_in'
        self.marker_array_out = 'path_colored'

        if self.get_parameter('colorizer').value.lower() == 'rm520n':
            self.colorizer = Rm520nRsrpColorizer()
            self.get_logger().info(f'Colorizer of type: RM520N initialized')
        else:
            self.colorizer = DummyColorizer()
            self.get_logger().info(f'Colorizer of type: DUMMY initialized')

        self.subscriber = self.create_subscription(
            Path,
            self.path_in,
            self.colorize_path,
            10)
        self.get_logger().info(f'Subscribed to Path:{self.path_in}')

        self.publisher = self.create_publisher(MarkerArray, self.marker_array_out, 10)
        self.get_logger().info(f'Publish MarkerArray on:{self.marker_array_out}')

    def new_marker(self, path_msg, color):
        marker = Marker()
        marker.header = path_msg.header
        marker.id = self.m_id
        self.m_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # line width

        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.points = []
        return marker

    def colorize_path(self, path_msg):
        curr_pos = path_msg.poses[-1].pose.position
        if not self.last_marker:
            color = self.colorizer.query_current_color()
            self.last_marker = {'color': color, 'pos': curr_pos}
            return
        
        if not self.movement_detected(curr_pos):
            return

        color = self.colorizer.query_current_color()

        if not self.pub_marker:
            self.pub_marker = self.new_marker(path_msg, self.last_marker['color'])
            self.markers.markers.append(self.pub_marker)
        
        self.pub_marker.points.append(self.last_marker['pos'])
        
        if not self.colorizer.is_same_as_current_color(self.last_marker['color']):
            self.pub_marker.points.append(curr_pos)
            self.publisher.publish(self.markers) 
            self.last_marker = {'color': color, 'pos': curr_pos}           
            self.pub_marker = None
        else:
            self.last_marker = {'color': color, 'pos': curr_pos}
            self.publisher.publish(self.markers)
        

    def movement_detected(self, curr_pos):
        distance = math.sqrt( math.pow((self.last_marker['pos'].x - curr_pos.x), 2) +
                              math.pow((self.last_marker['pos'].y - curr_pos.y), 2) + 
                              math.pow((self.last_marker['pos'].z - curr_pos.z), 2) )
        # movement has to be at least minimum distance
        return (distance > self.movement_min_distance)


def main(args=None):
    rclpy.init(args=args)
    colored_path = ColoredPath()

    rclpy.spin(colored_path)

    colored_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
