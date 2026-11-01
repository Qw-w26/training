import rclpy

from .controller import WheelLegController


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
