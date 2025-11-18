#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import LoadController, SwitchController

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller_node')

        # List of controllers from your YAML
        self.controllers = [
            'limb1_position_controller',
            'limb2_position_controller',
            'limb3_position_controller'
        ]

        # Clients
        self.load_client = self.create_client(LoadController, '/controller_manager/load_controller')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        while not self.load_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for LoadController service...')

        while not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SwitchController service...')

        self.load_and_start_controllers()

    def load_and_start_controllers(self):
        loaded = []

        # Load each controller
        for c in self.controllers:
            req = LoadController.Request()
            req.name = c
            future = self.load_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().ok:
                self.get_logger().info(f"Loaded controller: {c}")
                loaded.append(c)
            else:
                self.get_logger().error(f"Failed to load controller: {c}")

        if not loaded:
            self.get_logger().error("No controllers loaded. Exiting.")
            return

        # Switch controllers to start them
        switch_req = SwitchController.Request()
        switch_req.start_controllers = loaded
        switch_req.stop_controllers = []
        switch_req.strictness = SwitchController.Request.STRICT
        switch_req.start_asap = True
        switch_req.timeout = 0.0

        future = self.switch_client.call_async(switch_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().ok:
            self.get_logger().info(f"Controllers started: {loaded}")
        else:
            self.get_logger().error(f"Failed to start controllers: {loaded}")


def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
