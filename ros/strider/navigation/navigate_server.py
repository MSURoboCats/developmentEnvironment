import time
import random

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Import Navigaton action
from strider.action import Navigate


class NavigateActionServer(Node):
    def __init__(self):
        # Call constructor for parent Node class
        super().__init__('navigate_server')

        # Create a new instance of an ActionServer
        # Node -> self (NavigateServerClient instance)
        # Action -> Navigation
        # Name -> navigate
        # Action CB -> self.execute_callback
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Mock recieving data from depth sensor topic
        feedback = Navigate.Feedback()
        feedback.current_depth = random.randint(0, 10)
        
        # While the target depth has not bene reached,
        while feedback.current_depth != goal_handle.request.target_depth:
            if (feedback.current_depth > goal_handle.request.target_depth):
                feedback.current_depth -= 1
            else:
                feedback.current_depth += 1
            
            self.get_logger().info('Feedback: {0}'.format(feedback.current_depth))

            # Publish a feedback object
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()

        # Set the result object
        result = Navigate.Result()
        result.new_depth = feedback.current_depth

        return result


def main(args=None):
    rclpy.init(args=args)
    navigate_action_server = NavigateActionServer()
    rclpy.spin(navigate_action_server)

if __name__ == '__main__':
    main()