import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import Navigaton action
from strider.action import Navigate


class NavigateActionClient(Node):
    def __init__(self):
        # Call constructor for parent Node class
        super().__init__('navigate_client')

        # Create a new instance of an ActionClient
        # Node -> self (NavigateActionClient instance)
        # Action -> Navigation
        # Name -> navigate
        self._action_client = ActionClient(self, Navigate, 'navigate')

    def send_goal(self, depth):
        goal_msg = Navigate.Goal()
        goal_msg.target_depth = depth

        # Blocking call... wait for navigate action server to respond
        self._action_client.wait_for_server()

        # If the action server accepts the goal, call the feedback callback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # If the action server completes the goal, call the response callback
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # If the accepted property is not defined, halt execution bc the action server rejected the goal
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal was rejected')
            return

        self.get_logger().info('Navigation goal was accepted')

        # Create a future that resolves to the result form the action 
        self._get_result_future = goal_handle.get_result_async()
        
        # Associate a callback with completion of the goal
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Upon completion of the goal, fetch the returned result from the resolved result object
        result = future.result().result

        self.get_logger().info('Result: {0}'.format(result.new_depth))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # Upon recieving a feedback message, fetch the message from the resolved feedback object
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_depth))


def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()