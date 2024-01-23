#! /usr/bin/env python3

import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rmf_dispenser_msgs.msg import DispenserState
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_dispenser_msgs.msg import DispenserRequest

import time

from std_msgs.msg import String

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ManualDispenser(Node):

    def __init__(self, manual_dispenser_name="manual_dispenser_0"):
        super().__init__('manual_dispenser')

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()

        self._manual_dispenser_name = manual_dispenser_name

        self._state_pub = self.create_publisher(
            DispenserState, "/dispenser_states", 10)
        self._result_pub = self.create_publisher(
            DispenserResult, "/dispenser_results", 10)

        self.current_state = DispenserState()
        self.latest_request = DispenserRequest()

        self.dispensing_now_flag = False
        self.processing_request_flag = False

        self._request_sub = self.create_subscription(
            DispenserRequest,
            '/dispenser_requests',
            self.dispenser_request_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group1)

        self.DEFAULT_GUI_ACTION = "no_button_pressed"
        self.gui_action = self.DEFAULT_GUI_ACTION

        self._hri_gui_message_sub = self.create_subscription(
            String,
            '/hri_gui_message',
            self.hri_gui_message_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group2)

        timer_period = 0.1
        self.timer = self.create_timer(
            timer_period, self.timer_callback, callback_group=self.group3)

    def hri_gui_message_cb(self, msg):
        self.gui_action = msg.data

    def get_gui_action(self):
        return self.gui_action

    def dispenser_state_publish(self, dispense):

        self.current_state.time = self.get_clock().now().to_msg()

        if dispense:
            self.current_state.mode = DispenserState.BUSY
            self.current_state.request_guid_queue = {
                self.latest_request.request_guid}
        else:
            self.current_state.mode = DispenserState.IDLE
            self.current_state.request_guid_queue.clear()

        self._state_pub.publish(self.current_state)

    def send_dispenser_response(self, status):

        response = DispenserResult()
        response.time = self.get_clock().now().to_msg()
        response.status = status
        response.request_guid = self.latest_request.request_guid
        response.source_guid = self._manual_dispenser_name

        self._result_pub.publish(response)

    def alert_to_human_request_incoming(self):
        self.get_logger().info("FAKE----> ALERT TO HUMAN REQUEST INCOMING")

    def wait_until_manual_despense_signal(self):
        self.get_logger().info("---> WAITING HUMAN ....")

        action = self.get_gui_action()

        if action == "close_window":
            self.get_logger().warning("SHUTING DOWN...")
            rclpy.shutdown()

        return action

    def dispenser_request_cb(self, msg):

        if msg.target_guid == self._manual_dispenser_name:
            if not self.processing_request_flag:
                # To avoid that the callback publishes states
                self.dispensing_now_flag = True

                self.latest_request = msg

                self.dispenser_state_publish(dispense=self.dispensing_now_flag)
                self.send_dispenser_response(DispenserResult.ACKNOWLEDGED)

                self.alert_to_human_request_incoming()

                self.processing_request_flag = True
        else:
            self.get_logger().warning("This Dispenser request isn't for me =" +
                                      str(msg.target_guid)+" not ="+str(self._manual_dispenser_name))

    def timer_callback(self):

        if not self.dispensing_now_flag:
            action = self.get_gui_action()
            if action == "close_window":
                rclpy.shutdown()

            self.dispenser_state_publish(dispense=False)
            # self.get_logger().info("Manual Dispenser "+str(self._manual_dispenser_name) +
            #                        " state IDLE, action="+str(action))

        else:

            manual_dispense_action = self.wait_until_manual_despense_signal()
            self.get_logger().info("---> ACTION DETECTED="+str(manual_dispense_action))

            if (manual_dispense_action == "LOADED"):
                self.send_dispenser_response(DispenserResult.SUCCESS)
                self.get_logger().info("---> HUMAN DISPENSING....DONE")
                self.dispensing_now_flag = False
                self.processing_request_flag = False
            elif (manual_dispense_action == "CANCEL"):
                self.send_dispenser_response(DispenserResult.FAILED)
                self.get_logger().info("Unable to dispense item")

            else:
                pass


def main(args=None):

    rclpy.init(args=args)

    simple_publisher = ManualDispenser()

    # We have potentially 3 callback at the same time, 2 for sure
    num_threads = 3

    simple_publisher.get_logger().info('NUM threads='+str(num_threads))

    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(simple_publisher)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        simple_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
