#! /usr/bin/env python3

import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rmf_ingestor_msgs.msg import IngestorState
from rmf_ingestor_msgs.msg import IngestorResult
from rmf_ingestor_msgs.msg import IngestorRequest

import time

from std_msgs.msg import String

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


"""
Subscribers:
    /ingestor_requests: rmf_ingestor_msgs/msg/IngestorRequest
  Publishers:
    /ingestor_results: rmf_ingestor_msgs/msg/IngestorResult
    /ingestor_states: rmf_ingestor_msgs/msg/IngestorState
"""


class ManualIngestor(Node):

    def __init__(self, manual_ingestor_name="manual_ingestor_0"):
        super().__init__('manual_ingestor')

        self.TRIGGER_ACTION = "UNLOADED"

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()

        self._manual_ingestor_name = manual_ingestor_name

        self._state_pub = self.create_publisher(
            IngestorState, "/ingestor_states", 10)
        self._result_pub = self.create_publisher(
            IngestorResult, "/ingestor_results", 10)

        self.current_state = IngestorState()
        self.latest_request = IngestorRequest()

        self.ingesting_now_flag = False
        self.processing_request_flag = False

        self._request_sub = self.create_subscription(
            IngestorRequest,
            '/ingestor_requests',
            self.ingestor_request_cb,
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

    def ingestor_state_publish(self, dispense):

        self.current_state.time = self.get_clock().now().to_msg()

        if dispense:
            self.current_state.mode = IngestorState.BUSY
            self.current_state.request_guid_queue = {
                self.latest_request.request_guid}
        else:
            self.current_state.mode = IngestorState.IDLE
            self.current_state.request_guid_queue.clear()

        self._state_pub.publish(self.current_state)

    def send_ingestor_response(self, status):

        response = IngestorResult()
        response.time = self.get_clock().now().to_msg()
        response.status = status
        response.request_guid = self.latest_request.request_guid
        response.source_guid = self._manual_ingestor_name

        self._result_pub.publish(response)

    def alert_to_human_request_incoming(self):
        self.get_logger().info("FAKE----> ALERT TO HUMAN REQUEST INCOMING")

    def wait_until_manual_despense_signal(self):
        #self.get_logger().info("---> WAITING HUMAN ....")

        action = self.get_gui_action()

        if action == "close_window":
            self.get_logger().warning("SHUTING DOWN...")
            rclpy.shutdown()

        return action

    def ingestor_request_cb(self, msg):

        if msg.target_guid == self._manual_ingestor_name:
            if not self.processing_request_flag:
                # To avoid that the callback publishes states
                self.ingesting_now_flag = True

                self.latest_request = msg

                self.ingestor_state_publish(dispense=self.ingesting_now_flag)
                self.send_ingestor_response(IngestorResult.ACKNOWLEDGED)

                self.alert_to_human_request_incoming()

                self.processing_request_flag = True
        else:
            self.get_logger().warning("This Ingestor request isn't for me =" +
                                      str(msg.target_guid)+" not ="+str(self._manual_ingestor_name))

    def timer_callback(self):

        if not self.ingesting_now_flag:
            action = self.get_gui_action()
            if action == "close_window":
                rclpy.shutdown()

            self.ingestor_state_publish(dispense=False)
            # self.get_logger().info("Manual ingestor "+self._manual_ingestor_name +
            #                        " state IDLE, action="+str(action))

        else:

            manual_dispense_action = self.wait_until_manual_despense_signal()
            #self.get_logger().info("---> ACTION DETECTED="+str(manual_dispense_action))

            if (manual_dispense_action == self.TRIGGER_ACTION):
                self.send_ingestor_response(IngestorResult.SUCCESS)
                self.get_logger().info("---> HUMAN DISPENSING....DONE")
                self.ingesting_now_flag = False
                self.processing_request_flag = False
            elif (manual_dispense_action == "CANCEL"):
                self.send_ingestor_response(IngestorResult.FAILED)
                self.get_logger().info("Unable to dispense item")

            else:
                pass


def main(args=None):

    rclpy.init(args=args)

    simple_publisher = ManualIngestor()

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
