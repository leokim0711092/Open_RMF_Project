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

from rclpy.action import ActionClient
from panda_action_msgs.action import PandaMove
from geometry_msgs.msg import Pose

from object_teleport_msgs.srv import ObjectTeleport
from gazebo_msgs.srv import GetEntityState


class PandaRobotArmDispenser(Node):

    def __init__(self, manual_dispenser_name="ur3_robot_arm_dispenser"):
        super().__init__('ur3_robot_arm_dispenser')

        self.reset_vars()

        # Init PandaArm Client to be able to send commands to the arm
        self._action_client = ActionClient(self, PandaMove, 'panda_arm_as')

        # Setting up client for knowing the position of the robotmaking the request
        # ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "name: 'cute_mug_1' reference_frame: 'world'"
        self.get_state_srv_name = '/gazebo/get_entity_state'
        self.client_entity_state = self.create_client(
            GetEntityState, self.get_state_srv_name)
        while not self.client_entity_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service "+self.get_state_srv_name +
                                   "not available, waiting again...")
        self.req_entity_state = GetEntityState.Request()
        self.get_logger().info("---> SERVICE READY ==> " +
                               str(self.get_state_srv_name))

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()

        self._manual_dispenser_name = manual_dispenser_name

        self._state_pub = self.create_publisher(
            DispenserState, "/dispenser_states", 10)
        self._result_pub = self.create_publisher(
            DispenserResult, "/dispenser_results", 10)

        self.current_state = DispenserState()
        self.latest_request = DispenserRequest()

        self._request_sub = self.create_subscription(
            DispenserRequest,
            '/dispenser_requests',
            self.dispenser_request_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group1)

        self.object_teleport_name = '/object_teleporter'
        self.object_teleport_client = self.create_client(
            ObjectTeleport, self.object_teleport_name)
        while not self.object_teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ObjectTeleport.Request()
        self.get_logger().info("---> SERVICE READY ==> " +
                               str(self.object_teleport_name))

        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback, callback_group=self.group3)

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

    def set_robot_arm_task(self, task_name):
        self.dispenser_phase = "moving_arm"
        self.get_logger().info("---> Setting Robot ARm task ==> " +
                               str(task_name))
        # Set positions to move to
        pose_array = []
        if task_name == "make_coffee":
            pose_1 = Pose()

            pose_1.position.x = 0.5
            pose_1.position.y = 0.0
            pose_1.position.z = 0.25

            pose_1.orientation.x = 1.0
            pose_1.orientation.y = 0.0
            pose_1.orientation.z = 0.0
            pose_1.orientation.w = 0.0

            pose_2 = Pose()

            pose_2.position.x = 0.0
            pose_2.position.y = 0.5
            pose_2.position.z = 0.25

            pose_2.orientation.x = 1.0
            pose_2.orientation.y = 0.0
            pose_2.orientation.z = 0.0
            pose_2.orientation.w = 0.0
            pose_array.append(pose_1)
            pose_array.append(pose_2)

            self.get_logger().info("Created trajectories for make_coffee TASK")
        else:
            self.get_logger().error("TASK NOT SUPPORTED")

        if len(pose_array) > 0:
            self.get_logger().error("Sending pose array generated=..."+str(pose_array))
            self.send_goal(pose_array)
        else:
            pass

    def dispenser_request_cb(self, msg):

        # We wont process new request until we finish the one we are executing first
        if not self.dispensing_now_flag:
            if msg.target_guid == self._manual_dispenser_name:
                # To avoid that the callback publishes states
                self.get_logger().info("---> Accepted Request for => " +
                                       str(self._manual_dispenser_name))
                self.dispensing_now_flag = True

                self.latest_request = msg

                self.dispenser_state_publish(dispense=self.dispensing_now_flag)
                self.send_dispenser_response(DispenserResult.ACKNOWLEDGED)

                # self.object_to_teleport_name = msg.items[0]
                # TODO: This is hardcoded, we have to extract it from msg.items.type_guid
                self.object_to_teleport_name = "portable_cup_2"
                self.requester_name = msg.transporter_type
                # TODO: This is hardcoded, this means that it will only work for the ROBOT_NAMEFLEET_1
                self.name_with_number = self.requester_name + \
                    "_"+str(self.robot_counter)
                self.make_request_get_state(self.name_with_number)

            else:
                self.get_logger().warning("This Dispenser request isn't for me =" +
                                          str(msg.target_guid)+" not ="+str(self._manual_dispenser_name))
        else:
            self.get_logger().warning(
                "Request not processed because there is one still being executed")

    def timer_callback(self):

        if not self.dispensing_now_flag:
            self.dispenser_state_publish(dispense=False)
            #self.get_logger().info("---> Robot Arm Waiting...")
        else:
            self.get_logger().info("---> Dispensing...PHASE="+str(self.dispenser_phase))
            if self.dispenser_phase == "getting_robot_pose":

                if self.future_get_state.done():
                    try:
                        response_get_state = self.future_get_state.result()

                        # We check it went ok:
                        if response_get_state.success:

                            self.teleport_location = response_get_state.state.pose
                            self.set_robot_arm_task(task_name="make_coffee")

                        else:
                            self.get_logger().error("ERROR: No model in Gazebo with name=" +
                                                    str(self.name_with_number))
                            self.reset_vars()

                    except Exception as e:
                        self.get_logger().error(
                            'Service call failed %r' % (e,))
                else:
                    self.get_logger().info("Waiting for Get State service call to finish")

            elif self.dispenser_phase == "moving_arm":
                # The change of state is done in an action callback
                self.get_logger().info("Robot Arm Moving...")
            elif self.dispenser_phase == "teleporting_object":

                if self.future_teleport.done():
                    try:
                        response_teleport = self.future_teleport.result()

                        # We check it went ok:
                        if response_teleport.success:
                            self.get_logger().info("DISPENSE HAS FINISHED!")
                            # We tell RMF that we finished
                            self.send_dispenser_response(
                                DispenserResult.SUCCESS)
                            self.reset_vars()

                        else:
                            self.get_logger().error("ERROR: In Teleport Model name=" +
                                                    str(self.object_to_teleport_name)+" to pose="+str(self.teleport_location))
                            self.reset_vars()

                    except Exception as e:
                        self.get_logger().error(
                            'Service call failed %r' % (e,))
                else:
                    self.get_logger().info("Waiting for Get State service call to finish")
            else:
                self.get_logger().error("ERROR, not supported dispenser_phase="+str(self.dispenser_phase))

    def reset_vars(self):
        self.requester_name = "no_requester_name"
        self.object_to_teleport_name = "no_object"
        self.dispensing_now_flag = False
        self.dispenser_phase = "nothing"
        self.robot_counter = 1

    # TODO: Remove when finished new version

    def timer_callback_old(self):

        if not self.dispensing_now_flag:
            self.dispenser_state_publish(dispense=False)
            self.get_logger().info("---> Robot Arm Waiting...")
        else:
            self.get_logger().info("---> Dispensing...")

    def send_goal(self, pose_list):
        self.get_logger().error("Start Sending Goal...")
        goal_msg = PandaMove.Goal()

        goal_msg.goal_pose_array = pose_list

        self.get_logger().info('PandaArmClient send_goal_waiting for server...')
        self._action_client.wait_for_server()
        self.get_logger().info('PandaArmClient send_goal_waiting for server...Done, Sending goal')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('PandaArmClient goal sent..')
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result_status))

        # We call the teleport service to teleport the mug
        self.send_teleport_request(
            self.teleport_location, self.object_to_teleport_name)

    def send_teleport_request(self, in_pose, in_name, default_height=0.5):

        self.req.new_pose.position.x = in_pose.position.x
        self.req.new_pose.position.y = in_pose.position.y
        self.req.new_pose.position.z = default_height
        self.req.object_name = in_name

        self.get_logger().info("Teleport Object ="+str(self.req))
        self.future_teleport = self.object_teleport_client.call_async(self.req)
        self.get_logger().info("Teleport Object DONE...")
        self.dispenser_phase = "teleporting_object"

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.feedback_current_pose))

    def make_request_get_state(self, new_name, ref_frame="world"):
        self.get_logger().info("Requesting Get Model state in Gazebo...")
        self.req_entity_state.name = new_name
        self.req_entity_state.reference_frame = ref_frame
        self.future_get_state = self.client_entity_state.call_async(
            self.req_entity_state)
        self.dispenser_phase = "getting_robot_pose"
        self.get_logger().info("Requesting Get Model state in Gazebo...DONE")


def main(args=None):

    rclpy.init(args=args)

    simple_publisher = PandaRobotArmDispenser()

    # We have potentially 3 callback at the same time, 2 for sure
    num_threads = 2

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
