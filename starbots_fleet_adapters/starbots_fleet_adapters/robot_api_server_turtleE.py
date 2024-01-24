#! /usr/bin/env python3
import sys
import math
import threading
import rclpy
import uvicorn
from fastapi import FastAPI
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rmf_fleet_msgs.msg import RobotState
from rmf_fleet_msgs.msg import PathRequest
from rmf_fleet_msgs.msg import Location
from std_msgs.msg import String
from pydantic import BaseModel
from typing import Optional
from rmf_fleet_msgs.msg import RobotMode

app = FastAPI(title="ROS2 Robot Rest API",
              description="This is the API for a ROS2 Fleet of robots using RMF",
              version="1.0.0",)

# CLASSES FOR THE API RETURN AND REQUEST


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


class Request(BaseModel):
    map_name: str
    task: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None

#######


###
# MAIN CLASS


class SimpleRobotApiServer(Node):

    def __init__(self, fleet_name, robot_in_fleet_list, linear_vel, angular_vel, timer_period=0.1):
        print("Init Start Complete Robot Api server for "+str(fleet_name))
        self._fleet_name = fleet_name
        self._robot_in_fleet_list = robot_in_fleet_list
        self._linear_vel = linear_vel
        self._angular_vel = angular_vel
        self._timer_update_period = timer_period

        super().__init__(self._fleet_name+'_simple_robot_api_server_node')

        # INIT Section
        self.init_vars()
        self.init_subscribers()
        self.init_publishers()

        ################################
        # APP REST API METHOD DEFINITION
        ################################
        # Check Connection
        @app.get('/'+self._fleet_name+'/check_connection/',
                 response_model=Response)
        async def check_connection():
            """
            Return True if connection to the robot API server is successful:
            """
            message = "Connection to fleet "+self._fleet_name+" OK"
            data = {'success': True, 'msg': message}

            return data

        @app.get('/'+self._fleet_name+'/status/',
                 response_model=Response)
        async def status(robot_name: Optional[str] = None):
            data = {'data': {},
                    'success': False,
                    'msg': ''}
            state = self.robots_fleet.get(robot_name)
            if state is None:
                return data
            else:
                data['data'] = self.get_robot_state(state, robot_name)
                data['success'] = True
                return data

        @app.post('/'+self._fleet_name+'/navigate/',
                  response_model=Response)
        async def navigate(robot_name: str, dest: Request):
            data = {'success': False, 'msg': ''}
            if robot_name not in self.robots_fleet:
                data = {'success': False, 'msg': "Robot " +
                        str(robot_name)+" not in fleet "+str(self._fleet_name)}
                return data
            elif len(dest.destination) < 1:
                data = {'success': False, 'msg': 'Destination empty=' +
                        str(dest.destination)+"="}
                return data
            else:
                target_x = dest.destination['x']
                target_y = dest.destination['y']
                target_yaw = dest.destination['yaw']

                result_send_path_request = self.send_path_request(
                    target_x, target_y, target_yaw, robot_name)

                data['success'] = result_send_path_request
                data['msg'] = "Teleport Nav Goal sent"
                return data

        @app.post('/'+self._fleet_name+'/start_task/',
                  response_model=Response)
        async def start_process(robot_name: str, task: Request):
            data = {'success': False, 'msg': ''}

            if robot_name not in self.robots_fleet:
                data['msg'] = "Robot " + \
                    str(robot_name)+" not in fleet "+str(self._fleet_name)
                return data
            elif len(task.task) < 1:
                data['msg'] = 'Task empty=' + str(task.task)+"="
                return data
            else:

                process_name = task.task

                if "pick" in process_name:
                    self.get_logger().info('Starting pick task..')
                    self.publish_task_done(
                        object_picked_name="cube_rubish", robot_name=robot_name)
                    self.get_logger().info('Starting pick task..DONE')
                else:
                    self.get_logger().info('Process not supported by RobotHWI ='+str(process_name))

                self.get_logger().info('Process Done...')

                data['success'] = True
                return data

        @app.get('/'+self._fleet_name+'/stop_robot/',
                 response_model=Response)
        async def stop(robot_name: str):
            data = {'success': False, 'msg': ''}
            if robot_name not in self.robots_fleet:
                data['msg'] = "Robot " + \
                    str(robot_name)+" not in fleet "+str(self._fleet_name)
                return data

            self.send_stop_request(robot_name)

            data['success'] = True
            return data

        #################################

        self.get_logger().info('Init DONE')

    def init_vars(self):
        """
        We initialise all teh variables that we will use to store the state
        and commands issued
        """
        # We create the var robots_fleet, to store their current state
        none_list = len(self._robot_in_fleet_list) * [None]
        self.robots_fleet = dict(zip(self._robot_in_fleet_list, none_list))

        # Init Fleet Vehicles Traits
        self.vehicle_traits_linear_nominal_velocity = self._linear_vel
        self.vehicle_traits_rotational_nominal_velocity = self._angular_vel

        # We init the dictionary that states if robot has a pending task
        # Example: {"box_bot_1":None,"box_bot_2":32}, box_bot_1 has nothing pending
        # box_bot_2 has 32 task pending.
        none_list = len(self._robot_in_fleet_list) * [None]
        self.robot_pending_tasks = dict(
            zip(self._robot_in_fleet_list, none_list))

        # We need a list of each of teh task id's given to the robots
        # The reason is taht we can't repeat task ids in each robot.
        task_id_list = len(self._robot_in_fleet_list) * [0]
        self.robot_next_task_id = dict(
            zip(self._robot_in_fleet_list, task_id_list))

        # We need to initialise also the data from estimated duration and completed task
        dest_duration_list = len(self._robot_in_fleet_list) * [0.0]
        self.robot_dest_duration = dict(
            zip(self._robot_in_fleet_list, dest_duration_list))

    def init_subscribers(self):

        # Init Subscribers
        robot_state_topic = "/robot_state"
        self.create_subscription(
            RobotState,
            robot_state_topic,
            self.robot_state_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.get_logger().info("init_subscribers DONE")

    def robot_state_callback(self, msg):
        if msg.name in self.robots_fleet:
            # We save the state of the robot
            self.robots_fleet[msg.name] = msg
            # self.get_logger().info(str(msg.name)+"==robot_state_callback, update pos=" +
            #                        str(self.robots_fleet[msg.name].location)+", bat="+str(self.robots_fleet[msg.name].battery_percent))
            self.update_pending_tasks(msg.name, msg.mode.mode)
        else:
            self.get_logger().error(str(msg.name)+"==robot_state_callback, Robot Not in fleet==" +
                                    str(msg.name)+","+str(self.robots_fleet))

    def update_pending_tasks(self, robot_name, robot_mode):
        pending_task = self.robot_pending_tasks[robot_name]

        self.get_logger().info(str(robot_name)+",pending=>"+str(pending_task) +
                               "=== robot_mode=>"+str(robot_mode)+"<==")
        # We check if the robot is Iddle, which means that it has finished that task
        if robot_mode == RobotMode.MODE_IDLE:
            # We remove that task form the list of that robot
            self.robot_pending_tasks[robot_name] = None
            # We set duration to 0.0
            self.robot_dest_duration[robot_name] = 0.0
        else:
            self.get_logger().info(str(robot_name)+",MOVING STILL")

    def robot_completed_request(self, robot_name):
        return self.robot_pending_tasks[robot_name] is None

    def init_publishers(self):
        self.robot_request_seq = 0

        task_done_topic = "/task_done"
        self.task_done_pub = self.create_publisher(
            String, task_done_topic, 1)

        self.path_pub = self.create_publisher(
            PathRequest,
            '/robot_path_requests',
            qos_profile=qos_profile_system_default)

    def get_robot_state(self, state, robot_name):
        """
        We format from the following message: RobotState
        """
        data = {}
        position = [state.location.x, state.location.y]
        angle = state.location.yaw
        data['robot_name'] = robot_name
        data['map_name'] = state.location.level_name
        data['position'] =\
            {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = state.battery_percent
        # This is hardcoded because its teleporting
        data['destination_arrival_duration'] = self.robot_dest_duration[robot_name]
        data['completed_nav_request'] = self.robot_completed_request(
            robot_name)
        # TODO: We supose all teh tasks are finished inmediatelly when executed
        data['completed_task_request'] = True
        return data

    def publish_task_done(self, object_picked_name, robot_name):

        task_done_msg = String()
        task_done_msg.data = str(robot_name) + \
            ">>Task Done: "+str(object_picked_name)
        self.task_done_pub.publish(task_done_msg)

    def send_path_request(self, in_x, in_y, in_yaw, robot_name, map_name="L1", speed_limit=100.0):

        #######

        target_x = in_x
        target_y = in_y
        target_yaw = in_yaw
        target_map = map_name
        target_speed_limit = speed_limit

        t = self.get_clock().now().to_msg()

        path_request = PathRequest()

        robot_state = self.robots_fleet[robot_name]
        cur_x = robot_state.location.x
        cur_y = robot_state.location.y
        cur_yaw = robot_state.location.yaw
        cur_loc = robot_state.location
        path_request.path.append(cur_loc)

        # We calculate the estimated duration of this path
        # Based on current_location and destination_location
        disp = self.disp([target_x, target_y], [cur_x, cur_y])
        duration = int(disp/self.vehicle_traits_linear_nominal_velocity) +\
            int(abs(abs(cur_yaw) - abs(target_yaw)) /
                self.vehicle_traits_rotational_nominal_velocity)
        # We store this duration in dict
        self.robot_dest_duration[robot_name] = duration

        t.sec = t.sec + duration
        target_loc = Location()
        target_loc.t = t
        target_loc.x = target_x
        target_loc.y = target_y
        target_loc.yaw = target_yaw
        target_loc.level_name = target_map
        if target_speed_limit > 0:
            target_loc.obey_approach_speed_limit = True
            target_loc.approach_speed_limit = target_speed_limit

        path_request.fleet_name = self._fleet_name
        path_request.robot_name = robot_name
        path_request.path.append(target_loc)

        # We set a New CMD_id
        cmd_id = self.generate_task_robot(robot_name)
        path_request.task_id = str(cmd_id)
        self.path_pub.publish(path_request)

        self.get_logger().info(
            f'Sending path request for {robot_name}: {cmd_id}')

        return True

    def send_stop_request(self, robot_name):

        robot_state = self.robots_fleet[robot_name]
        path_request = PathRequest()
        path_request.fleet_name = self._fleet_name
        path_request.robot_name = robot_name
        path_request.path = []
        # Appending the current location twice will effectively tell the
        # robot to stop
        path_request.path.append(robot_state.location)
        path_request.path.append(robot_state.location)

        # We set the cmd_id
        cmd_id = self.generate_task_robot(robot_name)
        path_request.task_id = str(cmd_id)
        self.path_pub.publish(path_request)

        self.get_logger().info(
            f'Sending stop request for {robot_name}: {cmd_id}')

        return True

    def generate_task_robot(self, robot_name):
        """
        We generate the task id
        We assign it to the pensidng task id of the robot
        We Generate the next one
        """
        next_id = self.robot_next_task_id[robot_name]
        self.robot_pending_tasks[robot_name] = next_id
        self.robot_next_task_id[robot_name] += 1
        return self.robot_pending_tasks[robot_name]

    def disp(self, A, B):
        """
        Disparity Between two 2D points
        """
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


def main(argv=None):
    # Init rclpy and adapter
    rclpy.init(args=argv)

    fleet_name = "turtlebot_5"
    robots_in_fleet_list = ["turtlebot_5"]
    linear_vel = 0.5
    angular_vel = 0.6

    robot_api_server_obj = SimpleRobotApiServer(
        fleet_name, robots_in_fleet_list, linear_vel, angular_vel)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(robot_api_server_obj,))
    spin_thread.start()

    uvicorn.run(app,
                host='127.0.0.1',
                port='8000',
                log_level='warning')


if __name__ == '__main__':
    main(sys.argv)