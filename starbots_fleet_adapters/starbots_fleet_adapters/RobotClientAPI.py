#! /usr/bin/env python3

import requests
from urllib.error import HTTPError
import urllib


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str, fleet_name: str, timeout: float = 5.0, debug: bool = True):
        self.fleet_name = fleet_name
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False

        self.timeout = timeout
        self.debug = debug

        self.url_base = self.prefix + f'/'+self.fleet_name

        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        url = self.url_base+'/check_connection/'

        print("CHECK CONNECTION ROBOTCLIENTAPI SIDE="+url)
        try:
            response = requests.get(url, timeout=float(self.timeout))

            data = response.json()
            if self.debug:
                print("Response:" + str(data['msg']))
            return data['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''

        url = self.url_base+f'/status/?robot_name={robot_name}'

        try:
            response = requests.get(url, timeout=float(self.timeout))
            response.raise_for_status()
            data = response.json()
            if self.debug:
                #print(f'Response: {data}')
                pass
            if not data['success']:
                return None
            x = data['data']['position']['x']
            y = data['data']['position']['y']
            angle = data['data']['position']['yaw']
            return [x, y, angle]
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return None

    def navigate(self,
                 robot_name: str,
                 pose,
                 map_name: str,
                 speed_limit=0.0):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''

        assert(len(pose) > 2)
        url = self.url_base+f'/navigate/?robot_name={robot_name}'

        data = {}  # data fields: map_name, destination{}, speed_limit
        data['map_name'] = map_name
        data['destination'] = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
        data['speed_limit'] = speed_limit
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False

    def start_process(self,
                      robot_name: str,
                      process: str,
                      map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''

        info = None

        url = self.url_base+f'/start_task/?robot_name={robot_name}'

        # data fields: task, map_name
        data = {'task': process, 'map_name': map_name}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            info = response.json()
            return response.json()['success'], info
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
            info = http_err
        except Exception as err:
            print(f'Other error: {err}')
            info = err
        return False, info

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        url = self.url_base+f'/stop_robot/?robot_name={robot_name}'
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        response = self.data(robot_name)
        if response is not None:
            return response['data']['destination_arrival_duration']
        else:
            return 0.0

    def navigation_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        response = self.data(robot_name)
        if response is not None and response.get('data') is not None:
            return response['data']['completed_nav_request']
        else:
            return False

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''

        response = self.data(robot_name)
        if response is not None and response.get('data') is not None:
            return response['data']['completed_task_request']
        else:
            return False

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        response = self.data(robot_name)
        if response is not None:
            return response['data']['battery']/100.0
        else:
            return None

    # Added method, used internally
    def data(self, robot_name=None):
        url = self.url_base+f'/status/?robot_name={robot_name}'

        try:
            response = requests.get(url, timeout=float(self.timeout))
            response.raise_for_status()
            data = response.json()
            if self.debug:
                print(f'Response: {data}')
            return data
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return None
