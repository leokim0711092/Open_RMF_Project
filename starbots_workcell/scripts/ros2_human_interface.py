#! /usr/bin/env python3

from ast import Str
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory

import PySimpleGUI as sg


class HRIGui(object):

    def __init__(self, function=None):

        self.button_1_lable = "LOADED"
        self.button_2_lable = "UNLOADED"

        logo_image_path = os.path.join(
            get_package_share_directory('starbots_workcell'), 'images', 'starbotslogo.png')

        self.layout = [[sg.Image(logo_image_path, size=(300, 300))], [sg.Text("HRI RMF Interface")], [
            sg.Button(self.button_1_lable)], [sg.Button(self.button_2_lable)]]

        # Create the window
        self.window = sg.Window("HRI", self.layout)

        self.function = lambda x: function(x)

    def __del__(self):
        self.window.close()

    def update(self):
        event, values = self.window.read(timeout=100)

        if event == self.button_1_lable:
            message = self.button_1_lable
            self.do_something(message)
        elif event == self.button_2_lable:
            message = self.button_2_lable
            self.do_something(message)
        elif event == sg.WIN_CLOSED:
            self.window.close()
            message = "close_window"
            self.do_something(message)
        else:
            message = "no_button_pressed"
            self.do_something(message)

        return event

    def do_something(self, message):
        if self.function:
            self.function(message)


class WorkCellPublisher(Node):

    def __init__(self):
        super().__init__('workcell_publisher')

        self.publisher_ = self.create_publisher(String, 'hri_gui_message', 1)
        self.hri_obj = HRIGui(self.publish_message)
        self.get_logger().info("WorkCellPublisher Ready...")

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        action = self.update_GUI()
        if action == "close_window":
            self.destroy_node()
            rclpy.shutdown()

        # self.get_logger().info("#############################################")

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        # self.get_logger().info("Publishing:"+str(msg.data))

    def update_GUI(self):
        return self.hri_obj.update()


def main(args=None):
    rclpy.init(args=args)

    wcp_publisher = WorkCellPublisher()
    rclpy.spin(wcp_publisher)
    wcp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
