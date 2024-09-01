import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32, String
import logging

from robot import Robot

logging.basicConfig(level=logging.WARNING)


class LiveStats(Node):
    """
    @class: Create a GUI based off of the number of robots connected and stats we care about
    """
    def __init__(self, grid):
        super().__init__("gui_updater")
        self.grid = grid
        self.active_subscribers = {}
        self.__create_subscribers()
        self.robot = Robot()
        self.total_hosts = {}

    def __create_subscribers(self):
        topic_info = {
            "{name}/du/velocity": {
                "type": Int32MultiArray,
                "callback": self.parameterized_callback_du_velocity
            },
            "{name}/battery_voltage": {
                "type": Float32,
                "callback": self.parameterized_callback_battery_voltage
            },
            "{name}/user_ip": {
                "type": String,
                "callback": self.parameterized_callback_user_ip_cb
            }
        }

        """
        Create a subscription for each number of vehicles connected to the network
        name is the hostname of the robot.
        Ex:
        edu-robot-1@192.168.1.130
        hostname = edu-robot-1
        """
        for name, stats in self.grid.items():
            for topic, info in topic_info.items():
                if not self.active_subscribers.get(name):
                    self.active_subscribers[name] = []

                self.active_subscribers[name].append(self.create_subscription(
                    msg_type=info["type"],
                    topic=topic.format(name=name),
                    callback=info["callback"](name),
                    qos_profile=10
                ))

    def update_stats_label(self, label, data, robot_name):
        """
        Label in the GUI grid to update
        :param label: specific label to update
        :param data: the new value to update for the specific label
        :return:
        """
        for name, grid_stats in self.grid.items():
            if name == robot_name and grid_stats.get(label):
                grid_stats[label].setText(f"{label}: {data}")

    def parameterized_callback_du_velocity(self, name):
        return lambda msg: self.update_stats_label(self.robot.get_stats()["VELOCITY_LABEL"], msg.data, name)

    def parameterized_callback_battery_voltage(self, name):
        return lambda msg: self.update_stats_label(self.robot.get_stats()["BATTERY_LABEL"], msg.data, name)

    def parameterized_callback_user_ip_cb(self, name):
        return lambda msg: self.user_ip_cb(msg, name)

    def du_velocity_cb(self, msg, name):
        """
        self.vehicle.get_stats() contains all label references, like Battery Voltage and Actual Velocity, on the GUI
        :param msg: vehicle velocity messages that come from topic connection
        :return:
        """

        self.update_stats_label(self.robot.get_stats()["VELOCITY_LABEL"], msg.data, name)

    def battery_voltage_cb(self, msg, name):
        """

        :param msg:
        :param name:
        :return:
        """
        self.update_stats_label(self.robot.get_stats()["BATTERY_LABEL"], msg.data, name)

    def user_ip_cb(self, msg, name):
        """
        msg.data comes in the format as user_name@ip_address
        example: edu-robot-1@192.168.1.130, field[0] = edu-robot-1, field[1] = 192.168.1.130
        :param msg: messages coming from {name}/user_ip topic
        :param name:
        :return: None
        """
        field = msg.data.split('@')
        self.total_hosts[field[0]] = field[1]
        self.update_stats_label(self.robot.get_stats()["USER_LABEL"], field[0], name)
        self.update_stats_label(self.robot.get_stats()["IP_LABEL"], field[1], name)
        # self.check_liveliness(name)

    def check_liveliness(self, name):
        """
        Check if existing hosts are still connected
        failed = 0, means robot is connected
        failed = 1, means robot is disconnected
        :return: None
        """
        for user, ip in self.ips.items():
            failed = self.robot.ping_host(ip)
            if not failed:
                value = "Connected"
            else:
                value = "Disconnected"
                self.destroy_subscribers(user)
            self.update_stats_label(self.robot.get_stats()["STATUS_LABEL"], value, name)

    def destroy_subscribers(self, user):
        """
        Destroy existing topic connections when we see that a robot is no longer connected
        :param user: Robot username
        :return:
        """
        for sub in self.active_subscribers[user]:
            self.destroy_subscription(sub)
