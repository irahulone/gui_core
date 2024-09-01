import os
import logging

# Custom imports
from connect import Connect

logging.basicConfig(level=logging.WARNING)


class Robot:
    def __init__(self):
        self.prefix = "Mobile Robot "
        self.cmds = {
            "ros_topics": "ros2 topic list",
            "ping": "ping {} -c 3 -W 2"
        }
        self.connect = Connect()
        self.stats = self.__init_stats()
        self.robot_count = self.__init_count()

    def __init_stats(self) -> dict:
        """
        stats of interest should be set in the config file
        :return:
        """
        return {
            "USER_LABEL": os.environ.get("USER_LABEL", "User"),
            "IP_LABEL": os.environ.get("IP_LABEL", "IP"),
            "STATUS_LABEL": os.environ.get("STATUS_LABEL", "Status"),
            "BATTERY_LABEL": os.environ.get('BATTERY_LABEL', "Battery Voltage"),
            "VELOCITY_LABEL": os.environ.get('VELOCITY_LABEL', "Actual Velocity")
        }

    def __init_count(self) -> int:
        """
        run subprocess to get all topics and
        count number of robots based off of number of mr (mobile robot) topics
        :return:
        """
        count = 0
        for topic in self.__get_topics():
            if self.stats.get("USER_LABEL").lower() in topic:
                count += 1
        return count

    def ping_host(self, host):
        return self.connect.handle_shell_ping(self.cmds["ping"].format(host))

    def __get_topics(self) -> list:
        response = self.connect.handle_shell_request(self.cmds["ros_topics"])
        return response.split('\n')

    def get_users_from_topics(self) -> list:
        users = []

        for topic in self.get_user_topics():
            try:
                users.append(topic.split("/")[1])
            except IndexError:
                logging.warning(f"Unable to find user in topic: {topic}")

        if not users:
            users.append("test_robot_user")

        return users

    def get_user_topics(self) -> list:
        user_topics = []
        for topic in self.__get_topics():
            if self.stats.get("USER_LABEL").lower() in topic:
                user_topics.append(topic)
        return user_topics

    def get_prefix(self) -> str:
        return self.prefix

    def get_count(self) -> int:
        return self.robot_count

    def get_stats(self) -> dict:
        return self.stats
