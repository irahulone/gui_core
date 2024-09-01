import rclpy
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QPushButton,
    QWidget
)
import threading
from robot import Robot
from subscribers.window import LiveStats

import logging
logging.basicConfig(level=logging.WARNING)


class GridWindow(QMainWindow):
    """
    @class Main window containing a grid layout that provides the
    foundation for frames containing information to a specific robot
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.robot = Robot()
        self.row_limit = 3
        self.__init_window()
        self.grid = {}

        self.__create_gui()
        self.__create_boxes()

    def __init_window(self):
        title = "Command Center"
        left = 500
        top = 200
        width = 400
        height = 100
        self.setWindowTitle(title)
        self.setGeometry(left, top, width, height)

    def get_grid(self):
        return self.grid

    def __create_gui(self):
        self.central_widget = QWidget()
        self.central_widget_layout = QGridLayout(self.central_widget)

        self.main = QFrame()
        self.central_widget_layout.addWidget(self.main)
        self.staff_main_layout = QGridLayout(self.main)

        self.main_grid = QGridLayout()
        self.staff_main_layout.addLayout(self.main_grid, 0, 0)

        self.setCentralWidget(self.central_widget)

    def __create_boxes(self):
        robot_count = self.robot.get_count()
        robot_count = robot_count if robot_count else 5
        columns = int(robot_count / self.row_limit) + 1
        users = self.robot.get_users_from_topics()
        counter = 1
        for y in range(columns):
            for x in range(self.row_limit):
                if len(users) and x < robot_count:
                    vehicle_header = f"{self.robot.get_prefix()}{counter}"
                    counter += 1
                    main_sub = QGroupBox(vehicle_header)
                    self.main_grid.addWidget(main_sub, y, x)
                    grid_layout = QGridLayout()
                    self.__create_labels(users.pop(0), grid_layout)
                    main_sub.setLayout(grid_layout)

    def __create_labels(self, user, grid_layout):
        count = 0

        for _, name in self.robot.get_stats().items():
            label = QLabel(text=f"{name}: """)
            grid_layout.addWidget(label, count, 0)
            if not self.grid.get(user):
                self.grid[user] = {}

            self.grid[user][name] = label
            count += 1


def main(args=None):
    app = QApplication([])
    rclpy.init(args=args)
    window = GridWindow()
    executor = MultiThreadedExecutor()
    executor.add_node(LiveStats(window.get_grid()))
    window.show()
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    app.exec()


if __name__ == '__main__':
    main()
