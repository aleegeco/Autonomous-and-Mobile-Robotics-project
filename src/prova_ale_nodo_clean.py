import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
import numpy as np

class class_occupancy_grid_sanitized():
    def __init__(self):
        self.n = 2
        self.m = 2
        self.grid = np.zeros((self.n,self.m))
        self.discretized_grid = np.zeros((self.n,self.m))

        self.sanitized_map = OccupancyGrid(header=rospy.Header())
        self.area_to_sanitize = GridCells(header=rospy.Header())
        self.sanitized_area = GridCells(header=rospy.Header())

        self.walls_cells = GridCells(header=rospy.Header())
        self.unexplored_cells = GridCells(header=rospy.Header())

    def callback_sanitize(self, data):
        self.sanitized_map.header.stamp = rospy.Time.now()
        self.sanitized_map.header.frame_id = data.header.frame_id
        # self.sanitized_map.info.resolution = data.info.resolution
        self.sanitized_map.info.resolution = 0.2

        self.n = self.sanitized_map.info.width
        self.m = self.sanitized_map.info.height
        self.grid = np.reshape(np.array(data.data), (self.n, self.m))

        self.area_to_sanitize.header.stamp = rospy.Time.now()
        self.area_to_sanitize.header.frame_id = data.header.frame_id
        self.area_to_sanitize.cell_width = 0.15
        self.area_to_sanitize.cell_height = 0.15

        self.sanitized_area.header.stamp = rospy.Time.now()
        self.sanitized_area.header.frame_id = data.header.frame_id
        self.sanitized_area.cell_width = 0.15
        self.sanitized_area.cell_height = 0.15

        self.walls_cells.header.stamp = rospy.Time.now()
        self.walls_cells.header.frame_id = data.header.frame_id
        self.walls_cells.cell_width = 0.15
        self.walls_cells.cell_height = 0.15

