import rospy
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetModelState

import numpy as np
from math import sqrt
import time

# i dont know if it is correct from ROS and python point of view (i need to keep this variable seen from anyone)
global GlobalRateTime  # its a FREQUENCY HZ
global Stanza


class occupancy_grid_class_copy():
    '''
    This class will be used to manage the data recived from different topics
    '''
    Zero = 0  # this is class attribute, it is the same for all the objects of this class, it is declared outside of "def _init_(self):" method

    def __init__(self):  # this is colled "method of the class" ?
        self.Uno = 1  # this is a istance attributre, it is different for all the objects of this class, it is declared inside of "def _init_(delf):" method

        self.n = 2  # lenth in x direction (zero axes) of self.grid
        self.m = 2  # lenth in y direction (one axes) of self.grid
        self.grid = np.zeros((self.n, self.m))  # it's a matrix that will be copied from "/map" topic and reshaped afterwards

        self.disc_n = 2  # discretized lenth in x direction (zero axes) of self.discretized_grid
        self.disc_m = 2  # discretized lenth in y direction (one axes) of self.discretized_grid
        self.discretized_grid = np.zeros((self.disc_n,
                                          self.disc_m))  # it's a discretized matrix of the envirmant that will be obtained from self.grid

        # the object below is used only for tests, it can be removed without compromising the functionality of the node
        self.SanitizedMap = OccupancyGrid(header=rospy.Header())  # OLD COMMENT# we will use this element as a OccupacyGrid for storing sanification data
        self.AreaToSanitize = GridCells(header=rospy.Header())  # this is an object that will be sent to rviz where we will see the area to be sanitized
        self.SanitizedArea = GridCells(header=rospy.Header())  # this is an object that will be sent to rviz where we will see the areas that have been sanitized
        self.WallsCells = GridCells(header=rospy.Header())  # this is an object that will be sent to rviz where we will see the cell that aproximate the walls
        self.UnexploredCells = GridCells(header=rospy.Header())  # this is an object that will be sent to rviz where we will see the cells that werent explored when the map was created

    def callbackFillSanitizazionMap(self, data):  # the callback take always an additional argument (in this case "data") that represent the object that you will manage
        '''
        This function will fill and reshape the matrices self.SanitizedMap.data, self.AreaToSanitize.cells, self.SanitizedArea.cells in a proper manner.
        '''
        # copy data from the "/map" argument in self.SanitizedMap object
        self.SanitizedMap.header.stamp = rospy.Time.now()  # in the header we need to have a time information about the message
        self.SanitizedMap.header.frame_id = data.header.frame_id  # they have the same frame ( in this case 'map')
        self.SanitizedMap.info.resolution = data.info.resolution  # for now i will keep the same resolution
        self.SanitizedMap.info.height = data.info.height  # same height
        self.SanitizedMap.info.width = data.info.width  # same width
        self.SanitizedMap.info.origin = data.info.origin  # same origin
        self.SanitizedMap.info.map_load_time = data.info.map_load_time  # i dont know for what is used for this data
        self.SanitizedMap.data = data.data  # coping the tuple containing the map data (it is one dimensional array/matrix)
        # one thing to be noticed, OccupacyGrid.data is a tuple so we need to transform it in somthing else if we want to change its values

        # filling self.SanitizedMap with correct data
        self.n = self.SanitizedMap.info.width
        self.m = self.SanitizedMap.info.height
        self.grid = np.reshape(np.array(data.data), (self.n, self.m))

        # display all unique different elements in an array
        # i reshape the matrix in to a long vector that is passed to "unique" function
        # it returns "-1" for unexplored reagion, "0" for free space and "100" for walls
        print("\nElements of self.grid = {0}".format(np.unique(np.reshape(self.grid, (self.n * self.m)))))

        # fill self.AreaToSanitize with required data
        self.AreaToSanitize.header.stamp = rospy.Time.now()
        self.AreaToSanitize.header.frame_id = data.header.frame_id  # 'map'# -> look documentation or frame_id of other topics
        self.AreaToSanitize.cell_width = 0.15  # I have decided to make the squares a little smaller than 0.2 [m], so we can see the map below
        self.AreaToSanitize.cell_height = 0.15  # same as abowe

        # fill self.SanitizedArea with required data
        self.SanitizedArea.header.stamp = rospy.Time.now()
        self.SanitizedArea.header.frame_id = data.header.frame_id  # 'map'# -> look documentation or frame_id of other topics
        self.SanitizedArea.cell_width = 0.15  # I have decided to make the squares a little smaller than 0.2 [m], so we can see the map below
        self.SanitizedArea.cell_height = 0.15  # same as abowe

        # fill self.WallsCells with required data
        self.WallsCells.header.stamp = rospy.Time.now()
        self.WallsCells.header.frame_id = data.header.frame_id  # 'map'# -> look documentation or frame_id of other topics
        self.WallsCells.cell_width = 0.15  # I have decided to make the squares a little smaller than 0.2 [m], so we can see the map below
        self.WallsCells.cell_height = 0.15  # same as abowe

        # fill self.UnexploredCells with required data
        self.UnexploredCells.header.stamp = rospy.Time.now()
        self.UnexploredCells.header.frame_id = data.header.frame_id  # 'map'# -> look documentation or frame_id of other topics
        self.UnexploredCells.cell_width = 0.15  # I have decided to make the squares a little smaller than 0.2 [m], so we can see the map below
        self.UnexploredCells.cell_height = 0.15  # same as abowe

        self.ReshapeMapLorenzo()  # with this function you assigne the correct values (discretized) to self.discretized_grid from self.grid

        # displays the unique elements of the self.discretized grid where "-1" represents the unexplored region
        # ,"0" represents the areas to be sanitized, "10" represents the walls and "1" represents the areas that have been sanitized or are free
        print("\nElements of self.discretized_grid = {0}".format(
            np.unique(np.reshape(self.discretized_grid, (np.shape(self.discretized_grid)[0] ** 2)))))

        # displays data about self.discretized_grid
        print("\nself.disc_n: {0} \tself.disc_m: {1}\tself.discretized_grid: {2}\n".format(self.disc_n, self.disc_m,
                                                                                           self.discretized_grid))

        # the following if conditions will fill self.discretized_grid[q][p] according to the chosen option
        if Stanza == "ALL":
            pass  # i wrote pass becouse it is alredy correct the self.discretized_grid[q][p] matrix

        if Stanza == "KITCHEN":  # modify the values of self.discretized_grid[q][p] in such a way that only the kitchen cells will need to be sanitized
            for q in range(0, self.disc_n):
                for p in range(0, self.disc_m):
                    if ((self.discretized_grid[q][p] == 0) and (not (50 <= q <= 90) or not (25 <= p <= 48))):
                        self.discretized_grid[q][p] = 1

        if Stanza == "OFFICE":  # modify the values of self.discretized_grid[q][p] in such a way that only the office cells will need to be sanitized
            for q in range(0, self.disc_n):
                for p in range(0, self.disc_m):
                    if ((self.discretized_grid[q][p] == 0) and (not (31 <= q <= 48) or not (24 <= p <= 73))):
                        self.discretized_grid[q][p] = 1

        if Stanza == "GARAGE":  # modify the values of self.discretized_grid[q][p] in such a way that only the garage cells will need to be sanitized
            for q in range(0, self.disc_n):
                for p in range(0, self.disc_m):
                    if ((self.discretized_grid[q][p] == 0) and (not (31 <= q <= 53) or not (13 <= p <= 23))):
                        self.discretized_grid[q][p] = 1

        if Stanza == "GARDEN":  # modify the values of self.discretized_grid[q][p] in such a way that only the garage cells will need to be sanitized
            for q in range(0, self.disc_n):
                for p in range(0, self.disc_m):
                    if ((self.discretized_grid[q][p] == 0) and (not (19 <= q <= 30) or not (43 <= p <= 66))):
                        self.discretized_grid[q][p] = 1

        # load a position in self.AreaToSanitize.cells if the value of self.discretized_grid in that position is == 0 (that cell has been sanitized)
        for q in range(0, self.disc_n):
            for p in range(0, self.disc_m):
                if self.discretized_grid[q][p] == 1:
                    self.SanitizedArea.cells.append(
                        Point(x=p * 0.2 + self.SanitizedMap.info.origin.position.y + 0.15 / 2,
                              y=q * 0.2 + self.SanitizedMap.info.origin.position.x + 0.15 / 2, z=0))

        # load discretized walls positions in self.WallsCells.cells (walls area rappresented with "10" in self.discretized_grid)
        for q in range(0, self.disc_n):
            for p in range(0, self.disc_m):
                if self.discretized_grid[q][p] == 10:
                    self.WallsCells.cells.append(Point(x=p * 0.2 + self.SanitizedMap.info.origin.position.y + 0.15 / 2,
                                                       y=q * 0.2 + self.SanitizedMap.info.origin.position.x + 0.15 / 2,
                                                       z=0))

        # load positions of unexplored areas in self.UnexploredCells.cells (unexplored areas rappresented with "-1" in self.discretized_grid)
        for q in range(0, self.disc_n):
            for p in range(0, self.disc_m):
                if self.discretized_grid[q][p] == -1:
                    print("\nim here")
                    self.UnexploredCells.cells.append(
                        Point(x=p * 0.2 + self.SanitizedMap.info.origin.position.y + 0.15 / 2,
                              y=q * 0.2 + self.SanitizedMap.info.origin.position.x + 0.15 / 2, z=0))

    #################################################################################################################################################
    # da qua in giu' c'e' la aprte di Alessandro
    #################################################################################################################################################

    def compute_angles(self, xx_pose, yy_pose, i,
                       j):  # <------------------------------------------------------------------------------------ NOTE what represent those variables exactly ?
        ''' This function was written by Alessandro and it return ??? given tha position of the robot(in which reference system) and
         the cell position in the discretized map
        '''
        verbose = False
        # Remember that the division gives you the value in radians and not in degrees
        ### Alpha_min and Alpha_max must be returned as integer values from 0 to 360

        pi = np.pi  ### TAKE BUILT IN CONSTANT

        cellDimensionInMeters = 0.2
        # from cell dimension to meter dimension
        fromRadtoDegree = 180 / pi

        xx_robot = np.round(xx_pose * cellDimensionInMeters + cellDimensionInMeters / 2,
                            decimals=2)  # posizione non discretizzata
        yy_robot = np.round(yy_pose * cellDimensionInMeters + cellDimensionInMeters / 2, decimals=2)
        xx_cell_left = np.round(xx_robot + (i - xx_pose) * cellDimensionInMeters - 0.1, decimals=2)
        xx_cell_right = np.round(xx_robot + (i - xx_pose) * cellDimensionInMeters + 0.1, decimals=2)
        yy_cell_top = np.round(yy_robot + (j - yy_pose) * cellDimensionInMeters - 0.1, decimals=2)
        yy_cell_bottom = np.round(yy_robot + (j - yy_pose) * cellDimensionInMeters + 0.1, decimals=2)

        if verbose:
            print("xx_robot " + str(xx_robot))
            print("yy_robot " + str(yy_robot))
            print("xx_cell_left " + str(xx_cell_left))
            print("xx_cell_right " + str(xx_cell_right))
            print("yy_cell_top " + str(yy_cell_top))
            print("yy_cell_bottom " + str(yy_cell_bottom))

        if (j - yy_pose) < 0 and (i - xx_pose) > 0:  # the cell is in 1st quadrant
            alpha_max = np.rint(np.arctan((yy_robot - yy_cell_top) / (xx_cell_left - xx_robot)) * fromRadtoDegree)
            alpha_min = np.rint(np.arctan((yy_robot - yy_cell_bottom) / (xx_cell_right - xx_robot)) * fromRadtoDegree)
        elif (j - yy_pose) < 0 and (i - xx_pose) < 0:  # the cell is in 2nd quadrant
            alpha_max = 180 - np.rint(
                np.arctan((yy_robot - yy_cell_bottom) / (xx_robot - xx_cell_left)) * fromRadtoDegree)
            alpha_min = 180 - np.rint(
                np.arctan((yy_robot - yy_cell_top) / (xx_robot - xx_cell_right)) * fromRadtoDegree)
        elif (j - yy_pose) > 0 and (i - xx_pose) < 0:  # the cell is in 3rd quadrant
            alpha_max = 180 + np.rint(
                np.arctan((yy_cell_bottom - yy_robot) / (xx_robot - xx_cell_right)) * fromRadtoDegree)
            alpha_min = 180 + np.rint(np.arctan((yy_cell_top - yy_robot) / (xx_robot - xx_cell_left)) * fromRadtoDegree)
        elif (j - yy_pose) > 0 and (i - xx_pose) > 0:  # the cell is in 4th quadrant
            alpha_max = 270 + np.rint(
                np.arctan((xx_cell_right - xx_robot) / (yy_cell_top - yy_robot)) * fromRadtoDegree)
            alpha_min = 270 + np.rint(
                np.arctan((xx_cell_left - xx_robot) / (yy_cell_bottom - yy_robot)) * fromRadtoDegree)
        elif (j - yy_pose) == 0 and (i - xx_pose) > 0:  # the cell is On the right
            alpha_max = 360 - np.rint(
                np.arctan((cellDimensionInMeters / 2) / (xx_cell_left - xx_robot)) * fromRadtoDegree)
            alpha_min = np.rint(np.arctan((cellDimensionInMeters / 2) / (xx_cell_left - xx_robot)) * fromRadtoDegree)
        elif (j - yy_pose) < 0 and (i - xx_pose) == 0:  # the cell is On the top
            alpha_max = 90 + np.rint(
                np.arctan((cellDimensionInMeters / 2) / (yy_robot - yy_cell_bottom)) * fromRadtoDegree)
            alpha_min = 90 - np.rint(
                np.arctan((cellDimensionInMeters / 2) / (yy_robot - yy_cell_bottom)) * fromRadtoDegree)
        elif (j - yy_pose) == 0 and (i - xx_pose) < 0:  # the cell is On the left
            alpha_max = 180 + np.rint(
                np.arctan((cellDimensionInMeters / 2) / (xx_robot - xx_cell_right)) * fromRadtoDegree)
            alpha_min = 180 - np.rint(
                np.arctan((cellDimensionInMeters / 2) / (xx_robot - xx_cell_right)) * fromRadtoDegree)
        elif (j - yy_pose) > 0 and (i - xx_pose) == 0:  # the cell is On the bottom
            alpha_max = 270 + np.rint(
                np.arctan((cellDimensionInMeters / 2) / (yy_cell_top - yy_robot)) * fromRadtoDegree)
            alpha_min = 270 - np.rint(
                np.arctan((cellDimensionInMeters / 2) / (yy_cell_top - yy_robot)) * fromRadtoDegree)
        else:  # giving the same positions the result is zero
            alpha_max = 0
            alpha_min = 0
        return int(alpha_min), int(alpha_max)

    def roomLimitCheck2(self, xx, yy, room):
        ''' This function is used to understand whether the single cell is contained in the room. Written by Alessandro '''
        if (xx) < room.shape[1] and (xx) >= 0 and (yy) < room.shape[0] and (yy) >= 0:
            return True
        else:
            return False

    def roomLimitCheck(self, xx_pose, yy_pose, radius, room):
        ''' This function is used to understand when the room is finished and the cycle could be stopped. Written by Alessandro '''
        if (xx_pose + radius) < room.shape[1] or (xx_pose - radius) > 0 or (yy_pose + radius) <= room.shape[0] or (
                yy_pose - radius) >= 0:
            return True
        else:
            return False

    #################################################################################################################################################
    # fino a qua c'e' la parte di Alessandro
    #################################################################################################################################################

    def PerformeSanitization(self):
        '''
        This function manages squares that need to be sanitized and which have been sanitized by moving them from
         self.AreaToSanitize.cells[iii] to self.SanitizedArea.cells[jjj] and updaiting self.discretized_grid[bbb][fff]
         with the percentage of total energy they must receive to be considered sanitized.
        '''
        modelState = rospy.ServiceProxy('/gazebo/get_model_state',
                                        GetModelState)  # use a service to take data from '/gazebo/get_model_state' and save them in an object of type GetModelState
        # RobotContinuousPosX and RobotContinuousPosY are coordinates in real world not discretized (position of the robot respect "map" frame)
        RobotContinuousPosX, RobotContinuousPosY = modelState('turtlebot3', 'ground_plane').pose.position.x, modelState(
            'turtlebot3', 'ground_plane').pose.position.y  # get needed data giving name and reference frame

        # xx_pose and yy_pose are discretized coordinates of the robot in the world (those coordinates will be used to identify cell positions in self.discretized_grid and room matrix)
        xx_pose = int((RobotContinuousPosX - 0.15 / 2 - self.SanitizedMap.info.origin.position.y) * 5)
        yy_pose = int((RobotContinuousPosY - 0.15 / 2 - self.SanitizedMap.info.origin.position.x) * 5)

        #################################################################################################################################################
        # da qua in giu' c'e' la parte di Alessandro
        #################################################################################################################################################

        # Initialization
        angles = np.zeros((360))
        angles_backup = np.zeros((360))
        angles[0] = 1  # this initialization allows to the while to work
        angles_backup[0] = 1  # this initialization allows to the while to work
        radius = 0
        # WARNING in order to avoid errors i copied self.discretized_grid (matrix containing the discretized map) into room matrix,
        # after the alghorithm filled all the cells with the proper amouth of energy the room matrix is copied in self.discretized_grid
        # changing the variables containing the lenth and with of the matrix.
        # self.disc_n ,self.disc_m rapresent the number of columns and rows in self.discretized_grid.
        room = np.copy(self.discretized_grid)
        cellDimensionInMeters = 0.2

        while np.unique(angles).size == 2 and self.roomLimitCheck(xx_pose, yy_pose, radius,
                                                                  room):  # Until the unique elements of the array angle is have a size of one
            radius += 1
            for i in range(-radius, radius + 1):  # the plus one is because the range function skip the last element
                for j in range(-radius, radius + 1):
                    # the functions determine whether the cell is inside the room and skip all the cells that was previously computed.
                    # The former control is useful when along one direction we hit the border
                    if sqrt(i * 2 + j * 2) >= radius and self.roomLimitCheck2(xx_pose + i, yy_pose + j, room):
                        alpha_min, alpha_max = self.compute_angles(xx_pose, yy_pose, xx_pose + i, yy_pose + j)

                        # you see an obstacle here
                        if room[
                            yy_pose + j, xx_pose + i] == 10:  # <--------------------------------------------------------------------------------------------- NOTE here you have to change the value of obstacle
                            # You cannot change the values of angles also for the cells that have your same radius
                            # for this reason it has been used a backup variable
                            # The plus one is because the operator misses the last element
                            angles_backup[alpha_min:alpha_max + 1] = 1
                        else:
                            # we are sure that the cell inside this range are zeros since we start from closer cells that can cover smaller
                            # of the circle arc. it is sufficient to check the borders
                            # the plus one and minus one is because if there is a match of the angles, the cell is still completely uncovered
                            # All those exceptions are for
                            if angles[alpha_min + 1] == 0 and angles[alpha_max - 1] == 0 or angles[
                                alpha_min - 1] == 0 and angles[alpha_max + 1] == 0:
                                room[yy_pose + j, xx_pose + i] += 0.5 / ((i * cellDimensionInMeters) * 2 + (
                                            j * cellDimensionInMeters) * 2)  # <--------------------------------------------------------------------------------------------- NOTE here you have to change the value of Uncovered_area
                                if (room[
                                    yy_pose + j, xx_pose + i] >= 1):  # check if the energy level is higher than 1 (more than 100%) after dt has occured
                                    room[
                                        yy_pose + j, xx_pose + i] = 1  # sets the value to 1 (100%) in this way it will be saved inside self.SanitizedArea.cells array

            angles = np.copy(angles_backup)

        self.discretized_grid = np.copy(room)

        #################################################################################################################################################
        # fino a qua c'e' la parte di Alessandro
        #################################################################################################################################################

        self.disc_n = self.discretized_grid.shape[0]
        self.disc_m = self.discretized_grid.shape[1]

        self.AreaToSanitize.cells = []
        self.SanitizedArea.cells = []
        print("\nlen1 : {0}\tlen2 : {1}".format(len(self.AreaToSanitize.cells), len(self.SanitizedArea.cells)))

        for q in range(0, self.disc_n):
            for p in range(0, self.disc_m):
                if ((self.discretized_grid[q][p] >= 0) and (
                        self.discretized_grid[q][p] < 1)):  # select the area to sanityze
                    self.AreaToSanitize.cells.append(
                        Point(x=p * 0.2 + self.SanitizedMap.info.origin.position.y + 0.15 / 2,
                              y=q * 0.2 + self.SanitizedMap.info.origin.position.x + 0.15 / 2, z=0))

        for q in range(0, self.disc_n):
            for p in range(0, self.disc_m):
                if ((self.discretized_grid[q][p] == 1)):  # sanitized areas
                    self.SanitizedArea.cells.append(
                        Point(x=p * 0.2 + self.SanitizedMap.info.origin.position.y + 0.15 / 2,
                              y=q * 0.2 + self.SanitizedMap.info.origin.position.x + 0.15 / 2, z=0))

        print("\nlen11 : {0}\tlen22 : {1}".format(len(self.AreaToSanitize.cells), len(self.SanitizedArea.cells)))

    def ReshapeMapLorenzo(self):
        '''With this function you assigne the correct values (discretized) to self.discretized_grid from self.grid,
        this part of code was written by Roman Sudin coping from Lorenzo Frangiamone code prototype'''
        # YOU HAVE TO MODIFY THE VALUES OF THE IF FUNCTION BECOUSE THE VALUES FROM LORENZO CODE DONT METCH WITH THE VALUES OF THIS MAP
        # YOU HAVE -1 IN YOUR MATRIX WHILE IN LORENZOS CODE THIS VALUE IS NOT ALLOWED BECOUSE YOU MAKE A SUM
        map = self.grid  # temporary copy of the self.grid map

        # i dont know what is happening below
        while 1:
            if (np.sum(map[0, :]) == 0):
                map = np.delete(map, (0), axis=0)
            else:
                break
        while 1:
            if np.sum(map[-1, :]) == 0:
                map = np.delete(map, (-1), axis=0)
            else:
                break
        while 1:
            if np.sum(map[:, 0]) == 0:
                map = np.delete(map, (0), axis=1)
            else:
                break
        while 1:
            if np.sum(map[:, -1]) == 0:
                map = np.delete(map, (-1), axis=1)
            else:
                break

        discret_unit = 4  # this value tells how many times the new matrix will be smaller (maybe it should make it a global variable) <----------------------------------------------------------------- NOTE
        sizex_cut = np.shape(map)[0]  # lenth in x direction (first axis) of map matrix
        sizey_cut = np.shape(map)[1]  # lenth in y direction (second axis) of map matrix
        discret_sizex = int(sizex_cut / discret_unit)  # discretized lenth in x direction (first axis) of the new matrix
        discret_sizey = int(
            sizey_cut / discret_unit)  # discretized lenth in y direction (second axis) of the new matrix
        map_02 = np.zeros([discret_sizey, discret_sizex])  # declaration of the new discretized matrix

        ## fill the discretized map with: 10 if the selected quadrant contains at least a pixel of wall,
        ## 1 if it doesn't
        for i in range(discret_sizey):
            for j in range(discret_sizex):
                map_02[i, j] = np.ceil(np.sum(
                    map[discret_unit * i:discret_unit * i + discret_unit, discret_unit * j:discret_unit * j + discret_unit]) / (
                                                   discret_unit ** 2))
                if map_02[i, j] > 1:
                    map_02[i, j] = 10
                elif map_02[i, j] == 1:
                    map_02[i, j] = 1

                self.disc_n = np.shape(map_02)[0]  # assigne the lenth of discretized map to the class method
                self.disc_m = np.shape(map_02)[1]
                self.discretized_grid = map_02


    def DisplaySanitizedMapData(self):
        '''This function is used to display the data of self.SanitizeMap without displaying the matrix at the end'''
        # it don't print the data in a nice way
        print("\nHeader\n\tstamp : {0}\n\tframe_id : {1}".format(self.SanitizedMap.header.stamp,
                                                                 self.SanitizedMap.header.frame_id))
        print("\nInfo\n\tresolution : {0}\n\theight: {1}\n\twidth : {2}\n\torigin : {3}\n\tmap_load_time {4}".format(
            self.SanitizedMap.info.resolution, \
            self.SanitizedMap.info.height, \
            self.SanitizedMap.info.width, \
            self.SanitizedMap.info.origin, \
            self.SanitizedMap.info.map_load_time))


if __name__ == '__main__':  # this line of code is used to execute the code below if it is run from terminal, if this python file is imported from another python code the part below will not be executed
    try:
        ''' Here are listed the steps i will follow for the implementation of the node
            1) get and reshape map from /map topic. 
            2) get position of turtlebot.
            3) fill the boxes in the sanitization map
        '''

        # time.sleep(5)# this line is here because I need delay in startup file, if node is started at the same time as other nodes it will crash because '/map' data is not available yet <-------------------------------- NOTE

        GlobalRateTime = 2  # declared as global variable at the start of the code it represent the frequency at which the topics are published (dt)
        Stanza = "KITCHEN"
        rospy.init_node("TestNode")  # creating the node
        grid = occupancy_grid_class_copy()  # creating an object (it will beused for data manipulation)
        grid.pub = rospy.Publisher("sanitizazion_map_publisher", OccupancyGrid,
                                   queue_size=10)  # publisher that will publish a OccupancyGrid object, it's used to perform different tests (you can delete this publisherbecouse it is not needed for the correct functionality of the node)
        grid.pub2 = rospy.Publisher("points_to_sanitize", GridCells,
                                    queue_size=10)  # is used to view all the points that need to be sanitized in rviz
        grid.pub3 = rospy.Publisher("sanitized_points", GridCells,
                                    queue_size=10)  # is used to view all the points that have been sanitized
        grid.pub4 = rospy.Publisher("walls", GridCells,
                                    queue_size=10)  # is used to view all the points that aproximate the wall position with the discretization
        grid.pub5 = rospy.Publisher("unexplored_cells", GridCells,
                                    queue_size=10)  # is used to view all the points that werent explored when the map was created
        rate = rospy.Rate(GlobalRateTime)  # how many times the message will be published in one second
        rospy.Subscriber("map", OccupancyGrid,
                         grid.callbackFillSanitizazionMap)  # read the environment map from topic "/map" and create a copy of it (also creates a discretized version that will be used later)
        while not rospy.is_shutdown():  # continue to publish until a shutdown interrupt don't arrives ( CTRL + C )

            grid.PerformeSanitization()  # fill the cels of grid.discretized_grid with the correct amouth of energy that they have recived and update grid.AreaToSanitize, grid.SanitizedArea

            grid.pub.publish(grid.SanitizedMap) #
            grid.pub2.publish(grid.AreaToSanitize)
            grid.pub3.publish(grid.SanitizedArea)
            grid.pub4.publish(grid.WallsCells)
            grid.pub4.publish(grid.UnexploredCells)

            '''something = rospy.ServiceProxy('/global_localization', Empty)# Is it possible to send a command that clears the estimated position of the turtlebot with the monte carlo algorithm?'''

            print("\n\t\t\t A dt of time has passed\n")

            rate.sleep()  # sleep for a certain amout of time
    except rospy.ROSInterruptException:
        print("\n\nAn error has occurred in main function\n\n")  # when some error occures
