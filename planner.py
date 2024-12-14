from mapUtilities import *
from a_star import *
from probabilistic_road_map import *
import time

POINT_PLANNER=0; TRAJECTORY_PLANNER=1; ASTAR_PLANNER=2; PRM_PLANNER=3

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        #### If using the map, you can leverage on the code below originally implemented for A*
        self.m_utilities=mapManipulator(laser_sig=0.4)    
        self.costMap=self.m_utilities.make_likelihood_field()

        # List of obstacles to plot (in x and y coordiantes)
        self.obstaclesList = np.array(self.m_utilities.getAllObstacles())     

        # List of obstacles for PRM, in cell indices
        self.obstaclesListCell = np.array(self.m_utilities.getAllObstaclesCell())  

        
    def trajectory_planner(self, startPoseCart, endPoseCart, type):

        startPose=self.m_utilities.position_2_cell(startPoseCart)
        endPose=self.m_utilities.position_2_cell(endPoseCart)

        # [Part 3] TODO Use the PRM and search_PRM to generate the path
        # Hint: see the example of the ASTAR case below, there is no scaling factor for PRM
        if type == PRM_PLANNER:
            startPose = [int(i) for i in startPose]
            endPose   = [int(j) for j in endPose]
            start_time = time.time()
            sample_points_tuple, roadmap = prm_graph(startPose, endPose, self.obstaclesListCell, robot_radius=30, m_utilities=self.m_utilities)
            # print("sample_points")
            # print(sample_points_tuple)
            # print("roadmap")
            # print(roadmap)
            path = search_PRM(sample_points_tuple, roadmap, startPose, endPose)

            end_time = time.time()


            print(f"the time took for PRM a_star calculation was {end_time - start_time}")
            # print(path)
            path_ = [[x, y] for x,y in path ]

        elif type == ASTAR_PLANNER: # This is the same planner you should have implemented for Lab4
            scale_factor = 4 # Depending on resolution, this can be smaller or larger
            startPose = [int(i/scale_factor) for i in startPose]
            endPose   = [int(j/scale_factor) for j in endPose]
            start_time = time.time()

            path = search(self.costMap, startPose, endPose, scale_factor)

            end_time = time.time()


            print(f"the time took for a_star calculation was {end_time - start_time}")

            path_ = [[x*scale_factor, y*scale_factor] for x,y in path ]

        Path = np.array(list(map(self.m_utilities.cell_2_position, path_ )))

        # Plot the generated path
        plt.plot(self.obstaclesList[:,0], self.obstaclesList[:,1], '.')
        plt.plot(Path[:,0], Path[:,1], '-*')
        plt.plot(startPoseCart[0],startPoseCart[1],'*')
        plt.plot(endPoseCart[0],
                 endPoseCart[1], '*')

        plt.show()
        
        return Path.tolist()
    

if __name__=="__main__":

    m_utilities=mapManipulator()
    
    map_likelihood=m_utilities.make_likelihood_field()
    
    # You can test your code here...
