import rospy as rp
from grid_map import GridMap
from geometry_msgs.msg import Point, Twist, PoseStamped
import numpy as np
import math

np.random.seed(1024)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        # Length of edge of the tree
        self.step = 1.
        # Radius of neigbhourhood to search for beeter point
        self.neighbourhood = 2.0

    # Restore fastest past from Goal to start
    def restorePath(self):
        startReached = False
        path = []
        parent = self.start
        prev_dist = 1000
        for child in self.parent:
            if self.rrt_route(0,child) > 8. and child[0]<4.3:
                if self.edge_distance([3.5,0.5],child) < prev_dist:
                    parent = child
                    prev_dist = self.edge_distance([3.5,0.5],child)
                else:
                    pass
            else:
                pass
        path.append(self.end)
        while (not startReached):
            row = parent[0]
            col = parent[1]
            path.append(parent)
            if (row == self.start[0] and col == self.start[1]):
                startReached = True
            parent = self.parent[parent[0], parent[1]]
        return path

    # Deacreses neighbourhood radius over time
    def neighbourhood_star(self,i):
        if self.neighbourhood > self.step+0.2:
            self.neighbourhood = 2.0 - 0.001*i

    # Calculate length of route from point to root
    def rrt_route(self,newp, closest):
        length = 0.0
        while(closest is not None):
            parent = self.parent[closest[0],closest[1]]
            if parent is None:
                break
            length = self.edge_distance(parent, closest) + length
            closest = [parent[0],parent[1]]
        return length

    # Calculate distance from point to parent
    def edge_distance(self,point,parent):
        v = np.subtract(point,parent)
        m = np.linalg.norm(v)
        return m

    # Calculate angle between new edge end parent edge for car kinematics
    def car_kinematics(self, pt,closest):
        try:
            parent = self.parent[closest[0],closest[1]]
            angle_parents = math.atan((closest[1] - parent[1]) / (closest[0] - parent[0]))
            angle_child = math.atan((pt[1] - closest[1]) / (pt[0] - closest[0]))
            angle = angle_parents - angle_child
            if (angle < 0):
                angle = angle_parents + angle_child
            if (angle < 0):
                angle = -angle_parents - angle_child
            if (angle < 0):
                angle = -angle_parents + angle_child
        except:
            angle = 0
        return angle

    # Check if new point is in free space over cost map
    def check_if_valid(self, pt, closest,i):
        angle = math.atan((pt[1] - closest[1]) / (pt[0] - closest[0]))
        distance = math.sqrt(pow((pt[0] - closest[0]), 2) + pow((pt[1] - closest[1]), 2))
        point = np.zeros(2, np.int8)
        in_free_space = True
        for i in range(0, 100):
            check_step = i / 100.0 * distance
            x = int(10 * (pt[0] + check_step * math.cos(angle)))
            y = int(10 * (pt[1] + check_step * math.cos(angle)))
            try:

                if self.map[y, x] > 50:
                    in_free_space = False
                    break
                else:
                    in_free_space = True
            except:
                in_free_space = False
            if self.rrt_route(pt,closest) <15.0 and i >2000:
                in_free_space = False
        return in_free_space

    # Check if new point is allowed - prevent from turning quickly on first corner
    def check_if_allowed(self,point, closest):
        if closest[0] == self.start[0] and closest[1] == self.start[1]:
            return 1
        else:
            route = self.rrt_route(0,closest) + self.edge_distance(point,closest)
            if (point[1] > 4.0 or point[1] < 2.2) and route < self.step*5:

                return 0
            else:
                return 1

    # Draw a random point
    def random_point(self,i):
        x = self.width * np.random.random()
        y = self.height * np.random.random()

        return np.array([x, y])

    # Find closest point
    def find_closest(self, pos):
        closest = pos
        prev = -1
        for points in self.parent:
            dist = pow(pos[0]-points[0],2)+pow(pos[1]-points[1],2)
            if(prev < 0):
                closest = points
                prev = dist

            else:
                if(dist < prev):
                    closest = points
                    prev = dist
                else:
                    pass
        return np.array([closest[0], closest[1]])

    # Verify if any other other point over neighbourhood has faster connection to root
    def check_neighbours(self,point,closest):
        prev = self.rrt_route(0,closest) + self.edge_distance(point,closest)
        better_neigh = closest
        for neigh in self.parent:
            distance = self.edge_distance(point,neigh)
            if distance < self.neighbourhood:
                route = self.rrt_route(0,neigh) + distance
                if route < prev:
                    prev = route
                    better_neigh = neigh
                else:
                    pass
        return better_neigh

    # Calculates a quarter for edge angle detection
    def vector_quarter(self,newp, closest):
        vec = np.subtract(newp,closest)
        quarter = -1
        if vec[0]>=0 and vec[1] >0:
            quarter = 1
        if vec[0]<0 and vec[1] >=0:
            quarter = 2
        if vec[0]<=0 and vec[1] <0:
            quarter = 3
        if vec[0]>0 and vec[1] <=0:
            quarter = 4
        return quarter

    # Calculates edge angle
    def vector_angle(self, newp, closest):
        v1 = np.subtract(newp,closest)
        v2 = np.subtract(closest,self.start)
        u1 = v1 / np.linalg.norm(v1)
        u2 = v2 / np.linalg.norm(v2)
        dot_product = np.dot(u1,u2)
        return np.arccos(dot_product)

    # Return proper angle for car kinematics
    def kin_vec_angle(self,rad1, quarter1, rad2, quarter2):
        if(quarter1 == 1 and quarter2 == 1):
            return abs(rad1-rad2)
        elif (quarter1 == 1 and quarter2 == 2):
            return abs(rad1-rad2)
        elif (quarter1 == 1 and quarter2 == 4):
            return abs(rad1-rad2)
        elif (quarter1 == 2 and quarter2 == 1):
            return abs(rad1-rad2)
        elif (quarter1 == 2 and quarter2 == 2):
            return abs(rad1-rad2)
        elif (quarter1 == 2 and quarter2 == 3):
            return abs(np.radians(360)-rad1-rad2)
        elif (quarter1 == 3 and quarter2 == 2):
            return abs(np.radians(360)-rad1-rad2)
        elif (quarter1 == 3 and quarter2 == 3):
            return abs(rad1-rad2)
        elif (quarter1 == 3 and quarter2 == 4):
            return abs(rad1-rad2)
        elif (quarter1 == 4 and quarter2 == 1):
            return abs(rad1-rad2)
        elif (quarter1 == 4 and quarter2 == 3):
            return abs(rad1-rad2)
        elif (quarter1 == 4 and quarter2 == 4):
            return abs(rad1-rad2)
        else:
            return np.radians(360)

    # Return new point with define edge distance - self.step
    def new_pt(self, pt, closest):
        newp = np.subtract(pt,closest)
        newp = newp / np.linalg.norm(newp) * self.step
        new_pt = np.add(closest,newp)
        return new_pt

    # Main search
    def search(self):
        self.parent[self.start] = None
        i =0
        path= []
        path.append(self.start)
        path.append(self.end)
        self.publish_path(path)
        while not rp.is_shutdown():
            i = i+1
            print "iter: ", i
            rp.sleep(0.01)
            self.publish_search()
            new_point = self.random_point(i)
            closest = self.find_closest(new_point)
            new_point = self.new_pt(new_point, closest)
            closest = self.check_neighbours(new_point,closest)
            if self.check_if_valid(new_point, closest,i) and self.check_if_allowed(new_point,closest):
                #if self.car_kinematics(new_point,closest) < np.radians(15):
                self.parent[new_point[0], new_point[1]] = closest
                self.publish_search()
                self.neighbourhood_star(i)
            if i >200:
                path = self.restorePath()
                self.publish_path(path)
            if len(self.parent) > 1000 :
                break


if __name__ == '__main__':
    rrt = RRT()
    rrt.search()
