import numpy as np
from scipy.optimize import minimize
from .graph_search import graph_search
from .occupancy_map import OccupancyMap

 # source: https://karthaus.nl/rdp/, adapted into python
def rdp(points, eps):
    def perpendicularDistance(p3, p1, p2):
        return np.linalg.norm(np.cross(p2-p1, p3-p1))/np.linalg.norm(p2-p1)
    dmax = 0
    index = -1
    end = len(points) - 1
    for i in range(1, end):
        d = perpendicularDistance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d
    ResultList = []
    if dmax > eps:
        recResults1 = rdp(points[:index+1], eps)
        recResults2 = rdp(points[index:], eps)

        ResultList = recResults1[:-1] + recResults2
    else:
        ResultList = [points[0], points[end]]
    return ResultList

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!

        # self.resolution = np.array([0.25, 0.25, 0.25])
        # self.margin = 0.30

        # maze 1
        self.resolution = np.array([0.1, 0.1, 0.1])
        self.margin = 0.40
        self.mid_eps = 2.5
        self.rdp_eps = 0.28
        self.v = 1.0

        # maze 2
        # self.resolution = np.array([0.22, 0.22, 0.22])
        # self.margin = 0.2
        # self.mid_eps = 1.0
        # self.rdp_eps = 0.1
        # self.v = 1.0

        # maze 3
        # self.resolution = np.array([0.1, 0.1, 0.1])
        # self.margin = 0.3
        # self.mid_eps = 1.0
        # self.rdp_eps = 0.2
        # self.v = 1.0
        

        self.start = np.array(start)
        self.goal = np.array(goal)




        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        occ_map = OccupancyMap(world, self.resolution, self.margin)

        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1,3)) # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        
        # self.path_to_points()
        self.points = np.array(rdp(self.path, eps=self.rdp_eps))
        # print(self.points)
        self.fill_midpoints(eps=self.mid_eps)
        # print(self.points)

        self.set_times()
        self.make_bs()
        # print(self.bs)
        self.make_A()
        # print(self.A)
        self.make_H()
        # print(self.H)

        self.x_constraints = [
            {'type': 'eq', 'fun': self.linear_constraint_x},  
        ]
        self.y_constraints = [
            {'type': 'eq', 'fun': self.linear_constraint_y},  
        ]
        self.z_constraints = [
            {'type': 'eq', 'fun': self.linear_constraint_z},  
        ]

        x0 = np.ones((6*len(self.ts)))
        self.x_result = minimize(self.cost_func, x0, constraints=self.x_constraints, method="SLSQP")
        self.x_coeff = self.x_result.x
        print("x optimization sucess: ", self.x_result.success)
        self.y_result = minimize(self.cost_func, x0, constraints=self.y_constraints, method="SLSQP")
        self.y_coeff = self.y_result.x
        print("y optimization sucess: ", self.y_result.success)
        self.z_result = minimize(self.cost_func, x0, constraints=self.z_constraints, method="SLSQP")
        self.z_coeff = self.z_result.x
        print("z optimization sucess: ", self.z_result.success)


    def path_to_points(self):
        sg_diff_eps = np.linalg.norm(self.start-self.goal)/6

        if self.path.any():
            self.points = [self.path[0]]
            last_disp = self.path[1]-self.path[0]
            last_pt = self.path[1]
            for i in range(2,self.path.shape[0]):
                cur_pt = self.path[i]
                cur_disp = cur_pt-last_pt
                if not np.allclose(cur_disp,last_disp):
                    self.points.append(last_pt)
                elif np.linalg.norm(cur_pt-self.points[-1])>sg_diff_eps:
                    self.points.append(last_pt)
                last_pt = cur_pt
                last_disp = cur_disp
            self.points.append(self.path[-1])  
    
    def fill_midpoints(self, eps):
        mid_idx = []
        mid_pts = []
        for i in range(1,self.points.shape[0]):
            disp = self.points[i] - self.points[i-1]
            if np.linalg.norm(disp) > 2*eps:
                mid_idx.append(i)
                mid_idx.append(i)
                interp = np.linspace(self.points[i-1], self.points[i], 4)
                mid_pts.append(interp[1])
                mid_pts.append(interp[2])
            elif np.linalg.norm(disp) > eps:
                mid_idx.append(i)
                mid_pts.append((self.points[i] + self.points[i-1])/2)
        if mid_idx:
            self.points = np.insert(self.points, mid_idx, mid_pts, axis=0)

    def set_times(self):
        self.true_ts = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.true_ts[0] *= 1.4
        self.true_ts[-1] *= 1.4
        self.true_ts /= self.v
        # self.ts /= self.ts.sum()
        self.start_true_ts = np.insert(np.cumsum(self.true_ts), 0, 0)

        
        # self.ts = np.ones(len(self.points)-1)*self.v
        self.ts = np.ones(len(self.points)-1)
        self.start_t = np.insert(np.cumsum(self.ts), 0, 0)


    def make_bs(self):
        k = len(self.ts)
        self.bs = np.zeros((6+4*(k-1),3))
        
        # Setting p1(0) and pk(tk)
        self.bs[0] = self.points[0]
        self.bs[3] = self.points[-1]

        # Setting Interm Points
        for i in range(k-1):
            j = 2*i + 6
            self.bs[j] = self.points[i+1]
            self.bs[j+1] = self.points[i+1]

    def make_A(self):
        k = len(self.ts)
        self.A = np.zeros(((6+4*(k-1),6*k)))
        self.A[:3,:6] = np.array([
            [0,0,0,0,0,1],
            [0,0,0,0,1,0],
            [0,0,0,2,0,0]
        ])
        tk = self.ts[-1]
        self.A[3:6,-6:] = np.array([
            [tk**5,    tk**4,       tk**3,     tk**2,      tk**1,  1],
            [5*tk**4,  4*tk**3,     3*tk**2,   2*tk**1,    1,      0],
            [20*tk**3, 12*tk**2,    6*tk**1,   2,          0,      0]
        ])
        
        for i in range(k-1):
            ti = self.ts[i]
            r = 2*i + 6
            c1 = 6*i
            c2 = 6*(i+1)+5
            self.A[r,c1:c1+6] = np.array([ti**5, ti**4, ti**3, ti**2, ti**1, 1])
            self.A[r+1,c2] = 1
        
        for i in range(k-1):
            ti = self.ts[i]
            r = 2*i + 6 + 2*(k-1)
            c = 6*i
            self.A[r:r+2,c:c+12] = np.array([
                [5*ti**4,   4*ti**3,   3*ti**2,  2*ti**1,  1,  0,  0,  0,  0,  0, -1,  0],
                [20*ti**3,  12*ti**2,  6*ti**1,  2,        0,  0,  0,  0,  0, -2,  0,  0],
            ])

    def make_H(self):
        k = len(self.ts)
        self.H = np.zeros((6*k,6*k))
        for i in range(k):
            ti = self.ts[i]
            self.H[6*i:6*i+3,6*i:6*i+3] = np.array([
                [720*ti**5, 360*ti**4, 120*ti**3],
                [360*ti**4, 192*ti**3,  72*ti**2],
                [120*ti**3,  72*ti**2,  36*ti],
            ])

    def cost_func(self, x):
        return x.T @ self.H @ x

    def linear_constraint_x(self, x):
        return self.A @ x - self.bs[:,0]

    def linear_constraint_y(self, x):
        return self.A @ x - self.bs[:,1]

    def linear_constraint_z(self, x):
        return self.A @ x - self.bs[:,2]
    

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE

        if t>np.cumsum(self.true_ts)[-1]: 
            x = self.points[-1]
            flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
            return flat_output
        
        i = 0
        for j in range(len(self.true_ts)): 
            if t<np.cumsum(self.true_ts)[j]: 
                i = j 
                break
        
        t_now = (t-self.start_true_ts[i])/(self.start_true_ts[i+1]-self.start_true_ts[i])

        time_arr = np.array([t_now**5, t_now**4, t_now**3, t_now**2, t_now, 1])
        x = np.array([
            np.dot(self.x_coeff[6*i:6*i+6],time_arr),
            np.dot(self.y_coeff[6*i:6*i+6],time_arr),
            np.dot(self.z_coeff[6*i:6*i+6],time_arr)
        ])

        time_arr = np.array([5*t_now**4, 4*t_now**3, 3*t_now**2, 2*t_now, 1, 0])
        x_dot = np.array([
            np.dot(self.x_coeff[6*i:6*i+6],time_arr),
            np.dot(self.y_coeff[6*i:6*i+6],time_arr),
            np.dot(self.z_coeff[6*i:6*i+6],time_arr)
        ])

        time_arr = np.array([20*t_now**3, 12*t_now**2, 6*t_now, 2, 0, 0])
        x_ddot = np.array([
            np.dot(self.x_coeff[6*i:6*i+6],time_arr),
            np.dot(self.y_coeff[6*i:6*i+6],time_arr),
            np.dot(self.z_coeff[6*i:6*i+6],time_arr)
        ])

        time_arr = np.array([60*t_now**2, 24*t_now, 6, 0, 0, 0])
        x_dddot = np.array([
            np.dot(self.x_coeff[6*i:6*i+6],time_arr),
            np.dot(self.y_coeff[6*i:6*i+6],time_arr),
            np.dot(self.z_coeff[6*i:6*i+6],time_arr)
        ])

        time_arr = np.array([120*t_now, 24, 0, 0, 0, 0])
        x_ddddot = np.array([
            np.dot(self.x_coeff[6*i:6*i+6],time_arr),
            np.dot(self.y_coeff[6*i:6*i+6],time_arr),
            np.dot(self.z_coeff[6*i:6*i+6],time_arr)
        ])
        
        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        

        return flat_output
