import pygame 
import random
from webcolors import name_to_rgb
import math
import copy
import time
from scipy.interpolate import BSpline, make_interp_spline,CubicSpline
import numpy as np
from occupancy2coords import *
coordinates = convert_ocgrid2coord()


def main():
    pygame.init()

    # Units are in metres and radians
    obs_radius = 0.1
    robot_radius = 0.10

    robot_width = 2* robot_radius
    safedistance = robot_radius * 0.5

    max_vel = 0.5
    max_accel = 0.5

    obs_vel_range = 0.15   # obstacle velocity range, if moving

    display_bound = (-5.0,-5.0,5.0,5.0)



    # Params for display
    render_scale = 4

    width = 1500
    height = 1500

    size = (width,height)

    # Screen center at (x,y) = (0,0)
    # u0 = width/2
    # v0 = height/2
    u0 = 0
    v0 = height


    #pixels per metre for graphics
    k = 19
 

    # red_tuple = name_to_rgb('red')
    # print(tuple(red_tuple))

    # Initialise Pygame display screen
    screen = pygame.display.set_mode(size)
    # This makes the normal mouse pointer invisible in graphics window
    pygame.mouse.set_visible(0)


    #Initialize states: x,y,theta
    x,y,theta = 50.0,40,0.0
    location_history= []  #history of x and y

    # Initial velocity of left and right wheel
    vL = 0.0
    vR = 0.0

    #Params
    dt = 0.1  
    steps_ahead_to_plan = 15
    tau = dt * steps_ahead_to_plan

    #Obstacles
    obs = []  # contain x,y and visibility mask(2 values)
    # num_obstalces = 20
    
    # #Initialize obstacles
    # for i in range(num_obstalces):
    #     (ox,oy, vx,vy) = (random.uniform(display_bound[0],display_bound[2]),random.uniform(display_bound[1],display_bound[3]),random.gauss(0.0, obs_vel_range), random.gauss(0.0, obs_vel_range))
    #     ob = [ox,oy,vx,vy]
    #     obs.append(ob)


    num_obstacles = len(coordinates)

    for coord in coordinates:
        (ox,oy,vx,vy) = (coord[0],coord[1],random.gauss(0.0, obs_vel_range),random.gauss(0.0, obs_vel_range))
        ob = [ox,oy,vx,vy]
        obs.append(ob)

    # Goal between (-5,-5) and (5,5) 
    goal = (66,71)
    # goal = (random.randint(-5,5), random.randint(-5,5))

    def move_obs(obs,display_bound,dt):
        for i in range(len(obs)):
            obs[i][0] += obs[i][2] * dt
            if (obs[i][0] < display_bound[0]):
                obs[i][2] = -obs[i][2]
            if (obs[i][0] > display_bound[2]):
                 obs[i][2] = -obs[i][2]

            obs[i][1] += obs[i][3] * dt
            if (obs[i][1] < display_bound[1]):
                obs[i][3] = -obs[i][3]
            if (obs[i][1] > display_bound[3]):
                 obs[i][3] = -obs[i][3]


    def predict_next(vL,vR, x,y, theta, dt):

        if (round(vL,3) == round(vR,3)):
            # if moving in a straight line
            x_new = x + vL * math.cos(theta)*dt
            y_new = y * vL * math.sin(theta)*dt
            theta_new = theta 
            motion = (0,vL*dt) # 0 indicates pure translation, as it is

        elif (round(vL,3) == - round(vR,3)):
            # if pure rotation
            x_new = x 
            y_new = y
            theta_new = theta + ((vR-vL)*dt/robot_width)  # width to convert to radians
            motion = (1,0)  # 1 indicates pure rotation

        else: 
            R = robot_width/2.0 * ((vR+vL)/(vR-vL))
            d_theta = ((vR-vL)/robot_width) * dt
            x_new = x + R * (math.sin(d_theta + theta) - math.sin(theta))
            y_new = y - R * (math.cos(d_theta + theta) - math.cos(theta))
            theta_new = theta + d_theta

            # To calculate parameters for arc drawing - pygame params
            # centre of circle - cx,cy
            (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))

            # Turn this into Rect
            Rabs = abs(R)
            ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
            if (R > 0):
                    start_angle = theta - math.pi/2.0
            else:
                    start_angle = theta + math.pi/2.0
            stop_angle = start_angle + d_theta
            motion = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion

        return (x_new, y_new, theta_new, motion)
    

    def dist_to_closest_obstacle(x,y,obs):
        dist2closest_obs = 10000000.0
        for i,ob in enumerate(obs):
             if ob != goal:
                  dx = ob[0] - x
                  dy = ob[1] - y
                  d = math.sqrt(dx**2+dy**2)
                  dist = d - (obs_radius+robot_radius)
                  if dist < dist2closest_obs:
                       dist2closest_obs = dist
        return dist2closest_obs
    
    def fit_bspline(path, k=3, num_points=100):
        t = np.linspace(0, 1, len(path))
        t_new = np.linspace(0, 1, num_points)
        x, y ,_ = zip(*path)
        x_spline = make_interp_spline(t, x, k=k)
        y_spline = make_interp_spline(t, y, k=k)
        x_new = x_spline(t_new)
        y_new = y_spline(t_new)
        return list(zip(x_new, y_new))
    
    # def generate_path(x_start, y_start, x_end, y_end, theta_end, num_points=100):
    #     path = []

    #     dx = x_end - x_start
    #     dy = y_end - y_start
    #     dtheta = theta_end

    #     for i in range(num_points):
    #         t = i / (num_points - 1)
    #         x = x_start + t * dx
    #         y = y_start + t * dy
    #         theta = t * dtheta
    #         path.append((x, y, theta))

    #     return path
    
    
    def generate_path(x_start, y_start, x_end, y_end, theta_end, num_points=100):
        path = []

        # Define control points
        control_points = np.array([
            [x_start, y_start],
            [(x_start + x_end) / 2, (y_start + y_end) / 2],
            [x_end, y_end]
        ])

        # Calculate the knot vector for the B-spline
        t_control = np.linspace(0, 1, control_points.shape[0])
        t_spline = np.linspace(0, 1, num_points)

        # Create the B-spline representation of the curve for x and y coordinates separately
        spline_x = CubicSpline(t_control, control_points[:, 0], bc_type='natural')
        spline_y = CubicSpline(t_control, control_points[:, 1], bc_type='natural')

        # Evaluate the B-spline at t_spline values
        curve_points_x = spline_x(t_spline)
        curve_points_y = spline_y(t_spline)
        curve_points = np.vstack((curve_points_x, curve_points_y)).T

        # Calculate the linear interpolation of the angle
        dtheta = theta_end
        theta_values = np.linspace(0, dtheta, num_points)

        # Combine the curve points and theta values
        for i in range(num_points):
            x, y = curve_points[i]
            theta = theta_values[i]
            path.append((x, y, theta))

        return path
    
    
    
    # def generate_path(x_start, y_start, x_end, y_end, theta_end, num_points=100):
    #     path = []

    #     # Define control points
    #     control_points = np.array([
    #         [x_start, y_start],
    #         [(x_start + x_end) / 2, (y_start + y_end) / 2],
    #         [x_end, y_end]
    #     ])

    #     # Degree of the B-spline curve
    #     degree = 2

    #     # Create the B-spline representation of the curve for x and y coordinates separately
    #     t_control = np.linspace(0, 1, control_points.shape[0])
    #     spline_x = make_interp_spline(t_control, control_points[:, 0], k=degree)
    #     spline_y = make_interp_spline(t_control, control_points[:, 1], k=degree)

    #     # Evaluate the B-spline at t_spline values
    #     t_spline = np.linspace(0, 1, num_points)
    #     x_values = spline_x(t_spline)
    #     y_values = spline_y(t_spline)

    #     # Calculate the linear interpolation of the angle
    #     dtheta = theta_end
    #     theta_values = np.linspace(0, dtheta, num_points)

    #     # Combine the x, y, and theta values
    #     for i in range(num_points):
    #         x, y, theta = x_values[i], y_values[i], theta_values[i]
    #         path.append((x, y, theta))

    #     return path


    # draw the obstalces and goal
    def draw_objects(obstacles, goals):
        for ob in obstacles:
            pygame.draw.rect(screen, tuple(name_to_rgb('red')), (int(u0 + k * ob[0]), int(v0 - k * ob[1]),1,1), width=0)
        # for goal in goals:
        pygame.draw.circle(screen,tuple(name_to_rgb('yellow')) , (int(u0 + k * goal[0]), int(v0 - k * goal[1])), int(k * 0.1* render_scale), 0)



    while (True):
        event_list = pygame.event.get()
        
        # Save trail for displaying
        location_history.append((x,y))


        # Planning
        # We want to find the best benefit where we have a positive component for closeness to target,
        # and a negative component for closeness to obstacles, for each of a choice of possible actions
        best_benefit = -100000
        forward_weight = 0.1
        obs_weight = 1000
        
        # Copy of barriers so we can predict their positions
        obs_copied = copy.deepcopy(obs)
        goal_copied = copy.deepcopy(goal)



        # for i in range(steps_ahead_to_plan):
        #     move_obs(obs, display_bound, dt)



        '''Choose the best motion from the list of posiibilities'''
        vL_list = (vL - max_accel*dt, vL, vL+ max_accel*dt)
        vR_list = (vR - max_accel*dt, vR, vR+ max_accel*dt)

        paths_to_draw = []
        new_pos_to_draw = []
        splined_paths = []


        for vl in vL_list:
            for vr in vR_list:
                #chack if the vl and vr within limits
                if (vl <= max_vel and vr <= max_vel and vl >= -max_vel and vr >= -max_vel):

                    x_predict, y_predict, theta_predict, motion = predict_next(vl,vr,x,y,theta,tau)
                    path = generate_path(x, y, x_predict, y_predict, theta_predict)  # You need to implement this function
                    splined_path = fit_bspline(path)
                    paths_to_draw.append(motion)
                    new_pos_to_draw.append((x_predict,y_predict))

                    distance2obs = dist_to_closest_obstacle(x_predict,y_predict,obs)

                    prev_distance_to_goal = math.sqrt((x - goal[0])**2+(y-goal[1])**2)
                    updated_distance_to_goal = math.sqrt((x_predict-goal[0])**2+(y_predict-goal[1])**2)
                    distance_forward = prev_distance_to_goal - updated_distance_to_goal

                    distance_benefit = forward_weight* distance_forward

                    if distance2obs<safedistance:
                        #if away from collision, linearly increase cost
                        obstacle_cost = obs_weight *(safedistance - distance2obs)

                    else:
                        obstacle_cost = 0.0
                    # total benefit
                    benefit = distance_benefit - obstacle_cost

                    if benefit > best_benefit:
                        vl_chosen = vl
                        vr_chosen = vr
                        best_benefit = benefit

        vL = vl_chosen
        vR = vr_chosen

        obs = copy.deepcopy(obs_copied)
        goal = copy.deepcopy(goal_copied)


        '''Pygame rendering'''
        
        # Planning is finished; now do graphics
        screen.fill(tuple(name_to_rgb('black')))
        for loc in location_history:
                pygame.draw.circle(screen,tuple(name_to_rgb('green')), (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 3)
        draw_objects(obs,goal)

        
        # Draw robot
        u = u0 + k * x
        v = v0 - k * y
        pygame.draw.circle(screen,tuple(name_to_rgb('white')) , (int(u), int(v)), int(k * robot_radius))
        # Draw wheels as little blobs so you can see robot orientation
        # left wheel centre 
        wlx = x - (robot_width/2.0) * math.sin(theta)
        wly = y + (robot_width/2.0) * math.cos(theta)
        ulx = u0 + k * wlx
        vlx = v0 - k * wly
        WHEELBLOB = 0.04
        pygame.draw.circle(screen,tuple(name_to_rgb('blue')), (int(ulx), int(vlx)), int(k * WHEELBLOB),)
        # right wheel centre 
        wrx = x + (robot_width/2.0) * math.sin(theta)
        wry = y - (robot_width/2.0) * math.cos(theta)
        urx = u0 + k * wrx
        vrx = v0 - k * wry
        pygame.draw.circle(screen, tuple(name_to_rgb('blue')), (int(urx), int(vrx)), int(k * WHEELBLOB))


        if (1):
                # Draw paths: little arcs which show the different paths the robot is selecting between
                # A bit complicated so don't worry about the details!
                for path in paths_to_draw:
                        #if path[0] = 1:    # Pure rotation: nothing to draw
                        if path[0] == 0:    # Straight line
                                straightpath = path[1]
                                linestart = (u0 + k * x, v0 - k * y)
                                lineend = (u0 + k * (x + straightpath * math.cos(theta)), v0 - k * (y + straightpath * math.sin(theta)))
                                pygame.draw.line(screen, (0, 200, 0), linestart, lineend, 1)
                        if path[0] == 2:    # General case: circular arc
                                # path[2] and path[3] are start and stop angles for arc but they need to be in the right order to pass
                                if (path[3] > path[2]):
                                        startangle = path[2]
                                        stopangle = path[3]
                                else:
                                        startangle = path[3]
                                        stopangle = path[2]
                                # Pygame arc doesn't draw properly unless angles are positive
                                if (startangle < 0):
                                        startangle += 2*math.pi
                                        stopangle += 2*math.pi
                                if (path[1][1][0] > 0 and path[1][0][0] > 0 and path[1][1][1] > 1):
                                        #print (path[1], startangle, stopangle)
                                        pygame.draw.arc(screen, (0, 200, 0), path[1], startangle, stopangle, 1)

     
        
        # Update display
        pygame.display.flip()

        # Actually now move robot based on chosen vL and vR
        (x, y, theta, tmppath) = predict_next(vL, vR, x, y, theta, dt)

        
        '''Make Obstacles Dynamic'''
        # move_obs(obs, display_bound, dt)

        
        # Wraparound: check if robot has reached target; if so reset it to the other side, randomise
        # target position and add some more barriers to go again
        disttotarget = math.sqrt((x - goal[0])**2 + (y - goal[1])**2)
        if (disttotarget < (obs_radius + robot_radius)):
                break
                 
        # Sleeping dt here runs simulation in real-time
        time.sleep(0.00001)

   

if __name__ == '__main__':
    
    main()

