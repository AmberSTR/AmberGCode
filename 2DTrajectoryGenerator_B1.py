import csv

from loop_rate_limiters import RateLimiter

from pink.utils import custom_configuration_vector
import math
import time
import matplotlib.pyplot as plt
import socket
from ctypes import *
import pink
from pink import solve_ik
from pink.tasks import BodyTask, PostureTask
from pink.visualization import start_meshcat_visualizer
import os
import pinocchio as pin
import meshcat_shapes
import qpsolvers
# import _thread
import threading

semaphore_START = threading.Semaphore(0)
semaphore_READ = threading.Semaphore(0)
semaphore_SOLVE = threading.Semaphore(0)
semaphore_OVER = threading.Semaphore(0)

joint1 = []
joint2 = []
joint3 = []
joint4 = []
joint5 = []
joint6 = []
joint7 = []


class Cartesian:
    X = 0.0
    Y = 0.3
    Z = 0
    roll = 0
    pitch = 3.14
    yaw = 0
    FREQ = 50
    OVER = False


def pink_solver():
    print("pink_solver_ON")
    """
    Amber B1 arm tracking a moving target.
    """
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        #"urdf_l1",
        #"amber_l1.urdf",
        "urdf_b1",
        "amber_b1.urdf",
    )
    print("Preparing URDF")
    robot = pin.RobotWrapper.BuildFromURDF(
        filename=urdf_path,
        package_dirs=["."],
        root_joint=None,
    )
    # robot = load_robot_description("gen2_description", root_joint=None)
    print("Starting meshcat")
    viz = start_meshcat_visualizer(robot)
    end_effector_task = BodyTask(
        "seven_Link",
        position_cost=[1.0,1.0,1.0],  # [cost] / [m]
        orientation_cost=0.9,  # [cost] / [rad]
    )
    posture_task = PostureTask(
        # cost=1e-3,  # [cost] / [rad]
        cost=0.02,  # [cost] / [rad]
    )
    tasks = [end_effector_task, posture_task]
    q = custom_configuration_vector(
        robot,  # joint2=1.575, joint4=0
        # ,   #one = -1.575,two=0.525, four=-0.525,six=0.525
    )
    print("Press Enter to Start! ")
    configuration = pink.apply_configuration(robot, q)
    for task in tasks:
        task.set_target_from_configuration(configuration)
    viz.display(configuration.q)
    viewer = viz.viewer
    meshcat_shapes.frame(viewer["end_effector_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["end_effector"], opacity=1.0)
    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"
    rate = RateLimiter(frequency=Cartesian.FREQ)
    dt = rate.period
    # t = 0.0  # [s]
    semaphore_START.acquire()
    semaphore_START.release()
    while True:

        semaphore_READ.acquire()
        # Update task targets
        end_effector_target = end_effector_task.transform_target_to_world
        end_effector_target.translation[0] = Cartesian.X
        end_effector_target.translation[1] = Cartesian.Y
        end_effector_target.translation[2] = Cartesian.Z
        end_effector_target.rotation = pin.utils.rpyToMatrix(Cartesian.roll, Cartesian.pitch, Cartesian.yaw)
        # Update visualization frames
        viewer["end_effector_target"].set_transform(end_effector_target.np)
        viewer["end_effector"].set_transform(
            configuration.get_transform_body_to_world(
                end_effector_task.body
            ).np
        )
        # Compute velocity and integrate it into next configuration
        velocity = solve_ik(configuration, tasks, dt, solver=solver)
        q = configuration.integrate(velocity, dt)

        joint1.append(q[0])
        joint2.append(q[1])
        joint3.append(q[2])
        joint4.append(q[3])
        joint5.append(q[4])
        joint6.append(q[5])
        joint7.append(q[6])
        # udp_to_ros(q)
        configuration = pink.apply_configuration(robot, q)
        # Visualize result at fixed FPS
        viz.display(q)
        # time.sleep()
        # rate.sleep()
        # print(dt)
        # t += dt
        time.sleep(0.005)
        print(q)
        if Cartesian.OVER:
            semaphore_OVER.release()
            break
        semaphore_SOLVE.release()


semaphore_READ.release()
t1 = threading.Thread(target=pink_solver)
t1.start()
# t1.join()
semaphore_SOLVE.release()
input()
semaphore_START.release()
# print("Playing")
lineindex = 0
Cx = []
Cy = []

with open('XY.csv', 'r') as csv_file_R:
    csv_reader = csv.reader(csv_file_R)
    print(f"Solving frame 0")
    for line in csv_reader:
        lineindex += 1
        semaphore_SOLVE.acquire()
        Cartesian.X = float(line[0])
        Cartesian.Z = float(line[1])- 0.3
        Cx.append(float(line[0]))
        Cy.append(float(line[1]))
        if lineindex % 50 == 0:
            print(f"Solving frame {lineindex}")
        semaphore_READ.release()
    Cartesian.OVER = True
    semaphore_OVER.acquire()
    print(f"Total frame {lineindex}")
    print("OVER")
    # print(joint1)
with open('out.csv', 'w+', encoding='utf-8', newline='') as csv_file_W:
    rowindex = 0
    writer = csv.writer(csv_file_W)
    for items in joint1:
        if rowindex > 100:
            writer.writerow([0, 1, 0, 0,
                             -1 * joint1[rowindex],
                             -1 * joint2[rowindex],
                             -1 * joint3[rowindex],
                             -1 * joint4[rowindex],
                             -1 * joint5[rowindex],
                             -1 * joint6[rowindex],
                             -1 * joint7[rowindex], Cx[rowindex], Cy[rowindex]])
        rowindex += 1
