import numpy as np
import os, sys
import pybullet as p
import math
import time
sys.path.insert(0, 'utils/')
from Robot import Robot 
import dqrobotics 
from dqrobotics import robot_modeling 
from dqrobotics.utils import DQ_Geometry
from dqrobotics import *


class SimulatorSetup:
    def __init__(self):
        if not p.isConnected():
            p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        print("\nSimulator setup completed.\n")

    def reset(self):
        p.resetSimulation()
        print("\nSimulation reset.\n")


class RobotSetup:
    def __init__(self, robot_data, grip_path):
        self.robot_data = robot_data
        self.grip_path = grip_path
        self.robot = None
        self.initialize_robot()

    def initialize_robot(self):
        self.robot = Robot(self.robot_data, gripPath=self.grip_path)
        self.add_base_constraint()

    def add_base_constraint(self):
        p.createConstraint(
            self.robot.robot_model,
            -1,
            -1,
            -1,
            p.JOINT_FIXED,
            [0, 0, 0],
            [0, 0, 0],
            self.robot.start_pos,
        )

    def get_robot(self):
        return self.robot


class SimulationTests:
    def __init__(self, robot):
        self.robot = robot

    def sim_step(self, steps=240, timestep=1 / 90.0):
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(timestep)

    def test_move_arm_to_target(self):
        xd = self.robot.serialManipulator.fkm(
            np.array([0, 0, 0, math.pi / 4, 0, -math.pi / 2, 0])
        )
        self.robot.moveArmToEETarget(xd, 0.2)

        theta = np.zeros(self.robot.numArmJoints)
        for j in range(self.robot.numArmJoints):
            theta[j] = p.getJointState(self.robot.robot_model, j)[0]
            time.sleep(0.1)

        xTrue = self.robot.serialManipulator.fkm(theta)
        err = DQ_Geometry.vec4(xTrue - xd)

        print("Error vector:", err)
        assert np.allclose(err, np.zeros(len(err)), 0.1, 0.2), "Kuka does not move properly!"

    def test_set_joint_position(self):
        target_pos = np.array([0, -(math.pi / 2), 0, -(math.pi / 2), 0, -(math.pi / 2), 0])
        self.robot.setJointPosition(target_pos)

        theta = np.zeros(self.robot.numArmJoints)
        for j in range(self.robot.numArmJoints):
            theta[j] = p.getJointState(self.robot.robot_model, j)[0]
            self.sim_step()

        print("Final joint positions:", theta)
        assert np.allclose(theta, target_pos, 0.0001, 0.00001), "setJointPosition does not work for Kuka"


def main():
    simulator = SimulatorSetup()

    robot_data = {
        "arm": "Kuka",
        "gripper": "RG6",
        "robot_start_pos": [0, 0, 0],
        "robot_start_orientation_euler": [0, 0, 0],
        "K": 0.5,
        "camera": True,
        "camera_pos": [0, 0, 0],
    }
    grip_path = "./data/grip/threeFingers.dat"

    robot_setup = RobotSetup(robot_data, grip_path)
    robot = robot_setup.get_robot()

    tests = SimulationTests(robot)
    # Uncomment the desired test to run
    # tests.test_move_arm_to_target()
    tests.test_set_joint_position()


if __name__ == "__main__":
    main()

