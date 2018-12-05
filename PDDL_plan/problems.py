
import numpy as np
from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import ManipulationStation, IiwaCollisionModel
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile

from systems import build_manipulation_station
from iiwa_utils import DOOR_CLOSED, DOOR_OPEN, open_wsg50_gripper
from utils import get_model_name, create_transform, get_movable_joints, \
    get_model_bodies, get_bodies, set_joint_position, set_world_pose


class Task(object):
    def __init__(self, diagram, mbp, scene_graph, robot, gripper,
                 movable=[], surfaces=[], doors=[],
                 initial_positions={}, initial_poses={},
                 goal_poses={}, goal_holding=[], goal_on=[], goal_cooked=[],
                 reset_robot=True, reset_doors=True):
        # Drake systems
        self.diagram = diagram
        self.mbp = mbp
        self.scene_graph = scene_graph
        self.diagram_context = diagram.CreateDefaultContext()
        self.plant_context = diagram.GetMutableSubsystemContext(mbp, self.diagram_context)
        # context = plant.CreateDefaultContext()

        # Semantic information about models
        self.robot = robot
        self.gripper = gripper
        self.movable = movable
        self.surfaces = surfaces
        self.doors = doors

        # Initial values
        self.initial_positions = initial_positions
        self.initial_poses = initial_poses

        # Goal conditions
        self.goal_poses = goal_poses
        self.goal_holding = goal_holding
        self.goal_on = goal_on
        self.goal_cooked = goal_cooked
        self.reset_robot = reset_robot
        self.reset_doors = reset_doors
    @property
    def plant(self):
        return self.mbp
    def movable_bodies(self):
        movable = {self.mbp.tree().get_body(index) for index in self.doors}
        for model in [self.robot, self.gripper] + list(self.movable):
            movable.update(get_model_bodies(self.mbp, model))
        return movable
    def fixed_bodies(self):
        return set(get_bodies(self.mbp)) - self.movable_bodies()
    def set_initial(self):
        for joint, position in self.initial_positions.items():
            set_joint_position(joint, self.plant_context, position)
        for model, pose in self.initial_poses.items():
            set_world_pose(self.plant, self.plant_context, model, pose)
        open_wsg50_gripper(self.plant, self.plant_context, self.gripper)
    def publish(self):
        self.diagram.Publish(self.diagram_context)
    def __repr__(self):
        return '{}(robot={}, gripper={}, movable={}, surfaces={})'.format(
            self.__class__.__name__,
            get_model_name(self.mbp, self.robot),
            get_model_name(self.mbp, self.gripper),
            [get_model_name(self.mbp, model) for model in self.movable],
            self.surfaces)

##################################################

def load_station(time_step=0.0, plan=None):
    # TODO: map names to SDF paths
    # https://github.com/RobotLocomotion/drake/blob/master/bindings/pydrake/examples/manipulation_station_py.cc
    #object_file_path = FindResourceOrThrow(
    #    "drake/external/models_robotlocomotion/ycb_objects/061_foam_brick.sdf")
    object_file_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf")

    # RuntimeError: Error control wants to select step smaller than minimum allowed (1e-14)
    station = ManipulationStation(time_step, IiwaCollisionModel.kBoxCollision)
    plant = station.get_mutable_multibody_plant()
    scene_graph = station.get_mutable_scene_graph()

    robot = plant.GetModelInstanceByName('iiwa')
    gripper = plant.GetModelInstanceByName('gripper')
    station.AddCupboard()
    brick = AddModelFromSdfFile(file_name=object_file_path, model_name="brick",
                                plant=plant, scene_graph=scene_graph)
    station.Finalize()

    door_names = ['left_door', 'right_door']
    doors = [plant.GetBodyByName(name).index() for name in door_names]

    initial_conf = [0, 0.6 - np.pi / 6, 0, -1.75, 0, 1.0, 0]
    #initial_conf[1] += np.pi / 6
    initial_positions = {
        plant.GetJointByName('left_door_hinge'): -DOOR_CLOSED,
        #plant.GetJointByName('left_door_hinge'): -np.pi / 2,
        plant.GetJointByName('right_door_hinge'): np.pi/2,
        #plant.GetJointByName('right_door_hinge'): np.pi,
    }
    initial_positions.update(zip(get_movable_joints(plant, robot), initial_conf))

    initial_poses = {
        brick: create_transform(translation=[0.3, 0, 0], rotation=[0, 0, np.pi/2]),
    }
    goal_poses = {
        brick: create_transform(translation=[0.8, 0.2, 0.2927], rotation=[0, 0, 5*np.pi/4]),
    }

    diagram, state_machine = build_manipulation_station(station, plan)

    task = Task(diagram, plant, scene_graph, robot, gripper, movable=[brick], doors=doors,
                initial_positions=initial_positions, initial_poses=initial_poses,
                goal_poses=goal_poses, reset_robot=True, reset_doors=False)
    task.set_initial()

    return task, diagram, state_machine
