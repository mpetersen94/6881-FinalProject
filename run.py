#!/usr/bin/env python2

from __future__ import print_function

#import sys
#sys.path.append('pddlstream')

import argparse
import random
import numpy as np

from generators import Pose, Conf, get_grasp_gen_fn, get_ik_gen_fn, \
    get_motion_fn, get_pull_fn, get_collision_test, get_open_trajectory, Trajectory
from iiwa_utils import get_door_positions, DOOR_OPEN
from simulation import compute_duration, convert_splines, step_trajectories, simulate_splines
from problems import load_station
from systems import RenderSystemWithGraphviz
from utils import get_world_pose, get_configuration, get_model_name, get_joint_positions, get_parent_joints, \
    get_state, set_state, get_movable_joints

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, PDDLProblem
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_gen_fn, from_fn
from pddlstream.utils import print_solution, read, INF, get_file_path


def get_pddlstream_problem(task, context, collisions=True):
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    plant = task.mbp
    robot = task.robot
    robot_name = get_model_name(plant, robot)

    world = plant.world_frame() # mbp.world_body()
    robot_joints = get_movable_joints(plant, robot)
    robot_conf = Conf(robot_joints, get_configuration(plant, context, robot))
    init = [
        ('Robot', robot_name),
        ('CanMove', robot_name),
        ('Conf', robot_name, robot_conf),
        ('AtConf', robot_name, robot_conf),
        ('HandEmpty', robot_name),
    ]
    goal_literals = []
    if task.reset_robot:
        goal_literals.append(('AtConf', robot_name, robot_conf),)

    for obj in task.movable:
        obj_name = get_model_name(plant, obj)
        obj_pose = Pose(plant, world, obj, get_world_pose(plant, context, obj))
        init += [('Graspable', obj_name),
                 ('Pose', obj_name, obj_pose),
                 ('AtPose', obj_name, obj_pose)]
        for surface in task.surfaces:
            init += [('Stackable', obj_name, surface)]

    for surface in task.surfaces:
        surface_name = get_model_name(plant, surface.model_index)
        if 'sink' in surface_name:
            init += [('Sink', surface)]
        if 'stove' in surface_name:
            init += [('Stove', surface)]

    for door in task.doors:
        door_body = plant.tree().get_body(door)
        door_name = door_body.name()
        door_joints = get_parent_joints(plant, door_body)
        door_conf = Conf(door_joints, get_joint_positions(door_joints, context))
        init += [
            ('Door', door_name),
            ('Conf', door_name, door_conf),
            ('AtConf', door_name, door_conf),
        ]
        for positions in [get_door_positions(door_body, DOOR_OPEN)]:
            conf = Conf(door_joints, positions)
            init += [('Conf', door_name, conf)]
        if task.reset_doors:
            goal_literals += [('AtConf', door_name, door_conf)]

    for obj, transform in task.goal_poses.items():
        obj_name = get_model_name(plant, obj)
        obj_pose = Pose(plant, world, obj, transform)
        init += [('Pose', obj_name, obj_pose)]
        goal_literals.append(('AtPose', obj_name, obj_pose))
    for obj in task.goal_holding:
        goal_literals.append(('Holding', robot_name, get_model_name(plant, obj)))
    for obj, surface in task.goal_on:
        goal_literals.append(('On', get_model_name(plant, obj), surface))
    for obj in task.goal_cooked:
        goal_literals.append(('Cooked', get_model_name(plant, obj)))

    goal = And(*goal_literals)
    print('Initial:', init)
    print('Goal:', goal)

    stream_map = {
        'sample-grasp': from_gen_fn(get_grasp_gen_fn(task)),
        'plan-ik': from_gen_fn(get_ik_gen_fn(task, context, collisions=collisions)),
        'plan-motion': from_fn(get_motion_fn(task, context, collisions=collisions)),
        'plan-pull': from_gen_fn(get_pull_fn(task, context, collisions=collisions)),
        'TrajPoseCollision': get_collision_test(task, context, collisions=collisions),
        'TrajConfCollision': get_collision_test(task, context, collisions=collisions),
    }
    #stream_map = 'debug' # Runs PDDLStream with "placeholder streams" for debugging

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


def postprocess_plan(plant, gripper, plan):
    trajectories = []
    if plan is None:
        return trajectories
    open_traj = get_open_trajectory(plant, gripper)
    close_traj = open_traj.reverse()

    for name, args in plan:
        if name in ['clean', 'cook']:
            continue
        traj = args[-1]
        if name == 'pick':
            trajectories.extend([Trajectory(reversed(traj.path)), close_traj, traj])
        elif name == 'place':
            trajectories.extend([traj.reverse(), open_traj, Trajectory(traj.path)])
        elif name == 'pull':
            trajectories.extend([close_traj, traj, open_traj])
        elif name == 'move':
            trajectories.append(traj)
        else:
            raise NotImplementedError(name)
    return trajectories


def plan_trajectories(task, context, collisions=True, max_time=180):
    stream_info = {
        'TrajPoseCollision': FunctionInfo(p_success=1e-3),
        'TrajConfCollision': FunctionInfo(p_success=1e-3),
    }
    problem = get_pddlstream_problem(task, context, collisions=collisions)
    solution = solve_focused(problem, stream_info=stream_info, planner='ff-wastar2',
                             max_time=max_time, search_sampling_ratio=0)
    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is None:
        print('Unable to find a solution in under {} seconds'.format(max_time))
        return None
    return postprocess_plan(task.mbp, task.gripper, plan)

##################################################

def main():
    #time_step = 0.0002 # TODO: context.get_continuous_state_vector() fails
    time_step = 2e-3

    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--cfree', action='store_true',
                        help='Disables collisions when planning')
    parser.add_argument('-d', '--deterministic', action='store_true',
                        help='Manually sets the random seeds used by the stream generators')
    parser.add_argument('-s', '--simulate', action='store_true',
                        help='Simulates the system')
    args = parser.parse_args()

    if args.deterministic:
        # TODO: still not fully deterministic
        random.seed(0)
        np.random.seed(0)

    import meshcat
    meshcat_vis = meshcat.Visualizer()
    task, diagram, state_machine = load_station(time_step=time_step)
    print(task)

    plant = task.mbp
    #dump_plant(plant)
    #dump_models(plant)
    RenderSystemWithGraphviz(diagram) # Useful for getting port names
    context = diagram.GetMutableSubsystemContext(plant, task.diagram_context)

    task.publish()
    initial_state = get_state(plant, context)
    trajectories = plan_trajectories(task, context, collisions=not args.cfree)
    if trajectories is None:
        return

    ##################################################

    set_state(plant, context, initial_state)
    if args.simulate:
        from manipulation_station.robot_plans import JointSpacePlan
        splines, gripper_setpoints = convert_splines(plant, task.robot, task.gripper, context, trajectories)
        sim_duration = compute_duration(splines)
        plan_list = [JointSpacePlan(spline) for spline in splines]
        print('Splines: {}\nDuration: {:.3f} seconds'.format(len(splines), sim_duration))

        task, diagram, state_machine = load_station(time_step=time_step, plan=(plan_list, gripper_setpoints))
        task.set_initial()
        #set_state(plant, context, initial_state)
        #state_machine.Load(plan_list, gripper_setpoints)
        simulate_splines(task.diagram, task.diagram_context, sim_duration)
    else:
        step_trajectories(diagram, task.diagram_context, context, trajectories, time_step=0.001)


if __name__ == '__main__':
    main()
