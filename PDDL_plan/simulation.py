import time

import numpy as np
from pydrake.systems.analysis import Simulator
from pydrake.trajectories import PiecewisePolynomial

from utils import get_configuration, user_input


def step_trajectories(diagram, diagram_context, plant_context, trajectories, time_step=0.001, teleport=False):
    diagram.Publish(diagram_context)
    user_input('Step?')
    for traj in trajectories:
        if teleport:
            traj.path = traj.path[::len(traj.path)-1]
        for _ in traj.iterate(plant_context):
            diagram.Publish(diagram_context)
            if time_step is None:
                user_input('Continue?')
            else:
                time.sleep(time_step)
    user_input('Finish?')

##################################################


def get_hold_spline(mbp, context, robot, duration=2.0):
    q_knots_kuka = np.zeros((2, 7))
    q_knots_kuka[0] = get_configuration(mbp, context, robot)  # Second is velocity
    return PiecewisePolynomial.ZeroOrderHold([0, duration], q_knots_kuka.T)


def get_gripper_setpoint(mbp, context, gripper):
    _, gripper_setpoint = get_configuration(mbp, context, gripper)
    return abs(gripper_setpoint)


def convert_splines(mbp, robot, gripper, context, trajectories):
    splines = [
        get_hold_spline(mbp, context, robot),
    ]
    gripper_setpoints = [
        get_gripper_setpoint(mbp, context, gripper),
    ]
    for traj in trajectories:
        traj.path[-1].assign(context)
        if len(traj.joints) == 8:
            traj.joints.pop()
            # TODO: remove inclusion of door joints

        if len(traj.joints) == 2:
            splines.append(get_hold_spline(mbp, context, robot))
        elif len(traj.joints) == 7:
            splines.append(traj.spline())
        else:
            raise ValueError('Invalid number of joints: {}'.format(len(traj.joints)))
        gripper_setpoints.append(get_gripper_setpoint(mbp, context, gripper))

    print
    for i, (spline, setpoint) in enumerate(zip(splines, gripper_setpoints)):
        d = spline.rows()
        n = spline.get_number_of_segments()
        print('{}) d={}, n={}, setpoint={}, duration={:.3f}'.format(
            i, d, n, setpoint, spline.end_time()))
    return splines, gripper_setpoints


def compute_duration(splines, extra_time=5.0):
    from manipulation_station.robot_plans import PlanBase
    sim_duration = 0.
    for spline in splines:
        if isinstance(spline, PlanBase):
            sim_duration += spline.get_duration() * 1.1
        else:
            sim_duration += spline.end_time() + 0.5
    sim_duration += extra_time
    return sim_duration

##################################################

def simulate_splines(diagram, diagram_context, sim_duration, real_time_rate=1.0):
    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(real_time_rate)
    simulator.Initialize()

    diagram.Publish(diagram_context)
    user_input('Simulate?')
    simulator.StepTo(sim_duration)
    user_input('Finish?')