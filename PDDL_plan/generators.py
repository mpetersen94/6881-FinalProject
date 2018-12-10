from itertools import product

import numpy as np

from iiwa_utils import open_wsg50_gripper, get_box_grasps, get_close_wsg50_positions, get_open_wsg50_positions
from motion import plan_joint_motion, plan_waypoints_joint_motion, \
    get_extend_fn, interpolate_translation, plan_workspace_motion, get_collision_fn, get_distance_fn
from utils import get_relative_transform, set_world_pose, set_joint_position, get_body_pose, \
    get_base_body, sample_aabb_placement, get_movable_joints, get_model_name, set_joint_positions, get_box_from_geom, \
    exists_colliding_pair, get_model_bodies, aabb_contains_point, bodies_from_models, get_model_aabb
from trajectory import get_signed_dist_fn, optimal_trajectory

from pydrake.trajectories import PiecewisePolynomial
from pydrake.multibody import inverse_kinematics
from pydrake.all import (RotationMatrix,RollPitchYaw,SolutionResult)

RADIANS_PER_SECOND = np.pi / 4

class Pose(object):
    # TODO: unify Pose & Conf?
    def __init__(self, mbp, parent, child, transform):
        self.mbp = mbp
        self.parent = parent # body_frame
        self.child = child # model_index
        self.transform = transform

    @property
    def bodies(self):
        return get_model_bodies(self.mbp, self.child)

    def assign(self, context):
        parent_pose = get_relative_transform(self.mbp, context, self.parent)
        child_pose = parent_pose.multiply(self.transform)
        set_world_pose(self.mbp, context, self.child, child_pose)

    def __repr__(self):
        return '{}({}->{})'.format(self.__class__.__name__, get_model_name(self.mbp, self.child), self.parent.name())


class Conf(object):
    def __init__(self, joints, positions):
        assert len(joints) == len(positions)
        self.joints = joints
        self.positions = tuple(positions)

    @property
    def bodies(self): # TODO: descendants
        return {joint.child_body() for joint in self.joints}

    def assign(self, context):
        for joint, position in zip(self.joints, self.positions):
            set_joint_position(joint, context, position)

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, len(self.joints))


class Trajectory(object):
    def __init__(self, path, attachments=[], force_control=False):
        self.path = tuple(path)
        self.attachments = attachments
        self.force_control = force_control
        # TODO: store a common set of joints instead (take in joints and path and convert to confs)

    @property
    def joints(self):
        return self.path[0].joints

    @property
    def bodies(self):
        joint_bodies = {joint.child_body() for joint in self.joints}
        for attachment in self.attachments:
            joint_bodies.update(attachment.bodies)
        return joint_bodies

    def reverse(self):
        return self.__class__(self.path[::-1], self.attachments, self.force_control)

    def iterate(self, context):
        for conf in self.path[1:]:
            conf.assign(context)
            for attach in self.attachments: # TODO: topological sort
                attach.assign(context)
            yield

    def spline(self):
        distance_fn = get_distance_fn(self.joints)
        path = [q.positions[:len(self.joints)] for q in self.path]
        q_knots_kuka = np.vstack(path).T
        distances = [0.] + [distance_fn(q1, q2) for q1, q2 in zip(path, path[1:])]
        # TODO: increase time for pick/place & hold
        t_knots = np.cumsum(distances) / RADIANS_PER_SECOND  # TODO: this should be a max
        d, n = q_knots_kuka.shape
        return PiecewisePolynomial.Cubic(
            breaks=t_knots,
            knots=q_knots_kuka,
            knot_dot_start=np.zeros(d),
            knot_dot_end=np.zeros(d))

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, len(self.joints), len(self.path))


def get_open_trajectory(plant, gripper):
    gripper_joints = get_movable_joints(plant, gripper)
    gripper_extend_fn = get_extend_fn(gripper_joints)
    gripper_closed_conf = get_close_wsg50_positions(plant, gripper)
    gripper_path = list(gripper_extend_fn(gripper_closed_conf, get_open_wsg50_positions(plant, gripper)))
    gripper_path.insert(0, gripper_closed_conf)
    return Trajectory(Conf(gripper_joints, q) for q in gripper_path)

##################################################


def get_grasp_gen_fn(task):
    mbp = task.mbp
    gripper_frame = get_base_body(mbp, task.gripper).body_frame()
    box_from_geom = get_box_from_geom(task.scene_graph)
    pitch = 2 * np.pi / 5  # 0 | np.pi/3 | 2 * np.pi / 5

    def gen(obj_name):
        obj = mbp.GetModelInstanceByName(obj_name)
        obj_aabb, obj_from_box, obj_shape = box_from_geom[int(obj), get_base_body(mbp, obj).name(), 0]
        for gripper_from_box in get_box_grasps(obj_aabb, pitch_range=(pitch, pitch)):
            gripper_from_obj = gripper_from_box.multiply(obj_from_box.inverse())
            grasp = Pose(mbp, gripper_frame, obj, gripper_from_obj)
            yield (grasp,)
    return gen

##################################################


def plan_frame_motion(plant, joints, frame, frame_path,
                      initial_guess=None, resolutions=None, collision_fn=lambda q: False):
    waypoints = plan_workspace_motion(plant, joints, frame, frame_path,
                                      initial_guess=initial_guess, collision_fn=collision_fn)
    if waypoints is None:
        return None
    return plan_waypoints_joint_motion(joints, waypoints, resolutions=resolutions, collision_fn=collision_fn)


def get_ik_gen_fn(task, context, collisions=True, max_failures=10, approach_distance=0.2, step_size=0.035):
    approach_vector = approach_distance * np.array([0, -1, 0])
    gripper_frame = get_base_body(task.mbp, task.gripper).body_frame()
    fixed = task.fixed_bodies() if collisions else []
    initial_guess = None
    #initial_guess = get_joint_positions(get_movable_joints(task.mbp, task.robot), context)

    def fn(robot_name, obj_name, obj_pose, obj_grasp):
        # TODO: if gripper/block in collision, return
        robot = task.mbp.GetModelInstanceByName(robot_name)
        joints = get_movable_joints(task.mbp, robot)
        collision_pairs = set(product(bodies_from_models(task.mbp, [robot, task.gripper]), fixed))
        collision_fn = get_collision_fn(task.diagram, task.diagram_context, task.mbp, task.scene_graph,
                                        joints, collision_pairs=collision_pairs) # TODO: while holding

        gripper_pose = obj_pose.transform.multiply(obj_grasp.transform.inverse())
        gripper_path = list(interpolate_translation(gripper_pose, approach_vector, step_size=step_size))

        attempts = 0
        last_success = 0
        while (attempts - last_success) < max_failures:
            attempts += 1
            obj_pose.assign(context)
            path = plan_frame_motion(task.plant, joints, gripper_frame, gripper_path,
                                     initial_guess=initial_guess, collision_fn=collision_fn)
            if path is None:
                continue
            traj = Trajectory([Conf(joints, q) for q in path], attachments=[obj_grasp])
            conf = traj.path[-1]
            yield (conf, traj)
            last_success = attempts
    return fn

##################################################

def get_door_grasp(door_body, box_from_geom):
    pitch = np.pi/3 # np.pi/2
    grasp_length = 0.02
    target_shape, target_ori = 'cylinder', 1  # Second grasp is np.pi/2, corresponding to +y
    for i in range(2):
        handle_aabb, handle_from_box, handle_shape = box_from_geom[int(door_body.model_instance()), door_body.name(), i]
        if handle_shape == target_shape:
            break
    else:
        raise RuntimeError(target_shape)
    [gripper_from_box] = list(get_box_grasps(handle_aabb, orientations=[target_ori],
                                             pitch_range=(pitch, pitch), grasp_length=grasp_length))
    return gripper_from_box.multiply(handle_from_box.inverse())


def get_pull_fn(task, context, collisions=True):
    # TODO: could also push the door either perpendicular or parallel
    # TODO: allow small rotation error perpendicular to handle
    # DoDifferentialInverseKinematics
    box_from_geom = get_box_from_geom(task.scene_graph)
    gripper_frame = get_base_body(task.mbp, task.gripper).body_frame()
    fixed = task.fixed_bodies() if collisions else []

    def fn(robot_name, door_name, door_conf1, door_conf2):
        """
        :param robot_name: The name of the robot (should be iiwa)
        :param door_name: The name of the door (should be left_door or right_door)
        :param door_conf1: The initial door configuration
        :param door_conf2: The final door configuration
        :return: A triplet composed of the initial robot configuration, final robot configuration,
                 and combined robot & door position trajectory to execute the pull
        """
        robot = task.mbp.GetModelInstanceByName(robot_name)
        robot_joints = get_movable_joints(task.mbp, robot)
        collision_pairs = set(product(bodies_from_models(task.mbp, [robot, task.gripper]), fixed))
        collision_fn = get_collision_fn(task.diagram, task.diagram_context, task.mbp, task.scene_graph,
                                        robot_joints, collision_pairs=collision_pairs)

        door_body = task.mbp.GetBodyByName(door_name)
        door_joints = door_conf1.joints
        combined_joints = robot_joints + door_joints
        # The transformation from the door frame to the gripper frame that corresponds to grasping the door handle
        gripper_from_door = get_door_grasp(door_body, box_from_geom)

        ##### BEGIN YOUR CODE HERE #####
        plant=task.mbp
        def GetKukaQKnots(q_knots):
            if len(q_knots.shape) == 1:
                q_knots.resize(1, q_knots.size)
            n = q_knots.shape[0]
            q_knots_kuka = np.zeros((n, 7))
            for i, q_knot in enumerate(q_knots):
                q_knots_kuka[i] = plant.tree().GetPositionsFromArray(robot, q_knot)

            return q_knots_kuka

        world_frame = plant.world_frame()
        n_iterations=36
        combined_joint_path = []
        q_knots = np.zeros((n_iterations + 2, plant.num_positions()))
        door_angle=np.zeros(n_iterations+1)
        for i in range(n_iterations+1):
            door_angle[i]=door_conf1.positions[0]+(door_conf2.positions[0]-door_conf1.positions[0])/n_iterations*i
            door_joints[0].set_angle(context,door_angle[i])

            W_from_door = plant.tree().EvalBodyPoseInWorld(context,door_body)
            W_from_gripper=W_from_door.multiply(gripper_from_door.inverse())

            ik = inverse_kinematics.InverseKinematics(plant)
            p_WQ_desired = W_from_gripper.translation()
            p_WQ_desired_upper = p_WQ_desired + 0.005
            p_WQ_desired_lower = p_WQ_desired - 0.005

            ik.AddPositionConstraint(
                    frameB=gripper_frame, p_BQ=np.zeros(3),
                    frameA=world_frame,
                    p_AQ_lower=p_WQ_desired_lower, p_AQ_upper=p_WQ_desired_upper)


            # include angle constraint in ik problem
            theta_bound = 0.01 * np.pi
            R_WEa = RotationMatrix(W_from_gripper.rotation())
            R_EEa = RotationMatrix(np.eye(3))
            # R_EEa = RollPitchYaw([0,0,-0.1]).ToRotationMatrix()

            ik.AddOrientationConstraint(
                frameAbar=world_frame, R_AbarA=R_WEa,
                frameBbar=gripper_frame, R_BbarB=R_EEa,
                theta_bound=theta_bound)


            prog = ik.get_mutable_prog()
            prog.SetInitialGuess(ik.q(), q_knots[i])  # this is very important!
            result = prog.Solve()
            if result != SolutionResult.kSolutionFound:
                print result
            q_knots[i] = prog.GetSolution(ik.q())
            q_knots[i + 1] = q_knots[i]


        # combined_joint_path is a joint position path for the concatenated robot and door joints.
        # It should be a list of 8 DOF configurations (7 robot DOFs + 1 door DOF).
        # Additionally, combined_joint_path[0][len(robot_joints):] should equal door_conf1.positions
        # and combined_joint_path[-1][len(robot_joints):] should equal door_conf2.positions.

        # print GetKukaQKnots(q_knots[0:-1]).shape, np.expand_dims(door_angle,axis=1).shape
        combined_joint_path = np.hstack((GetKukaQKnots(q_knots[0:-1]),np.expand_dims(door_angle,axis=1)))
        # print combined_joint_path.shape

        ##### END YOUR CODE HERE #####

        robot_conf1 = Conf(robot_joints, combined_joint_path[0][:len(robot_joints)])
        robot_conf2 = Conf(robot_joints, combined_joint_path[-1][:len(robot_joints)])
        traj = Trajectory(Conf(combined_joints, combined_conf) for combined_conf in combined_joint_path)
        yield (robot_conf1, robot_conf2, traj)
    return fn

##################################################

def parse_fluents(fluents, context, obstacles):
    attachments = []
    for fact in fluents:
        predicate = fact[0]
        if predicate == 'AtConf'.lower():
            name, conf = fact[1:]
            conf.assign(context)
            obstacles.update(conf.bodies)
        elif predicate == 'AtPose'.lower():
            name, pose = fact[1:]
            pose.assign(context)
            obstacles.update(pose.bodies)
        elif predicate == 'AtGrasp'.lower():
            robot, name, grasp = fact[1:]
            attachments.append(grasp)
        else:
            raise ValueError(predicate)
    return attachments


def get_motion_fn(task, context, collisions=True):
    gripper = task.gripper

    def fn(robot_name, conf1, conf2, fluents=[]):
        robot = task.mbp.GetModelInstanceByName(robot_name)
        joints = get_movable_joints(task.mbp, robot)

        moving = bodies_from_models(task.mbp, [robot, gripper])
        obstacles = set(task.fixed_bodies())
        attachments = parse_fluents(fluents, context, obstacles)
        for grasp in attachments:
            moving.update(grasp.bodies)
        obstacles -= moving

        collision_pairs = set(product(moving, obstacles)) if collisions else set()
        collision_fn = get_collision_fn(task.diagram, task.diagram_context, task.mbp, task.scene_graph,
                                        joints, collision_pairs=collision_pairs, attachments=attachments)
        signed_dist_fn = get_signed_dist_fn(task.diagram, task.diagram_context, task.mbp, task.scene_graph,
                                            joints, collision_pairs=collision_pairs, attachments=attachments)
        open_wsg50_gripper(task.mbp, context, gripper)
        # path = plan_joint_motion(joints, conf1.positions, conf2.positions, collision_fn=collision_fn,
                                 # restarts=10, iterations=75, smooth=50)
        path = optimal_trajectory(joints, conf1.positions, conf2.positions,
                                  signed_dist_fn, len(collision_pairs))
        if path is None:
            return None
        traj = Trajectory([Conf(joints, q) for q in path], attachments=attachments)
        return (traj,)
    return fn

##################################################

def get_collision_test(task, context, collisions=True):
    # TODO: precompute and hash?
    def test(traj, obj_name, pose):
        if not collisions:
            return False
        moving = bodies_from_models(task.mbp, [task.robot, task.gripper])
        moving.update(traj.bodies)
        obstacles = set(pose.bodies) - moving
        collision_pairs = set(product(moving, obstacles))
        if not collision_pairs:
            return False
        pose.assign(context)
        for _ in traj.iterate(context):
            if exists_colliding_pair(task.diagram, task.diagram_context, task.mbp, task.scene_graph, collision_pairs):
                return True
        return False
    return test
