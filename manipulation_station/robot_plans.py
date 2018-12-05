import numpy as np
from pydrake.trajectories import PiecewisePolynomial
from pydrake.math import RollPitchYaw
from pydrake.util.eigen_geometry import Isometry3


PlanTypes = [
    "JointSpacePlan",
    "TaskSpacePlan",
    "PlanarTaskSpacePlan",
    "PlanarHybridPositionForcePlan",
    "OpenLeftDoorPositionPlan",
    "OpenLeftDoorImpedancePlan",
]


class PlanBase:
    def __init__(self,
                 type = None,
                 trajectory = None,):
        self.type = type
        self.traj = trajectory
        self.traj_d = None
        self.duration = None
        if trajectory is not None:
            self.traj_d = trajectory.derivative(1)
            self.duration = trajectory.end_time()

        self.start_time = None

    def get_duration(self):
        return self.duration

    def set_start_time(self, time):
        self.start_time = time


class JointSpacePlan(PlanBase):
    def __init__(self,
                 trajectory=None):
        PlanBase.__init__(self,
                          type=PlanTypes[0],
                          trajectory=trajectory)


# Define positions and transforms used in various tasks, e.g. opening left door.
'''
Ea, or End_Effector_world_aligned is a frame fixed w.r.t the gripper.
Ea has the same origin as the end effector's body frame, but
its axes are aligned with those of the world frame when the system
has zero state, i.e. the robot is upright with all joint angles
equal to zero.
This frame is defined so that it is convenient to define end effector orientation
relative to the world frame using RollPitchYaw.
'''
def GetEndEffectorWorldAlignedFrame():
    X_EEa = Isometry3.Identity()
    X_EEa.set_rotation(np.array([[0., 1., 0,],
                                 [0, 0, 1],
                                 [1, 0, 0]]))
    return X_EEa

# home position of point Q in world frame.
p_WQ_home = np.array([0.5, 0, 0.41])

# L: frame the cupboard left door, whose origin is at the center of the door body.
p_WL = np.array([0.7477, 0.1445, 0.4148]) #+  [-0.1, 0, 0]
# center of the left hinge of the door in frame L and W
p_LC_left_hinge = np.array([0.008, 0.1395, 0])
p_WC_left_hinge = p_WL + p_LC_left_hinge

# center of handle in frame L and W
p_LC_handle = np.array([-0.033, -0.1245, 0])
p_WC_handle = p_WL + p_LC_handle

# distance between the hinge center and the handle center
p_handle_2_hinge = p_LC_handle - p_LC_left_hinge
r_handle = np.linalg.norm(p_handle_2_hinge)

# angle between the world y axis and the line connecting the hinge cneter to the
# handle center when the left door is fully closed (left_hinge_angle = 0).
theta0_hinge = np.arctan2(np.abs(p_handle_2_hinge[0]),
                          np.abs(p_handle_2_hinge[1]))

# position of point Q in EE frame.  Point Q is fixed in the EE frame.
p_EQ = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0., 0.095]))

# orientation of end effector aligned frame
R_WEa_ref = RollPitchYaw(0, np.pi / 180 * 135, 0).ToRotationMatrix()


class OpenLeftDoorPlan(PlanBase):
    def __init__(self, angle_start, angle_end=np.pi/4, duration=10.0, type=None):
        t_knots = [0, duration/2, duration]
        door_angle_knots = np.zeros((3,1))
        door_angle_knots[0] = angle_start
        door_angle_knots[2] = angle_end
        door_angle_knots[1] = (door_angle_knots[0] + door_angle_knots[2])/2
        angle_traj = PiecewisePolynomial.Cubic(t_knots, door_angle_knots.T,
                                               np.zeros(1), np.zeros(1))
        PlanBase.__init__(self,
                          type=type,
                          trajectory=angle_traj)

    def CalcKinematics(
            self, X_L7E, l7_frame, world_frame, tree_iiwa, context_iiwa, handle_angle_ref):
        # calculate Geometric jacobian (6 by 7 matrix) of point Q in frame L7.
        p_L7Q = X_L7E.multiply(p_EQ)
        Jv_WL7q = tree_iiwa.CalcFrameGeometricJacobianExpressedInWorld(
            context=context_iiwa, frame_B=l7_frame,
            p_BoFo_B=p_L7Q)

        # Translation
        # Hr: handle reference frame
        # p_HrQ position of point Q relative to frame Hr.
        # X_WHr: transformation from Hr to world frame W.
        X_WHr = Isometry3()
        X_HrW = X_WHr.inverse()

        X_WL7 = tree_iiwa.CalcRelativeTransform(
            context_iiwa, frame_A=world_frame,
            frame_B=l7_frame)

        p_WQ = X_WL7.multiply(p_L7Q)
        p_HrQ = X_HrW.multiply(p_WQ)

        return Jv_WL7q, p_HrQ


class OpenLeftDoorPositionPlan(OpenLeftDoorPlan):
    def __init__(self, angle_start, angle_end=np.pi/4, duration=10.0):
        OpenLeftDoorPlan.__init__(
            self,
            angle_start=angle_start,
            angle_end=angle_end,
            duration=duration,
            type=PlanTypes[4])

    def CalcPositionCommand(self, t_plan, q_iiwa, Jv_WL7q, p_HrQ, control_period):
        return q_iiwa

    def CalcTorqueCommand(self):
        return np.zeros(7)


class OpenLeftDoorImpedancePlan(OpenLeftDoorPlan):
    def __init__(self, angle_start, angle_end=np.pi/4, duration=10.0):
        OpenLeftDoorPlan.__init__(
            self,
            angle_start=angle_start,
            angle_end=angle_end,
            duration=duration,
            type=PlanTypes[5])

    def CalcPositionCommand(self, q_iiwa):
            return q_iiwa

    def CalcTorqueCommand(self, t_plan, Jv_WL7q, p_HrQ):
        return np.zeros(7)


