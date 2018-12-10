import numpy as np

from pydrake.all import (AutoDiffXd, MathematicalProgram, SolutionResult)

from utils import set_joint_positions

# How do I do collision constraints?
# Can I have the most recent lab_2 pddl (and what drake binary does it use)?

def get_signed_dist_fn(diagram, diagram_context, plant, scene_graph,
                     joints, collision_pairs=set(), attachments=[]):
    # TODO: self-collisions
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    nc = len(collision_pairs)

    def fn(q):
        if not collision_pairs:
            return False
        if q.dtype == np.object:
            tol = 0.001
            q_val = np.array([j.value() for j in q])
            collision_val = fn(q_val)
            diffs = np.zeros((nc, len(q)))
            for ii in range(len(q)):
                q_plus = q_val
                q_plus[ii] += tol
                q_minus = q_val
                q_minus[ii] -= tol
                diffs[:,ii] = (fn(q_plus) - fn(q_minus))/(2*tol)

            collision_dist_ad = np.ndarray(nc, dtype=AutoDiffXd)
            for ii in range(nc):
                collision_dist_ad[ii] = AutoDiffXd(collision_val[ii], diffs[ii])
            return collision_dist_ad
        else:
            set_joint_positions(joints, plant_context, q)
            for attachment in attachments:
                attachment.assign(plant_context)
            sg_context = diagram.GetMutableSubsystemContext(scene_graph, diagram_context)
            query_object = scene_graph.get_query_output_port().Eval(sg_context)
            inspector = query_object.inspector()
            signed_dist_pair = query_object.ComputeSignedDistancePairwiseClosestPoints()

            collision_dist = 1e4*np.ones(nc)
            ii = 0
            for pair in collision_pairs:
                for dist_pair in signed_dist_pair:
                    bodyA = plant.GetBodyFromFrameId(inspector.GetFrameId(dist_pair.id_A))
                    bodyB = plant.GetBodyFromFrameId(inspector.GetFrameId(dist_pair.id_B))
                    if (bodyA == pair[0] and bodyB == pair[1]) or (bodyA == pair[1] and bodyB == pair[0]):
                        collision_dist[ii] = dist_pair.distance
                        break
                ii += 1
            return collision_dist
    return fn

def optimal_trajectory(joints, start_position, end_position, signed_dist_fn,
                       nc, time=10, knot_points=100):
    assert len(joints) == len(start_position)
    assert len(joints) == len(end_position)

    h = time/(knot_points-1)
    nq = len(joints)
    prog = MathematicalProgram()
    q_var = []
    v_var = []
    for ii in range(knot_points):
        q_var.append(prog.NewContinuousVariables(nq, "q[" + str(ii) + "]"))
        v_var.append(prog.NewContinuousVariables(nq, "v[" + str(ii) + "]"))

    # ---------------------------------------------------------------
    # Initial & Final Pose Constraints ------------------------------
    x_i = np.append(start_position, np.zeros(nq))
    x_i_vars = np.append(q_var[0], v_var[0])
    prog.AddLinearEqualityConstraint(np.eye(2*nq), x_i, x_i_vars)
    tol = 0.01 * np.ones(2*nq)
    x_f = np.append(end_position, np.zeros(nq))
    x_f_vars = np.append(q_var[-1], v_var[-1])
    prog.AddBoundingBoxConstraint(x_f - tol, x_f + tol, x_f_vars)

    # ---------------------------------------------------------------
    # Dynamics Constraints ------------------------------------------
    for ii in range(knot_points-1):
        dyn_con1 = np.hstack((np.eye(nq), np.eye(nq), -np.eye(nq)))
        dyn_var1 = np.concatenate((q_var[ii], v_var[ii], q_var[ii+1]))
        prog.AddLinearEqualityConstraint(dyn_con1, np.zeros(nq), dyn_var1)

    # ---------------------------------------------------------------
    # Joint Limit Constraints ---------------------------------------
    q_min = np.array([j.lower_limits() for j in joints])
    q_max = np.array([j.upper_limits() for j in joints])
    for ii in range(knot_points):
        prog.AddBoundingBoxConstraint(q_min, q_max, q_var[ii])

    # ---------------------------------------------------------------
    # Collision Constraints -----------------------------------------
    for ii in range(knot_points):
        prog.AddConstraint(signed_dist_fn, np.zeros(nc), 1e8*np.ones(nc),
                           q_var[ii])

    # ---------------------------------------------------------------
    # Dynamics Constraints ------------------------------------------
    for ii in range(knot_points):
        prog.AddQuadraticErrorCost(np.eye(nq), np.zeros(nq), v_var[ii])

    xi = np.array(start_position)
    xf = np.array(end_position)
    for ii in range(knot_points):
        prog.SetInitialGuess(q_var[ii], ii*(xf - xi)/(knot_points-1) + xi)
        prog.SetInitialGuess(v_var[ii], np.zeros(nq))

    result = prog.Solve()
    print prog.GetSolverId().name()
    if result != SolutionResult.kSolutionFound:
        print result
        return None
    q_path = []
    # print signed_dist_fn(prog.GetSolution(q_var[0]))
    for ii in range(knot_points):
        q_path.append(prog.GetSolution(q_var[ii]))
    return q_path
