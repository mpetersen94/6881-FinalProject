import numpy as np

from pydrake.all import (MathematicalProgram, SolutionResult)

def optimal_trajectory(joints, start_position, end_position, time=10, knot_points=100):
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
    # Collision Constraints ---------------------------------------

    # ---------------------------------------------------------------
    # Dynamics Constraints ------------------------------------------
    for ii in range(knot_points):
        prog.AddQuadraticErrorCost(np.eye(nq), np.zeros(nq), v_var[ii])

    result = prog.Solve()
    if result != SolutionResult.kSolutionFound:
        print result
        return None
    q_path = []
    for ii in range(knot_points):
        q_path.append(prog.GetSolution(q_var[ii]))
    return q_path
