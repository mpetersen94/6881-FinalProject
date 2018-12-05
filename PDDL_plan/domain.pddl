(define (domain manipulation-station)
  (:requirements :strips :equality)
  (:predicates

    ; Model static predicates
    (Robot ?robot)
    (Graspable ?obj)
    (Door ?door)

    ; "Type" static predicates
    (Conf ?robot ?conf)
    (Pose ?obj ?pose)
    (Grasp ?obj ?grasp)
    (Traj ?traj)

    ; Stream-certified static predicates
    (Motion ?robot ?conf1 ?conf2 ?traj)
    (Kin ?robot ?obj ?pose ?grasp ?conf ?traj)
    (Pull ?robot ?robot_conf1 ?robot_conf2 ?door ?door_conf1 ?door_conf2 ?traj)

    ; Collision static predicates
    (TrajPoseCollision ?traj ?obj ?pose)
    (TrajConfCollision ?traj ?door ?conf)

    ; Fluents predicates
    (AtPose ?obj ?pose)
    (AtGrasp ?robot ?obj ?grasp)
    (AtConf ?robot ?conf)
    (HandEmpty ?robot)
    (CanMove ?robot)

    ; Derived predicates (also called axioms)
    (UnsafeTraj ?traj)
  )

  ; General movement action
  (:action move
    :parameters (?robot ?conf1 ?conf2 ?traj)
    :precondition (and (Motion ?robot ?conf1 ?conf2 ?traj)
                       (AtConf ?robot ?conf1) (CanMove ?robot))
    :effect (and (AtConf ?robot ?conf2)
                 (not (AtConf ?robot ?conf1)) (not (CanMove ?robot)))
  )

  ; Manipulation actions
  (:action pick
    :parameters (?robot ?obj ?pose ?grasp ?conf ?traj)
    :precondition (and (Kin ?robot ?obj ?pose ?grasp ?conf ?traj)
                       (AtPose ?obj ?pose) (HandEmpty ?robot) (AtConf ?robot ?conf)
                       (not (UnsafeTraj ?traj)))
    :effect (and (AtGrasp ?robot ?obj ?grasp) (CanMove ?robot)
                 (not (AtPose ?obj ?pose)) (not (HandEmpty ?robot)))
  )

  (:action place
    :parameters (?robot ?obj ?pose ?grasp ?conf ?traj)
    :precondition (and (Kin ?robot ?obj ?pose ?grasp ?conf ?traj)
                       (AtGrasp ?robot ?obj ?grasp) (AtConf ?robot ?conf)
                       (not (UnsafeTraj ?traj)))
    :effect (and (AtPose ?obj ?pose) (HandEmpty ?robot) (CanMove ?robot)
                 (not (AtGrasp ?robot ?obj ?grasp)))
  )

  (:action pull
    :parameters (?robot ?robot_conf1 ?robot_conf2 ?door ?door_conf1 ?door_conf2 ?traj)
    ;;;;; BEGIN YOUR CODE HERE ;;;;;
    :precondition (and (AtConf ?robot ?robot_conf1) (HandEmpty ?robot)
                       (AtConf ?door ?door_conf1)
                       (Pull ?robot ?robot_conf1 ?robot_conf2 ?door ?door_conf1
                          ?door_conf2 ?traj)
                       (not (UnsafeTraj ?traj)))
    :effect (and (AtConf ?robot ?robot_conf2) (not (AtConf ?robot ?robot_conf1))
                 (AtConf ?door ?door_conf2) (not (AtConf ?door ?door_conf1))
                 (CanMove ?robot))
    ;;;;; END YOUR CODE HERE ;;;;;
  )

  (:derived (UnsafeTraj ?traj)
    (or
      (exists (?obj ?pose) (and (TrajPoseCollision ?traj ?obj ?pose) (Traj ?traj) (Pose ?obj ?pose)
                                (AtPose ?obj ?pose)))
      (exists (?door ?conf) (and (TrajConfCollision ?traj ?door ?conf) (Traj ?traj) (Conf ?door ?conf)
                                 (AtConf ?door ?conf)))
    )
  )
)