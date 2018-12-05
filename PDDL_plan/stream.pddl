(define (stream manipulation-station)

  (:stream sample-grasp
    :inputs (?obj)
    :domain (Graspable ?obj)
    :outputs (?grasp)
    :certified (Grasp ?obj ?grasp)
  )

  (:stream plan-ik
    :inputs (?robot ?obj ?pose ?grasp)
    :domain (and (Robot ?robot) (Grasp ?obj ?grasp) (Pose ?obj ?pose))
    :outputs (?conf ?traj)
    :certified (and (Conf ?robot ?conf) (Traj ?traj)
                    (Kin ?robot ?obj ?pose ?grasp ?conf ?traj))
  )

  (:stream plan-pull
    ;;;;; BEGIN YOUR CODE HERE ;;;;;
    :inputs (?robot ?door ?door_conf1 ?door_conf2)
    :domain (and (Robot ?robot) (Door ?door)
                 (Conf ?door ?door_conf1) (Conf ?door ?door_conf2))
    :outputs (?robot_conf1 ?robot_conf2 ?traj)
    :certified (and (Traj ?traj) (Conf ?robot ?robot_conf1)
                    (Conf ?robot ?robot_conf2) (Pull ?robot ?robot_conf1
                        ?robot_conf2 ?door ?door_conf1 ?door_conf2 ?traj))
    ;;;;; END YOUR CODE HERE ;;;;;
  )

  (:stream plan-motion
    :inputs (?robot ?conf1 ?conf2)
    :domain (and (Conf ?robot ?conf1) (Conf ?robot ?conf2) (Robot ?robot))
    ; Advanced feature that considers a subset of the fluent state as an input
    :fluents (AtConf AtPose AtGrasp)
    :outputs (?traj)
    :certified (Motion ?robot ?conf1 ?conf2 ?traj)
  )

  ; Boolean functions (i.e. predicates) that are similar to test streams
  (:predicate (TrajPoseCollision ?traj ?obj ?pose)
    (and (Traj ?traj) (Pose ?obj ?pose))
  )

  (:predicate (TrajConfCollision ?traj ?d ?conf)
    (and (Traj ?traj) (Conf ?d ?conf))
  )
)