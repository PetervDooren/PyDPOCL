
(define (problem test-0-problem)
  (:domain manipulation)
  (:objects robot_0 robot_1  - robot
            box_0 box_1 box_2 box_3  - physical_item
            table - area
            goal_0  - area
            reach_robot_0 reach_robot_1  - area
            )
  (:init )
  (:goal (and
              (within box_0 goal_0)
              )))
    