
(define (problem test-3-problem)
  (:domain manipulation)
  (:objects robot_0 robot_1  - robot
            box_0 box_1 box_2 box_3 box_4 box_5 box_6 box_7  - physical_item
            table - area
            goal_0 goal_1 goal_2 goal_3  - area
            reach_robot_0 reach_robot_1  - area
            )
  (:init )
  (:goal (and
              (within box_0 goal_0)
              (within box_1 goal_1)
              (within box_2 goal_2)
              (within box_3 goal_3)
              )))
    