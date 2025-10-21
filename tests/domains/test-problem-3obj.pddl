(define (problem test-problem)
  (:domain test)
  (:objects robot_0 robot_1  - robot
            box_0 box_1 box_2 box_3 box_4 - physical_item
            table - area
            goal_0 goal_1 goal_2 - area
            reach_robot_0 reach_robot_1  - area
            )
  (:init )
  (:goal (and
              (within box_0 goal_0)
              (within box_1 goal_1)
              (within box_2 goal_2)
              )))