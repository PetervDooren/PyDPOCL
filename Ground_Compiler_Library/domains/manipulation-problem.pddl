(define (problem manipulation-problem)
  (:domain manipulation)
  (:objects leftpanda rightpanda - robot
            boxa boxb boxc - item
            goal_left goal_right table reach_robot_left reach_robot_right - area
            )
  (:init )
  (:goal (and
              (within boxc goal_left)
              )))