(define (problem test-problem)
  (:domain test)
  (:objects leftpanda rightpanda - robot
            boxa boxb boxc - item
            goal_left goal_right table reach_robot_left reach_robot_right - area
            )
  (:init )
  (:goal (and
              (within boxa goal_left)
              )))