(define (problem manipulation-problem)
  (:domain manipulation)
  (:objects left_panda right_panda - robot
            boxa boxb boxc - item
            goal_left goal_right table reach_left_panda reach_right_panda - area
            )
  (:init )
  (:goal (and
              (within boxc goal_left)
              )))