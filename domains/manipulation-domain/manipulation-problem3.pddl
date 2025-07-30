(define (problem manipulation-problem)
  (:domain manipulation)
  (:objects left_panda right_panda - robot
            boxa boxb - physical_item
            goal_a goal_b table reach_left_panda reach_right_panda - area
            )
  (:init )
  (:goal (and
              (within boxa goal_a)
              (within boxb goal_b)
              )))