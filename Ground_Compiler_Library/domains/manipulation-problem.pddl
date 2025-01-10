(define (problem manipulation-problem)
  (:domain manipulation)
  (:objects leftpanda rightpanda - robot
            boxa boxb boxc - item
            loca locb locc locd loce - area
            )
  (:init (within boxa locd)
         (within boxb locc)
         (within boxc loca)
         (free locb)
         (free loce)
         (in_reach loca leftpanda)
         (in_reach locb leftpanda)
         (in_reach locc leftpanda)
         (in_reach locc rightpanda)
         (in_reach locd rightpanda)
         (in_reach loce rightpanda)
         )
  (:goal (and
              (within boxa loca)
              )))