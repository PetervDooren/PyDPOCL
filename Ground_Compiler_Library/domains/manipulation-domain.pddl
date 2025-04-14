(define (domain manipulation)
  (:requirements)
  (:types symbol area - object
          item robot - symbol)
  (:predicates (free ?area - area)
               (in_reach ?area - area ?robot - robot)
               (within ?obj - item ?area - area))


  (:action movemono
    :parameters   (?robot - robot ?obj - item ?startarea - area ?goalarea - area)
	:precondition (and
	                   (free ?goalarea)
                       (within ?obj ?startarea)
                       (in_reach ?startarea ?robot)
                       (in_reach ?goalarea ?robot)
                       )
	:effect       (and (within ?obj ?goalarea)
                        (free ?startarea)
                        (not (within ?obj ?startarea))
                        (not (free ?goalarea)))
    :agents       (?robot))


  ;;(:action bimanualmove
  ;;  :parameters   (?leftrobot - robot ?rightrobot - robot ?obj - item ?area - area)
;;	:precondition (and (free ?area)
;;                       (in_reach ?obj ?leftrobot)
;;                       (in_reach ?obj ?rightrobot))
;;	:effect       (and (within ?obj ?area))
 ;;   :agents       (?leftrobot ?rightrobot))

)