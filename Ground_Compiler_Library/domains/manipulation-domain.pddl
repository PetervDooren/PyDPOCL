(define (domain manipulation)
  (:requirements)
  (:types symbol area - object
          item robot - symbol)
  (:predicates (within ?obj - item ?area - area))


  (:action movemono
    :parameters   (?robot - robot ?obj - item ?startarea - area ?goalarea - area)
	:precondition (and
	                 (within ?obj ?startarea)
                )
	:effect       (and (within ?obj ?goalarea)
                        (not (within ?obj ?startarea))
                )
    :agents       (?robot))


  ;;(:action bimanualmove
  ;;  :parameters   (?leftrobot - robot ?rightrobot - robot ?obj - item ?area - area)
;;	:precondition (and (free ?area)
;;                       (in_reach ?obj ?leftrobot)
;;                       (in_reach ?obj ?rightrobot))
;;	:effect       (and (within ?obj ?area))
 ;;   :agents       (?leftrobot ?rightrobot))

)