(define (domain manipulation)
  (:requirements)
  (:types symbol - object
          item robot discarea - symbol)
  (:predicates (free ?area - discarea)
               (in_reach ?area - discarea ?robot - robot)
               (within ?obj - item ?area - discarea))


  (:action movemono
    :parameters   (?robot - robot ?obj - item ?startarea - discarea ?goalarea - discarea)
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
  ;;  :parameters   (?leftrobot - robot ?rightrobot - robot ?obj - item ?area - discarea)
;;	:precondition (and (free ?area)
;;                       (in_reach ?obj ?leftrobot)
;;                       (in_reach ?obj ?rightrobot))
;;	:effect       (and (within ?obj ?area))
 ;;   :agents       (?leftrobot ?rightrobot))

)