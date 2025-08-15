(define (domain manipulation)
  (:requirements)
  (:types symbol area path - object
          physical_item robot - symbol)
  (:predicates (within ?obj - physical_item ?area - area)
               (in_reach ?area - area ?robot - robot)
               (is_path ?path - path ?startarea - area ?goalarea - area)
  )


  (:action movemono
    :parameters   (?robot - robot ?obj - physical_item ?startarea - area ?goalarea - area ?path - path)
	:precondition (and
	                 (within ?obj ?startarea)
                   (in_reach ?startarea ?robot)
                   (in_reach ?goalarea ?robot)
                   (is_path ?path ?startarea ?goalarea)
                )
	:effect       (and (within ?obj ?goalarea)
                        (not (within ?obj ?startarea))
                )
    :agents       (?robot))


  ;;(:action bimanualmove
  ;;  :parameters   (?leftrobot - robot ?rightrobot - robot ?obj - physical_item ?area - area)
;;	:precondition (and (free ?area)
;;                       (in_reach ?obj ?leftrobot)
;;                       (in_reach ?obj ?rightrobot))
;;	:effect       (and (within ?obj ?area))
 ;;   :agents       (?leftrobot ?rightrobot))

)