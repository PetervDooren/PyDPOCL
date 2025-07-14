(define (domain test)
  (:requirements)
  (:types symbol area - object
          physical_item robot - symbol)
  (:predicates (within ?obj - physical_item ?area - area)
               (in_reach ?area - area ?robot - robot)
  )


  (:action movemono
    :parameters   (?robot - robot ?obj - physical_item ?startarea - area ?goalarea - area)
	:precondition (and
	                 (within ?obj ?startarea)
                   (in_reach ?startarea ?robot)
                   (in_reach ?goalarea ?robot)
                )
	:effect       (and (within ?obj ?goalarea)
                        (not (within ?obj ?startarea))
                )
    :agents       (?robot))
)