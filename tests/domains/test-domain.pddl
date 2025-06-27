(define (domain test)
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
)