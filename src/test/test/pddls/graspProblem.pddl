(define (problem task)
(:domain graspdomain)
(:objects
    c - cup
    d - desk
)
(:init

    (not_inhand c)

    (ondesk d c)



)
(:goal (and
    (inhand c)
))
)
