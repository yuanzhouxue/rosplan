;Header and description

(define (domain graspDomain)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    cup
    desk
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (inhand ?c - cup)
    (not_inhand ?c - cup)
    (ondesk ?d - desk ?c - cup)
    (observe)
    (finish_observe)
)


(:functions ;todo: define numeric functions here
)

;define actions here
(:durative-action grasp
    :parameters (?c - cup ?d - desk)
    :duration (= ?duration 60)
    :condition (and 
        ; (at start (finish_observe))
        (at start (not_inhand ?c))
        (at start (ondesk ?d ?c))
        
    )
    :effect (and 
        (at start (not (finish_observe)))
        (at end (observe))

    )
)

(:durative-action observing
    :parameters (?c - cup)
    :duration (= ?duration 60)
    :condition (and 
        (at start (observe))
        
    )
    :effect (and 
        (at end (not (observe)))
        (at end (finish_observe))
    )
)

(:durative-action then_action
    :parameters (?c - cup ?d - desk)
    :duration (= ?duration 60)
    :condition (and 
        (at start (finish_observe))
    )
    :effect (and 
        (at end (inhand ?c))
        (at end (not (not_inhand ?c)))
        (at end (not (ondesk ?d ?c)))
        (at start (not (finish_observe)))
    )
)

)