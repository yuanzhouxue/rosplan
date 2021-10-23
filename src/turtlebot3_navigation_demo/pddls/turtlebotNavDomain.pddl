;Header and description

(define (domain turtlebotNavDomain)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot
    waypoint
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (robot_at ?wp - waypoint ?v - robot)
    (visited ?wp - waypoint)
)


(:functions ;todo: define numeric functions here
)

;define actions here
(:durative-action goto_waypoint
    :parameters (?from - waypoint ?to - waypoint ?v - robot)
    :duration (= ?duration 20)
    :condition (and 
        (at start (robot_at ?from ?v))
    )
    :effect (and 
        (at start (not (robot_at ?from ?v)))
        (at end (robot_at ?to ?v))
        (at end (visited ?to))
    )
)

)