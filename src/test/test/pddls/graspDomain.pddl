;Header and description

(define (domain graspDomain)

    ;remove requirements that are not needed
    (:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

    (:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
        robot glass table waypoint
    )

    ; un-comment following line if constants are needed
    ;(:constants )

    (:predicates ;todo: define predicates here
        (in_hand ?g - glass ?r - robot)
        (emptyhand ?r - robot)
        (on_table ?g - glass ?t - table)
        (robot_at ?wp - waypoint ?r - robot)
    )

    (:functions ;todo: define numeric functions here
    )

    ;define actions here
    (:durative-action grasp
        :parameters (?g - glass ?t - table ?r - robot)
        :duration (= ?duration 60)
        :condition (and
            (at start (on_table ?g ?t))
            (at start (emptyhand ?r))
        )
        :effect (and
            (at end (in_hand ?g ?r))
            (at end (not (emptyhand ?r)))
            (at end (not (on_table ?g ?t)))
        )
    )

    (:durative-action goto_waypoint
        :parameters (?from ?to - waypoint ?g - glass ?r - robot)
        :duration (= ?duration 20)
        :condition (and
            (over all (in_hand ?g ?r))
            ; (at start (robot_at ?from ?r))
        )
        :effect (and
            ; (at start (not (robot_at ?from ?r)))
            (at end (robot_at ?to ?r))
        )
    )
    (:durative-action goto_waypoint_without_glass
        :parameters (?v - robot ?from ?to - waypoint ?l - landmark)
        ; :duration ( = ?duration (distance ?from ?to))
        :duration (= ?duration 10)
        :condition (and
            (at start (robot_at ?from ?v))
            ; (over all (connected ?from ?to))
        )
        :effect (and
            ; (at end (visited ?l))
            (at start (not (robot_at ?from ?v)))
            (at end (robot_at ?to ?v))
        )
    )
)