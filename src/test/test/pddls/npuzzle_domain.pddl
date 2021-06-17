;Header and description

(define (domain npuzzle_domain)

    ;remove requirements that are not needed
    (:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

    (:predicates ;todo: define predicates here
        (tile ?x)
        (position ?x)
        (at ?t ?x ?y)
        (blank ?x ?y)
        (inc ?p ?pp)
        (dec ?p ?pp)
    )

    ;define actions here
    (:action move-up
        :parameters (?t ?px ?py ?by)
        :precondition (and
            (tile ?t) (position ?px) (position ?py) (position ?by)
            (dec ?by ?py) (blank ?px ?by) (at ?t ?px ?py)
        )
        :effect (and (not (blank ?px ?by)) (not (at ?t ?px ?py))
            (blank ?px ?py) (at ?t ?px ?by))
    )

    (:action move-down
        :parameters (?t ?px ?py ?by)
        :precondition (and
            (tile ?t) (position ?px) (position ?py) (position ?by)
            (inc ?by ?py) (blank ?px ?by) (at ?t ?px ?py))
        :effect (and (not (blank ?px ?by)) (not (at ?t ?px ?py))
            (blank ?px ?py) (at ?t ?px ?by))
    )

    (:action move-left
        :parameters (?t ?px ?py ?bx)
        :precondition (and
            (tile ?t) (position ?px) (position ?py) (position ?bx)
            (dec ?bx ?px) (blank ?bx ?py) (at ?t ?px ?py))
        :effect (and (not (blank ?bx ?py)) (not (at ?t ?px ?py))
            (blank ?px ?py) (at ?t ?bx ?py))
    )

    (:action move-right
        :parameters (?t ?px ?py ?bx)
        :precondition (and
            (tile ?t) (position ?px) (position ?py) (position ?bx)
            (inc ?bx ?px) (blank ?bx ?py) (at ?t ?px ?py))
        :effect (and (not (blank ?bx ?py)) (not (at ?t ?px ?py))
            (blank ?px ?py) (at ?t ?bx ?py))
    )

)