;Header and description

(define (domain turtlebotNavDomain)

    ;remove requirements that are not needed
    (:requirements :durative-actions :typing :numeric-fluents)

    (:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
        robot waypoint
    )

    ; un-comment following line if constants are needed
    ;(:constants )

    (:predicates ;todo: define predicates here
        (robot_at ?wp - waypoint ?v - robot)
        (visited ?wp - waypoint)
        ; check whether the photo exists
        (photo_taken ?wp - waypoint)
        (connected ?wp1 ?wp2 - waypoint)

        ; ; need to be checked
        ; (docked ?v - robot)
        ; (undocked ?v - robot)
        ; (dock_at ?wp - waypoint)
    )

    (:functions ;todo: define numeric functions here
        ; (time)
        (distance ?wp1 ?wp2 - waypoint)
    )

    ;define actions here
    (:durative-action goto_waypoint
        :parameters (?from ?to - waypoint ?v - robot)
        :duration (= ?duration (distance ?from ?to))
        :condition (and
            ; (over all (undocked ?v))
            (at start (robot_at ?from ?v))
            (over all (connected ?from ?to))
        )
        :effect (and
            (at start (not (robot_at ?from ?v)))
            (at end (robot_at ?to ?v))
            (at end (visited ?to))
            ; (at start (increase (time) 20))
        )
    )

    (:durative-action take_photo
        :parameters (?v - robot ?wp - waypoint)
        :duration (= ?duration 1)
        :condition (and
            (at start (and 
                ; (undocked ?v)
                (robot_at ?wp ?v)
            ))
        )
        :effect (and 
            (at end (and 
                (photo_taken ?wp)
                ; (increase (time) 1)
            ))
        )
    )
    

    ; (:durative-action dock
    ;     :parameters (?v - robot ?wp - waypoint)
    ;     :duration (= ?duration 10)
    ;     :condition (and
    ;         (over all (and
    ;             (dock_at ?wp)
    ;         ))
    ;         (at start (and 
    ;             (robot_at ?wp ?v)
    ;         ))
    ;     )
    ;     :effect (and
    ;         (at end (and
    ;             (docked ?v)
    ;             (not (undocked ?v))
    ;         ))
    ;     )
    ; )

    ; (:durative-action undock
    ;     :parameters (?v - robot ?wp - waypoint)
    ;     :duration (= ?duration 5)
    ;     :condition (and
    ;         (over all (and
    ;             (docked ?v)
    ;             (dock_at ?wp)
    ;             (robot_at ?wp ?v)
    ;         ))
            
    ;     )
    ;     :effect (and
    ;         (at end (and
    ;             (undocked ?v)
    ;             (not (docked ?v))
    ;         ))
    ;     )
    ; )
    

)