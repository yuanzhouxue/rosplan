(define (problem turtlebotNav)
    (:domain turtlebotNavDomain)
    (:objects
        v - robot
        wp0 wp1 wp2 wp3 - waypoint
    )

    (:init
        ;todo: put the initial state's facts and numeric values here
        (robot_at wp0 v)
        ; (visited wp0)
        ; (visited wp0)
        ; (= (time) 0)
        ; (docked v)
        ; (dock_at wp0)
        (= (distance wp0 wp1) 1)
        (= (distance wp0 wp2) 1)
        (= (distance wp0 wp3) 1.414)
        (= (distance wp1 wp0) 1)
        (= (distance wp1 wp2) 1.414)
        (= (distance wp1 wp3) 1)
        (= (distance wp2 wp0) 1)
        (= (distance wp2 wp3) 1)
        (= (distance wp2 wp1) 1.414)
        (= (distance wp3 wp0) 1.414)
        (= (distance wp3 wp1) 1)
        (= (distance wp3 wp2) 1)
        (connected wp0 wp1)
        (connected wp0 wp2)
        (connected wp0 wp3)
        (connected wp1 wp0)
        (connected wp1 wp2)
        (connected wp1 wp3)
        (connected wp2 wp1)
        (connected wp2 wp2)
        (connected wp2 wp3)
        (connected wp3 wp0)
        (connected wp3 wp1)
        (connected wp3 wp2)
    )

    (:goal
        (and
            ;todo: put the goal condition here
            (photo_taken wp1)
            (photo_taken wp2)
            (photo_taken wp3)
            ; (visited wp0)
            ; (photo_taken wp0)
            ; (robot_at wp0 v)
            ; (docked v)
        )
    )

    ;un-comment the following line if metric is needed
    ; (:metric minimize (time))
)