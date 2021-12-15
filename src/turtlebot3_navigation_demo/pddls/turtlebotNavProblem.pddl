(define (problem turtlebotNav)
    (:domain turtlebotNavDomain)
    (:objects
        v - robot
        wp0 wp1 wp2 wp3 - waypoint
    )

    (:init
        ;todo: put the initial state's facts and numeric values here
        (robot_at wp0 v)
        (docked v)
        (dock_at wp0)
    )

    (:goal
        (and
            ;todo: put the goal condition here
            (photo_taken wp1)
            (photo_taken wp2)
            (photo_taken wp3)
            (docked v)
        )
    )

    ;un-comment the following line if metric is needed
    ;(:metric minimize (???))
)