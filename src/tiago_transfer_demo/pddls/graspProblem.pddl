(define (problem task)
    (:domain graspDomain)
    (:objects
        glass_cup - glass
        table0 table1 - table
        wp0 wp1 wp2 wp3 - waypoint
    )
    (:init
        (robot_at wp2)
        (emptyhand)
        (guest_not_near)
        (near wp0 table0)
        (near wp1 table1)
        (on_table glass_cup table0)
    )
    (:goal
        (and
            (on_table glass_cup table1)
            (robot_at wp2)
        )
    )
)