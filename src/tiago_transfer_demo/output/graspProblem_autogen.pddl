(define (problem task)
(:domain graspdomain)
(:objects
    glass_cup - glass
    table0 table1 - table
    wp0 wp1 wp2 - waypoint
)
(:init
    (in_hand glass_cup)



    (robot_at wp0)

    (near wp0 table0)
    (near wp1 table1)

    (guest_not_near)

)
(:goal (and
    (on_table glass_cup table1)
))
)
