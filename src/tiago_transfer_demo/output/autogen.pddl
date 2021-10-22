(define (problem task)
(:domain graspdomain)
(:objects
    r - robot
    g - glass
    t - table
    wp0 wp1 - waypoint
)
(:init

    (emptyhand r)

    (on_table g t)

    (robot_at wp0 r)

)
(:goal (and
    (in_hand g r)
    (robot_at wp1 r)
))
)
