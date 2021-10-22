(define (problem task)
(:domain graspDomain)
(:objects
    g - glass
    t - table
    r - robot
    wp0 wp1 - waypoint
)
(:init

    (on_table g t)
    (emptyhand r)
    (robot_at wp0 r)



)
(:goal (and
    (in_hand g r)
    (robot_at wp1 r)
))
)
