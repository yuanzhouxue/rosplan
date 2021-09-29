(define (problem task)
(:domain graspdomain)
(:objects
    g - glass
    t - table
    r - robot
    wp0 - waypoint
)
(:init

    (on_table g t)
    (emptyhand r)



)
(:goal (and
    (in_hand g r)
    (robot_at wp0 r)
))
)
