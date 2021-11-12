(define (problem task)
(:domain turtlebotnavdomain)
(:objects
    v - robot
    wp0 wp1 wp2 - waypoint
)
(:init
    (robot_at wp0 v)


)
(:goal (and
    (visited wp1)
    (visited wp2)
))
)
