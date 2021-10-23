(define (problem turtlebotNav) (:domain turtlebotNavDomain)
(:objects 
    v - robot
    wp0 - waypoint
    wp1 - waypoint
    wp2 - waypoint
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (robot_at wp0 v)

)

(:goal (and
    ;todo: put the goal condition here
    (visited wp1)
    (visited wp2)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
