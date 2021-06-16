(define (problem task)
(:domain turtlebot_demo)
(:objects
    wp0 wp1 wp2 wp3 wp4 wp5 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp0)


    (= (distance wp0 wp1) 2)
    (= (distance wp0 wp2) 3.91705)
    (= (distance wp0 wp3) 2.24695)
    (= (distance wp0 wp4) 0.688968)
    (= (distance wp1 wp0) 2)
    (= (distance wp1 wp2) 2)
    (= (distance wp1 wp3) 2)
    (= (distance wp1 wp4) 2.38701)
    (= (distance wp1 wp5) 3.55125)
    (= (distance wp2 wp0) 3.91705)
    (= (distance wp2 wp1) 2)
    (= (distance wp2 wp3) 2.78541)
    (= (distance wp2 wp4) 4.14535)
    (= (distance wp2 wp5) 3.85261)
    (= (distance wp3 wp0) 2.24695)
    (= (distance wp3 wp1) 2)
    (= (distance wp3 wp2) 2.78541)
    (= (distance wp3 wp4) 2)
    (= (distance wp3 wp5) 1.57162)
    (= (distance wp4 wp0) 0.688968)
    (= (distance wp4 wp1) 2.38701)
    (= (distance wp4 wp2) 4.14535)
    (= (distance wp4 wp3) 2)
    (= (distance wp5 wp1) 3.55125)
    (= (distance wp5 wp2) 3.85261)
    (= (distance wp5 wp3) 1.57162)

)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (visited wp5)
))
)
