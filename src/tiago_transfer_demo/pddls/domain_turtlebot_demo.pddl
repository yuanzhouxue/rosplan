(define (domain turtlebot_demo)

	(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

	(:types
		waypoint robot landmark
	)

	(:predicates
		(robot_at ?v - robot ?wp - waypoint)
		(connected ?from ?to - waypoint)
		(visited ?wp - waypoint)
        (landmark_at ?l - landmark ?wp - waypoint)
	)

	(:functions
		(distance ?wp1 ?wp2 - waypoint)
	)

	;; Move between any two waypoints, avoiding terrain
	(:durative-action goto_waypoint
		:parameters (?v - robot ?from ?to - waypoint ?l - landmark)
		; :duration ( = ?duration (distance ?from ?to))
		:duration (= ?duration 10)
		; 添加了动作代价，语法错误
		; :action-costs ( = (distance ?from ?to))
		:condition (and
			(at start (robot_at ?v ?from))
			(over all (connected ?from ?to))
		)
		:effect (and
			(at end (visited ?l))
			(at start (not (robot_at ?v ?from)))
			(at end (robot_at ?v ?to))
		)
	)
)