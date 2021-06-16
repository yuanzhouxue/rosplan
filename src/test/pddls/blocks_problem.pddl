(define (problem blocks-domiain_task)
    (:domain blocks_domain)
    (:objects
        a b c d t1 t2 t3 t4 - block_t
    )
    (:init
        (clear b)
        (clear t3)
        (clear t2)
        (clear c)
        (emptyhand)
        (on a t1)
        (on b a)
        (on d t4)
        (on c d)
    )
    (:goal
        (and
            (on d t2)
            (on a d)
            (on c a)
            (on b t4)
        )
    )
)