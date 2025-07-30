(define (domain healthcare_facility_4_base)
    (:requirements :strips :typing :equality :durative-actions)

    (:types
        location carrier - object
        med_unit robot box supply patient - locatable
        worker_bot helper_bot - robot
    )

    (:predicates
        ; location related
        (at ?obj - locatable ?loc - location) ; locatable is at a location
        (linked ?loc1 - location ?loc2 - location) ; two locations are linked (roadmap)
        
        ; General Robot related
        (bot_is_free ?bot - robot) ; bot is free (not carrying a box or helping a patient)
        (bot_is_unlocked ?bot - robot) ; predicate for temporary lockout of a bot (e.g., during a task)

        ; Worker bots
        (bot_owns_carrier ?bot - robot ?carr - carrier) ; a bot owns a carrier
        
        ; Helper bots
        (helping_patient ?bot - helper_bot ?pat - patient) ; helper bot is helping the patient

        ; Carrier related
        (carrying_box ?carr - carrier ?obj - box) ; carrier has the box loaded
        (carrier_can_1 ?carr - carrier)  ; carrier can hold box 1
        (carrier_can_2 ?carr - carrier)   ; carrier can hold box 2   
        (carrier_can_3 ?carr - carrier)  ; carrier can hold box 3 (maximum)
        ; Carrier capacity settings
        (carrier_empty ?carr - carrier) ; carrier has 0 boxes
        (carrier_has_1 ?carr - carrier); carrier has 1 box
        (carrier_has_2 ?carr - carrier) ; carrier has 2 boxes
        (carrier_has_3 ?carr - carrier) ; carrier has 3 boxes (maximum)
        
        ; Box related
        (box_is_empty ?box - box) ; box is empty
        (box_on_ground ?box - box) ; box is on the ground (not being carried by a bot)

        ; Patient related
        (patient_on_ground ?pat - patient) ; patient is on the ground (not being helped)

        ; Supply related
        (supply_in_box ?sup - supply ?box - box) ; supply is in a box
        
        ; Goal Oriented
        (needs_supply ?mu - med_unit ?sup - supply) ; medical unit needs a supply
        (needs_patient ?mu - med_unit ?pat - patient) ; medical unit needs a patient so he they can treat them
        (has_supply ?mu - med_unit ?sup - supply) ; medical unit has a supply
        (has_patient ?mu - med_unit ?pat - patient) ; medical unit has a patient
    )

    ;;
    ;; Move Actions
    ;;
    
    (:durative-action move_bot_alone
        :parameters (?bot - robot ?from - location ?to - location)
        :duration (= ?duration 3)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check
            
            ; location consistency
            (at start (at ?bot ?from))
            (over all (linked ?from ?to))
            ; the bot is free (carrier is empty / not helping a patient)
            (over all (bot_is_free ?bot))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)
            
            ; change location (bot)
            (at start (not (at ?bot ?from)))
            (at start (at ?bot ?to)) ; note: for movement, location is updated instantly.

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )
    
    (:durative-action move_worker_bot_with_1_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?from - location ?to - location)
        :duration (= ?duration 5)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (at start (at ?bot ?from))
            (over all (linked ?from ?to))
            ; carrier checks
            (over all (bot_owns_carrier ?bot ?carr))
            (over all (carrier_has_1 ?carr)) ; the bot carries a single box
            (over all (carrying_box ?carr ?box)) ; carrying one box
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; update location (bot)
            (at start (not (at ?bot ?from)))
            (at start (at ?bot ?to))
            ; update location (box)
            (at start (not (at ?box ?from)))
            (at start (at ?box ?to))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )
    
    (:durative-action move_worker_bot_with_2_boxes
        :parameters (?bot - worker_bot ?carr - carrier ?box1 - box ?box2 - box ?from - location ?to - location)
        :duration (= ?duration 5)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (at start (at ?bot ?from))
            (over all (linked ?from ?to))
            ; carrier is owned by the bot and has capacity for 2 boxes
            (over all (bot_owns_carrier ?bot ?carr))
            (over all (carrier_has_2 ?carr))
            ; both boxes are on the carrier
            (over all (carrying_box ?carr ?box1))
            (over all (carrying_box ?carr ?box2))
            ; boxes must be different
            (over all (not (= ?box1 ?box2)))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; update location (bot)
            (at start (not (at ?bot ?from)))
            (at start (at ?bot ?to))
            ; update location of both boxes
            (at start (not (at ?box1 ?from)))
            (at start (at ?box1 ?to))
            (at start (not (at ?box2 ?from)))
            (at start (at ?box2 ?to))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    (:durative-action move_worker_bot_with_3_boxes
        :parameters (?bot - worker_bot ?carr - carrier ?box1 - box ?box2 - box  ?box3 - box  ?from - location ?to - location)
        :duration (= ?duration 5)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (at start (at ?bot ?from))
            (over all (linked ?from ?to))
            ; carrier is owned by the bot and has capacity for 3 boxes
            (over all (bot_owns_carrier ?bot ?carr))
            (over all (carrier_has_3 ?carr))
            ; all three boxes are on the carrier
            (over all (carrying_box ?carr ?box1))
            (over all (carrying_box ?carr ?box2))
            (over all (carrying_box ?carr ?box3))
            ; boxes must be different
            (over all (not (= ?box1 ?box2)))
            (over all (not (= ?box1 ?box3)))
            (over all (not (= ?box2 ?box3)))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; update location (bot)
            (at start (not (at ?bot ?from)))
            (at start (at ?bot ?to))
            ; update location of all three boxes
            (at start (not (at ?box1 ?from)))
            (at start (at ?box1 ?to))
            (at start (not (at ?box2 ?from)))
            (at start (at ?box2 ?to))
            (at start (not (at ?box3 ?from)))
            (at start (at ?box3 ?to))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    (:durative-action move_helper_bot_patient
        :parameters (?bot - helper_bot ?from - location ?to - location ?pat - patient)
        :duration (= ?duration 5)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (at start (at ?bot ?from))
            (over all (linked ?from ?to))
            ; helper bot is helping a patient
            (over all (helping_patient ?bot ?pat))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; update location (bot)
            (at start (not (at ?bot ?from)))
            (at start (at ?bot ?to))
            ; update location (patient)
            (at start (not (at ?pat ?from)))
            (at start (at ?pat ?to))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )
    

    ;;
    ;; Worker Bots : These are the bots that carry boxes with supplies  
    ;;

    ; pick up first box when carrier is empty (0 -> 1)
    (:durative-action pick_up_1st_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?loc - location)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (over all (at ?bot ?loc))
            (over all (at ?box ?loc))
            ; box not being carried
            (at start (box_on_ground ?box))
            ; carrier has space
            (over all (bot_owns_carrier ?bot ?carr))
            (at start (carrier_empty ?carr))
            (over all (carrier_can_1 ?carr))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; robot now has the box
            (at end (carrying_box ?carr ?box))
            (at end (not (box_on_ground ?box)))
            ; update carrier capacity
            (at end (not (carrier_empty ?carr)))
            (at end (carrier_has_1 ?carr))
            ; bot becomes busy
            (at end (not (bot_is_free ?bot)))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    ; pick up second box when carrier has one (1 -> 2)
    (:durative-action pick_up_2nd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?loc - location)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (over all (at ?bot ?loc))
            (over all (at ?box ?loc))
            ; box not being carried
            (at start (box_on_ground ?box))
            ; carrier has space
            (over all (bot_owns_carrier ?bot ?carr))
            (at start (carrier_has_1 ?carr)) ; one box is already being carried
            (over all (carrier_can_2 ?carr)) ; carrier must have capacity for 2 boxes
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; robot now has the box
            (at end (carrying_box ?carr ?box))
            (at end (not (box_on_ground ?box)))
            ; update carrier capacity
            (at end (not (carrier_has_1 ?carr)))
            (at end (carrier_has_2 ?carr))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    ; pick up third box when carrier has two (2 -> 3)
    (:durative-action pick_up_3rd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?loc - location)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (over all (at ?bot ?loc))
            (over all (at ?box ?loc))
            ; box not being carried
            (at start (box_on_ground ?box))
            ; carrier has space
            (over all (bot_owns_carrier ?bot ?carr))
            (at start (carrier_has_2 ?carr)) ; two boxes are already being carried
            (over all (carrier_can_3 ?carr)) ; carrier must have capacity for 3 boxes
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; robot now has the box
            (at end (carrying_box ?carr ?box))
            (at end (not (box_on_ground ?box)))
            ; update carrier capacity
            (at end (not (carrier_has_2 ?carr)))
            (at end (carrier_has_3 ?carr))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    ; drop down a box when carrier is full (3 -> 2)
    (:durative-action drop_down_3rd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; the bot carries the box
            (at start (carrying_box ?carr ?box))
            ; bot owns the carrier
            (over all (bot_owns_carrier ?bot ?carr))
            (at start (carrier_has_3 ?carr)) ; carrier has 3 boxes on (maximum capacity)
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; leave box there
            (at end (not (carrying_box ?carr ?box)))
            (at end (box_on_ground ?box))
            ; update carrier capacity
            (at end (not (carrier_has_3 ?carr)))
            (at end (carrier_has_2 ?carr))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    ; drop down a box when carrier has two boxes (2 -> 1)
    (:durative-action drop_down_2nd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; the bot carries the box
            (at start (carrying_box ?carr ?box))
            ; bot owns the carrier
            (over all (bot_owns_carrier ?bot ?carr))
            (at start (carrier_has_2 ?carr)) ; carrier has 2 boxes on
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; leave box there
            (at end (not (carrying_box ?carr ?box)))
            (at end (box_on_ground ?box))
            ; update carrier capacity
            (at end (not (carrier_has_2 ?carr)))
            (at end (carrier_has_1 ?carr))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    ; drop down a box when carrier has 1 boxes (1 -> 0)
    (:durative-action drop_down_1st_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; the bot carries the box
            (at start (carrying_box ?carr ?box))
            ; bot owns the carrier
            (over all (bot_owns_carrier ?bot ?carr))
            (at start (carrier_has_1 ?carr))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; leave box there
            (at end (not (carrying_box ?carr ?box)))
            (at end (box_on_ground ?box))
            ; update carrier capacity
            (at end (not (carrier_has_1 ?carr)))
            (at end (carrier_empty ?carr)) ; carrier is now empty
            ; robot is once again free
            (at end (bot_is_free ?bot))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )


    (:durative-action fill_box
        :parameters (?bot - worker_bot ?box - box ?sup - supply ?loc - location)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (over all (at ?bot ?loc))
            (over all (at ?sup ?loc))
            (over all (at ?box ?loc))
            ; the box is not being carried
            (over all (box_on_ground ?box))
            ; box must be empty
            (at start (box_is_empty ?box))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; fill the box with the supply
            (at end (supply_in_box ?sup ?box))
            (at end (not (box_is_empty ?box)))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )


    (:durative-action deliver_supply
        :parameters (?bot - worker_bot ?box - box ?sup - supply ?loc - location ?mu - med_unit)
        :duration (= ?duration 1)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (over all (at ?bot ?loc))
            (over all (at ?mu ?loc))
            (over all (at ?box ?loc))
            ; the box is NOT being carried
            (over all (box_on_ground ?box))
            ; the supply is in the box
            (at start (supply_in_box ?sup ?box))
            ; the medical unit needs such supply
            (at start (needs_supply ?mu ?sup))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; empty the box
            (at end (not (supply_in_box ?sup ?box)))
            (at end (box_is_empty ?box))
            ; satisfy the medical unit's need and hand over the supply
            (at end (not (needs_supply ?mu ?sup))) 
            (at end (has_supply ?mu ?sup))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )

    ;;
    ;; Helper Bots : These are the bots that accompany patients to medical units
    ;;

    (:durative-action pick_up_patient
        :parameters (?bot - helper_bot ?pat - patient ?loc - location)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (over all (at ?bot ?loc))
            (over all (at ?pat ?loc))
            ; patient is not being helped by some other bot
            (at start (patient_on_ground ?pat))
            ; helper bot is free
            (at start (bot_is_free ?bot))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; helper bot accompanies the patient
            (at end (helping_patient ?bot ?pat))
            (at end (not (patient_on_ground ?pat)))
            ; helper bot is now busy
            (at end (not (bot_is_free ?bot)))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )
    (:durative-action drop_down_patient
        :parameters (?bot - helper_bot ?pat - patient)
        :duration (= ?duration 2)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; helper bot accompanies the patient
            (at start (helping_patient ?bot ?pat))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; helper bot no longer accompanies the patient
            (at end (not (helping_patient ?bot ?pat)))
            (at end (patient_on_ground ?pat))
            ; helper bot is no longer busy
            (at end (bot_is_free ?bot))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )
    (:durative-action deliver_patient
        :parameters (?bot - helper_bot ?pat - patient ?mu - med_unit ?loc - location)
        :duration (= ?duration 1)
        :condition (and
            (at start (bot_is_unlocked ?bot)) ; unlock check

            ; location consistency
            (over all (at ?bot ?loc))
            (over all (at ?mu ?loc))
            (over all (at ?pat ?loc))
            ; the patient is not being helped by some other bot
            (over all (patient_on_ground ?pat)) 
            ; the patient needs treatment at the medical unit
            (at start (needs_patient ?mu ?pat))
        )
        :effect (and
            (at start (not (bot_is_unlocked ?bot))) ; LOCK (during execution)

            ; satisfy the medical unit's need for the patient and hand over the patient
            (at end (not (needs_patient ?mu ?pat)))
            (at end (has_patient ?mu ?pat))

            (at end (bot_is_unlocked ?bot)) ; UNLOCK (after execution ends)
        )
    )
)