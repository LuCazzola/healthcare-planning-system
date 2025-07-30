(define (domain healthcare_facility_2_base)
    (:requirements :strips :typing :equality)

    (:types
        location carrier - object
        worker_bot helper_bot - robot
        med_unit robot box supply patient - locatable
    )

    (:predicates
        ; location related
        (at ?obj - locatable ?loc - location) ; locatable is at a location
        (linked ?loc1 - location ?loc2 - location) ; two locations are linked (roadmap)
        
        ; General Robot related
        (bot_is_free ?bot - robot) ; bot is free (not carrying a box or helping a patient)
        
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
    
    (:action move_bot_alone
        :parameters (?bot - robot ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; the bot is free (carrier is empty / not helping a patient)
            (bot_is_free ?bot)
        )
        :effect (and
            ; change location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
        )
    )
    
    (:action move_worker_bot_with_1_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; carrier checks
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_1 ?carr) ; the bot carries a single box
            (carrying_box ?carr ?box) ; carrying one box
        )
        :effect (and
            ; update location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
            ; update location (box)
            (not (at ?box ?from))
            (at ?box ?to)
        )
    )
    
    (:action move_worker_bot_with_2_boxes
        :parameters (?bot - worker_bot ?carr - carrier ?box1 - box ?box2 - box ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; carrier is owned by the bot and has capacity for 2 boxes
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_2 ?carr)
            ; both boxes are on the carrier
            (carrying_box ?carr ?box1)
            (carrying_box ?carr ?box2)
            ; boxes must be different
            (not (= ?box1 ?box2))
        )
        :effect (and
            ; update location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
            ; update location of both boxes
            (not (at ?box1 ?from))
            (at ?box1 ?to)
            (not (at ?box2 ?from))
            (at ?box2 ?to)
        )
    )

    (:action move_worker_bot_with_3_boxes
        :parameters (?bot - worker_bot ?carr - carrier ?box1 - box ?box2 - box  ?box3 - box  ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; carrier is owned by the bot and has capacity for 3 boxes
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_3 ?carr)
            ; all three boxes are on the carrier
            (carrying_box ?carr ?box1)
            (carrying_box ?carr ?box2)
            (carrying_box ?carr ?box3)
            ; boxes must be different
            (not (= ?box1 ?box2))
            (not (= ?box1 ?box3))
            (not (= ?box2 ?box3))
        )
        :effect (and
            ; update location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
            ; update location of all three boxes
            (not (at ?box1 ?from))
            (at ?box1 ?to)
            (not (at ?box2 ?from))
            (at ?box2 ?to)
            (not (at ?box3 ?from))
            (at ?box3 ?to)
        )
    )

    (:action move_helper_bot_patient
        :parameters (?bot - helper_bot ?from - location ?to - location ?pat - patient)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; helper bot is helping a patient
            (helping_patient ?bot ?pat)
        )
        :effect (and
            ; update location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
            ; update location (patient)
            (not (at ?pat ?from))
            (at ?pat ?to)
        )
    )
    

    ;;
    ;; Worker Bots : These are the bots that carry boxes with supplies  
    ;;

    ; pick up first box when carrier is empty (0 -> 1)
    (:action pick_up_1st_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?box ?loc)
            ; box not being carried
            (box_on_ground ?box)
            ; carrier has space
            (bot_owns_carrier ?bot ?carr)
            (carrier_empty ?carr)
            (carrier_can_1 ?carr)
        )
        :effect (and
            ; robot now has the box
            (carrying_box ?carr ?box)
            (not (box_on_ground ?box))
            ; update carrier capacity
            (not (carrier_empty ?carr))
            (carrier_has_1 ?carr)
            ; bot becomes busy
            (not (bot_is_free ?bot))
        )
    )

    ; pick up second box when carrier has one (1 -> 2)
    (:action pick_up_2nd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?box ?loc)
            ; box not being carried
            (box_on_ground ?box)
            ; carrier has space
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_1 ?carr) ; one box is already being carried
            (carrier_can_2 ?carr) ; carrier must have capacity for 2 boxes
        )
        :effect (and
            ; robot now has the box
            (carrying_box ?carr ?box)
            (not (box_on_ground ?box))
            ; update carrier capacity
            (not (carrier_has_1 ?carr))
            (carrier_has_2 ?carr)
        )
    )

    ; pick up third box when carrier has two (2 -> 3)
    (:action pick_up_3rd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?box ?loc)
            ; box not being carried
            (box_on_ground ?box)
            ; carrier has space
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_2 ?carr) ; two boxes are already being carried
            (carrier_can_3 ?carr) ; carrier must have capacity for 3 boxes
        )
        :effect (and
            ; robot now has the box
            (carrying_box ?carr ?box)
            (not (box_on_ground ?box))
            ; update carrier capacity
            (not (carrier_has_2 ?carr))
            (carrier_has_3 ?carr)
        )
    )

    ; drop down a box when carrier is full (3 -> 2)
    (:action drop_down_3rd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box)
        :precondition (and
            ; the bot carries the box
            (carrying_box ?carr ?box)
            ; bot owns the carrier
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_3 ?carr) ; carrier has 3 boxes on (maximum capacity)
        )
        :effect (and
            ; leave box there
            (not (carrying_box ?carr ?box))
            (box_on_ground ?box)
            ; update carrier capacity
            (not (carrier_has_3 ?carr))
            (carrier_has_2 ?carr)
        )
    )

    ; drop down a box when carrier has two boxes (2 -> 1)
    (:action drop_down_2nd_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box)
        :precondition (and
            ; the bot carries the box
            (carrying_box ?carr ?box)
            ; bot owns the carrier
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_2 ?carr) ; carrier has 2 boxes on
        )
        :effect (and
            ; leave box there
            (not (carrying_box ?carr ?box))
            (box_on_ground ?box)
            ; update carrier capacity
            (not (carrier_has_2 ?carr))
            (carrier_has_1 ?carr)
        )
    )

    ; drop down a box when carrier has 1 boxes (1 -> 0)
    (:action drop_down_1st_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box)
        :precondition (and
            ; the bot carries the box
            (carrying_box ?carr ?box)
            ; bot owns the carrier
            (bot_owns_carrier ?bot ?carr)
            (carrier_has_1 ?carr)
        )
        :effect (and
            ; leave box there
            (not (carrying_box ?carr ?box))
            (box_on_ground ?box)
            ; update carrier capacity
            (not (carrier_has_1 ?carr))
            (carrier_empty ?carr) ; carrier is now empty
            ; robot is once again free
            (bot_is_free ?bot)
        )
    )


    (:action fill_box
        :parameters (?bot - worker_bot ?box - box ?sup - supply ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?sup ?loc)
            (at ?box ?loc)
            ; the box is not being carried
            (box_on_ground ?box)
            ; box must be empty
            (box_is_empty ?box)
        )
        :effect (and
            ; fill the box with the supply
            (supply_in_box ?sup ?box)
            (not (box_is_empty ?box))
        )
    )


    (:action deliver_supply
        :parameters (?bot - worker_bot ?box - box ?sup - supply ?loc - location ?mu - med_unit)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?mu ?loc)
            (at ?box ?loc)
            ; the box is NOT being carried
            (box_on_ground ?box)
            ; the supply is in the box
            (supply_in_box ?sup ?box)
            ; the medical unit needs such supply
            (needs_supply ?mu ?sup)
        )
        :effect (and
            ; empty the box
            (not (supply_in_box ?sup ?box))
            (box_is_empty ?box)
            ; satisfy the medical unit's need and hand over the supply
            (not (needs_supply ?mu ?sup)) 
            (has_supply ?mu ?sup)
        )
    )

    ;;
    ;; Helper Bots : These are the bots that accompany patients to medical units
    ;;

    (:action pick_up_patient
        :parameters (?bot - helper_bot ?pat - patient ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?pat ?loc)
            ; patient is not being helped by some other bot
            (patient_on_ground ?pat)
            ; helper bot is free
            (bot_is_free ?bot)
        )
        :effect (and
            ; helper bot accompanies the patient
            (helping_patient ?bot ?pat)
            (not (patient_on_ground ?pat))
            ; helper bot is now busy
            (not (bot_is_free ?bot))
        )
    )
    (:action drop_down_patient
        :parameters (?bot - helper_bot ?pat - patient)
        :precondition (and
            ; helper bot accompanies the patient
            (helping_patient ?bot ?pat)
        )
        :effect (and
            ; helper bot no longer accompanies the patient
            (not (helping_patient ?bot ?pat))
            (patient_on_ground ?pat)
            ; helper bot is no longer busy
            (bot_is_free ?bot)
        )
    )
    (:action deliver_patient
        :parameters (?bot - helper_bot ?pat - patient ?mu - med_unit ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?mu ?loc)
            (at ?pat ?loc)
            ; the patient is not being helped by some other bot
            (patient_on_ground ?pat) 
            ; the patient needs treatment at the medical unit
            (needs_patient ?mu ?pat)
        )
        :effect (and
            ; satisfy the medical unit's need for the patient and hand over the patient
            (not (needs_patient ?mu ?pat))
            (has_patient ?mu ?pat)
        )
    )
)