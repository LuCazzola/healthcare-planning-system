(define (domain healthcare_facility_2_numeric)
    (:requirements :strips :typing :numeric-fluents :conditional-effects)

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

    (:functions
        (carrier_max_capacity ?carr - carrier) - number
        (carrier_cur_capacity ?carr - carrier) - number
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
            ; the bot is free (carrier is empty or helping a patient)
            (bot_is_free ?bot)
        )
        :effect (and
            ; change location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
        )
    )
    
    (:action move_worker_bot_with_boxes
        :parameters (?bot - worker_bot ?carr - carrier ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; the bot is busy carrying the box
            (bot_owns_carrier ?bot ?carr)
            (> (carrier_cur_capacity ?carr) 0)
        )
        :effect (and
            ; change location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
            ; change location (boxex)
            (forall (?box - box)
                (when (carrying_box ?carr ?box)
                    (and
                        (not (at ?box ?from))
                        (at ?box ?to)
                    )
                )
            )
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

    (:action pick_up_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?box ?loc)
            ; the box is not being carried
            (box_on_ground ?box)
            ; carrier has space
            (bot_owns_carrier ?bot ?carr)
            (< (carrier_cur_capacity ?carr) (carrier_max_capacity ?carr))
        )
        :effect (and
            ; robot now has the box
            (not (box_on_ground ?box))
            (carrying_box ?carr ?box)
            ; bot is busy
            (when (= (carrier_cur_capacity ?carr) 0) ; Was empty before pickup
                (not (bot_is_free ?bot))
            )
            ; increase the current capacity of the carrier
            (increase (carrier_cur_capacity ?carr) 1)
        )
    )

    (:action drop_down_box
        :parameters (?bot - worker_bot ?carr - carrier ?box - box)
        :precondition (and
            ; bot owns the carrier
            (bot_owns_carrier ?bot ?carr)
            ; the bot carries the box
            (carrying_box ?carr ?box)

        )
        :effect (and
            ; leave box there
            (not (carrying_box ?carr ?box))
            (box_on_ground ?box)
            ; bot becomes free if capacuty was 1, as now it will become 0
            (when (= (carrier_cur_capacity ?carr) 1) 
                (bot_is_free ?bot)
            )
            ; decrease the current capacity of the carrier
            (decrease (carrier_cur_capacity ?carr) 1)
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
            ; helper bot is not busy
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