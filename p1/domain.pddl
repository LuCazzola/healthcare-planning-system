(define (domain healthcare_facility_1)
    (:requirements :strips :typing)

    (:types
        location - object
        worker_bot helper_bot - robot
        med_unit robot box supply patient - locatable
    )

    (:predicates
        ; location related
        (at ?obj - locatable ?loc - location) ; locatable is at a location
        (linked ?loc1 - location ?loc2 - location) ; two locations are linked (roadmap)

        ; General Robot related
        (bot_is_free ?bot - robot) ; bot is free (not carrying a box or helping a patient)
        
        ; Worker Bot related
        (carrying_box ?bot - worker_bot ?obj - box) ; robot is carrying an object
        ; Helper Bot related
        (helping_patient ?bot - helper_bot ?pat - patient) ; helper bot is helping a patient
        
        ; Box related
        (box_is_empty ?box - box) ; box doesn't contain any supply in
        (box_on_ground ?box - box) ; box is on the ground (not being carried by a bot)

        ; Patient related
        (patient_on_ground ?pat - patient) ; patient is on the ground (not being helped)

        ; Supply related
        (supply_in_box ?sup - supply ?box - box) ; the supply is in the box
        
        ; Goal Oriented
        (needs_supply ?mu - med_unit ?sup - supply) ; medical unit needs a supply
        (needs_patient ?mu - med_unit ?pat - patient) ; medical unit needs a patient so he they can treat them
        (has_supply ?mu - med_unit ?sup - supply) ; medical unit has a supply
        (has_patient ?mu - med_unit ?pat - patient) ; medical unit has a patient
    )
    
    ;;
    ;; Robot (General)
    ;;

    (:action move_bot_alone
        :parameters (?bot - robot ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; the bot is free (not carrying a box or helping a patient)
            (bot_is_free ?bot)
        )
        :effect (and
            ; change location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
        )
    )

    (:action move_worker_bot_with_box
        :parameters (?bot - worker_bot ?box - box ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; the bot is busy carrying the box
            (carrying_box ?bot ?box)
        )
        :effect (and
            ; change location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
            ; change location (box)
            (not (at ?box ?from))
            (at ?box ?to)
        )
    )

    (:action move_helper_bot_with_patient
        :parameters (?bot - helper_bot ?pat - patient ?from - location ?to - location)
        :precondition (and
            ; location consistency
            (at ?bot ?from)
            (linked ?from ?to)
            ; the bot is busy helping the patient
            (helping_patient ?bot ?pat)
        )
        :effect (and
            ; change location (bot)
            (not (at ?bot ?from))
            (at ?bot ?to)
            ; change location (patient)
            (not (at ?pat ?from))
            (at ?pat ?to)
        )
    )

    ;;
    ;; Worker Bots : These are the bots that carry boxes which can be loaded with supplies (1 box per bot, 1 supply per box) 
    ;;

    (:action pick_up_box
        :parameters (?bot - worker_bot ?box - box ?loc - location)
        :precondition (and
            ; location consistency
            (at ?bot ?loc)
            (at ?box ?loc)
            ; bot is not carrying a box yet
            (bot_is_free ?bot)
            ; the box is not being carried by some other bot
            (box_on_ground ?box)    
        )
        :effect (and
            ; robot now has the box
            (carrying_box ?bot ?box)
            (not (box_on_ground ?box))
            ; flag the bot as non-free (it's carrying a box)
            (not (bot_is_free ?bot))
        )
    )


    (:action drop_down_box
        :parameters (?bot - worker_bot ?box - box)
        :precondition (and
            ; the bot carries the box
            (carrying_box ?bot ?box)
        )
        :effect (and
            ; bot no longer carries the box
            (not (carrying_box ?bot ?box))
            (box_on_ground ?box)
            ; robot is no once again free
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
            ; the box is not being carried by any bot
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
            ; the box was previously dropped down
            (box_on_ground ?box)
            ; the box contains the supply
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
            ; patient is not being helped by another bot
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
            ; helper bot is now free to help other patients
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
            ; the patient was previously dropped down
            (patient_on_ground ?pat)
            ; medical unit needs to treat the patient
            (needs_patient ?mu ?pat)
        )
        :effect (and
            ; satisfy the medical unit's need and hand over the patient
            (not (needs_patient ?mu ?pat))
            (has_patient ?mu ?pat)
        )
    )
)
