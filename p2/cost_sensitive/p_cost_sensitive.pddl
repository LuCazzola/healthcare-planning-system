(define (problem healthcare_facility_2_cost_sensitive)
    (:domain healthcare_facility_2_cost_sensitive)

    (:objects 
        ;; Locations
        entrance reception diagnostic_lab corridor1 corridor2 central_warehouse
        surgical_block emergency_room cardiology_ward neurology_ward
        - location

        ;; Robots
        wBot1 - worker_bot
        hBot1 - helper_bot

        ;; Carrier (new in this domain)
        carrier1 - carrier

        ;; Boxes
        box1 box2 - box

        ;; Supplies
        scalpel defibrillator - supply

        ;; Medical Units
        trauma_unit resuscitation_unit triage_unit operation_unit cardiology_unit neurology_unit - med_unit

        ;; Patients
        patient1 - patient
    )

    (:init
        ;; Initialize total cost
        (= (total-cost) 0)

        ;; Roadmap
        (linked entrance reception)
        (linked reception entrance)

        (linked reception emergency_room)
        (linked emergency_room reception)

        (linked emergency_room corridor2)
        (linked corridor2 emergency_room)

        (linked corridor2 diagnostic_lab)
        (linked diagnostic_lab corridor2)

        (linked corridor1 diagnostic_lab)
        (linked diagnostic_lab corridor1)
        
        (linked surgical_block corridor1)
        (linked corridor1 surgical_block)

        (linked corridor1 central_warehouse)
        (linked central_warehouse corridor1)

        (linked reception corridor1)
        (linked corridor1 reception)

        (linked diagnostic_lab neurology_ward)
        (linked neurology_ward diagnostic_lab)

        (linked surgical_block cardiology_ward)
        (linked cardiology_ward surgical_block)

        ;; Carrier setup
        (bot_owns_carrier wBot1 carrier1)
        (carrier_empty carrier1)
        (carrier_can_1 carrier1) (carrier_can_2 carrier1) (carrier_can_3 carrier1) ; Carrier has 3 slots for boxes

        ;; Medical unit locations
        (at trauma_unit emergency_room)
        (at resuscitation_unit emergency_room)
        (at triage_unit reception)
        (at operation_unit surgical_block)
        (at cardiology_unit cardiology_ward)
        (at neurology_unit neurology_ward)

        ;; Robots are free at their initial positions
        (at wBot1 central_warehouse)
        (at hBot1 entrance)
        
        ;; Bots are free
        (bot_is_free wBot1)
        (bot_is_free hBot1)

        ;; Boxes at central_warehouse
        (at box1 central_warehouse)
        (at box2 central_warehouse)
        
        ;; Boxes are empty
        (box_is_empty box1)
        (box_is_empty box2)

        ;; Boxes on the ground
        (box_on_ground box1)
        (box_on_ground box2)

        ;; Supplies at central_warehouse
        (at scalpel central_warehouse)
        (at defibrillator central_warehouse)

        ;; Patient at entrance, not being helped
        (at patient1 entrance) (patient_on_ground patient1)

        ;; Medical unit needs
        (needs_supply trauma_unit scalpel)
        (needs_supply resuscitation_unit defibrillator)
        (needs_patient operation_unit patient1)
    )

    (:goal
        (and
            ;; Satisfy all needs
            (has_supply trauma_unit scalpel)
            (has_supply resuscitation_unit defibrillator)
            
            ;; Satisfy patient need
            (has_patient operation_unit patient1)
            
            ;; bot return to starting conditions
            (at wBot1 central_warehouse)
            (at hBot1 entrance)
            (at patient1 surgical_block) ; Added, otherwise the hbot carries around the patient as it's more cost effective! (after delivery)
            (bot_is_free wBot1)
            (bot_is_free hBot1)

            ;; Carrier is empty
            (carrier_empty carrier1)

            ;; Return boxes (empty) to the central warehouse
            (at box1 central_warehouse)
            (at box2 central_warehouse)
            (box_is_empty box1)
            (box_is_empty box2)
            (box_on_ground box1)
            (box_on_ground box2)
        )
    )

    (:metric minimize (total-cost))
)