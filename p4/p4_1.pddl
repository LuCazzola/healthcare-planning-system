(define (problem healthcare_facility_4_base_hard)
    (:domain healthcare_facility_4_base)

    (:objects 
        ;; Locations
        entrance reception diagnostic_lab corridor1 corridor2 central_warehouse
        surgical_block emergency_room cardiology_ward neurology_ward
        - location

        ;; Robots - 2 worker bots and 2 helper bots
        wBot1 wBot2 - worker_bot
        hBot1 hBot2 - helper_bot

        ;; Carriers - different capacities
        carrier1 carrier2 - carrier

        ;; Boxes - 4 boxes for 5 supply deliveries
        box1 box2 box3 box4 - box

        ;; Supplies - 5 different supplies
        scalpel defibrillator stethoscope blood_bag surgical_mask - supply

        ;; Medical Units
        trauma_unit resuscitation_unit triage_unit operation_unit cardiology_unit neurology_unit - med_unit

        ;; Patients - 3 patients for 3 deliveries
        patient1 patient2 patient3 - patient
    )

    (:init
        ;; Roadmap - same as original
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

        ;; Carrier setup - different capacities
        (bot_owns_carrier wBot1 carrier1)
        (bot_owns_carrier wBot2 carrier2)
        
        ;; Carrier 1 - capacity 3
        (carrier_empty carrier1)
        (carrier_can_1 carrier1) (carrier_can_2 carrier1) (carrier_can_3 carrier1)
        
        ;; Carrier 2 - capacity 2
        (carrier_empty carrier2)
        (carrier_can_1 carrier2) (carrier_can_2 carrier2)

        ;; Medical unit locations - same as simple version
        (at trauma_unit emergency_room)
        (at resuscitation_unit emergency_room)
        (at triage_unit reception)
        (at operation_unit surgical_block)
        (at cardiology_unit cardiology_ward)
        (at neurology_unit neurology_ward)

        ;; Robot initial positions
        (at wBot1 central_warehouse)
        (at wBot2 central_warehouse)
        (at hBot1 entrance)
        (at hBot2 entrance)
        
        ;; All bots are free initially
        (bot_is_free wBot1)
        (bot_is_free wBot2)
        (bot_is_free hBot1)
        (bot_is_free hBot2)
        (bot_is_unlocked wBot1)
        (bot_is_unlocked wBot2)
        (bot_is_unlocked hBot1)
        (bot_is_unlocked hBot2)

        ;; All boxes at central_warehouse, empty, on ground
        (at box1 central_warehouse) (box_is_empty box1) (box_on_ground box1)
        (at box2 central_warehouse) (box_is_empty box2) (box_on_ground box2)
        (at box3 central_warehouse) (box_is_empty box3) (box_on_ground box3)
        (at box4 central_warehouse) (box_is_empty box4) (box_on_ground box4)

        ;; All supplies at central_warehouse
        (at scalpel central_warehouse)
        (at defibrillator central_warehouse)
        (at stethoscope central_warehouse)
        (at blood_bag central_warehouse)
        (at surgical_mask central_warehouse)

        ;; Patients at different locations, not being helped
        (at patient1 entrance) (patient_on_ground patient1)
        (at patient2 entrance) (patient_on_ground patient2)
        (at patient3 diagnostic_lab) (patient_on_ground patient3)

        ;; Medical unit needs - 5 supply deliveries
        (needs_supply trauma_unit scalpel)
        (needs_supply resuscitation_unit defibrillator)
        (needs_supply cardiology_unit stethoscope)
        (needs_supply neurology_unit blood_bag)
        (needs_supply triage_unit surgical_mask)
        
        ;; Patient needs - 3 patient deliveries
        (needs_patient operation_unit patient1)
        (needs_patient cardiology_unit patient2)
        (needs_patient neurology_unit patient3)
    )

    (:goal
        (and
            ;; Satisfy all 5 supply needs
            (has_supply trauma_unit scalpel)
            (has_supply resuscitation_unit defibrillator)
            (has_supply cardiology_unit stethoscope)
            (has_supply neurology_unit blood_bag)
            (has_supply triage_unit surgical_mask)
            
            ;; Satisfy all 3 patient needs
            (has_patient operation_unit patient1)
            (has_patient cardiology_unit patient2)
            (has_patient neurology_unit patient3)
        )
    )
)