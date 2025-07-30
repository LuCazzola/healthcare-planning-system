(define (problem healthcare_facility_1_medium)
    (:domain healthcare_facility_1)

    (:objects
        ;; Locations
        entrance reception diagnostic_lab corridor1 corridor2 central_warehouse
        surgical_block emergency_room cardiology_ward neurology_ward
        - location

        ;; Robots
        wBot1 - worker_bot
        hBot1 - helper_bot

        ;; Boxes
        box1 - box

        ;; Supplies
        scalpel defibrillator anesthetic - supply

        ;; Medical Units
        trauma_unit resuscitation_unit triage_unit
        operation_unit cardiology_unit neurology_unit
        - med_unit

        ;; Patients
        patient1 patient2 - patient
    )

    (:init
        ;; Roadmap connectivity
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

        ;;  [Entrance]——————[Reception]—————————————[Emergency_Room]
        ;;                       |                         |
        ;;                       |                    [Corridor2]
        ;;                       |                         |
        ;; [Central_warehouse]———————[Corridor1]————[Diagnostic_Lab]————[Neurology_Ward]
        ;;                       |
        ;;                [Surgical_Block]
        ;;                       |
        ;;               [Cardiology_Ward]
 
        ;; Medical unit locations

        (at trauma_unit emergency_room)
        (at resuscitation_unit emergency_room)
        (at triage_unit reception)
        (at operation_unit surgical_block)
        (at cardiology_unit cardiology_ward)
        (at neurology_unit neurology_ward)

        ;; Robot initial positions
        (at wBot1 central_warehouse)
        (at hBot1 entrance)

        ;; Robots are at start
        (bot_is_free wBot1)
        (bot_is_free hBot1)

        ;; Boxes
        (at box1 central_warehouse)
        (box_is_empty box1)
        (box_on_ground box1)

        ;; Supplies
        (at scalpel central_warehouse)
        (at defibrillator central_warehouse)
        (at anesthetic central_warehouse)

        ;; Patients
        (at patient1 entrance)
        (at patient2 entrance)
        (patient_on_ground patient1)
        (patient_on_ground patient2)

        ;; Needs
        (needs_supply trauma_unit scalpel)
        (needs_supply resuscitation_unit defibrillator)
        (needs_supply neurology_unit anesthetic)

        (needs_patient operation_unit patient1)
        (needs_patient cardiology_unit patient2)
    )

    (:goal
        (and
            ;; Satisfy all needs
            (has_supply trauma_unit scalpel)
            (has_supply resuscitation_unit defibrillator)
            (has_supply neurology_unit anesthetic)
            (has_patient operation_unit patient1)
            (has_patient cardiology_unit patient2)
        )
    )
)