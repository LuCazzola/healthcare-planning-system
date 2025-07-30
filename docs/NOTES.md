
# World : Healthcare facility

Schema
---

- ***Units***: they provide medical care and services
	- ATTRIBUTES:
		- Location: where's the unit located in the center. It's a necessary information to instruct the robot.
		- has(item): There exist a box in the unit such that it contains such item
		- items = \[item1, item2, ...]
	- ACTIONS: 
		- submit_treatment (medical_devices, drugs)
- ***Robotic Agents*** : they provide necessary resources to units as well as accompany patients
	- ATTRIBUTES:
		- .
	- ACTIONS:
		- fill (box, item): requires empty box and having an item
		- empty (box, unit) : leaves content of the box to the unit.
		- .pick_up (box): requires: same position
		- deliver (box)
		- move (from, to): (also the box moves of course)
		- accompany (patient) ???
		- deliver(box, unit): 
	
- ***Boxes*** : they contain medical_devices and drugs. They're transported by robots and delivered to units
	- ATTRIBUTE:
		- Location = 
		- content = item
		- is_empty = False / True
	- .ACTIONS:
		- .

- ***Item***: abstraction of something which can be put into a box.
	- ATTRIBUTE:
		- Type = scalpel, tongue depressor, aspirine, ...



- There can be more than one medical unit at any given location (it's not sufficient to keep track of where a box is in order to know which medical units have been given boxes)
- robotic agents move only between connected locations according to a roadmap which specifies the connections which must be taken
- 