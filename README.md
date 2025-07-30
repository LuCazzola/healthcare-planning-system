# Healthcare Planning System

Assignment for the course Automated Planning Theory and Practice (A.Y. 2024-2025)  
University of Trento - Artificial Intelligence Systems

This project explores automated planning techniques through a healthcare facility logistics scenario involving robotic agents for supply delivery and patient escort. The implementation progresses from basic PDDL formulations to hierarchical decomposition, temporal planning, and ROS2 integration.

**For detailed methodology, results, and analysis, see [`docs/report.pdf`](docs/report.pdf)**  
**For complete assignment specifications, see [`docs/assignment.pdf`](docs/assignment.pdf)**

```bash
healthcare-planning-system
├─ p1/ # Basic PDDL formulation
├─ p2/ # Carrier-enhanced transportation
├─ p3/ # Hierarchical HDDL implementation  
├─ p4/ # Temporal planning with durative actions
└─ p5/ # ROS2 integration with PlanSys2
```

## Quick Start

### Problems 1-4 (PDDL/HDDL/Temporal Planning)

Build and run the planning environment:
```bash
docker build --rm -t myplanutils . --file Dockerfile-planutils
docker run -v $(pwd):/computer -it --privileged --rm myplanutils bash
cd /computer
```

Then follow the instructions :

<details>
  <summary><b>Problem 1 - Basic PDDL</b></summary>

<br>

Enter the problem folder
```bash
cd p1
```
Available problems are: `p1_0`, `p1_1`, `p1_2`, `p1_3`,


To evaluate with Fast-Downward
```bash
planutils run downward -- domain.pddl p1_0.pddl --search "astar(lmcut())"
```

To evaluate with LAMA
```bash
planutils run lama -- domain.pddl p1_0.pddl
```

To evaluate with LAMA-First
```bash
planutils run lama-first -- domain.pddl p1_0.pddl
```
</details>

<details>
  <summary><b>Problem 2 - Carrier-Enhanced Transportation</b></summary>

<br>

Enter the problem folder
```bash
cd p2
```
Available problems are: `p2_0`, `p2_1`

To evaluate with Fast-Downward
```bash
planutils run downward -- domain.pddl p2_0.pddl --search "astar(lmcut())"
```

To evaluate with LAMA
```bash
planutils run lama -- domain.pddl p2_0.pddl
```

To evaluate with LAMA-First
```bash
planutils run lama-first -- domain.pddl p2_0.pddl
```
</details>

<details>
  <summary><b>Problem 2 Variant - "Numeric"</b></summary>

<br>

Enter the problem folder
```bash
cd p2/numeric
```
Available problems are: `p_numeric`

To evaluate with ENHSP
```bash
planutils run enhsp -- -o domain.pddl -f p_numeric.pddl -wh 0.25 -anytime
```

</details>

<details>
  <summary><b>Problem 2 Variant - "Cost-Sensitive"</b></summary>

<br>

Enter the problem folder
```bash
cd p2/const_sensitive
```
Available problems are: `p_cost_sensitive`

To evaluate with Fast-Downward
```bash
planutils run downward -- domain.pddl p_cost_sensitive.pddl --search "astar(lmcut())"
```

To evaluate with LAMA
```bash
planutils run lama -- domain.pddl p_cost_sensitive.pddl
```

To evaluate with LAMA-First
```bash
planutils run lama-first -- domain.pddl p_cost_sensitive.pddl
```

</details>

<details>
  <summary><b>Problem 3 - HDDL</b></summary>

<br>

Enter the problem folder
```bash
cd p3
```
Available problems are: `p3_0`, `p3_1`

To evaluate with PANDA
```bash
planutils run panda -- domain.hddl p3_0.hddl
```
</details>


<details>
  <summary><b>Problem 4 - Temporal Planning</b></summary>

<br>

Enter the problem folder
```bash
cd p4
```
Available problems are: `p4_0`, `p4_1`

To evaluate with OPTIC
```bash
planutils run optic -- domain.pddl p4_0.pddl
```
</details>

### Problem 5 (ROS2 Integration)

Build ROS2 environment:
```bash
docker build --rm --tag ros-humble . --file Dockerfile-humble
xhost +local:docker
docker run -v /tmp/.X11-unix/:/tmp/.X11-unix/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --volume="$PWD/p5:/workspace/p5" --network=host --env="DISPLAY" --rm -i -t ros-humble bash
```

Build and run the Problem with dependancies:
```bash
source plansys2_ws/install/setup.bash
cd /workspace/p5
colcon build --symlink-install
source install/setup.bash
```

For 2 terminal execution it's suggested to launch `terminator` and then split the terminal. One you have 2 terminals operating within the docker you can proceed with:

**Terminal 1** (Launch system):
```bash
ros2 launch p5 p5_launch.py
```

**Terminal 2** (Execute planning):
```bash
ros2 run plansys2_terminal plansys2_terminal
source /workspace/p5/launch/commands
get plan
run
```

## Cite 💜

If you use this work in your research or projects, please reference this repository:

```bash
@misc{cazzola2025healthcare,
  title={Healthcare Planning System},
  author={Luca Cazzola},
  year={2025},
  url={https://github.com/LuCazzola/healthcare-planning-system}
}
```