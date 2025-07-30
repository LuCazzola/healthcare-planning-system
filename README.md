# Robot planning in Health-Care scenario

<br>

Assignment for the course Automated Planning Theory and Practice Academic Year 2024-2025


## Init

<br>

Problems 1 to 4

* Build Docker
```bash
docker build --rm -t myplanutils . --file Dockerfile-planutils
```

* Run Docker
```bash
docker run -v $(pwd):/computer -it --privileged --rm myplanutils bash
```

Problem 5


* Build Docker
```bash
sudo docker build --rm  --tag ros-humble . --file Dockerfile-humble
```

* Run Docker
```bash
xhost +local:docker
sudo docker run -v /tmp/.X11-unix/:/tmp/.X11-unix/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --volume="$PWD/p5:/workspace/p5" --network=host --name ubuntu_bash --env="DISPLAY" --rm -i -t ros-humble bash
```

* Buid Problem 5
```bash
source plansys2_ws/install/setup.bash
cd /workspace/p5
colcon build --symlink-install
source install/setup.bash
```

## Problems

<br>

```
planutils activate
```

### P1

<br>


Fast-Downward
```
planutils run downward -- domain.pddl p1_mid.pddl --search "astar(lmcut())"
```

ENHSP
```
planutils run enhsp -- -o domain.pddl -f p0_easy.pddl -sp p0.plan -s WAStar -wh 0.25
```

# p5

Terminal 1
```bash
ros2 launch p5 p5_launch.py
```

Terminal 2
```bash
ros2 run plansys2_terminal plansys2_terminal
source /workspace/p5/launch/commands
get plan
run
```