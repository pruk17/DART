# Docker Setup — Ubuntu 22.04 (ROS 2 Humble) in Docker

The host runs **Ubuntu 24.04**, but the project needs an **Ubuntu 22.04
(Humble)** environment (the TurtleBot3 runs Humble, and PC and robot must use
the same ROS 2 distribution). Docker provides a separate Ubuntu 22.04 box
without touching the host.

---

## Part 1 — Install Docker (once)

### 1.1 Install Docker
```bash
sudo apt update
sudo apt install -y docker.io
```

### 1.2 Use docker without sudo
```bash
sudo usermod -aG docker $USER
```
> ⚠️ **Log out and log back in** (or reboot) once for this to take effect.

### 1.3 Check that it works
```bash
docker run hello-world      # a welcome message = it works
```

---

## Part 2 — The two files

| File | Purpose |
|------|---------|
| `docker/Dockerfile` | Image recipe: starts from `osrf/ros:humble-desktop` (Ubuntu 22.04) and adds colcon, TurtleBot3 (incl. Gazebo simulation), Navigation2 (map_server, AMCL) and gazebo_ros |
| `docker/run.sh` | Shortcut script for the docker commands: `build / up / exec / down` |

Every shell in the container automatically sources ROS 2 Humble and the
workspace (`/ws/install/setup.bash`), sets `TURTLEBOT3_MODEL=burger`, and
installs the `dart` pip package (`pip install -e /ws/src/DART`) if it is
missing. Gazebo uses software rendering by default.

---

## Part 3 — Build & Run

| Command | What it does |
|---------|--------------|
| `./docker/run.sh build` | Builds the image from the Dockerfile (first time only, takes a while) |
| `./docker/run.sh up`    | Enters the container — mounts `DART_ws` → `/ws`, connects GUI + network |
| `./docker/run.sh exec`  | Opens another shell in the same running container (several windows) |
| `./docker/run.sh down`  | Stops the container |

### 3.1 Build the image
```bash
cd ~/DART_ws
./docker/run.sh build
```

### 3.2 Enter the container
```bash
./docker/run.sh up
# the prompt changes to  root@<host>:/ws#  = you are in Ubuntu 22.04
```

### 3.3 Open more windows (same container)
```bash
./docker/run.sh exec
```

### 3.4 Stop when finished
```bash
./docker/run.sh down
```

---

## Part 4 — What `run.sh` sets up automatically on `up`

```
--network host         → the container shares the host network (reaches the real robot)
--ipc host             → shared memory between host and container
-v DART_ws:/ws         → host and container edit the same files (same folder)
-e DISPLAY + X11 mount → GUI programs in the container open on the host screen
-e ROS_DOMAIN_ID=30    → must match the robot (override: ROS_DOMAIN_ID=<n> ./docker/run.sh up)
-e TURTLEBOT3_MODEL=burger
```

Optional: `DART_GPU=1 ./docker/run.sh up` passes the host GPU (`/dev/dri`)
instead of software rendering, which makes Gazebo faster.

---

## Common problems

| Symptom | Cause / fix |
|---------|-------------|
| `docker: permission denied` | `usermod -aG docker $USER` not done yet, or no logout/login since |
| Files in `DART_ws` show a lock 🔒 and cannot be edited | Docker created them as root → `sudo chown -R $USER:$USER ~/DART_ws` |
| No GUI (Gazebo / RViz) | `xhost +local:root` missing (run.sh does it on `up`) |
| Building the image takes very long | Normal — the first build downloads and installs the whole stack |
| `ros2` commands on the host and in the container interfere | The host (Jazzy) ROS 2 daemon is on the same network: run `ros2 daemon stop` on the host, or use `--no-daemon` |
