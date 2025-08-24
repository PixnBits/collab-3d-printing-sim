# Collaborative 3D Printing Simulation

This repository provides a Dockerized environment for simulating a collaborative 3D printing system with 2-4 robotic arms using ROS 2 Humble and Gazebo Harmonic on Ubuntu 22.04. The setup supports testing for synchronization, collision avoidance, and efficiency (aiming for 50% time reduction vs. single-head printing).

## Features
- **Open-Source Core**: MIT-licensed Dockerfile, Makefile, and basic ROS 2 package for simulations.
- **Dockerized Setup**: Installs ROS 2 Humble, Gazebo Harmonic, MoveIt 2, and Cura dependencies.
- **Simulation Goals**: Model multi-arm printing for a 30cm vase, optimizing for speed and collision-free operation.
- **Profitability Hooks**: Placeholder for premium AI path optimization (Grok Optimizer, future SaaS) and hardware kits.

## Prerequisites
- Ubuntu 22.04 LTS
- Docker (`sudo apt install docker.io`)
- Make (`sudo apt install make`)
- X11 for GUI (`export DISPLAY=:0`)

## Setup Instructions
1. **Clone Repository**:
   ```bash
   git clone https://github.com/<your-username>/collab-3d-printing-sim.git
   cd collab-3d-printing-sim
   ```

2. **Build Docker Image**:
   ```bash
   make build
   ```

3. **Run Docker Container**:
   ```bash
   make run
   ```

4. **Inside Container**:
   - Source workspace: `source /root/collab_3d_printing_ws/install/setup.bash`
   - Build ROS package: `cd /root/collab_3d_printing_ws && colcon build --symlink-install`
   - Launch simulation: `ros2 launch multi_arm_sim simulation.launch.py`

## Project Structure
- `Dockerfile`: Sets up Ubuntu 22.04 with ROS 2, Gazebo, and dependencies.
- `Makefile`: Manages Docker build, run, and cleanup.
- `ws/src/multi_arm_sim`: ROS 2 package with URDFs and launch files for simulation.
- `.gitignore`: Ignores build artifacts and temporary files.

## Simulation Setup
- **URDFs**: Define xArm Lite models with custom extruders in `ws/src/multi_arm_sim/urdf`.
- **Launch File**: Configures Gazebo with 2-4 arms and a print bed.
- **Metrics**: Log print time, collisions, and energy use with Prometheus (to be integrated).

## Contributing
- Fork and submit pull requests via GitHub.
- Join our Discord for beta testing and feedback.
- Simulation datasets will be shared openly; contribute your results!

## Future Plans
- **Open-Source**: Share simulation datasets and basic scripts under MIT license.
- **Premium Features**: Grok Optimizer SaaS ($10/month) for AI-driven path planning.
- **Hardware Kits**: Pre-calibrated arm kits ($500 each) for easy adoption.
- **Consulting**: Services for industrial scaling (e.g., aerospace).

## License
MIT License for core components. Proprietary features (e.g., Grok Optimizer) reserved for future monetization.

---
Activity: Aim for 10k+ GitHub stars in year 1! Share feedback on Reddit’s r/3Dprinting or our Discord.
