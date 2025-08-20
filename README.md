# UltraHand

## Installation

### Base on LeRobot

You should install all the lerobot dependency fist

Don't forget `pip install -e ".[dynamixel]"`, because we're using dynamixel motors

### Gravity_Compensation

Install pinocchio with `conda install -c conda-forge pinocchio`

### Simulation

Model of UltraHand is provided, if you need to simulate in MuJoCo,run:
```
conda install -c conda-forge libstdcxx-ng wayland
pip install mujoco
```

## Run

### Try Gravity Compensation only

You can find the source file and edit your config in `src/lerobot/gravity_compensation.py` and `src/lerobot/teleoperators/ultrahand`

Run with ```python -m lerobot.gravity_compensation --port=/your/port --model_path=/your/path ```