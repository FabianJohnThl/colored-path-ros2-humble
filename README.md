# colored-path-ros2-humble
Package to publish a given path colored, depending on some measured values

# Contributors

[jpschreiter](https://github.com/jpschreiter)

[FabianJohnThl](https://github.com/FabianJohnThl)

# Installation

```bash
cd ~/ros2_ws/src
sudo rm -r colored-path-ros2-humble
git clone https://github.com/FabianJohnThl/colored-path-ros2-humble.git
cd ..

colcon build --packages-select col_pth
source ./install/setup.bash
```

# Configuration

- The script subscribes to a path on the topic: `path_in`
- The colored path is published as MarkerArray under: `path_colored`
- refer the remappings in the launch file to map your own topics to be used with the script
- the script is parametrized via the parameters in the launch file

```
'colorizer': 'RM520N',                       # string to select a source for colorizing: {'RM520N', 'DUMMY'}
'movement_min_distance': 1.0                 # Minimum distance to be moved on path before drawing/querying new actual value for colorizing
```

# Run

```bash
ros2 launch col_pth col_pth_launch.py
```