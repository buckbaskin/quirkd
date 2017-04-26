The Launch Files
================

# Active

## `software_only.launch`

Launch everything. Consider this for doing a demo of the complete system.

### Runs
- `test_sim.launch`
- `model.launch`
- `gui.launch`
- `controller.launch`

## `gui.launch`

Launch the Graphical User Interface

### Runs
- `rviz` with the config `rviz/UIConfig.rviz`
- `UIManager`

## `controller.launch`

The main image processor/matching/alert calculation logic

### Runs
- `DataController`

## `model.launch`

The backing data abstractions

### Runs
- `map_server` (static map option 1)
- `gmapping` (dynamic map)
- `TimeDelayMap` (static map option 2)

## `test_sim.launch`

The simulator. Used to replace a robot/environment.

### Runs
- `stage_ros`
- static transform publisher

# Deprecated

- `all_py.launch`