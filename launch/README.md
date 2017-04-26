The Launch Files
================

# Active

## `software_only.launch`

Launch everything. Consider this for doing a demo of the complete system.
This runs two controllers.

### Runs
- `test_sim.launch`
- `model.launch`
- `gui.launch`
- `controller_static_dynamic.launch`
- `controller_delay_dynamic.launch`

## `gui.launch`

Launch the Graphical User Interface

### Runs
- `rviz` with the config `rviz/UIConfig.rviz`
- `UIManager`

## `controller_static_dynamic.launch`

The main image processor/matching/alert calculation logic.
It is run using a static map as its base reference, and the dynamic map as its comparison.

### Runs
- `DataController` with arguments mapped

## `controller_delay_dynamic.launch`

The main image processor/matching/alert calculation logic.
It is run using a time delay map as its base reference, and the dynamic map as its comparison.

### Runs
- `DataController` with arguments mapped

## `model.launch`

The backing data abstractions

### Runs
- `map_server` (static map option 1)
- `gmapping` (dynamic map)
- `TimeDelayMap` (static map option 2)

## `model_free.launch`

The backing data abstractions

### Runs
- `gmapping` (dynamic map)
- `TimeDelayMap` (static map option 2)

## `test_sim.launch`

The simulator. Used to replace a robot/environment.

### Runs
- `stage_ros`
- static transform publisher

# Deprecated

- `all_py.launch`