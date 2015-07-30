# ED Cloud

## Experiment Scenario 1

### How to run

    # Start a roscore
    roscore

    # Start the simulator
    roscd ed_cloud/experiments
    rosrun fast_simulator simulator scenario1-sim.yaml

    # Start simulated moving objects
    rosrun ed_cloud scenario1-sim-objects.py

    # Start ED
    roscd ed_cloud/experiments
    rosrun ed ed scenio1-ed.yaml

    # Visualize
    rosrun ed_gui_server ed_rviz_publisher
    cloud-rviz

### How to edit

To change the simulated world:

    roscd ed_object_models/models/cloud_experiments/scenario1

Then edit the 'walls/heightmap.pgm' file. Pay attention to the following:

1. White is floor, black is wall
*  Make sure you always leave a white border around the image (only has to be one pixel)
*  The resolution, origin and height of the map can be changed in walls/model.yaml

To change the sensors in the simulated world:

    roscd ed_cloud/experiments

    Edit the file 'scenario1-sim.yaml'

To change the world model (ED) configuration

    roscd ed_cloud/experiments

    Edit the file 'scenarion1-ed.yaml'
