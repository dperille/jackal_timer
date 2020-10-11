# jackal_timer
This is a ROS package used to automate navigation trials in Gazebo. It was built intentionally for the Clearpath Jackal UGV, though it can be adapted to other models as well.

## Getting started
### Important information
This package contains two methods for timing navigation trials: with map (map-nav) and without map (odom-nav). The master branch is set up with map nav, and the no-map branch is set up with odom nav.

This package was developed on ROS melodic, though it seems to also work on ROS kinetic.

### Installation and dependencies
To begin, clone this repository into a ROS workspace. Then, navigate to your ROS workspace and run `catkin_make`.

This package can be modified to be used in any number of ways, but the default configuration requires a number of other packages. The installation instructions for all of these packages can be found [here](https://gist.github.com/vfdev-5/57a0171d8f5697831dc8d374839bca12).

If you want to use this package with map-nav, you'll have to make some changes to the `amcl.launch` file in the `jackal_navigation` package. First, you must add the following lines under the `<launch>` tag:
```
<arg name="initial_pose_x" default="0.0" />
<arg name="initial_pose_y" default="0.0" />
<arg name="initial_pose_a" default="0.0" />
```
And, you must add the following lines under the `<node>` tag:
```
<param name="initial_pose_x" value="$(arg initial_pose_x)" />
<param name="initial_pose_y" value="$(arg initial_pose_y)" />
<param name="initial_pose_a" value="$(arg initial_pose_a)" />
```

## Usage
### Current setup
Currently, the package runs time trials for every world in our [benchmarking dataset](https://www.cs.utexas.edu/~attruong/metrics_dataset.html), which was published alongside our [paper](https://arxiv.org/pdf/2008.13315.pdf) for the 2020 IEEE SSRR conference.

### Modifying the dataset
This package can be adapted to run time trials for different datasets, though it has not been tested on any others, so significant modifications may be necessary. Currently, the requirements for such a dataset are that it includes .world files, path files (as .npy), and map files (as .yaml and .pgm), though map files are only necessary if working with map-nav.

Any changes to use a dataset other than our default one will require some changes to `run_trials.py` and `time_trial.launch`. First, you'll have to change all the file paths in both files to reflect your dataset's location. Second, you'll have to change the RADIUS variable in `run_trials.py` to reflect the radius of the obstacles in your worlds, as well as the `path_coord_to_gazebo_coord` function. You'll also have to change lines 62-70 in `run_trials.py`. As stated before, this package has not been tested on other datasets, so additional modifications may be necessary.

### Changing the planner/parameters
To test a different planner or different planner parameters, simply alter the tags under the `move_base` node in `time_trial.launch`. Or, you can create a separate launch file for your `move_base` node and use the `<include file` tag to replace the node entirely.

### Running trials
Once the necessary packages have been downloaded and the desired modifications have been made, simply run `python run_trials.py`.

The results for each world are currently outputted to a .txt file (to save intermediate results in the case of a crash), one file for each batch of trials. The results are listed as
```
world 
traversal time
```
The script `results_to_arr.py` can be used to convert such .txt files into .npy arrays to more easily package and view the results.
