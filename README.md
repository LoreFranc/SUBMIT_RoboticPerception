# How to run the code

Important files: `mads.ini`, `visualizer.py`, `run_all.sh`, `odometry_filter.cpp`, `replay.cpp`
Important note: to allow us to work properly on a shared file (github) I found a way to use a local
mads.ini, instead of the one contained in the root folder. When one of us modifies the local file and
commits, it updates the mads.ini of all the other.

Useful commands are found in the UserGuide document.

## HOW TO CLONE REPO AND RUN ON LOCAL MACHINE

You can clone in whichever folder you like, the repo contains a folder with all the data and the
mads.ini file starts from inside the SIMULATION folder.
To work with the current setup, we need to tell mads to use the mads.ini inside our local
folder.
The next steps explain how to do so:
- Clone the git repository:

      git clone https://github.com/LoreFranc/SUBMIT_RoboticPerception.git
  
- Navigate in the terminal inside the SIMULATION folder:

      cd SUBMIT_RoboticPerception/SIMULATION/
  
- Locate where your root mads.ini file is located:

      mads ini -i
  
This will give you an error like: Cannot write or overwrite "/usr/local/etc/mads.ini"
What we are going to do in the next step is removing the mads.ini inside the core of our pc and
directing our local file in its place, like a pointer.
- To make MADS use your local mads.ini file:

      sudo ln -sf $(pwd)/mads.ini /usr/local/etc/mads.ini
  
- Check if the connection has been made:

      ls -l /usr/local/etc/mads.ini
  
It should return something like:

    lrwxrwxrwx 1 root root 63 Feb 6 11:06 /usr/local/etc/mads.ini -> /home/lorenzo/SUBMIT_RoboticPerception/MADS/SIMULATION/mads.ini
            
If it does not work:

      sudo rm /usr/local/etc/mads.ini
      sudo ln -sf $(pwd)/mads.ini /usr/local/etc/mads.ini
            
The handy thing about this is that now you can edit mads.ini directly locally without having to
access protected folders. So, by making a change in Git, everyone's mads.ini gets updated.
If you want to use the original mads.ini, you'll need to edit it and fix the file paths.
Similarly, we need to link the plugins (odometry and replay) to our local build folders:
- Link the odometry_filter plugin:

      sudo ln -sf $(pwd)/odometry_filter/build/odometry_filter.plugin /usr/local/lib/odometry_filter.plugin
  
To test it:

    ls -l /usr/local/lib/odometry_filter.plugin
            
- Link the replay plugin:

      sudo ln -sf $(pwd)/replay_plugin/build/replay.plugin /usr/local/lib/replay.plugin
  
To test it:

      ls -l /usr/local/lib/replay.plugin
            
You are ready to run everything!
- To run the simulation:

      ./run_all.sh

This script checks beforehand if the plugins have been built, as git saves only the modifications to
the .cpp files in the src, it will automatically build the scripts for you. It might take a while
depending on your pc. After completion, the rerunner window will open and the simulation selected
in the mads.ini file will be launched.

- Important: if the simulation launches but there is a parsing error from rerun, there could be a version
problem. The version that runs with the simulator is: 0.25.1

To install it:

    pip install rerun-sdk==0.25.1
    
## Configurations of the mads.ini files for the simulations:

- 1st Dataset (the most complex one, with high slippage and aruco frequently out of frame):
  
[imu_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion 1stDataset/SensorFusion.imu.csv"

[encoders_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion 1stDataset/SensorFusion.encoders.csv"

[pose_htc_source]

there is no htc sensor in the simulation, leave all file paths commented

[pose_rs_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion
1stDataset/SensorFusion.H_initial_walker_aruco.csv"

[odometry_filter]

uncomment the line:
sub_topic = ["encoders_source", "imu_source", "pose_rs_source"]

Parameters:

Set aruco_is_walker_center = false

- 2nd Dataset (mid-difficulty, some slippage and aruco sometimes out of range):

[imu_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion 2ndDataset/SensorFusion.imu.csv"

[encoders_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion 2ndDataset/SensorFusion.encoders.csv"

[pose_htc_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion
2ndDataset/SensorFusion2.H_initial_walker_htc.csv"

[pose_rs_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion
2ndDataset/SensorFusion.H_initial_walker_aruco.csv"

[odometry_filter]

uncomment the line:
sub_topic = ["encoders_source", "imu_source", "pose_rs_source","pose_htc_source"]

Parameters:

Set aruco_is_walker_center = true

- 3rd Dataset (no slippage, aruco always in frame):

[imu_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion 3rdDataset/SensorFusion.imu.csv"

[encoders_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion 3rdDataset/SensorFusion.encoders.csv"

[pose_htc_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion
3rdDataset/SensorFusion2.H_initial_walker_htc.csv"

[pose_rs_source]

uncomment the line:
csv_file = "./data/SF_23.12.2025_UPDATE/Sensor Fusion
3rdDataset/SensorFusion.H_initial_walker_aruco.csv"

[odometry_filter]

uncomment the line:
sub_topic = ["encoders_source", "imu_source", "pose_rs_source","pose_htc_source"]

Parameters:

Set aruco_is_walker_center = true
