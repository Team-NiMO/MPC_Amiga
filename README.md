# Description
This repository was used for our MRSD capstone's Fall Validation Demonstration (FVD). It is a global navigation program which runs a MPC controller to follow some waypoints. The following file will describe everything from collecting waypoints to setting up the amiga robot to run the controller which follows these waypoints. This repository is an extension of [MPC_Amiga](https://github.com/Kantor-Lab/MPC_Amiga) by the members of Kantor Lab at CMU.

# Installion
First, clone the repository into the 'src' folder of your ROS workspace.
'''
git clone git@github.com:Team-NiMO/MPC_Amiga.git
'''

The next step is to install the Acado tookkit. This implementation uses [Acado Toolkit](https://acado.github.io/index.html) to solve the optimization problem. We use qpoases to solve the linear quadratic programming. A python interface is also generated to use the solver with Python.

## Acado set up
- We need to set up and compile the problem definition. To do so, clone the Acado toolkit, and then place the file *src/simple_mpc_diff.cpp* in  *ACADOtoolkit/examples /getting_started/*. Then compile everything using cmake:
```
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
cd ACADOtoolkit
cp ${PATH_TO_CATKIN_WS}/src/MPC_Amiga/src/simple_mpc_diff.cpp ${PATH_TO_ACADO_TOOLKIT}/examples/getting_started/
mkdir build
cd build
cmake ..
make
```
- This will generate an executable in the folder *getting_started*. Run the executable to generate the files that will be used to generate the python API.
```
./simple_mpc_diff
```
- After running this executable, a folder named *simple_mpc_diff_export* will be generated inside *getting_started*.
- Copy the contents of *simple_mpc_diff_export* to the folder *{PATH_TO_CATKIN_WS}/src/MPC_Amiga/src/acado_diff/*. Replace the files, if asked.
- Now we have to compile the python API to import the solver:
```
cd ${PATH_TO_CATKIN_WS}/src/MPC_Amiga/src/acado_diff/
mkdir build_python
python setup.py build --build-base=build_python/
cp -i build_python/lib.linux-x86_64-2.7/acado.so ${PATH_TO_CATKIN_WS}/devel/lib/python3/dist-packages
```
- After these steps, we should be able to import acado from Python, while in the ROS workspace.

## Starting all the sensors
- The first step is to setup the RTK base station. Ensure that you don't set it too close to the testing location of the robot.
#INSERT IMAGE OF THE SETUP
- As seen in the above image, first connect the 5 pin Male power connector with DC jack connector at its end to the power connector on swift nav
- Then connect the DC jack to the 


Now we move on to the first step of collecting waypoints - 
## Collecting Waypoints
To collect waypoint, amiga robot needs to be in manual mode. The way this process works is as follows -
- The following command runs the script which collects waypoints
  '''
  cd /home/amiga/catkin_workspaces/nimo_ws/src/MPC_Amiga/scripts/
  python3 collect_goals_mrsd.py
  '''
- Move robot to the waypoint location you want to collect and enter 'y' in the terminal whenever you want to save the location coordinate as waypoint
- After you collect all the waypoints, simply exit from the script
- Collected waypoints will be at the following location -
  '''
  cd /home/amiga/catkin_workspaces/nimo_ws/src/MPC_Amiga/gps_coordinates/rows_1.txt
  '''
  NOTE: The waypoints collected before might still be there in the file, the new waypoints collected will be appended after them
  - Copy all the coordinates from the above file to 'barn_field_waypoints.txt' file which is located in the same folder
  NOTE: To avoid confusion because of the previous step, after you copy the points to 'barn_field_waypoints.txt', delete the points from rows_1.txt file (This issue needs to be fixed, and hopefully someday I will do it)
- Copy the last coordinate to pruning_points_real.txt file twice - this specifies the last waypoint (i.e. indicating the robot has reached the end of row), after which the robot should navigate to the next row
- Copy other coordinates (for our usecase: except the first point because that specifies the barn waypoint) to stopping_points.txt file - this specifies the waypoint locations the robot should stop at

At this point you are all ready to start the controller

