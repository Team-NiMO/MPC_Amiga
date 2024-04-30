# MPC_Controller
This implementation uses [Acado Toolkit](https://acado.github.io/index.html) to solve the optimization problem. We use qpoases to solve the linear quadratic programming. A python interface is also generated to use the solver with Python.

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

## Running the controller
- Option 1: The version of the controller that has been tested in field is [v4](https://github.com/Kantor-Lab/MPC_Amiga/blob/master/scripts/mpc_warthog_v4_field.py). It implements a basic MPC to follow waypoints. They can be gathered by remote controllig the robot on the desired path, or using a drone map and giving waypoints. This version only is used to do "global navigation", which basically is driving the robot from the barn to the field.
The controller will read the waypoints from the file [barn_field_waypoints](https://github.com/Kantor-Lab/MPC_Amiga/blob/master/gps_coordinates/barn_field_waypoints.txt). I
    If the robot needs to stop in positions while executing the path, the stopping points need to be set in the file [stopping_points](https://github.com/Kantor-Lab/MPC_Amiga/blob/master/gps_coordinates/stopping_points.txt). In order to ensure the information is accessible to the robot in case of unexpected stops (e.g., e-button push or killing the control node), a copy of the stopping points need to be saved as [stopping_points_copied](https://github.com/Kantor-Lab/MPC_Amiga/blob/master/gps_coordinates/stopping_points_copied.txt).

    To run, just launch the controller as:
    ```
    roslaunch mpc_amiga mpc_amiga.launch fresh_start:=1
    ```
    The fresh_start parameter tells the controller if it is starting from the barn (very begining of the path) or somewhere in the middle of the path.

- Option 2: This version of the controller is written for integration with the [corn insertion team](https://github.com/Team-NiMO/NiMo-FSM). The desired behavior of the controller here is to handle "global navigation" and "local navigation". Global navigation is the case mentioned above. Local navigation default behaviour will wait for the planner given by [amiga path planner](https://github.com/Kantor-Lab/amiga_path_planning/tree/main) and will execute it.

    In order to make a difference between the two cases, three parameters are used:
    *fresh_start*: 1 if we are starting at the begining of the path, 0 if needed to stop and restarting the run. Applies for both local and global navigation. Does not have a default, need to be set.
    *barn_field*: True for global navigation, false for local navigation. Default is True.
    *load_backup_plan*: This is a debug flag. Every time the controller receives a path from the planner, it saves it to a [file](https://github.com/Kantor-Lab/MPC_Amiga/blob/master/gps_coordinates/field_waypoints.txt). If for some reason, we need to load a previously generated path, set this flag to True. Default is False.

    To run the controller in mode "local navigation", do
    ```
    roslaunch mpc_amiga mpc_amiga_corn_ins.launch fresh_start:=1 barn_field:=false load_backup_plan:=false
    ```
    This will wait for a plan generated by the planner in order to follow it.


