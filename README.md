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
