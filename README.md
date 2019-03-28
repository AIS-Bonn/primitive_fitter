![alt text](misc/teaser_full.jpg?raw=true)
# NimbRo Primitive Fitter
This package fits primitive shapes (capsules and boxes) to complex meshes to reduce computational complexity in simulations and planning tasks. 
## Installation
### Prerequisites:

```
sudo apt install libtool libltdl-dev gfortran liblog4cxx-dev coinor-libipopt-dev libpugixml-dev
```

### The roboptim packages:
#### Clone: 
```
git clone --recursive https://github.com/roboptim/roboptim-core.git
git clone --recursive https://github.com/roboptim/roboptim-core-plugin-ipopt.git
git clone --recursive https://github.com/roboptim/roboptim-capsule.git
```

#### Build and install:
**Follow this order:**  roboptim-core > roboptim-core-plugin-ipopt > roboptim-capsule
(More detailed instructions can be found within the roboptim-core module)

Navigate to respective projects and: 
```
mkdir build          # (1) Create a build directory
cd build             # (2) Go to the newly created build directory
cmake [options] ..   # (3) Generate the build files

make                 # (4) Compile the package
make install         # (5) Install the package into the prefix 
```
*Change the `CMAKE_INSTALL_PREFIX` env variable if you want the installation to be in a non-standard location (for example, if you don't have sudo privileges).*

If using non-standard location be sure to execute immediately after installing roboptim-core:

```
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:<CMAKE_INSTALL_PREFIX>/lib/pkgconfig/
```
For your convinience, you might want to add this to your `~/.bashrc`. 

# How to use?

## Example
This package comes with examples which fits capsules or boxes to the links of an UR10 robotic arm and the NimbRo-Op2X robot. You will need the `ur_description` or the `nimbro_op_model`(https://github.com/AIS-Bonn/humanoid_op_ros/tree/master/src/nimbro/hardware/nimbro_op_model) ROS packages to execute the launch files. Once you have this packages, you can try for example: 
```
roslaunch nimbro_primitive_fitter capsule_ur10.launch
```
or 
```
roslaunch nimbro_primitive_fitter box_op2x.launch
```
The launch-file specifies the URDF description and **generates a new file** with the corresponding primitives. 
To make this pkg work with your model, simply change the corresponding ros parameters in the launch file. 
The box-fitting comes with a number of parameters, which might be required to change for specific meshes. If required, you can find the description of those parameters in the [ApproxMVBB Documentation](https://github.com/gabyx/ApproxMVBB/).

## Contributing
### How to add new primitive shapes

Create a class that inherits from `ShapeFit`, thus you need to define the functions `writeUrdf` and `writeUrdfXacro`. 
These functions are supposed to write the fitted shape to the URDF XML. 
Additionally, you will need to define a `static addMacroXacro` method, which writes the Xacro Macro that is neccessary in `writeUrdfXacro`.
Finally, the `addMacroXacro` needs to be included in the `Urdf::addXacroDef` function. 
For a simple non-macro example, refer to BoxFit, for help on using macros, refer to the CapsuleFit class. 

### How to add support for different mesh-files

Firstly, a class inheriting from `Meshfile` needs to be created, thus including a `getPoints` and `getTransformationMatrix` function.
The former returns the points of the mesh in form of a flat vector of coordinates like *(x1, y1, z1, x2, y2, z2,...)*. 
The latter returns the intrinsic transformation of the mesh, like it is neccessary e.g. for collada files. 
Finally, the file has to be integrated to the `VersatileFitter::fit` function, solely to check for file extensions.

