# pcdto2dmap
Simple code to obtain a binary obstacle map from a 3D map in a .pcd file.

Modify the path to the .pcd file in the .cpp code to include your .pcd file.
Compile with:
```
mkdir build
cd build
cmake ..
make
```
Run simply with:
```
./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]
```
## Explanation.
Code will search for a .pcd file in the specified path, then set any points between the specified minimum and maximum heights as obstacles, creating a 2D map with the specified resolution, and then it will save the resulting 2D map in a .pgm file in the /build folder.
If no parameters are specified it's set to define any points between 0.5 and 5m as obstacles, and set a resolution of 25 cm², saving the file as obstacle_map.pgm in the build folder.
