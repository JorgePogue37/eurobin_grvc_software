# filter_pcd
Simple code to clean .pcd files and remove outliers using the Statistical Outlier Removal filter.


```
mkdir build
cd build
cmake ..
make
```

Run with:
```
./filter_pcd path/to/input.pcd [path/to/output.pcd]
```
The code has an implementation of the Radius Outlier Removal filter commented out, if needed, uncomment the corresponding lines in the code.
