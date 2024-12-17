cd filter_pcd/build
cp ../../files/scans.pcd ~/catkin_ws/src/pcd_map_viewer/maps
./filter_pcd ../../files/scans.pcd ../../files/filteredPointCloud.pcd
cd ../../pcdto2dmap/build
./pcd_to_2dmap ../../files/filteredPointCloud.pcd 0 3 0.1 5 ../../files/obstacles2dmap
cp ../../files/obstacles2dmap.pgm ~/catkin_ws/src/target_point_publisher/map/output.pgm
