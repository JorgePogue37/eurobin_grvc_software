#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

int main(int argc, char** argv){
    cv::String pcd_path = "/home/grvc/Escritorio/filtered_scans_obstaculos.pcd";
    cv::String pgm_filename = "obstacle_map.pgm";

    // Define height to mark as obstacle
    float height_threshold_min = 0.5; // 0.5 meters height threshold to be considered obstacle
    float height_threshold_max = 5; // above 5 meters is considered ceiling
    float grid_resolution = 0.25; // 25cm² per grid cell
    int iter = 2;
    if(argc<2){
        std::cout << "Using default arguments." << std::endl;
    }
    else if(argc == 2){
        pcd_path = std::string(argv[1]);
        std::cout << "Using file at: \n" << pcd_path << "\nAnd default heights." << std::endl;
    }
    else if(argc == 4){
        height_threshold_min = std::atof(argv[2]);
        height_threshold_max = std::atof(argv[3]);
        if(height_threshold_min > height_threshold_max){
           std::cout << "Minimum height can't be larger than maximum height. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }
        pcd_path = std::string(argv[1]);
        std::cout << "Using file at:" << pcd_path << std::endl;
    }
    else if(argc == 5){
        height_threshold_min = std::atof(argv[2]);
        height_threshold_max = std::atof(argv[3]);
        grid_resolution = std::atof(argv[4]);
        if(height_threshold_min > height_threshold_max){
           std::cout << "Minimum height can't be larger than maximum height. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }
        if(grid_resolution <= 0){
           std::cout << "Grid resolution must be a positive float. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }
        pcd_path = std::string(argv[1]);
        std::cout << "Using file at:" << pcd_path << std::endl;
    }
    else if(argc == 6){
        height_threshold_min = std::atof(argv[2]);
        height_threshold_max = std::atof(argv[3]);
        grid_resolution = std::atof(argv[4]);
        iter = std::atof(argv[5]);
        if(height_threshold_min > height_threshold_max){
           std::cout << "Minimum height can't be larger than maximum height. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }
        if(grid_resolution <= 0){
           std::cout << "Grid resolution must be a positive float. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }
        if(iter <= 0){
           std::cout << "Size of obstacles must be greater than 0. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }

        pcd_path = std::string(argv[1]);
        std::cout << "Using file at:" << pcd_path << std::endl;
    }
    else if(argc == 7){
        height_threshold_min = std::atof(argv[2]);
        height_threshold_max = std::atof(argv[3]);
        grid_resolution = std::atof(argv[4]);
        iter = std::atof(argv[5]);
        if(height_threshold_min > height_threshold_max){
           std::cout << "Minimum height can't be larger than maximum height. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }
        if(grid_resolution <= 0){
           std::cout << "Grid resolution must be a positive float. \nUsage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
        }
        pcd_path = std::string(argv[1]);
        pgm_filename = std::string(argv[6]) + ".pgm";
        std::cout << "Using file at:" << pcd_path << std::endl;
    }
    else{
        std::cout << "Unexpected error. Usage: ./pcd_to_2dmap [/path/to/file.pcd] [minimum height in meters] [maximum height in meters] [grid resolution in meters] [integer, size of obstacles] [name for save file]" << std::endl;
        return -1;
    }

    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1){
        PCL_ERROR("Couldn't read the PCD file");
        return -1;
    }

    // Finding map dimensions
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& point : cloud->points) {
        if (point.x < min_x) min_x = point.x;
        if (point.x > max_x) max_x = point.x;
        
        if (point.y < min_y) min_y = point.y;
        if (point.y > max_y) max_y = point.y;
        
        if (point.z < min_z) min_z = point.z;
        if (point.z > max_z) max_z = point.z;
    }

    float width = max_x - min_x;
    float height = max_z - min_z;
    float depth = max_y - min_y;

    std::cout << "Width: " << width << " meters" << std::endl;
    std::cout << "Depth: " << depth << " meters" << std::endl;
    std::cout << "Height: " << height << " meters" << std::endl;
    std::cout << "Minimum height for obstacle: " << height_threshold_min << " meters" << std::endl;
    std::cout << "Maximum height for obstacle: " << height_threshold_max << " meters" << std::endl;
    std::cout << "Grid resolution: " << grid_resolution << "^2 meters squared per cell" << std::endl;
    /*std::cout << "xmin: " << min_x << " meters" << std::endl;
    std::cout << "xmax: " << max_x << " meters" << std::endl;
    std::cout << "ymin: " << min_y << " meters" << std::endl;
    std::cout << "ymax: " << max_y << " meters" << std::endl;
    std::cout << "zmin: " << min_z << " meters" << std::endl;
    std::cout << "zmax: " << max_z << " meters" << std::endl;*/

    int grid_width = static_cast<int>(width / grid_resolution);
    int grid_height = static_cast<int>(depth / grid_resolution); // Yes, this is intentional. Naming conventions for 2d-3d spaces, not my fault.

    std::cout << "grid w: " << grid_width << " cells" << std::endl;
    std::cout << "grid h: " << grid_height << " cells" << std::endl;

    // Create a 2D grid (vector of vectors) initialized to 0 (free space).
    std::vector<std::vector<int>> obstacle_map(grid_height, std::vector<int>(grid_width, 0));

    
    // Calculate map grid.
    for (const auto& point : cloud->points) {
        if (point.z > height_threshold_min && point.z < height_threshold_max) {
            int x_index = static_cast<int>((point.x + width / 2) / grid_resolution);
            int y_index = static_cast<int>((point.y + depth / 2) / grid_resolution); // Yes, this is intentional. Naming conventions for 2d-3d spaces, not my fault.

            if (x_index >= 0 && x_index < grid_width && y_index >= 0 && y_index < grid_height) {
                obstacle_map[y_index][x_index] = 1; // Mark as obstacle
            }
        }
    }

    // Create an OpenCV matrix to store the image
    cv::Mat image(grid_height, grid_width, CV_8UC1);

    // Convert the binary map to an image
    for (int i = 0; i < grid_height; i++) {
        for (int j = 0; j < grid_width; j++) {
            // Set pixels: 255 for free space (0) and 0 for obstacles (1)
            image.at<uchar>(i, j) = obstacle_map[grid_height-1-i][j] == 0 ? 255 : 0;
        }
    }
    cv::erode(image,image,cv::Mat(),cv::Point(-1,-1),iter);
    // Optionally, resize the image for better visualization
    // cv::resize(image, image, cv::Size(900,0.832*900), 0, 0, cv::INTER_NEAREST);
    cv::imwrite(pgm_filename, image);
    std::cout << "Map saved as " << pgm_filename << std::endl;
    // Display the image
    cv::imshow("Obstacle Map", image);
    std::cout << "Displaying map. Press any key to close the window." << std::endl;
    cv::waitKey(0); // Wait for a key press to close the window
    return 0;
}
