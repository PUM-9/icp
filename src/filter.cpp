#include <iostream>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>

int main(int argc, char **argv) {
    std::cout << "Loading files" << std::endl;

    //for (int i = 0; i < 10; i++) {

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::stringstream ss;
    /*if(i < 10) {
        ss << "cube000" << i << ".pcd";
    } else {
        ss << "cube00" << i << ".pcd";
    }*/
    ss << "halfcube190.pcd";

    pcl::PCLPointCloud2 cloud_blob;

    if (pcl::io::loadPCDFile(ss.str(), cloud_blob) == -1) {
        std::cout << "Couldn't read file " << ss.str() << std::endl;
    }

    pcl::fromPCLPointCloud2(cloud_blob, *cloud);

    // The data should be available in the *cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_first(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_second(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_third(new pcl::PointCloud<pcl::PointXYZ>);


    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(400, 513);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_filtered_first);

    /*pass.setInputCloud(cloud_filtered_first);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-300, 0);
    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud_filtered_second);*/

    pass.setInputCloud(cloud_filtered_first);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1, 1);
    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud_filtered_third);

    // Remove points that are far away from oneanother points
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud_filtered_third);
    outlier_filter.setRadiusSearch(0.8);
    outlier_filter.setMinNeighborsInRadius(2);
    outlier_filter.filter(*cloud_filtered);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    float theta = M_PI/2; // The angle of rotation in radians
    //transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
    //transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
    //transform_2.translation() << 0.0, 200.0, 0.0;
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform_2);

    std::cout << "Saving filtered cloud" << std::endl;
    pcl::io::savePCDFile("halfcube190filtered.pcd", *transformed_cloud);
    // }

    return (0);
}
