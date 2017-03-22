#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/poisson.h>

int main (int argc, char** argv)
{
    std::cout << "Loading files" << std::endl;

    for (int i = 0; i < 10; i++) {

        // Load input file into a PointCloud<T> with an appropriate type
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        std::stringstream ss;
        if(i < 10) {
            ss << "cube000" << i << ".pcd";
        } else {
            ss << "cube00" << i << ".pcd";
        }

        pcl::PCLPointCloud2 cloud_blob;

        if (pcl::io::loadPCDFile (ss.str(), cloud_blob) == -1) {
            std::cout << "Couldn't read file " << ss.str() << std::endl;
        }

        pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

        // The data should be available in the *cloud

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_first(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_second(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Create the filtering object
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0.0, 1.0);
        pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_filtered_first);

        pass.setInputCloud(cloud_filtered_first);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-10, 10);
        pass.setFilterLimitsNegative(true);
        pass.filter(*cloud_filtered_second);

        // Remove points that are far away from other points
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
        outlier_filter.setInputCloud(cloud_filtered_second);
        outlier_filter.setRadiusSearch(0.8);
        outlier_filter.setMinNeighborsInRadius(2);
        outlier_filter.filter(*cloud_filtered);

        std::cout << "Saving filtered cloud" << std::endl;
        pcl::io::savePCDFile(ss.str(), *cloud_filtered);
    }

    return (0);
}