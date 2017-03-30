#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

bool remove_stick(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out)
{
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    search_tree->setInputCloud(cloud_in);

    // Create the object for extracting cluster in cloud_in
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setInputCloud(cloud_in);

    // Set the maximum distance between two points in a cluster to 4 mm
    cluster_extraction.setClusterTolerance(4.0);

    // Set a cluster to be between 10 and 25000 points
    cluster_extraction.setMinClusterSize(10);
    cluster_extraction.setMaxClusterSize(25000);

    // Perform the euclidean cluster extraction algorithm
    cluster_extraction.setSearchMethod(search_tree);
    cluster_extraction.extract(cluster_indices);

    // Generate all the individual cluster point clouds
    std::vector<PointCloud::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cluster(new PointCloud);

        // For every list of indices, add all the points to a point cloud
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(cloud_in->points[*pit]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    // If we have exactly two clusters one will be the object and one will be the stick.
    // Look at which object is higher up to figure out which is which.
    if (clusters.size() == 2) {
        // Compute the centroids for both clusters to find out where they are located
        Eigen::Vector4f first_centroid(Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*clusters.at(0), first_centroid);

        Eigen::Vector4f second_centroid(Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*clusters.at(1), second_centroid);

        // Set the correct output cloud based on the x (height) value of the centroids
        if (first_centroid(0, 0) > second_centroid(0, 0)) {
            *cloud_out = *clusters.at(1);
        } else {
            *cloud_out = *clusters.at(0);
        }

        return true;
    } else {
        // We can't figure out where the stick is by clustering, perform a simple pass through filter instead
        pcl::PassThrough<pcl::PointXYZ> pt_filter;
        pt_filter.setInputCloud(cloud_in);
        pt_filter.setFilterFieldName("x");
        pt_filter.setFilterLimits(100, 510);
        pt_filter.setFilterLimitsNegative(false);
        pt_filter.filter(*cloud_out);

        return false;
    }
}

bool stick(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out)
{
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    search_tree->setInputCloud(cloud_in);

    // Create the object for extracting cluster in cloud_in
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setInputCloud(cloud_in);

    // Set the maximum distance between two points in a cluster to 4 mm
    cluster_extraction.setClusterTolerance(4.0);

    // Set a cluster to be between 10 and 25000 points
    cluster_extraction.setMinClusterSize(10);
    cluster_extraction.setMaxClusterSize(25000);

    // Perform the euclidean cluster extraction algorithm
    cluster_extraction.setSearchMethod(search_tree);
    cluster_extraction.extract(cluster_indices);

    // Generate all the individual cluster point clouds
    std::vector<PointCloud::Ptr> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cluster(new PointCloud);

        // For every list of indices, add all the points to a point cloud
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(cloud_in->points[*pit]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    // If we have exactly two clusters one will be the object and one will be the stick.
    // Look at which object is higher up to figure out which is which.
    if (clusters.size() == 2) {
        // Compute the centroids for both clusters to find out where they are located
        Eigen::Vector4f first_centroid(Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*clusters.at(0), first_centroid);

        Eigen::Vector4f second_centroid(Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*clusters.at(1), second_centroid);

        // Set the correct output cloud based on the x (height) value of the centroids
        if (first_centroid(0, 0) < second_centroid(0, 0)) {
            *cloud_out = *clusters.at(1);
        } else {
            *cloud_out = *clusters.at(0);
        }

        return true;
    } else {
        // We can't figure out where the stick is by clustering, perform a simple pass through filter instead
        pcl::PassThrough<pcl::PointXYZ> pt_filter;
        pt_filter.setInputCloud(cloud_in);
        pt_filter.setFilterFieldName("x");
        pt_filter.setFilterLimits(100, 510);
        pt_filter.setFilterLimitsNegative(false);
        pt_filter.filter(*cloud_out);

        return false;
    }
}

void filter(PointCloud::ConstPtr cloud_in, PointCloud::Ptr cloud_out, int rotation)
{
    // Use a pass through filter to remove all points outside of specific coordinates
    pcl::PassThrough<pcl::PointXYZ> pt_filter;
    pt_filter.setInputCloud(cloud_in);

    // Do initial rough filtering on the x axis to remove noise data
    pt_filter.setFilterFieldName("x");
    pt_filter.setFilterLimits(100, 568);
    pt_filter.setFilterLimitsNegative(false);
    pt_filter.filter(*cloud_out);

    // Filter on the z axis to remove the plane of noise data in the
    // beginning of the scan
    pt_filter.setInputCloud(cloud_out);
    pt_filter.setFilterFieldName("z");
    pt_filter.setFilterLimits(-1, 1);
    pt_filter.setFilterLimitsNegative(true);
    pt_filter.filter(*cloud_out);

    // Filter on the y axis
    pt_filter.setInputCloud(cloud_out);
    pt_filter.setFilterFieldName("y");
    pt_filter.setFilterLimits(100, 580);
    pt_filter.setFilterLimitsNegative(false);
    pt_filter.filter(*cloud_out);

    // Remove points that are far away from other points
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud_out);
    outlier_filter.setRadiusSearch(0.8);
    outlier_filter.setMinNeighborsInRadius(2);
    outlier_filter.filter(*cloud_out);

    // Remove the stick the object is attached to
    remove_stick(cloud_out, cloud_out);

    // Translate the object to move the center of the object to the origin (approximately).
    // This works but should be done in a better way. Right now these values
    // will be wrong if the scanner hardware is moved.
    Eigen::Affine3f translation_transform(Eigen::Affine3f::Identity());
    translation_transform.translation() << -528.0, -346.0, 591.0;
    pcl::transformPointCloud(*cloud_out, *cloud_out, translation_transform);

    // Scale the point cloud by half to make it the correct proportions.
    // The scaling factor 0.5 assumes the scans are run with cart speed 200 mm/s.
    float scale = 0.5;
    Eigen::Affine3f scale_transform(Eigen::Affine3f::Identity());
    scale_transform.scale(Eigen::Vector3f(1, 1, scale));
    pcl::transformPointCloud(*cloud_out, *cloud_out, scale_transform);

    // Rotate the object around the x axis to match the objects real world rotation
    Eigen::Matrix3f rotation_matrix(Eigen::AngleAxisf((rotation*M_PI) / 180, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f rotation_transform(Eigen::Affine3f::Identity());
    rotation_transform.rotate(rotation_matrix);
    pcl::transformPointCloud(*cloud_out, *cloud_out, rotation_transform);

    return;
}

int main(int argc, char **argv)
{
    PointCloud::Ptr cloud_in(new PointCloud), cloud_out(new PointCloud),
                    cloud_in2(new PointCloud);

    if (std::string(argv[1]) == "all") {
        int degs[] = {0, 45, 90, 135, 180, 225, 270, 315};

        for (size_t i = 0; i < (sizeof(degs)/sizeof(*degs)); ++i) {
            std::stringstream ss;
            ss << "bridge/bridge-" << degs[i] << ".pcd";
            std::cout << "Loading file " << ss.str() << std::endl;

            if (pcl::io::loadPCDFile(ss.str(), *cloud_in) == -1) {
                PCL_ERROR ("Couldn't read cloud file! \n");
                return -1;
            }
            std::stringstream ss2;
            ss2 << "sticks/bridge-" << degs[i] << ".pcd";

            filter(cloud_in, cloud_out, degs[i]);
            pcl::io::savePCDFile(ss2.str(), *cloud_out);
        }

        return 0;
    }

    std::cout << "Loading file " << argv[1] << std::endl;

    if (pcl::io::loadPCDFile(argv[1], *cloud_in) == -1) {
        PCL_ERROR("Couldn't read cloud file! \n");
        return -1;
    }

    if (pcl::io::loadPCDFile(argv[2], *cloud_in2) == -1) {
        PCL_ERROR ("Couldn't read cloud file! \n");
        return -1;
    }

    pcl::visualization::PCLVisualizer viewer;
    viewer.addCoordinateSystem(10);
    //viewer.addPointCloud(cloud);

    if (!stick(cloud_in, cloud_out)) {
        PCL_ERROR("Couldn't generate stick \n");
        return -1;
    }

    PointCloud::Ptr stick1(new PointCloud);
    stick(cloud_in, stick1);
    Eigen::Vector4f stick_centroid(Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*stick1, stick_centroid);
    std::cout << stick_centroid.head<3>() << std::endl;

    Eigen::Affine3f translation_transform(Eigen::Affine3f::Identity());
    translation_transform.translation() << 0.0, -stick_centroid(1, 0), -stick_centroid(2, 0);
    pcl::transformPointCloud(*cloud_in, *cloud_in, translation_transform);


    PointCloud::Ptr stick2(new PointCloud);
    stick(cloud_in2, stick2);
    Eigen::Vector4f stick2_centroid(Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*stick2, stick2_centroid);
    std::cout << stick2_centroid.head<3>() << std::endl;

    Eigen::Affine3f translation2_transform(Eigen::Affine3f::Identity());
    translation2_transform.translation() << 0.0, -stick2_centroid(1, 0), -stick2_centroid(2, 0);
    pcl::transformPointCloud(*cloud_in2, *cloud_in2, translation2_transform);


    //viewer.addPointCloud(cloud_in, "cloud_in");
    //viewer.addPointCloud(cloud_in2, "cloud_in2");

    remove_stick(cloud_in, cloud_in);
    remove_stick(cloud_in2, cloud_in2);

    std::cout << "Downsampling" << std::endl;
    float leaf_size = 1.5f;
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_in);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud(cloud_in2);
    sor2.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor2.filter(*cloud_in2);



    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Parameters for the ICP algorithm
    icp.setInputSource(cloud_in2);
    icp.setInputTarget(cloud_in);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-4);
    icp.setMaxCorrespondenceDistance(5);
    icp.setEuclideanFitnessEpsilon(0.1);
    icp.setRANSACOutlierRejectionThreshold(5);

    std::cout << "Performing ICP" << std::endl;
    icp.align(*cloud_in2);

    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }
    else std::cout << "ICP did not converge." << std::endl;

    pcl::transformPointCloud(*cloud_in2, *cloud_in2, icp.getFinalTransformation());

    PointCloud::Ptr final_cloud(new PointCloud);
    *final_cloud = *cloud_in + *cloud_in2;

    std::stringstream ss;
    ss << "gicp/" << argv[1];
    std::cout << "Saving file " << ss.str() << std::endl;
    pcl::io::savePCDFileASCII (ss.str(), *final_cloud);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 200, 25, 25);
    viewer.addPointCloud(cloud_in, cloud_in_color, "cl_in");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in2_color(cloud_in2, 25, 200, 25);
    viewer.addPointCloud(cloud_in2, cloud_in2_color, "cl_in2");

    viewer.spin();

    return 0;
}
