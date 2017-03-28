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

/**
   This function filters a cloud by performing a few steps:
   * It removes all noise data and outliers
   * It scales the cloud to the correct proportions
   * It makes sure the cloud has the correct rotation and position around origin
   @param cloud_in The input cloud, taken directly from a TreeD scan.
   @param cloud_out The filtered and scaled cloud is returned here.
   @param rotation The rotation the scan was taken from (in degrees).
 */
void
filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, int rotation)
{
    // Use a pass through filter to remove all points outside of specific coordinates
    pcl::PassThrough<pcl::PointXYZ> pt_filter;
    pt_filter.setInputCloud(cloud_in);

    // Filter on the x axis
    // This will remove the stick the object is attached to
    pt_filter.setFilterFieldName("x");
    pt_filter.setFilterLimits(528, 568);
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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char **argv) {
    PointCloud::Ptr cloud(new PointCloud), cloud_tf_1(new PointCloud);

    if (std::string(argv[1]) == "all") {
        int degs[] = {0, 45, 90, 135, 180, 225, 270, 315};

        for (size_t i = 0; i < (sizeof(degs)/sizeof(*degs)); ++i) {
            std::stringstream ss;
            ss << "bridge/bridge-" << degs[i] << ".pcd";
            std::cout << "Loading file " << ss.str() << std::endl;

            if (pcl::io::loadPCDFile(ss.str(), *cloud) == -1) {
                PCL_ERROR ("Couldn't read cloud file! \n");
                return -1;
            }
            std::stringstream ss2;
            ss2 << "sticks/bridge-" << degs[i] << ".pcd";

            filter(cloud, cloud_tf_1, degs[i]);
            pcl::io::savePCDFile(ss2.str(), *cloud_tf_1);
        }

        return 0;
    }

    std::cout << "Loading file " << argv[1] << std::endl;

    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
        PCL_ERROR ("Couldn't read cloud file! \n");
        return -1;
    }

    float scale = 0.001;
    Eigen::Affine3f scale_transform(Eigen::Affine3f::Identity());
    scale_transform.scale(Eigen::Vector3f(scale, scale, scale));
    pcl::transformPointCloud(*cloud, *cloud, scale_transform);

    pcl::visualization::PCLVisualizer viewer;
    viewer.addCoordinateSystem(0.01);
    //viewer.addPointCloud(cloud);

    PointCloud::Ptr cloud_f(new PointCloud);


    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_filtered = *cloud;


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    float leaf_size = 0.001f;
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    //vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
    //viewer.addPointCloud(cloud_filtered, "cloud_filtered");

    /*
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.005);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    */

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.004);
    ec.setMinClusterSize (40);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        pcl::io::savePCDFile(ss.str(), *cloud_cluster);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_cluster, rand()%255, rand()%255, rand()%255);
        viewer.addPointCloud(cloud_cluster, color, ss.str());
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
    std::cout << "Number of clusters: " << j << std::endl;

    //viewer.addPointCloud(cloud_out, "cloud_out");
    viewer.spin();

    return 0;
}
