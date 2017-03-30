#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char *argv[])
{
    PointCloudT::Ptr cloud (new PointCloudT),
            cloud_tf_1 (new PointCloudT),
            cloud_tf_2 (new PointCloudT);

    if(pcl::io::loadPCDFile (argv[1], *cloud)==-1)
    {
        PCL_ERROR ("Couldn't read cloud file! \n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer viewer;
    viewer.addCoordinateSystem(15);
    //viewer.addPointCloud(cloud);

    Eigen::Affine3f transform (Eigen::Affine3f::Identity());
    // To change the rotation, first make a AngleAxis with the desired change of degrees and then specify the
    // Axis with a unitvector. To change multiple axes at once, multiply the desired AngleAxises with eachother
    long int deg = strtol(argv[2],NULL,10);


    transform.translation() << -528.0, -346.0, 591.0;
    pcl::transformPointCloud(*cloud, *cloud, transform);
    transform.translation() << 0.0, 0.0, 0.0;

    // Scale the point cloud
    float scale = atof(argv[3]);
    Eigen::Affine3f scale_transform(Eigen::Affine3f::Identity());
    scale_transform.scale(Eigen::Vector3f(1, 1, scale));
    pcl::transformPointCloud(*cloud, *cloud_tf_1, scale_transform);
    
    Eigen::Matrix3f rotation (Eigen::AngleAxisf((deg*M_PI) / 180, Eigen::Vector3f::UnitX()));
    transform.rotate(rotation);
    pcl::transformPointCloud(*cloud_tf_1, *cloud_tf_1, transform);
    std::cout << transform.matrix() << std::endl << std::endl;

    Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
    centroid_new.head<3>() = rotation * centroid.head<3>();
    transform.translation() = centroid.head<3>() - centroid_new.head<3>();
    pcl::transformPointCloud(*cloud, *cloud_tf_2, transform);
    std::cout << transform.matrix() << std::endl << std::endl;

    // White PC
    pcl::io::savePCDFileASCII ("cloud.pcd", *cloud);
    std::cout << "cloud saved to file cloud.pcd" << std::endl;

    // Green PC
    pcl::io::savePCDFileASCII ("cloudtf1.pcd", *cloud_tf_1);
    std::cout << "cloudtf1 saved to file cloudtf1.pcd" << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tf_1_color_handler (cloud_tf_1, 25, 200, 25);
    viewer.addPointCloud (cloud_tf_1, cloud_tf_1_color_handler, "cloud_tf_1");

    // Red PC
    pcl::io::savePCDFileASCII ("cloudtf2.pcd", *cloud_tf_2);
    std::cout << "cloudtf2 saved to file cloudtf2.pcd" << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tf_2_color_handler (cloud_tf_2, 200, 25, 25);
    //viewer.addPointCloud (cloud_tf_2, cloud_tf_2_color_handler, "cloud_tf_2");
    viewer.spin();
    return 0;
}
