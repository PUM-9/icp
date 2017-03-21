#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>

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
  viewer.addCoordinateSystem(0.5);
  viewer.addPointCloud(cloud);

  Eigen::Affine3f transform (Eigen::Affine3f::Identity());
  Eigen::Matrix3f rotation (Eigen::AngleAxisf((60.0*M_PI) / 180, Eigen::Vector3f::UnitZ()));
  transform.rotate(rotation);
  pcl::transformPointCloud(*cloud, *cloud_tf_1, transform);
  std::cout << transform.matrix() << std::endl << std::endl;

  Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
  pcl::compute3DCentroid(*cloud, centroid);
  Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
  centroid_new.head<3>() = rotation * centroid.head<3>();
  transform.translation() = centroid.head<3>() - centroid_new.head<3>();
  pcl::transformPointCloud(*cloud, *cloud_tf_2, transform);
  std::cout << transform.matrix() << std::endl << std::endl;

  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tf_1_color_handler (cloud_tf_1, 25, 200, 25);
  viewer.addPointCloud (cloud_tf_1, cloud_tf_1_color_handler, "cloud_tf_1");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tf_2_color_handler (cloud_tf_2, 200, 25, 25);
  viewer.addPointCloud (cloud_tf_2, cloud_tf_2_color_handler, "cloud_tf_2");
  viewer.spin();
  return 0;
}