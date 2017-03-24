#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>

int
main(int argc, char **argv) {

    // for (int i = 0; i < 9; i++) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> finalCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut_new(new pcl::PointCloud<pcl::PointXYZI>);

    //if(i == 0) {
    if (pcl::io::loadPCDFile("cuberesult2.pcd", *cloudOut) == -1) {
        std::cout << argv[0] << std::endl;
        PCL_ERROR ("Couldn't read first file! \n");
        return (-1);
    }
    //} else {
    //if(pcl::io::loadPCDFile ("cuberesult.pcd", *cloudIn)==-1)
    //{
    // PCL_ERROR ("Couldn't read result file! \n");
    //  return (-1);
    // }
    //}

    //save aligned pair, transformed into the first cloud's frame
    //std::stringstream ss;
    //ss << "cube000" << i + 1 << ".pcd";

    //std::cout << ss.str() << std::endl;
    if (pcl::io::loadPCDFile("halfcube190filtered.pcd", *cloudIn) == -1) {
        PCL_ERROR ("Couldn't read second input file! \n");
        return (-1);
    }
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    //float theta = 0;//M_PI/2; // The angle of rotation in radians
    //transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
    transform_2.translation() << 0.0, -70.0, -752.0;
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloudOut, *transformed_cloud, transform_2);

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    // Parameters for the ICP algorithm

    icp.setInputSource(transformed_cloud);
    icp.setInputTarget(cloudIn);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-7);
    icp.setMaxCorrespondenceDistance(3);
    //icp.setEuclideanFitnessEpsilon(1);
    //icp.setRANSACOutlierRejectionThreshold(1);

    icp.align(*cloudOut);

    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    } else std::cout << "ICP did not converge." << std::endl;

    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    std::cout << "trans %n" << transformationMatrix << std::endl;

    pcl::transformPointCloud(*cloudOut, *cloudOut_new, transformationMatrix);

    finalCloud = *cloudIn + *cloudOut;

    pcl::io::savePCDFileASCII("cuberesult3.pcd", finalCloud);
    //}

    return (0);
}