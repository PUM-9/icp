#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef int degree;

struct Rectangle {
    CloudPtr point_cloud_ptr;
    Point origo;
    Point x;
    Point y;
    Point xy;
    degree pov;


};

struct Cuboid {
    CloudPtr point_cloud_ptr;
    Point origo;
    Point x;
    Point y;
    Point z;
    Point xy;
    Point yz;
    Point xz;
    Point xyz;
    degree pov;

};
/**
 * This function first rotates the Cubiod to match the Rectangle. It then translates the Cuboid to match the Rectangles
 * coodinates. And finaly runs the ICP algorithm to register the pointclouds.
 *
 * @param target    The Cuboid object containing the target pointcloud
 * @param source    The Rectangle object containing the source pointcloud
 * @return Ture if ICP converges false otherwise
 */
bool register_point_clouds_icp(Cuboid target, Rectangle source) {

    CloudPtr SourceCloud = source.point_cloud_ptr;
    CloudPtr TargetCloud = target.point_cloud_ptr;
    CloudPtr TargetCloudNew(new Cloud);
    CloudPtr FinalCloud(new Cloud);


    Eigen::Affine3f TransformRotate = Eigen::Affine3f::Identity();
    Eigen::Affine3f TransformTranslate = Eigen::Affine3f::Identity();

    // Compute rotation
    float theta = source.pov - target.pov; // The angle of rotation in radians
    TransformRotate.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // Executing the rotation
    CloudPtr transformed_cloud(new Cloud);
    pcl::transformPointCloud(*TargetCloud, *transformed_cloud, TransformRotate);

    // Update target
    target.point_cloud_ptr = transformed_cloud;
    //find_cuboid_corner(target);

    // Compute the translation
    float translation_x = source.origo.x - target.origo.x;
    float translation_y = source.origo.y - target.origo.y;
    float translation_z = source.origo.z - target.origo.z;

    // Execute the translation
    TransformTranslate.translation() << translation_x, translation_y, translation_z;

    // Start ICP
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    // Parameters for the ICP algorithm
    icp.setInputSource(SourceCloud);
    icp.setInputTarget(TargetCloud);
    icp.setMaximumIterations(25);
    icp.setTransformationEpsilon(1e-7);
    icp.setMaxCorrespondenceDistance(3);
    //icp.setEuclideanFitnessEpsilon(1);
    //icp.setRANSACOutlierRejectionThreshold(1);

    icp.align(*TargetCloud);

    if (icp.hasConverged()) {
        std::cout << "ICP converged." << std::endl
                  << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
        std::cout << "trans %n" << transformationMatrix << std::endl;

        pcl::transformPointCloud(*TargetCloud, *TargetCloudNew, transformationMatrix);

        FinalCloud = SourceCloud + TargetCloud;

        pcl::io::savePCDFileASCII("cuberesult3.pcd", *FinalCloud);
        return true;
    }

    else std::cout << "ICP did not converge." << std::endl;

    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    std::cout << "trans %n" << transformationMatrix << std::endl;

    pcl::transformPointCloud(*TargetCloud, *TargetCloudNew, transformationMatrix);

    FinalCloud = SourceCloud + TargetCloud;

    pcl::io::savePCDFileASCII("cuberesult3.pcd", *FinalCloud);

    return false;

}

int main(int argc, char **argv) {

    // Crerate target and source CloudPtr
    CloudPtr TargetCloudRead(new Cloud);
    CloudPtr SourceCloudRead(new Cloud);

    // Read Target pointcloud
    if (pcl::io::loadPCDFile("cuberesult2.pcd", *TargetCloudRead) == -1) {
        std::cout << argv[0] << std::endl;
        PCL_ERROR ("Couldn't read first file! \n");
        return (-1);
    }

    // Read source pointcloud
    if (pcl::io::loadPCDFile("halfcube190filtered.pcd", *SourceCloudRead) == -1) {
        PCL_ERROR ("Couldn't read second input file! \n");
        return (-1);
    }

    // Create Cuboid object
    Cuboid Target;
    Target.point_cloud_ptr = TargetCloudRead;
    //find_cuboid_corners(Target);

    // Create Rectangle object
    Rectangle Source;
    Source.point_cloud_ptr = SourceCloudRead;
    //find_rectangle_corner(Source);

    // Run registation functrion
    if(register_point_clouds_icp(Target, Source) == false){
        PCL_ERROR ("Registration failed! \n");
        return (-1);
    }
    //register_point_clouds_icp(Target, Source);


    return (0);
}



