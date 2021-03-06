#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>

int
 main (int argc, char** argv)
{

     for (int i = 0; i < 1; i++) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> finalCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut_new (new pcl::PointCloud<pcl::PointXYZI>) ;

        if(i == 0) {
            if(pcl::io::loadPCDFile (argv[1], *cloudIn)==-1)
            {
                PCL_ERROR ("Couldn't read first file! \n");
                return (-1);
            }
        } else {
            if(pcl::io::loadPCDFile ("cuberesult.pcd", *cloudIn)==-1)
            {
                PCL_ERROR ("Couldn't read result file! \n");
                return (-1);
            }
        }

        //save aligned pair, transformed into the first cloud's frame
        std::stringstream ss;
        ss << "cube000" << i + 1 << ".pcd";

        std::cout << ss.str() << std::endl;
         if(i == 0) {
             if(pcl::io::loadPCDFile (argv[2], *cloudOut)==-1)
             {
                 PCL_ERROR ("Couldn't read first file! \n");
                 return (-1);
             }
         } else {
             if(pcl::io::loadPCDFile (ss.str(), *cloudOut)==-1)
             {
                 PCL_ERROR ("Couldn't read result file! \n");
                 return (-1);
             }
         }

        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

        // Parameters for the ICP algorithm

        icp.setInputSource(cloudOut);
        icp.setInputTarget(cloudIn);
        icp.setMaximumIterations(1000);
        icp.setTransformationEpsilon(1e-7);
        icp.setMaxCorrespondenceDistance(1);
        icp.setEuclideanFitnessEpsilon(0.1);
        icp.setRANSACOutlierRejectionThreshold(20);

        icp.align(*cloudOut);

        if (icp.hasConverged())
        {
            std::cout << "ICP converged." << std::endl
                      << "The score is " << icp.getFitnessScore() << std::endl;
            std::cout << "Transformation matrix:" << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;
        }
        else std::cout << "ICP did not converge." << std::endl;

        Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation ();
        std::cout<<"trans %n"<<transformationMatrix<<std::endl;

        pcl::transformPointCloud( *cloudOut, *cloudOut_new, transformationMatrix);

        finalCloud = *cloudIn + *cloudOut;

        pcl::io::savePCDFileASCII ("cuberesult.pcd", finalCloud);
    }

    return (0);
}
