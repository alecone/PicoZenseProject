#include "MultiCamWorker.h"

MultiCamWorker::MultiCamWorker(void *picoZenseHandlers, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    info("MultiCam Object created");
    picos = (struct picoHandlers *)picoZenseHandlers;
    if (picos->pico1 == NULL && picos->pico2 == NULL)
    {
        error("PicoZenseHandlers are null, Stop it!");
        exit(1);
    }
    info("PicoZenseHandlers are OK!");
    m_visualizer = viewer;
}

//Hint: sometimes it could clash during visualization.
//in order to make it run again. compile it and run it without the
//spinOnce() line code. Then restore, and it'll work.

void MultiCamWorker::worker(boost::barrier &p_barier)
{
    while (true)
    {
        p_barier.wait();
        if (picos->pico1->IsPointCloudRGBEnabled() != picos->pico2->IsPointCloudRGBEnabled())
            continue;
        if (picos->pico1->IsPointCloudRGBEnabled() && picos->pico2->IsPointCloudRGBEnabled())
        {
            if (!m_visualizer->updatePointCloud(picos->pico1->GetRGBPointCloud(), "PointCloudSX"))
                m_visualizer->addPointCloud(picos->pico1->GetRGBPointCloud(), "PointCloudSX");
            if (!m_visualizer->updatePointCloud(picos->pico2->GetRGBPointCloud(), "PointCloudDX"))
                m_visualizer->addPointCloud(picos->pico2->GetRGBPointCloud(), "PointCloudDX");
                
            m_visualizer->spinOnce();
        }
        else
        {
            if (!m_visualizer->updatePointCloud(picos->pico1->GetPointCloud(), "PointCloudSX"))
                m_visualizer->addPointCloud(picos->pico1->GetPointCloud(), "PointCloudSX");
            if (!m_visualizer->updatePointCloud(picos->pico2->GetPointCloud(), "PointCloudDX"))
                m_visualizer->addPointCloud(picos->pico2->GetPointCloud(), "PointCloudDX");

            m_visualizer->spinOnce();
        }
    }
}

Eigen::Matrix4f MultiCamWorker::allignPointClouds(PointCloud<PointXYZ>::Ptr cloudSrc1, PointCloud<PointXYZ>::Ptr cloudSrc2)
{
    PointCloud<PointXYZ>::Ptr alligned(new PointCloud<PointXYZ>());

    debug("PointCloud 1 dimension: ", std::to_string(cloudSrc1->size()));
    debug("PointCloud 2 dimension: ", std::to_string(cloudSrc2->size()));

    PointCloud<PointXYZ>::Ptr src1Down(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr src2Down(new PointCloud<PointXYZ>());
    pcl::VoxelGrid<PointXYZ> grid;
    grid.setLeafSize(0.05, 0.05, 0.05);
    grid.setInputCloud(cloudSrc1);
    grid.filter(*src1Down);

    grid.setInputCloud(cloudSrc2);
    grid.filter(*src2Down);
    debug("PointCloud 1 downsampled dimension: ", std::to_string(src1Down->size()));
    debug("PointCloud 2 downsampled dimension: ", std::to_string(src2Down->size()));

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    // Set the input source and target
    icp.setInputCloud(src1Down);
    icp.setInputTarget(src2Down);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.08);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);
    // Perform the alignment
    icp.align(*alligned);
    if (icp.hasConverged())
        debug("ICP converged.");
    else
        debug("ICP did not converge.");
    debug("The score is ", icp.getFitnessScore());
    debug("Transformation matrix:");
    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    debug(transformationMatrix);

    return transformationMatrix;

    // transformPointCloud(*src2, *dst, transformationMatrix);
    // finalCloud1 = *cloudIn;
    // finalCloud1 += *cloudOut_new;
}

Eigen::Matrix4f MultiCamWorker::allignPointCloudsRGB(PointCloud<PointXYZRGB>::Ptr cloudSrc1, PointCloud<PointXYZRGB>::Ptr cloudSrc2)
{
    PointCloud<PointXYZRGB>::Ptr allignedRGB(new PointCloud<PointXYZRGB>());

    debug("PointCloud 1 dimension: ", std::to_string(cloudSrc1->size()));
    debug("PointCloud 2 dimension: ", std::to_string(cloudSrc2->size()));

    PointCloud<PointXYZRGB>::Ptr src1Down(new PointCloud<PointXYZRGB>());
    PointCloud<PointXYZRGB>::Ptr src2Down(new PointCloud<PointXYZRGB>());
    pcl::VoxelGrid<PointXYZRGB> grid;
    grid.setLeafSize(0.05, 0.05, 0.05);
    grid.setInputCloud(cloudSrc1);
    grid.filter(*src1Down);

    grid.setInputCloud(cloudSrc2);
    grid.filter(*src2Down);
    debug("PointCloud 1 dimension: ", std::to_string(src1Down->size()));
    debug("PointCloud 2 dimension: ", std::to_string(src2Down->size()));

    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
    // Set the input source and target
    icp.setInputCloud(src1Down);
    icp.setInputTarget(src2Down);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.08);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);
    // Perform the alignment
    icp.align(*allignedRGB);
    if (icp.hasConverged())
        debug("ICP converged.");
    else
        debug("ICP did not converge.");
    debug("The score is ", icp.getFitnessScore());
    debug("Transformation matrix:");
    Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation();
    debug(transformationMatrix);

    return transformationMatrix;

    // transformPointCloud(*src2, *dst, transformationMatrix);
    // finalCloud1 = *cloudIn;
    // finalCloud1 += *cloudOut_new;
}