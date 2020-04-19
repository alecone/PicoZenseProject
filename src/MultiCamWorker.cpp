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
    m_computeTransform = false;
    m_loop = true;
    T = Eigen::Matrix4f::Identity();
    finalCloudRGB = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>());
    srcRGB = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>());
    tgtRGB = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>());
    finalCloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
    src = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
    tgt = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
}

//Hint: sometimes it could clash during visualization.
//in order to make it run again. compile it and run it without the
//spinOnce() line code. Then restore, and it'll work.

void MultiCamWorker::worker(boost::barrier &p_barier)
{
    // T << 0.998038, 0.016504, 0.0603983, -0.197575, -0.0152705, 0.999667, -0.0208275, 0.0166649, -0.0607219, 0.0198643, 0.997957, 0.0132814, -0, -0, -0, 1;

    while (m_loop)
    {
        p_barier.wait();
        if (picos->pico1->IsPointCloudRGBEnabled() != picos->pico2->IsPointCloudRGBEnabled())
            continue;
        if (picos->pico1->IsPointCloudRGBEnabled() && picos->pico2->IsPointCloudRGBEnabled())
        {
            // Copy PointCloud
            copyPointCloud(*picos->pico1->GetRGBPointCloud(), *srcRGB);
            copyPointCloud(*picos->pico2->GetRGBPointCloud(), *tgtRGB);
            if (m_computeTransform)
            {
                allignPointClouds(srcRGB, tgtRGB, T);
                m_computeTransform = false;
            }
            *finalCloudRGB = *srcRGB;
            *finalCloudRGB += *tgtRGB;

            if (!m_visualizer->updatePointCloud(finalCloudRGB, "PointCloudAlligned"))
                m_visualizer->addPointCloud(finalCloudRGB, "PointCloudAlligned");

            m_visualizer->spinOnce();
        }
        else
        {
            // Copy PointCloud
            copyPointCloud(*picos->pico1->GetPointCloud(), *src);
            copyPointCloud(*picos->pico2->GetPointCloud(), *tgt);
            if (m_computeTransform)
            {
                allignPointClouds(src, tgt, T);
                m_computeTransform = false;
            }
            *finalCloud = *src;
            *finalCloud += *tgt;

            if (!m_visualizer->updatePointCloud(finalCloud, "PointCloudAlligned"))
                m_visualizer->addPointCloud(finalCloud, "PointCloudAlligned");

            m_visualizer->spinOnce();
        }
    }
}

bool MultiCamWorker::getTransform(Eigen::Matrix4f &p_tranform)
{
    bool computed = false;
    if (!m_computeTransform)
    {
        if (! (T == Eigen::Matrix4f::Identity()))
        {
            p_tranform = T;
            computed = true;
        }
    }
    return computed;
}

void MultiCamWorker::startComputeTransform()
{
    m_computeTransform = true;
}

void MultiCamWorker::ShutDown()
{
    info("Shutting down Cameras Worker...");
    m_loop = false;
    m_visualizer = nullptr;
    finalCloudRGB = nullptr;
    srcRGB = nullptr;
    tgtRGB = nullptr;
    finalCloud = nullptr;
    src = nullptr;
    tgt = nullptr;

    exit(0);
}

void MultiCamWorker::allignPointClouds(PointCloud<PointXYZ>::Ptr cloudSrc1, PointCloud<PointXYZ>::Ptr cloudSrc2, Eigen::Matrix4f &T)
{
    debug("PointCloud 1 dimension: ", std::to_string(cloudSrc1->size()));
    debug("PointCloud 2 dimension: ", std::to_string(cloudSrc2->size()));

    PointCloud<PointXYZ>::Ptr src1Down(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr src2Down(new PointCloud<PointXYZ>());
    pcl::VoxelGrid<PointXYZ> grid;
    // Consider to take the approach that calculate features and just keep them
    // or even betther to keep only the borders (with NARF)
    grid.setLeafSize(0.01, 0.01, 0.01);
    grid.setInputCloud(cloudSrc1);
    grid.filter(*src1Down);

    grid.setInputCloud(cloudSrc2);
    grid.filter(*src2Down);
    debug("PointCloud 1 downsampled dimension: ", std::to_string(src1Down->size()));
    debug("PointCloud 2 downsampled dimension: ", std::to_string(src2Down->size()));

    // Now computation of normals and curvature can be done

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    // Set the input source and target
    icp.setInputTarget(src2Down);
    // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-4);
    // Set the euclidean distance difference epsilon (criterion 3)
    // icp.setEuclideanFitnessEpsilon(1e-2);
    // icp.setRANSACIterations(5);
    // icp.setRANSACOutlierRejectionThreshold(0.1);
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloud<PointXYZ>::Ptr tmpResult = src1Down;
    int i = 0;
    do
    {
        debug("[ICP] Iteration n° ", std::to_string(++i));
        src1Down = tmpResult;
        icp.setInputSource(src1Down);
        // Perform the alignment
        icp.align(*tmpResult);

        //Accumulate tranformation between each iteration
        Ti = icp.getFinalTransformation() * Ti;
        //If the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (std::abs((icp.getLastIncrementalTransformation() - prev).sum()) < icp.getTransformationEpsilon())
            icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() - 0.001);

        prev = icp.getLastIncrementalTransformation();
        debug("[ICP] Current score: ", icp.getFitnessScore());
        debug("[ICP] Corrispondace distance", icp.getMaxCorrespondenceDistance());
    } while ((icp.getFitnessScore() > 0.0035) && icp.hasConverged() && i < 100);

    debug("Transformation matrix:");
    targetToSource = Ti.inverse();
    debug(targetToSource);

    T = targetToSource;
}

void MultiCamWorker::allignPointClouds(PointCloud<PointXYZRGB>::Ptr cloudSrc1, PointCloud<PointXYZRGB>::Ptr cloudSrc2, Eigen::Matrix4f &T)
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
    icp.setInputSource(src1Down);
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

    //Copy to transform arg
    // return transformationMatrix;
}