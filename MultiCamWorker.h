#ifndef MULTICAMWORKER_H
#define MULTICAMWORKER_H

#include "PicoZenseHandler.h"
#include <pcl-1.8/pcl/registration/icp.h>
#include <pcl-1.8/pcl/registration/icp_nl.h>
//This is particularly useful when solving for camera extrinsics
//using multiple observations. When given a single pair of clouds, 
//this reduces to vanilla ICP.
#include <pcl/registration/joint_icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>

struct picoHandlers
{
    PicoZenseHandler *pico1;
    PicoZenseHandler *pico2;
};

class MultiCamWorker
{
public:
    explicit MultiCamWorker(void *picoZenseHandlers, pcl::visualization::PCLVisualizer::Ptr viewer);
    ~MultiCamWorker();

    //Public functions
    void worker(boost::barrier &p_barier);

private:
    //Private functions
    Eigen::Matrix4f allignPointClouds(PointCloud<PointXYZ>::Ptr cloudSrc1, PointCloud<PointXYZ>::Ptr cloudSrc2);
    Eigen::Matrix4f allignPointCloudsRGB(PointCloud<PointXYZRGB>::Ptr cloudSrc1, PointCloud<PointXYZRGB>::Ptr cloudSrc2);

    //Private members
    pcl::visualization::PCLVisualizer::Ptr m_visualizer = nullptr;
    struct picoHandlers *picos;
};

#endif // !MULTICAMWORKER_H
