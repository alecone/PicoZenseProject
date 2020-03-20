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

void MultiCamWorker::worker(boost::barrier &p_barier)
{
    while (true)
    {
        p_barier.wait();
        if (picos->pico1->IsPointCloudRGBEnabled() != picos->pico2->IsPointCloudRGBEnabled())
            continue;
        if (picos->pico1->IsPointCloudRGBEnabled() && picos->pico2->IsPointCloudRGBEnabled())
        {
            vis_mutex.lock();
            if (!m_visualizer->updatePointCloud(picos->pico1->GetRGBPointCloud(), "PointCloudSX"))
                m_visualizer->addPointCloud(picos->pico1->GetRGBPointCloud(), "PointCloudSX");
            if (!m_visualizer->updatePointCloud(picos->pico2->GetRGBPointCloud(), "PointCloudDX"))
                m_visualizer->addPointCloud(picos->pico2->GetRGBPointCloud(), "PointCloudDX");
            m_visualizer->spinOnce();
            vis_mutex.unlock();
        }
        else
        {
            vis_mutex.lock();
            if (!m_visualizer->updatePointCloud(picos->pico1->GetPointCloud(), "PointCloudSX"))
                m_visualizer->addPointCloud(picos->pico1->GetPointCloud(), "PointCloudSX");
            if (!m_visualizer->updatePointCloud(picos->pico2->GetPointCloud(), "PointCloudDX"))
                m_visualizer->addPointCloud(picos->pico2->GetPointCloud(), "PointCloudDX");
            m_visualizer->spinOnce();
            vis_mutex.unlock();
        }
    }
}