#ifndef MULTICAMWORKER_H
#define MULTICAMWORKER_H

#include "PicoZenseHandler.h"

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

    void worker(boost::barrier &p_barier);

private:
    pcl::visualization::PCLVisualizer::Ptr m_visualizer = nullptr;
    struct picoHandlers *picos;
};

#endif // !MULTICAMWORKER_H
