#ifndef PICOZENSEHANDLER_PICOZENSEHANDLER_H
#define PICOZENSEHANDLER_PICOZENSEHANDLER_H

#include <opencv2/opencv.hpp>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/common/transforms.h>
#include <pcl-1.8/pcl/visualization/eigen.h>
#include <pcl-1.8/pcl/visualization/range_image_visualizer.h>
#include <pcl-1.8/pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl-1.8/pcl/keypoints/narf_keypoint.h>
#include <pcl-1.8/pcl/range_image/range_image.h>
#include <pcl-1.8/pcl/visualization/range_image_visualizer.h>
#include <pcl-1.8/pcl/features/range_image_border_extractor.h>
#include <pcl-1.8/pcl/keypoints/harris_3d.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/keypoints/iss_3d.h>
#include <PicoZense_api.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

using namespace pcl;
using namespace cv;

class PicoZenseHandler
{
public:
    explicit PicoZenseHandler(int32_t devIndex);
    ~PicoZenseHandler();

    void Visualize();
    int SavePCD(const std::string &filename);
    pcl::visualization::PCLVisualizer::Ptr InitializeInterations();
    void PrintCameraParameters();

private:
    //Private functions
    std::string PsStatusToString(PsReturnStatus p_status);
    void PointCloudCreatorXYZ(int p_height, int p_width, cv::Mat &p_image, uint8_t *p_data, PsCameraParameters params);
    void PointCloudCreatorXYZRGB(int p_height, int p_width, cv::Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters params);
    void Harris3DCornerDetection();
    void NARFCorenerDetection();
    void ISSCornerDetection();
    void CreateRangeImage();
    template <class... Args>
    void debug(Args... args)
    {
        (std::cout << BOLDBLUE << ... << args) << RESET << std::endl;
    }
    template <class... Args>
    void info(Args... args)
    {
        (std::cout << BOLDGREEN << ... << args) << RESET << std::endl;
    }
    template <class... Args>
    void warn(Args... args)
    {
        (std::cout << BOLDYELLOW << ... << args) << RESET << std::endl;
    }
    template <class... Args>
    void error(Args... args)
    {
        (std::cout << BOLDRED << ... << args) << RESET << std::endl;
    }

    //Provate members
    int32_t m_devIndex;
    pcl::visualization::CloudViewer *m_viewer = nullptr;
    pcl::visualization::PCLVisualizer::Ptr m_visualizer = nullptr;
    cv::Mat imageMatrix;
    cv::Mat imageMatrixRGB;
    PointCloud<PointXYZ>::Ptr pointCloud = nullptr;
    PointCloud<PointXYZRGB>::Ptr pointCloudRGB = nullptr;
    pcl::RangeImage::Ptr rangeImage = nullptr;
    int32_t m_deviceIndex;
    PsDepthRange depthRange;
    int32_t dataMode;
};

#endif // PICOZENSEHANDLER_PICOZENSEHANDLER_H