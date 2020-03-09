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
#include <pcl-1.8/pcl/filters/fast_bilateral.h>
#include <pcl-1.8/pcl/surface/bilateral_upsampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <PicoZense_api.h>
#include "pthread.h"
#include <ctime>

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
#define PCD_FILE_PATH   "/home/alecone/Documents/Università/Magistrale/Tesi/PCDs/"

template <class... Args>
void debug(Args... args)
{
    (std::cout << BLUE << ... << args) << RESET << std::endl;
}
template <class... Args>
void info(Args... args)
{
    (std::cout << GREEN << ... << args) << RESET << std::endl;
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

using namespace pcl;
using namespace cv;
using namespace Eigen;

class PicoZenseHandler
{
public:
    explicit PicoZenseHandler(int32_t devIndex);
    ~PicoZenseHandler();

    void *Visualize();
    void init();
    int SavePCD();
    pcl::visualization::PCLVisualizer::Ptr InitializeInterations();
    pcl::visualization::PCLVisualizer::Ptr m_visualizer = nullptr;

    // Getters PicoZense Params
    void GetCameraParameters();
    void GetImu();

    // Setters PicoZense Params
    PsReturnStatus SetDepthRange(PsDepthRange depthRange);
    PsReturnStatus SetColoPixelFormat(PsPixelFormat pixelFormat);
    PsReturnStatus SetDataMode(PsDataMode dataMode);
    PsReturnStatus SetThreshold(uint16_t threshold);
    PsReturnStatus SetFIlter(PsFilterType filterType, bool enable);
    PsReturnStatus SetDepthDistortionCorrectionEnabled(bool enable);
    PsReturnStatus SetRGBDistortionCorrectionEnabled(bool enable);
    PsReturnStatus SetComputeRealDepthCorrectionEnabled(bool enable);
    PsReturnStatus SetSmoothingFilterEnabled(bool enable);
    PsReturnStatus SetResolution(PsResolution resolution);
    PsReturnStatus SetSpatialFilterEnabled(bool enable);
    void SetBilateralNoiseFilter(bool enable);
    void SetBilateralUpsampling(bool enable);

    // Main PointCloud Feautures funtions
    void SetPointCloudRGB();
    void SetPointCloudClassic();
    void SetFeatureDetection(bool enable);
    void SetWDRDataMode();

private:
    //Private functions
    std::string PsStatusToString(PsReturnStatus p_status);
    void PointCloudCreatorXYZ(int p_height, int p_width, cv::Mat &p_image, uint8_t *p_data, PsCameraParameters params);
    void PointCloudMapRGBDepthCustom(int p_height, int p_width, cv::Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters paramsDepth, PsCameraParameters paramsRGB, Matrix3d R, Vector3d t);
    void PointCloudCreatorXYZRGB(int p_height, int p_width, cv::Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters params);
    void Harris3DCornerDetection();
    void NARFCorenerDetection();
    void ISSCornerDetection();
    void CreateRangeImage();
    PointCloud<PointXYZ>::Ptr ApplyBilateralFilter(PointCloud<PointXYZ>::Ptr cloud_in);
    PointCloud<PointXYZRGB>::Ptr ApplyBilateralUpsampling(PointCloud<PointXYZRGB>::Ptr cloud_in);

    //Provate members
    // pcl::visualization::CloudViewer *m_viewer = nullptr;
    cv::Mat imageMatrix;
    cv::Mat imageMatrixRGB;
    cv::Mat imageRGB;
    PointCloud<PointXYZ>::Ptr pointCloud = nullptr;
    PointCloud<PointXYZRGB>::Ptr pointCloudRGB = nullptr;
    pcl::RangeImage::Ptr rangeImage = nullptr;
    int32_t m_deviceIndex;
    PsDepthRange m_depthRange;
    PsDataMode m_dataMode;
    Eigen::Quaternionf q;
    bool m_pointCloudClassic;
    bool m_pointCloudMappedRGB;
    bool m_detectorHarris;
    bool m_detectorNARF;
    bool m_detectorISS;
    bool m_wdrDepth;
    bool m_fastBiFilter;
    bool m_bilateralUpsampling;
};

#endif // PICOZENSEHANDLER_PICOZENSEHANDLER_H