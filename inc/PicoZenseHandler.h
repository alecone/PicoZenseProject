#ifndef PICOZENSEHANDLER_H
#define PICOZENSEHANDLER_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
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
#include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl-1.8/pcl/keypoints/iss_3d.h>
#include <pcl-1.8/pcl/filters/fast_bilateral.h>
#include <pcl-1.8/pcl/surface/bilateral_upsampling.h>
#include <pcl-1.8/pcl/surface/poisson.h>
#include <pcl-1.8/pcl/surface/gp3.h>
#include <pcl-1.8/pcl/surface/mls.h>
#include <pcl-1.8/pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.8/pcl/filters/radius_outlier_removal.h>
#include <pcl-1.8/pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
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
#define TEST_PATH       "/home/alecone/Documents/Università/Magistrale/Tesi/TEST_PERFORMACE/"

static boost::mutex vis_mutex;

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
    explicit PicoZenseHandler(int32_t devIndex, pcl::visualization::PCLVisualizer::Ptr viewer);
    ~PicoZenseHandler();

    void *Visualize(boost::barrier &p_barier);
    void init();
    void SavePCD();
    void InitializeInterations(pcl::visualization::PCLVisualizer::Ptr viewer);
    pcl::visualization::PCLVisualizer::Ptr m_visualizer = nullptr;

    // PointCloud Getters
    PointCloud<PointXYZRGB>::Ptr GetRGBPointCloud();
    PointCloud<PointXYZ>::Ptr GetPointCloud();

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
    void SetNormalizedBoxFilter(bool enable);
    void SetGaussinFilter(bool enable);
    void SetBilateralFilter(bool enable);
    void SetStatisticalOutlierRemoval(bool enable);
    void SetRadialOutlierRemoval(bool enable);
    void SetMLSUpsampling(bool enable);
    void SetFastTriangolation(bool enable);
    void SetPolynomialReconstruction(bool enable);
    void SetTransform(Eigen::Matrix4f p_tranform);
    void UnSetTranform();

    // Main PointCloud Feautures funtions
    void SetPointCloudRGB();
    void SetPointCloudClassic();
    void SetFeatureDetection(bool enable);
    void SetWDRDataMode();
    bool IsPointCloudRGBEnabled();
    void ShutDown();

    void PerformTest();
    void SetTestName(std::string &p_testName);

private:
    //Private functions
    std::string PsStatusToString(PsReturnStatus p_status);
    void PointCloudCreatorXYZ(int p_height, int p_width, cv::Mat &p_image, uint8_t *p_data, PsCameraParameters params, boost::barrier &p_barier);
    void PointCloudCreatorXYZRGB(int p_height, int p_width, cv::Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters params, boost::barrier &p_barier);
    void Harris3DCornerDetection();
    void NARFCorenerDetection();
    void ISSCornerDetection();
    void CreateRangeImage();
    PointCloud<PointXYZ>::Ptr ApplyBilateralFilter(PointCloud<PointXYZ>::Ptr cloud_in);
    PointCloud<PointXYZRGB>::Ptr ApplyBilateralUpsampling(PointCloud<PointXYZRGB>::Ptr cloud_in);
    PointCloud<PointXYZ>::Ptr ApplyMLSUpsampling(PointCloud<PointXYZ>::Ptr cloud_in);
    PointCloud<PointXYZRGB>::Ptr ApplyMLSUpsamplingRGB(PointCloud<PointXYZRGB>::Ptr cloud_in);

    void PolynomialReconstructionRGB(PointCloud<PointXYZRGB>::Ptr cloud_in);
    void FastTriangolationRGB(PointCloud<PointXYZRGB>::Ptr cloud_in);

    void SendToVisualizer(PointCloud<PointXYZ>::Ptr cloud_in, int32_t dev_index, boost::barrier &p_barier);
    void SendToVisualizer(PointCloud<PointXYZRGB>::Ptr cloud_in, int32_t dev_index, boost::barrier &p_barier);
    void SaveDepthImage(cv::Mat p_image, std::string &info);

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
    bool m_normalizedBoxFilter;
    bool m_gaussianFilter;
    bool m_bilateralFilter;
    bool m_stattisticalOutlierRemoval;
    bool m_radialOutlierRemoval;
    bool m_save;
    bool m_mlsUpsampling;
    bool m_fastTriangolation;
    bool m_polynomialReconstraction;
    int32_t m_deviceCount;
    uint32_t m_saveIndex;
    PointCloud<PointXYZ>::Ptr copy = nullptr;
    PointCloud<PointXYZRGB>::Ptr copyRGB = nullptr;
    Eigen::Matrix4f transform;
    bool m_canUseTransform;
    bool m_loop;
    int m_savedImages;
    bool m_performTest;
    std::string m_testName;
};

#endif // PICOZENSEHANDLER_H
