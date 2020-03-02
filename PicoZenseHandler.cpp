#include "PicoZenseHandler.h"


using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

#define CUSTOM_MAPPED_DEPTH_RGB     0

static void keyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* pico);
static void mouseEventHandler(const pcl::visualization::MouseEvent &event, void* pico);
static void pointEventHandler(const pcl::visualization::PointPickingEvent &event, void *pico);
static bool m_loop;
static PointXYZ old;

PicoZenseHandler::PicoZenseHandler(int32_t devIndex)
{
    m_visualizer = InitializeInterations();
    m_visualizer->setBackgroundColor(0.0, 0.0, 0.0);
    pointCloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
    pointCloudRGB = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>());
    rangeImage = pcl::RangeImage::Ptr(new pcl::RangeImage());

    auto angle = M_PI;
    q.x() = 0 * sin(angle / 2);
    q.y() = 0 * sin(angle / 2);
    q.z() = 1 * sin(angle / 2);
    q.w() = 1 * std::cos(angle / 2);
    m_deviceIndex = devIndex;
    m_depthRange = PsNearRange;
    m_dataMode = PsDepthAndRGB_30;
    m_pointCloudClassic = true;
    m_wdrDepth = false;
    m_pointCloudMappedRGB = false;
    m_detectorHarris = false;
    m_detectorNARF = false;
    m_detectorISS = false;
    m_fastBiFilter = false;
    m_bilateralUpsampling = false;
    m_loop = true;

    info("Starting with:\n\tDepthRange: PsNearRange\n\tm_dataMode: PsDepthAndRGB_30\n\tPixelFormat: PsPixelFormatRGB888\nDevice #", std::to_string(m_deviceIndex));
}


PicoZenseHandler::~PicoZenseHandler()
{
    if (m_visualizer != nullptr)
    {
        m_visualizer->removeAllPointClouds();
        // m_visualizer->close();
        // m_visualizer = nullptr;
    }
    if (pointCloud != nullptr)
    {
        pointCloud->clear();
        pointCloud = nullptr;
    }
    if (pointCloudRGB != nullptr)
    {
        pointCloudRGB->clear();
        pointCloudRGB = nullptr;
    }
    if (rangeImage != nullptr)
    {
        rangeImage->clear();
        rangeImage = nullptr;
    }

    debug("PicoZense destroyed");    
}

void PicoZenseHandler::init()
{
    PsReturnStatus status;
    int32_t deviceCount = 0;
    // uint32_t slope = 1450;
    // uint32_t wdrSlope = 4400;

    // Introduced due to error 
    //[xcb] Unknown sequence number while processing queue
    //[xcb] Most likely this is a multi - threaded client and XInitThreads has not been called
    //[xcb] Aborting, sorry about that.

    XInitThreads();
    status = PsInitialize();
    if (status != PsReturnStatus::PsRetOK)
    {
        std::cout << "PsInitialize failed!\n";
        system("pause");
        return;
    }

    status = PsGetDeviceCount(&deviceCount);
    if (status != PsReturnStatus::PsRetOK)
    {
        error("PsGetDeviceCount failed!");
        system("pause");
        return;
    }

    //Set the Depth Range to Near through PsSetDepthRange interface
    status = PsSetDepthRange(m_deviceIndex, m_depthRange);
    if (status != PsReturnStatus::PsRetOK)
        error("PsSetDepthRange failed!");
    else
        debug("Set Depth Range to Near");

    status = PsOpenDevice(m_deviceIndex);
    if (status != PsReturnStatus::PsRetOK)
    {
        error("OpenDevice failed with status ", PsStatusToString(status));
        system("pause");
        return;
    }

    //Set PixelFormat as PsPixelFormatBGR888 for opencv display
    PsSetColorPixelFormat(m_deviceIndex, PsPixelFormatRGB888);
#if CUSTOM_MAPPED_DEPTH_RGB
    PsSetResolution(m_deviceIndex, PsRGB_Resolution_640_480);
#endif

    //Set to data mode
    status = PsSetDataMode(m_deviceIndex, (PsDataMode)m_dataMode);
    if (status != PsReturnStatus::PsRetOK)
    {
        std::cout << "Set DataMode Failed failed!\n";
    }
}

void *PicoZenseHandler::Visualize()
{
    // Printing threadID
    debug("Running threadID: ", pthread_self());
    PsReturnStatus status;

    PsCameraParameters depthCameraParameters;
    status = PsGetCameraParameters(m_deviceIndex, PsDepthSensor, &depthCameraParameters);
    if (status != PsRetOK)
        debug("PsGetCameraParameters for depth sensor failed with error ", PsStatusToString(status));

    PsCameraParameters rgbCameraParameters;
    status = PsGetCameraParameters(m_deviceIndex, PsRgbSensor, &rgbCameraParameters);
    if (status != PsRetOK)
        debug("PsGetCameraParameters for RGB sensor failed with error ", PsStatusToString(status));

    PsCameraExtrinsicParameters CameraExtrinsicParameters;
    status = PsGetCameraExtrinsicParameters(m_deviceIndex, &CameraExtrinsicParameters);
    if (status != PsRetOK)
        debug("PsGetCameraExtrinsicParameters failed with error ", PsStatusToString(status));


    // Main loop
    while (m_loop)
    {
        PsFrame depthFrame = {0};
        PsFrame wdrDepthFrame = {0};
        PsFrame mappedRGBFrame = {0};
#if CUSTOM_MAPPED_DEPTH_RGB
        PsFrame rgbFrame = {0};
#endif // CUSTOM_MAPPED_DEPTH_RGB

        // Read one frame before call PsGetFrame
        status = PsReadNextFrame(m_deviceIndex);
        // if (status != PsRetOK)
        //     warn("PsReadNextFrame gave ", PsStatusToString(status));

        //Get depth frame, depth frame only output in following data mode
        if (!m_wdrDepth && (m_dataMode == PsDepthAndRGB_30 || m_dataMode == PsDepthAndIR_30 || m_dataMode == PsDepthAndIRAndRGB_30 || m_dataMode == PsDepthAndIR_15_RGB_30))
        {
            PsGetFrame(m_deviceIndex, PsDepthFrame, &depthFrame);
#if CUSTOM_MAPPED_DEPTH_RGB
            PsGetFrame(m_deviceIndex, PsRGBFrame, &rgbFrame);
#endif

            if (!m_pointCloudMappedRGB && depthFrame.pFrameData != NULL)
            {
                //Generate and display PointCloud
#if CUSTOM_MAPPED_DEPTH_RGB
                PointCloudMapRGBDepthCustom(depthFrame.height, depthFrame.width, imageRGB, imageMatrix, rgbFrame.pFrameData, depthFrame.pFrameData, depthCameraParameters, rgbCameraParameters, CameraExtrinsicParameters);
#else
                PointCloudCreatorXYZ(depthFrame.height, depthFrame.width, imageMatrix, depthFrame.pFrameData, depthCameraParameters);
#endif
            }
        }

        //Get WDR depth frame(fusion or alternatively, determined by PsSetWDRStyle, default in fusion)
        //WDR depth frame only output in PsWDR_Depth data mode
        if (m_dataMode == PsWDR_Depth)
        {
            PsGetFrame(m_deviceIndex, PsWDRDepthFrame, &wdrDepthFrame);
            if (!m_pointCloudMappedRGB && wdrDepthFrame.pFrameData != NULL)
            {
                //Display the WDR Depth Image
                PointCloudCreatorXYZ(wdrDepthFrame.height, wdrDepthFrame.width, imageMatrix, wdrDepthFrame.pFrameData, depthCameraParameters);
            }
        }

        //Get mapped rgb frame which is mapped to depth camera space
        //Mapped rgb frame only output in following data mode
        //And can only get when the feature is enabled through api PsSetMapperEnabledDepthToRGB
        if (m_pointCloudMappedRGB && (m_dataMode == PsDepthAndRGB_30 || m_dataMode == PsDepthAndIRAndRGB_30 || m_dataMode == PsWDR_Depth || m_dataMode == PsDepthAndIR_15_RGB_30))
        {
            PsGetFrame(m_deviceIndex, PsMappedRGBFrame, &mappedRGBFrame);
            if (mappedRGBFrame.pFrameData != NULL)
            {
                if (m_dataMode == PsWDR_Depth)
                    PointCloudCreatorXYZRGB(mappedRGBFrame.height, mappedRGBFrame.width, imageMatrixRGB, imageMatrix, mappedRGBFrame.pFrameData, wdrDepthFrame.pFrameData, depthCameraParameters);
                else
                    PointCloudCreatorXYZRGB(mappedRGBFrame.height, mappedRGBFrame.width, imageMatrixRGB, imageMatrix, mappedRGBFrame.pFrameData, depthFrame.pFrameData, depthCameraParameters);
            }
        }
        imageMatrix.release();
        imageMatrixRGB.release();
#if CUSTOM_MAPPED_DEPTH_RGB
        imageRGB.release();
#endif
    }

    status = PsCloseDevice(m_deviceIndex);
    info("CloseDevice status: ", PsStatusToString(status));

    status = PsShutdown();
    info("Shutdown status: ", PsStatusToString(status));

    // Return in order to make thread exit
    return NULL;
}

void PicoZenseHandler::GetCameraParameters()
{
    PsCameraParameters cameraParameters;
    PsReturnStatus status = PsGetCameraParameters(m_deviceIndex, PsDepthSensor, &cameraParameters);
    info("Get PsGetCameraParameters PsDepthSensor status: ", PsStatusToString(status));
    info("Depth Camera Intinsic:\n", "Fx: ", std::to_string(cameraParameters.fx), "\n",
         "Cx: ", std::to_string(cameraParameters.cx), "\n",
         "Fy: ", std::to_string(cameraParameters.fy), "\n",
         "Cy: ", std::to_string(cameraParameters.cy));//, "\n",
        //  "Depth Distortion Coefficient: \n",
        //  "K1: ", std::to_string(cameraParameters.k1), "\n",
        //  "K2: ", std::to_string(cameraParameters.k2), "\n",
        //  "P1: ", std::to_string(cameraParameters.p1), "\n",
        //  "P2: ", std::to_string(cameraParameters.p2), "\n",
        //  "K3: ", std::to_string(cameraParameters.k3), "\n",
        //  "K4: ", std::to_string(cameraParameters.k4), "\n",
        //  "K5: ", std::to_string(cameraParameters.k5), "\n",
        //  "K6: ", std::to_string(cameraParameters.k6));

    status = PsGetCameraParameters(m_deviceIndex, PsRgbSensor, &cameraParameters);
    info("Get PsGetCameraParameters PsRgbSensor status: ", PsStatusToString(status));
    info("RGB Camera Intinsic: \n",
         "Fx: ", std::to_string(cameraParameters.fx), "\n",
         "Cx: ", std::to_string(cameraParameters.cx), "\n",
         "Fy: ", std::to_string(cameraParameters.fy), "\n",
         "Cy: ", std::to_string(cameraParameters.cy));//, "\n",
        //  "RGB Distortion Coefficient: \n",
        //  "K1: ", std::to_string(cameraParameters.k1), "\n",
        //  "K2: ", std::to_string(cameraParameters.k2), "\n",
        //  "P1: ", std::to_string(cameraParameters.p1), "\n",
        //  "P2: ", std::to_string(cameraParameters.p2), "\n",
        //  "K3: ", std::to_string(cameraParameters.k3), "\n");

    PsCameraExtrinsicParameters CameraExtrinsicParameters;
    status = PsGetCameraExtrinsicParameters(m_deviceIndex, &CameraExtrinsicParameters);

    info("Get PsGetCameraExtrinsicParameters status: ", PsStatusToString(status));
    info("Camera rotation: \n",
         std::to_string(CameraExtrinsicParameters.rotation[0]), " ", std::to_string(CameraExtrinsicParameters.rotation[1]), " ", std::to_string(CameraExtrinsicParameters.rotation[2]), "\n",
         std::to_string(CameraExtrinsicParameters.rotation[3]), " ", std::to_string(CameraExtrinsicParameters.rotation[4]), " ", std::to_string(CameraExtrinsicParameters.rotation[5]), "\n",
         std::to_string(CameraExtrinsicParameters.rotation[6]), " ", std::to_string(CameraExtrinsicParameters.rotation[7]), " ", std::to_string(CameraExtrinsicParameters.rotation[8]), "\n",
         "Camera transfer: \n",
         std::to_string(CameraExtrinsicParameters.translation[0]), " ", std::to_string(CameraExtrinsicParameters.translation[1]), " ", std::to_string(CameraExtrinsicParameters.translation[2]));
}

int PicoZenseHandler::SavePCD(const std::string &filename)
{
    pcl::PCDWriter w;
    debug("Going to write");
    int ret = w.write(filename, *pointCloud);
    return ret;
}

void PicoZenseHandler::GetImu()
{
    PsImuWithParams imuParam;
    PsReturnStatus status = PsGetImuWithParams(m_deviceIndex, &imuParam);
    if (status != PsRetOK)
        error("GetImuWithParam failed with error ", PsStatusToString(status));

    info("Imu Params:\n", "Acceleration (x, y, z) = [",  std::to_string(imuParam.acc.x),
        ", ", std::to_string(imuParam.acc.y), ", ", std::to_string(imuParam.acc.z),
        "]\nGyroscope (x, y, z) = [", std::to_string(imuParam.gyro.x), ", ", std::to_string(imuParam.gyro.y), ", ",
        std::to_string(imuParam.gyro.z), "]\nTemperature (°C) = ", std::to_string(imuParam.temp),
        "\nFrame n° ", std::to_string(imuParam.frameNo));
}

PsReturnStatus PicoZenseHandler::SetDepthRange(PsDepthRange depthRange)
{
    PsReturnStatus status = PsSetDepthRange(m_deviceIndex, depthRange);
    if (status != PsRetOK)
        error("PsSetDepthRange failed with error ", PsStatusToString(status));
    else
    {
        info("SetDepthRange done");
        m_depthRange = depthRange;
    }
    return status;
}

PsReturnStatus PicoZenseHandler::SetColoPixelFormat(PsPixelFormat pixelFormat)
{
    PsReturnStatus status = PsSetColorPixelFormat(m_deviceIndex, pixelFormat);
    if (status != PsRetOK)
        error("PsSetColorPixelFormat failed with error ", PsStatusToString(status));
    else
        info("SetColorPixelFormat done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetDataMode(PsDataMode dataMode)
{
    PsReturnStatus status = PsSetDataMode(m_deviceIndex, dataMode);
    if (status != PsRetOK)
        error("PsSetDataMode failed with error ", PsStatusToString(status));
    else
    {
        info("PsSetDataMode done");
        m_dataMode = dataMode;
    }
    return status;
}

PsReturnStatus PicoZenseHandler::SetThreshold(uint16_t threshold)
{
    PsReturnStatus status = PsSetThreshold(m_deviceIndex, threshold);
    if (status != PsRetOK)
        error("PsSetThreshold failed with error ", PsStatusToString(status));
    else
        info("PsSetThreshold done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetFIlter(PsFilterType filterType, bool enable)
{
    PsReturnStatus status = PsSetFilter(m_deviceIndex, filterType, enable);
    if (status != PsRetOK)
        error("PsSetFilter failed with error ", PsStatusToString(status));
    else
        info("PsSetFilter done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetDepthDistortionCorrectionEnabled(bool enable)
{
    PsReturnStatus status = PsSetDepthDistortionCorrectionEnabled(m_deviceIndex, enable);
    if (status != PsRetOK)
        error("PsSetDepthDistortionCorrectionEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetDepthDistortionCorrectionEnabled done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetRGBDistortionCorrectionEnabled(bool enable)
{
    PsReturnStatus status = PsSetRGBDistortionCorrectionEnabled(m_deviceIndex, enable);
    if (status != PsRetOK)
        error("PsSetRGBDistortionCorrectionEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetRGBDistortionCorrectionEnabled done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetComputeRealDepthCorrectionEnabled(bool enable)
{
    PsReturnStatus status = PsSetComputeRealDepthCorrectionEnabled(m_deviceIndex, enable);
    if (status != PsRetOK)
        error("PsSetComputeRealDepthCorrectionEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetComputeRealDepthCorrectionEnabled done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetSmoothingFilterEnabled(bool enable)
{
    PsReturnStatus status = PsSetSmoothingFilterEnabled(m_deviceIndex, enable);
    if (status != PsRetOK)
        error("PsSetSmoothingFilterEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetSmoothingFilterEnabled done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetResolution(PsResolution resolution)
{
    PsReturnStatus status = PsSetResolution(m_deviceIndex, resolution);
    if (status != PsRetOK)
        error("PsSetResolution failed with error ", PsStatusToString(status));
    else
        info("PsSetResolution done");
    return status;
}

PsReturnStatus PicoZenseHandler::SetSpatialFilterEnabled(bool enable)
{
    PsReturnStatus status = PsSetSpatialFilterEnabled(m_deviceIndex, enable);
    if (status != PsRetOK)
        error("PsSetSpatialFilterEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetSpatialFilterEnabled done");
    return status;
}

void PicoZenseHandler::SetFeatureDetection(bool enable)
{
    m_detectorNARF = true;
}

void PicoZenseHandler::SetPointCloudClassic()
{
    // Disable RGB Mapped feature first
    PsReturnStatus status;
    if (m_pointCloudMappedRGB || m_wdrDepth)
    {
        if (m_wdrDepth)
        {
            status = PsSetDataMode(m_deviceIndex, PsDepthAndRGB_30);
            if (status != PsRetOK)
                error("PsSetDataMode failed with error ", PsStatusToString(status));
            m_wdrDepth = false;
        }
        //Disable the Depth and RGB synchronize feature
        status = PsSetSynchronizeEnabled(m_deviceIndex, false);
        if (status != PsRetOK)
            error("PsSetSynchronizeEnabled failed with error ", PsStatusToString(status));
        status = PsSetMapperEnabledDepthToRGB(m_deviceIndex, false);
        if (status != PsRetOK)
            error("PsSetMapperEnabledDepthToRGB failed with error ", PsStatusToString(status));
        m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PointCloud");
        m_pointCloudMappedRGB = false;
        m_pointCloudClassic = true;
    }
}

void PicoZenseHandler::SetPointCloudRGB()
{
    // Disable Classic way
    PsReturnStatus status;
    if (m_pointCloudClassic || m_wdrDepth)
    {
        if (m_wdrDepth)
        {
            status = PsSetDataMode(m_deviceIndex, PsDepthAndRGB_30);
            if (status != PsRetOK)
                error("PsSetDataMode failed with error ", PsStatusToString(status));
            m_wdrDepth = false;
        }
        //Enable the Depth and RGB synchronize feature
        status = PsSetSynchronizeEnabled(m_deviceIndex, true);
        if (status != PsRetOK)
            error("PsSetSynchronizeEnabled failed with error ", PsStatusToString(status));
        status = PsSetMapperEnabledDepthToRGB(m_deviceIndex, true);
        if (status != PsRetOK)
            error("PsSetMapperEnabledDepthToRGB failed with error ", PsStatusToString(status));
        m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloud");
        m_pointCloudClassic = false;
        m_pointCloudMappedRGB = true;
    }
}

void PicoZenseHandler::SetWDRDataMode()
{
    // Disable Classic and RGB
    PsReturnStatus status;
    if (m_pointCloudClassic || m_pointCloudMappedRGB)
    {
        if (m_pointCloudMappedRGB)
        {
            //Disable the Depth and RGB synchronize feature
            status = PsSetSynchronizeEnabled(m_deviceIndex, false);
            if (status != PsRetOK)
                error("PsSetSynchronizeEnabled failed with error ", PsStatusToString(status));
            status = PsSetMapperEnabledDepthToRGB(m_deviceIndex, false);
            if (status != PsRetOK)
                error("PsSetMapperEnabledDepthToRGB failed with error ", PsStatusToString(status));
            m_pointCloudMappedRGB = false;
        }
        PsDataMode dataMode = PsWDR_Depth;
        status = PsSetDataMode(m_deviceIndex, dataMode);
        if (status != PsRetOK)
            error("PsSetDataMode failed with error ", PsStatusToString(status));
        //Set WDR Output Mode, three ranges Near/Middle/Far output from device every one frame
        PsWDROutputMode wdrMode = {PsWDRTotalRange_Three, PsNearRange, 1, PsMidRange, 1, PsFarRange, 1};
        status = PsSetWDROutputMode(m_deviceIndex, &wdrMode);
        if (status != PsRetOK)
            error("PsSetWDROutputMode failed with error ", PsStatusToString(status));
        //Set WDR fusion threshold
        status = PsSetWDRFusionThreshold(m_deviceIndex, 1000, 2500);
        if (status != PsRetOK)
            error("PsSetWDRFusionThreshold failed with error ", PsStatusToString(status));
        status = PsSetWDRStyle(m_deviceIndex, PsWDR_FUSION);
        if (status != PsRetOK)
            error("PsSetWDRStyle failed with error ", PsStatusToString(status));
        
        m_pointCloudClassic = false;
        m_wdrDepth = true;
    }
}

void PicoZenseHandler::SetBilateralNoiseFilter(bool enable)
{
    // Using Bilateral filter that smooths the signal and preserves strong edges
    m_fastBiFilter = enable;
}

void PicoZenseHandler::SetBilateralUpsampling(bool enable)
{
    m_bilateralUpsampling = enable;
}

/*  Private Functions  */

std::string PicoZenseHandler::PsStatusToString(PsReturnStatus p_status)
{
    std::string ret;
    switch (p_status)
    {
    case PsRetOK:
        ret = "PsRetOK";
        break;
    case PsRetNoDeviceConnected:
        ret = "PsRetNoDeviceConnected";
        break;
    case PsRetInvalidDeviceIndex:
        ret = "PsRetInvalidDeviceIndex";
        break;
    case PsRetDevicePointerIsNull:
        ret = "PsRetDevicePointerIsNull";
        break;
    case PsRetInvalidFrameType:
        ret = "PsRetInvalidFrameType";
        break;
    case PsRetFramePointerIsNull:
        ret = "PsRetFramePointerIsNull";
        break;
    case PsRetNoPropertyValueGet:
        ret = "PsRetNoPropertyValueGet";
        break;
    case PsRetNoPropertyValueSet:
        ret = "PsRetNoPropertyValueSet";
        break;
    case PsRetPropertyPointerIsNull:
        ret = "PsRetPropertyPointerIsNull";
        break;
    case PsRetPropertySizeNotEnough:
        ret = "PsRetPropertySizeNotEnough";
        break;
    case PsRetInvalidDepthRange:
        ret = "PsRetInvalidDepthRange";
        break;
    case PsRetReadNextFrameError:
        ret = "PsRetReadNextFrameError";
        break;
    case PsRetInputPointerIsNull:
        ret = "PsRetInputPointerIsNull";
        break;
    case PsRetCameraNotOpened:
        ret = "PsRetCameraNotOpened";
        break;
    case PsRetInvalidCameraType:
        ret = "PsRetInvalidCameraType";
        break;
    case PsRetInvalidParams:
        ret = "PsRetInvalidParams";
        break;
    case PsRetOthers:
        ret = "PsRetOthers";
        break;
    default:
        break;
    }
    return ret;
}


pcl::visualization::PCLVisualizer::Ptr PicoZenseHandler::InitializeInterations()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->registerKeyboardCallback(keyboardEventHandler, (void *)this);
    viewer->registerMouseCallback(mouseEventHandler, (void *)this);
    viewer->registerPointPickingCallback(pointEventHandler, (void *)this);

    return (viewer);
}

void PicoZenseHandler::PointCloudCreatorXYZRGB(int p_height, int p_width, Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters params)
{
    p_imageRGB = cv::Mat(p_height, p_width, CV_8UC3, p_dataRGB);
    p_imageDepth = cv::Mat(p_height, p_width, CV_16UC1, p_dataDepth);
    
    p_imageDepth.convertTo(p_imageDepth, CV_32F); // convert depth image data to float type
    if (p_imageDepth.cols != p_imageRGB.cols && p_imageDepth.rows != p_imageRGB.rows)
    {
        error("Images with different sizes!!!");
        return;
    }

    pointCloudRGB->clear();
    if (m_bilateralUpsampling)
    {
        //Dimension must be initialized to use 2-D indexing
        //so will have Pointcloud organized in order to apply the bilateral Filter
        pointCloudRGB->width = p_width;
        pointCloudRGB->height = p_height;
        pointCloudRGB->resize(p_width * p_height);
    }
    

    //From formula: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    // It has be done a rotation around the z azis
    // Quaternions are a really cool way to express 3D rotation and it doesn't have to use rotation matrix
    // since orientation is expressed by 1 + i + j + k. Using eulero fornula quaternion orientation goes like
    // q = exp[angle/2*(ax*i + ay*j + az*k)] = cos(angle/2) + (ax*i + ay*j + az*k)*sin(angle/2)
    // What will acually happen internally is that every point of the Point cloud acquired by the sensor will be sandwich multipy by q
    // in this following way --> p' = q * p * q-1
    pointCloudRGB->sensor_orientation_ = q;

    for (int v = 0; v < p_imageDepth.rows; v += 1)
    {
        for (int u = 0; u < p_imageDepth.cols; u += 1)
        {
            if (p_imageDepth.at<float>(v, u) == 0)
                continue;
            PointXYZRGB p;
            Vec3b intensity = p_imageRGB.at<Vec3b>(v, u);
            p.r = intensity.val[0];
            p.g = intensity.val[1];
            p.b = intensity.val[2];
            p.z = p_imageDepth.at<float>(v, u);
            p.x = (u - (float)params.cx) * p.z / (float)params.fx;
            p.y = (v - (float)params.cy) * p.z / (float)params.fy;

            // Converting to meters
            p.z = p.z / 1000;
            p.x = p.x / 1000;
            p.y = p.y / 1000;

            if (m_bilateralUpsampling)
            {
                pointCloudRGB->at(u, v) = p;
            }
            else
            {
                pointCloudRGB->push_back(p);
            }
        }
    }

    if (m_bilateralUpsampling && pointCloudRGB->isOrganized())
    {
        pointCloudRGB = ApplyBilateralUpsampling(pointCloudRGB);
    }
    else
    {
        //Can keep it unorganized (1-D indexing)
        //Note that using push_back it will break the organized structure in unorganized
        pointCloudRGB->width = (uint32_t)pointCloudRGB->points.size();
        pointCloudRGB->height = 1;
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudRGB);

    if (!m_visualizer->updatePointCloud<pcl::PointXYZRGB>(pointCloudRGB, rgb, "PointCloud"))
        m_visualizer->addPointCloud<pcl::PointXYZRGB>(pointCloudRGB, rgb, "PointCloud");

    m_visualizer->spinOnce();
}

void PicoZenseHandler::PointCloudMapRGBDepthCustom(int p_height, int p_width, cv::Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters paramsDepth, PsCameraParameters paramsRGB, PsCameraExtrinsicParameters extrinsecParam)
{
    p_imageRGB = cv::Mat(p_height, p_width, CV_8UC3, p_dataRGB);
    p_imageDepth = cv::Mat(p_height, p_width, CV_16UC1, p_dataDepth);

    p_imageDepth.convertTo(p_imageDepth, CV_32F); // convert depth image data to float type
    if (p_imageDepth.cols != p_imageRGB.cols && p_imageDepth.rows != p_imageRGB.rows)
    {
        error("Images with different sizes!!!");
        return;
    }

    pointCloudRGB->clear();
    pointCloudRGB->sensor_orientation_ = q;

    for (int v = 0; v < p_imageDepth.rows; v += 1)
    {
        for (int u = 0; u < p_imageDepth.cols; u += 1)
        {
            if (p_imageDepth.at<float>(v, u) == 0)
                continue;
            PointXYZRGB p;
            Vec3b intensity = p_imageRGB.at<Vec3b>(v, u);
            p.r = intensity.val[0];
            p.g = intensity.val[1];
            p.b = intensity.val[2];
            p.z = p_imageDepth.at<float>(v, u);
            //Reverting from camera coordinates to real word using Camera Projection theory
            p.x = (u - (float)paramsDepth.cx) * p.z / (float)paramsDepth.fx;
            p.y = (v - (float)paramsDepth.cy) * p.z / (float)paramsDepth.fy;

            // Converting to meters
            p.z = p.z / 1000;
            p.x = p.x / 1000;
            p.y = p.y / 1000;

            pointCloudRGB->push_back(p);
        }
    }

    pointCloudRGB->width = (uint32_t)pointCloudRGB->points.size();
    pointCloudRGB->height = 1;

    if (!m_visualizer->updatePointCloud(pointCloudRGB, "PointCloud"))
        m_visualizer->addPointCloud(pointCloudRGB, "PointCloud");

    m_visualizer->spinOnce();
}

    void PicoZenseHandler::PointCloudCreatorXYZ(int p_height, int p_width, Mat &p_image, uint8_t *p_data, PsCameraParameters params)
{
    p_image = cv::Mat(p_height, p_width, CV_16UC1, p_data);
    p_image.convertTo(p_image, CV_32F); // convert image data to float type
    if (!imageMatrix.data)
        error("No depth data");

    pointCloud->clear();

    //From formula: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    // It has be done a rotation around the z azis
    pointCloud->sensor_orientation_ = q;

    //Getting the world's points coordinate from the image relying on
    //Perspective projection using homogeneous coordinates.
    //Given a real world point as (X,Y,Z) the camera will retrieve
    //the the pixel points (2D:u,v) such as u=X*f/Z and v=Y*f/Z
    //As the depth camera stores inside each frame pixel (u,v) camera coords
    //the Z value I'll calculate the 

    for (int v = 0; v < p_image.rows; v += 1)
    {
        for (int u = 0; u < p_image.cols; u += 1)
        {
            if (p_image.at<float>(v, u) == 0)   continue;
            PointXYZ p;
            p.z = p_image.at<float>(v, u);
            p.x = (u - (float)params.cx) * p.z / (float)params.fx;
            p.y = (v - (float)params.cy) * p.z / (float)params.fy;

            // Converting to meters
            p.z = p.z / 1000;
            p.x = p.x / 1000;
            p.y = p.y / 1000;

            pointCloud->push_back(p);
        }
    }

    if(m_fastBiFilter)
    {
        //Dimension must be initialized to use 2-D indexing
        //so will have Pointcloud organized in order to apply the bilateral Filter
        pointCloud->width = p_width;
        pointCloud->height = p_height;
        pointCloud->resize(p_width * p_height);
        pointCloud = ApplyBilateralFilter(pointCloud);
    }
    else
    {
        // Keep it unorganized so 1-D indexing
        pointCloud->width = (uint32_t)pointCloud->points.size();
        pointCloud->height = 1;
    }

    if (!m_visualizer->updatePointCloud(pointCloud, "PointCloud"))
        m_visualizer->addPointCloud(pointCloud, "PointCloud");


    // Invoke a corner detection method
    if (m_detectorHarris)
        Harris3DCornerDetection();
    else if (m_detectorNARF)
        NARFCorenerDetection();
    else if (m_detectorISS)
        ISSCornerDetection();

    m_visualizer->spinOnce();
}

void PicoZenseHandler::Harris3DCornerDetection()
{
    // HarrisKeypoint3D uses the idea of 2D Harris keypoints, but instead of using image gradients, it uses surface normals.

    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
    detector.setNonMaxSupression(true);
    detector.setRadiusSearch(0.1f);
    detector.setInputCloud(pointCloud);
    detector.setRefine(false);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    detector.compute(*keypoints);

    debug("keypoints detected: ", keypoints->size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ tmp;
    double max = 0, min = 0;

    for (auto i = keypoints->begin(); i != keypoints->end(); i++)
    {
        tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
        if ((*i).intensity > max)
        {
            debug("Intensity: ", (*i), " Point: [", (*i).x, "; ", (*i).y, "; ", (*i).z, "]");
            max = (*i).intensity;
        }
        if ((*i).intensity < min)
        {
            min = (*i).intensity;
        }
        keypoints3D->push_back(tmp);
    }

    // Show point cloud on existing viewrer
    m_visualizer->addPointCloud(pointCloud, "Keypoints");
}

void PicoZenseHandler::NARFCorenerDetection()
{
    // NARF (Normal Aligned Radial Feature) keypoints.
    // Input is a range image, output the indices of the keypoints See B. Steder, R. B. Rusu, K. Konolige, and W. Burgard Point Feature Extraction
    // on 3D Range Scans Taking into Account Object Boundaries In Proc. of the IEEE Int. Conf. on Robotics &Automation (ICRA). 2011.
    // http://ais.informatik.uni-freiburg.de/publications/papers/steder11icra.pdf
    CreateRangeImage();
    // Show range image
    // pcl::visualization::RangeImageVisualizer rangeImageWidget("Range Image");
    // rangeImageWidget.showRangeImage(*rangeImage);
    
    //Now that rangeImage is created let's extract borders
    pcl::RangeImage &rangeImageRef = *rangeImage;
    pcl::RangeImageBorderExtractor borderExtractor(&rangeImageRef); 
    pcl::PointCloud<pcl::BorderDescription> borderDescriptions;
    borderExtractor.compute(borderDescriptions);
    //Showing now results on viewer. Border, Veil and Showdow border
    pcl::PointCloud<pcl::PointWithRange>::Ptr borderPointsPtr(new pcl::PointCloud<pcl::PointWithRange>),
        veilPointsPtr(new pcl::PointCloud<pcl::PointWithRange>),
        shadowPointsPtr(new pcl::PointCloud<pcl::PointWithRange>);
    
    pcl::PointCloud<pcl::PointWithRange> &borderPoints = *borderPointsPtr,
                                         &veilPoints = *veilPointsPtr,
                                         &shadowPoints = *shadowPointsPtr;
    for (int y = 0; y < (int)rangeImageRef.height; ++y)
    {
        for (int x=0; x<(int)rangeImageRef.width; ++x)
        {
            if (borderDescriptions.points[y * rangeImageRef.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                borderPoints.points.push_back(rangeImageRef.points[y * rangeImageRef.width + x]);
            if (borderDescriptions.points[y * rangeImageRef.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veilPoints.points.push_back(rangeImageRef.points[y * rangeImageRef.width + x]);
            if (borderDescriptions.points[y * rangeImageRef.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadowPoints.points.push_back(rangeImageRef.points[y * rangeImageRef.width + x]);
        }
    }
    borderPoints.sensor_orientation_ = q;
    veilPoints.sensor_orientation_ = q;
    shadowPoints.sensor_orientation_ = q;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> borderPointsColorHandler(borderPointsPtr, 0, 255, 0);
    if (!m_visualizer->updatePointCloud<pcl::PointWithRange>(borderPointsPtr, borderPointsColorHandler, "border points"))
        m_visualizer->addPointCloud<pcl::PointWithRange>(borderPointsPtr, borderPointsColorHandler, "border points");
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veilPointsColorHandler(veilPointsPtr, 255, 0, 0);
    if (!m_visualizer->updatePointCloud<pcl::PointWithRange>(veilPointsPtr, veilPointsColorHandler, "veil points"))
        m_visualizer->addPointCloud<pcl::PointWithRange>(veilPointsPtr, veilPointsColorHandler, "veil points");
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadowPointsColorHandler(shadowPointsPtr, 0, 255, 255);
    if (!m_visualizer->updatePointCloud<pcl::PointWithRange>(shadowPointsPtr, shadowPointsColorHandler, "shadow points"))
        m_visualizer->addPointCloud<pcl::PointWithRange>(shadowPointsPtr, shadowPointsColorHandler, "shadow points");
    m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
}

void PicoZenseHandler::ISSCornerDetection()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    double model_resolution = 0.1;
    // Compute model_resolution
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(6 * model_resolution); //The radius of the spherical neighborhood used to compute the scatter matrix.
    iss_detector.setNonMaxRadius(4 * model_resolution);  //The non maxima suppression radius.
    iss_detector.setThreshold21(0.975);                  //The upper bound on the ratio between the second and the first eigenvalue returned by the EVD.
    iss_detector.setThreshold32(0.975);                  //The upper bound on the ratio between the third and the second eigenvalue returned by the EVD.
    iss_detector.setMinNeighbors(2);                     //Minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
    iss_detector.setNumberOfThreads(4);
    iss_detector.setInputCloud(pointCloud);
    iss_detector.compute(*model_keypoints);

    debug("Keypoits size: ", model_keypoints->size());
    for (auto i = model_keypoints->begin(); i < model_keypoints->end(); i++)
    {
        debug("Point: [", (*i).x, "; ", (*i).y, "; ", (*i).z, "]");
    }
    
    m_visualizer->addPointCloud(model_keypoints, "Keypoints");
}

void PicoZenseHandler::CreateRangeImage()
{
    pcl::PointCloud<PointXYZ>& pCloud = *pointCloud;
    float noiseLevel = 0.0;
    float minRange = 0.0f;
    int borderSize = 1;
    Eigen::Affine3f sceneSensorPose = Eigen::Affine3f(Eigen::Translation3f(pCloud.sensor_origin_[0], pCloud.sensor_origin_[1], pCloud.sensor_origin_[2])) * Eigen::Affine3f(pCloud.sensor_orientation_);
    rangeImage->createFromPointCloud(pCloud, pcl::deg2rad(0.5f), pcl::deg2rad(360.0f),
                pcl::deg2rad(180.0f), sceneSensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel, minRange, borderSize);
    rangeImage->setUnseenToMaxRange();
    //If something doesn't work try this way
    // pcl::RangeImage &range_image = *rangeImage;
    // range_image.createFromPointCloud...
    // range_image.integrateFarRanges (far_ranges);
    // range_image.setUnseenToMaxRange();
}

PointCloud<PointXYZ>::Ptr PicoZenseHandler::ApplyBilateralFilter(PointCloud<PointXYZ>::Ptr cloud_in)
{
    PointCloud<PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::FastBilateralFilter<pcl::PointXYZ> bilateral_filter;
    bilateral_filter.setInputCloud(cloud_in);
    bilateral_filter.setSigmaS(2);
    bilateral_filter.setSigmaR(0.05f);
    bilateral_filter.filter(*cloud_out);
    return cloud_out;
}

PointCloud<PointXYZRGB>::Ptr PicoZenseHandler::ApplyBilateralUpsampling(PointCloud<PointXYZRGB>::Ptr cloud_in)
{
    PointCloud<PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::BilateralUpsampling<PointXYZRGB, PointXYZRGB> bilateral_upsampling;
    bilateral_upsampling.setInputCloud(cloud_in);
    bilateral_upsampling.setSigmaColor(15.f);
    bilateral_upsampling.setSigmaDepth(0.5f);
    bilateral_upsampling.process(*cloud_out);
    return cloud_out;
}

static void mouseEventHandler(const pcl::visualization::MouseEvent &event, void *pico)
{
    // std::cout << "Mouse: button" << std::to_string(event.getButton()) << " type -> " << event.getType() << std::endl;
}

static void keyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void *pico)
{
    PicoZenseHandler* picoHandler = static_cast<PicoZenseHandler *>(pico);
    if (event.getKeySym() == "p" && event.keyUp())
    {
        picoHandler->GetCameraParameters();
    }
    else if (event.getKeySym() == "Escape" && event.keyUp())
    {
        info("Shutting down ...");
        m_loop = false;
        // Free what allocated
    }
}

static void pointEventHandler(const pcl::visualization::PointPickingEvent &event, void *pico)
{
    PicoZenseHandler *picoHandler = static_cast<PicoZenseHandler *>(pico);
    float x, y, z;
    event.getPoint(x, y, z);
    debug("Point clicked [", x, "; ", y, "; ", z, "]");
    if (old.z == 0)
    {
        old.x = x;
        old.y = y;
        old.z = z;
    }
    else
    {
        //Calculate here euclidean 
        PointXYZ novo;
        novo.x = x;
        novo.y = y;
        novo.z = z;

        double distance = std::sqrt(std::pow((old.x - novo.x), 2) +
        std::pow((old.y - novo.y), 2) + std::pow((old.z - novo.z), 2));
        debug("Distance calculated is ", std::to_string(distance));

        old = novo;
        //Some issues here to be fixed
        // picoHandler->m_visualizer->removeShape("line");
        // picoHandler->m_visualizer->addLine(old, novo, 0, 1, 0, "line");
    }
 }
