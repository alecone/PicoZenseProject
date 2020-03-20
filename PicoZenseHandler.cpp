#include "PicoZenseHandler.h"


using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;


//Global variables
static void keyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* pico);
static void mouseEventHandler(const pcl::visualization::MouseEvent &event, void* pico);
static void pointEventHandler(const pcl::visualization::PointPickingEvent &event, void *pico);
static bool m_loop;
static bool m_pause;
static PointXYZ old;

static int KERNEL_LENGTH = 15;  // This might be tuned
static int KERNEL_LENGTH_BILATREL = 15; //See function explanation
static double SIGMA_X = 1.0;
static double SIGMA_Y = 1.0;
static double SIGMA_COLOR = 50;
static double SIGMA_SPACE = 75;

PicoZenseHandler::PicoZenseHandler(int32_t devIndex, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    InitializeInterations(viewer);
    m_visualizer = viewer;
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
    m_save = false;
    m_normalizedBoxFilter = false;
    m_gaussianFilter = false;
    m_bilateralFilter = false;
    m_stattisticalOutlierRemoval = false;
    m_radialOutlierRemoval = false;
    m_mlsUpsampling = false;
    m_saveIndex = 0;
    m_loop = true;
    m_pause = false;

    info("Starting with:\n\tDepthRange: PsNearRange\n\tm_dataMode: PsWDR_Depth\n\tPixelFormat: PsPixelFormatRGB888\nDevice #", std::to_string(m_deviceIndex));
}


PicoZenseHandler::~PicoZenseHandler()
{
    if (m_visualizer != nullptr)
    {
        m_visualizer->removeAllPointClouds();
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
    m_deviceCount = 0;
    // uint32_t slope = 1450;
    // uint32_t wdrSlope = 4400;
    PsGetDeviceCount(&m_deviceCount);

    status = PsInitialize();
    if (status != PsReturnStatus::PsRetOK)
    {
        error("PsInitialize failed!");
        exit(1);
    }

    // uint32_t slope = 1450;
    // uint32_t wdrSlope = 4400;

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

    //Set to data mode
    status = PsSetDataMode(m_deviceIndex, (PsDataMode)m_dataMode);
    if (status != PsReturnStatus::PsRetOK)
    {
        warn("Set DataMode Failed failed!");
    }
}

void *PicoZenseHandler::Visualize(boost::barrier &p_barier)
{
    // Printing threadID
    debug("Running threadID: ", boost::this_thread::get_id());
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

    Matrix3d R;
    R << CameraExtrinsicParameters.rotation[0], CameraExtrinsicParameters.rotation[1], CameraExtrinsicParameters.rotation[2],
        CameraExtrinsicParameters.rotation[3], CameraExtrinsicParameters.rotation[4], CameraExtrinsicParameters.rotation[5],
        CameraExtrinsicParameters.rotation[6], CameraExtrinsicParameters.rotation[7], CameraExtrinsicParameters.rotation[8];
    Vector3d t;
    t << CameraExtrinsicParameters.translation[0], CameraExtrinsicParameters.translation[1], CameraExtrinsicParameters.translation[2];

    // Main loop
    while (m_loop)
    {
        if (m_pause)
        {
            while (m_pause)
            {
                vis_mutex.lock();
                m_visualizer->spinOnce();
                vis_mutex.unlock();
                usleep(100000);
            }
        }
        PsFrame depthFrame = {0};
        PsFrame wdrDepthFrame = {0};
        PsFrame mappedRGBFrame = {0};

        // Read one frame before call PsGetFrame
        status = PsReadNextFrame(m_deviceIndex);
        if (status != PsRetOK)
        {
            warn("PsReadNextFrame gave ", PsStatusToString(status));
            usleep(50000);
            continue;
        }

        //Get depth frame, depth frame only output in following data mode
        if (!m_wdrDepth && (m_dataMode == PsDepthAndRGB_30 || m_dataMode == PsDepthAndIR_30 || m_dataMode == PsDepthAndIRAndRGB_30 || m_dataMode == PsDepthAndIR_15_RGB_30))
        {
            PsGetFrame(m_deviceIndex, PsDepthFrame, &depthFrame);

            if (!m_pointCloudMappedRGB && depthFrame.pFrameData != NULL)
            {
                //Generate and display PointCloud
                PointCloudCreatorXYZ(depthFrame.height, depthFrame.width, imageMatrix, depthFrame.pFrameData, depthCameraParameters, p_barier);
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
                PointCloudCreatorXYZ(wdrDepthFrame.height, wdrDepthFrame.width, imageMatrix, wdrDepthFrame.pFrameData, depthCameraParameters, p_barier);
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
                    PointCloudCreatorXYZRGB(mappedRGBFrame.height, mappedRGBFrame.width, imageMatrixRGB, imageMatrix, mappedRGBFrame.pFrameData, wdrDepthFrame.pFrameData, depthCameraParameters, p_barier);
                else
                    PointCloudCreatorXYZRGB(mappedRGBFrame.height, mappedRGBFrame.width, imageMatrixRGB, imageMatrix, mappedRGBFrame.pFrameData, depthFrame.pFrameData, depthCameraParameters, p_barier);
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

PointCloud<PointXYZRGB>::Ptr PicoZenseHandler::GetRGBPointCloud()
{
    if (pointCloudRGB != nullptr)
    {
        return pointCloudRGB;
    }
}

PointCloud<PointXYZ>::Ptr PicoZenseHandler::GetPointCloud()
{
    if (pointCloudRGB != nullptr)
    {
        return pointCloud;
    }
}

void PicoZenseHandler::SavePCD()
{
    m_save = true;
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
        info("PsSetDataMode done -> ", std::to_string(dataMode));
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
        vis_mutex.lock();
        if (m_deviceCount == 1)
            m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PointCloud");
        else
        {
            m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PointCloudSX");
            m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PointCloudDX");
        }
        vis_mutex.unlock();
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
        vis_mutex.lock();
        if (m_deviceCount == 1)
            m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloud");
        else
        {
            m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloudSX");
            m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloudDX");
        }
        vis_mutex.unlock();
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

bool PicoZenseHandler::IsPointCloudRGBEnabled()
{
    return m_pointCloudMappedRGB;
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

void PicoZenseHandler::SetNormalizedBoxFilter(bool enable)
{
    m_normalizedBoxFilter = enable;
}

void PicoZenseHandler::SetGaussinFilter(bool enable)
{
    m_gaussianFilter = enable;
}

void PicoZenseHandler::SetBilateralFilter(bool enable)
{
    m_bilateralFilter = enable;
}

void PicoZenseHandler::SetStatisticalOutlierRemoval(bool enable)
{
    m_stattisticalOutlierRemoval = enable;
}

void PicoZenseHandler::SetRadialOutlierRemoval(bool enable)
{
    m_radialOutlierRemoval = enable;
}

void PicoZenseHandler::SetMLSUpsampling(bool enable)
{
    m_mlsUpsampling = enable;
}

void PicoZenseHandler::SetFastTriangolation(bool enable)
{
    m_fastTriangolation = enable;
}

void PicoZenseHandler::SetPolynomialReconstruction(bool enable)
{
    m_polynomialReconstraction = enable;
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

void PicoZenseHandler::InitializeInterations(pcl::visualization::PCLVisualizer::Ptr viewer)
{
    viewer->registerKeyboardCallback(keyboardEventHandler, (void *)this);
    viewer->registerMouseCallback(mouseEventHandler, (void *)this);
    viewer->registerPointPickingCallback(pointEventHandler, (void *)this);
}

void PicoZenseHandler::PointCloudCreatorXYZRGB(int p_height, int p_width, Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters params, boost::barrier &p_barier)
{
    p_imageRGB = cv::Mat(p_height, p_width, CV_8UC3, p_dataRGB);
    p_imageDepth = cv::Mat(p_height, p_width, CV_16UC1, p_dataDepth);
    
    p_imageDepth.convertTo(p_imageDepth, CV_32F); // convert depth image data to float type
    if (p_imageDepth.cols != p_imageRGB.cols && p_imageDepth.rows != p_imageRGB.rows)
    {
        error("Images with different sizes!!!");
        return;
    }

    if (m_normalizedBoxFilter)
    {
        //This filter is the simplest of all !Each output pixel is the mean of its kernel neighbors(all of them contribute with equal weights)
        cv::Mat postProc = p_imageRGB.clone();
        blur(p_imageRGB, postProc, Size(KERNEL_LENGTH, KERNEL_LENGTH), Point(-1, -1));
        p_imageRGB = postProc;
    }
    else if (m_gaussianFilter)
    {
        //Gaussian filtering is done by convolving each point in the input array with a Gaussian kernel and then summing them all to produce the output array.
        cv::Mat postProc = p_imageRGB.clone();
        GaussianBlur(p_imageRGB, postProc, Size(KERNEL_LENGTH, KERNEL_LENGTH), SIGMA_X, SIGMA_Y);
        p_imageRGB = postProc;
    }
    else if (m_bilateralFilter)
    {
        // In an analogous way as the Gaussian filter, the bilateral filter also considers the neighboring pixels with
        // weights assigned to each of them. These weights have two components, the first of which is the same weighting
        // used by the Gaussian filter. The second component takes into account the difference in intensity between
        // the neighboring pixels and the evaluated one. Smooth but enhance edges
        cv::Mat postProc = p_imageRGB.clone();
        bilateralFilter(p_imageRGB, postProc, KERNEL_LENGTH_BILATREL, SIGMA_COLOR, SIGMA_SPACE);
        p_imageRGB = postProc;
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
            if (p_imageDepth.at<float>(v, u) == 0) continue;
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

    if (m_save)
    {
        std::string filename(PCD_FILE_PATH);
        filename.append("CAMERA_");
        filename.append(std::to_string(m_deviceIndex));
        filename.append("_");
        filename.append(std::to_string(m_saveIndex++));
        filename.append(".pcd");
        pcl::PCDWriter w;
        w.write(filename, *pointCloudRGB);
        debug("Saved for camera #", std::to_string(m_deviceIndex));
        m_save = false;
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

    if (m_stattisticalOutlierRemoval)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(pointCloudRGB);
        // Must be tuned
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        // sor.setUserFilterValue(/*The mean of the neighbors*/)
        sor.filter(*cloud_filtered);
        pointCloudRGB = cloud_filtered;
    }
    else if (m_radialOutlierRemoval)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        // build the filter
        outrem.setInputCloud(pointCloudRGB);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius(2);
        // outrem.setUserFilterValue();
        outrem.filter(*cloud_filtered);
        pointCloudRGB = cloud_filtered;
    }
    else if(m_mlsUpsampling)
    {
        pointCloudRGB = ApplyMLSUpsamplingRGB(pointCloudRGB);
    }
    else if (m_fastTriangolation)
    {
        // EDIT HERE in order to work with both camera too
        m_visualizer->removePointCloud("PointCloud");
        FastTriangolationRGB(pointCloudRGB);
        return;
    }
    else if (m_polynomialReconstraction)
    {
        // EDIT HERE in order to work with both camera too
        m_visualizer->removePointCloud("PointCloud");
        PolynomialReconstructionRGB(pointCloudRGB);
        return;
    }
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudRGB);

    // TODO move this to function for synchronization
    SendToVisualizer(pointCloudRGB, m_deviceIndex, p_barier);
}

void PicoZenseHandler::PointCloudCreatorXYZ(int p_height, int p_width, Mat &p_image, uint8_t *p_data, PsCameraParameters params, boost::barrier &p_barier)
{
    p_image = cv::Mat(p_height, p_width, CV_16UC1, p_data);
    p_image.convertTo(p_image, CV_32F); // convert image data to float type
    if (!imageMatrix.data)
        error("No depth data");

    if (m_normalizedBoxFilter)
    {
        //This filter is the simplest of all !Each output pixel is the mean of its kernel neighbors(all of them contribute with equal weights)
        cv::Mat postProc = p_image.clone();
        blur(p_image, postProc, Size(KERNEL_LENGTH, KERNEL_LENGTH), Point(-1, -1));
        p_image = postProc;
    }
    else if (m_gaussianFilter)
    {
        //Gaussian filtering is done by convolving each point in the input array with a Gaussian kernel and then summing them all to produce the output array.
        cv::Mat postProc = p_image.clone();
        GaussianBlur(p_image, postProc, Size(KERNEL_LENGTH, KERNEL_LENGTH), SIGMA_X, SIGMA_Y);
        p_image = postProc;
    }
    else if (m_bilateralFilter)
    {
        // In an analogous way as the Gaussian filter, the bilateral filter also considers the neighboring pixels with
        // weights assigned to each of them. These weights have two components, the first of which is the same weighting
        // used by the Gaussian filter. The second component takes into account the difference in intensity between
        // the neighboring pixels and the evaluated one. Smooth but enhance edges
        cv::Mat postProc = p_image.clone();
        bilateralFilter(p_image, postProc, KERNEL_LENGTH_BILATREL, SIGMA_COLOR, SIGMA_SPACE);
        p_image = postProc;
    }

    pointCloud->clear();

    //From formula: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    // It has be done a rotation around the z azis
    pointCloud->sensor_orientation_ = q;

    //Getting the world's points coordinate from the image relying on
    //Perspective projection using homogeneous coordinates.
    //Given a real world point as (X,Y,Z) the camera will retrieve
    //the the pixel points (2D:u,v) such as u=X*f/Z and v=Y*f/Z
    //As the depth camera stores inside each frame pixel (u,v) camera coords
    //the Z value I'll calculate the (x, y) pair according to the camera sys coord.
    //N.B. I'm taking into account also the pixelization process of objects

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

    if (m_save)
    {
        std::string filename(PCD_FILE_PATH);
        filename.append("CAMERA_");
        filename.append(std::to_string(m_deviceIndex));
        filename.append("_");
        filename.append(std::to_string(m_saveIndex++));
        filename.append(".pcd");
        pcl::PCDWriter w;
        w.write(filename, *pointCloud);
        debug("Saved for camera #", std::to_string(m_deviceIndex));
        m_save = false;
    }

    if (m_stattisticalOutlierRemoval)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(pointCloud);
        // Must be tuned
        sor.setMeanK(25);
        sor.setStddevMulThresh(1.0);
        // sor.setUserFilterValue(/*The mean of the neighbors*/)
        sor.filter(*cloud_filtered);
        pointCloud = cloud_filtered;
    }
    else if (m_radialOutlierRemoval)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(pointCloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius(2);
        // outrem.setUserFilterValue();
        outrem.filter(*cloud_filtered);
        pointCloud = cloud_filtered;
    }
    else if (m_mlsUpsampling)
    {
        pointCloud = ApplyMLSUpsampling(pointCloud);
    }

    // Invoke a corner detection method
    if (m_detectorHarris)
        Harris3DCornerDetection();
    else if (m_detectorNARF)
        NARFCorenerDetection();
    else if (m_detectorISS)
        ISSCornerDetection();

    SendToVisualizer(pointCloud, m_deviceIndex, p_barier);
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

    vis_mutex.lock();
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
    vis_mutex.unlock();
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

PointCloud<PointXYZ>::Ptr PicoZenseHandler::ApplyMLSUpsampling(PointCloud<PointXYZ>::Ptr cloud_in)
{
    PointCloud<PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // Output has the same type as the input one, it will be only smoothed
    pcl::PointCloud<pcl::PointXYZ> mls_points;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

    // Optionally, a pointer to a cloud can be provided, to be set by MLS
    // pcl::PointCloud<pcl::Normal>::Ptr mls_normals(new pcl::PointCloud<pcl::Normal>());
    // mls.setOutputNormals(mls_normals);
    // Set parameters
    mls.setInputCloud(cloud_in);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03); //5 cm
    // In order of efficienct: DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
    mls.setUpsamplingMethod(MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
    // mls.setDilationIterations();
    mls.setDilationVoxelSize(0.001);
    mls.setPolynomialOrder(2);
    // Reconstruct
    mls.process(*cloud_out);
    return cloud_out;
}

PointCloud<PointXYZRGB>::Ptr PicoZenseHandler::ApplyMLSUpsamplingRGB(PointCloud<PointXYZRGB>::Ptr cloud_in)
{
    MovingLeastSquares<PointXYZRGB, PointXYZRGB> mls;
    mls.setInputCloud(cloud_in);
    //The larger the radius the better performace, the higher computation cost
    mls.setSearchRadius(0.08);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZRGB, PointXYZRGB>::VOXEL_GRID_DILATION);
    mls.setUpsamplingRadius(0.10);
    mls.setDilationVoxelSize(0.02);
    mls.setUpsamplingStepSize(0.01);
    PointCloud<PointXYZRGB>::Ptr cloud_smoothed(new PointCloud<PointXYZRGB>());
    mls.process(*cloud_smoothed);

    NormalEstimationOMP<PointXYZRGB, Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(cloud_smoothed);
    ne.setRadiusSearch(0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloud_smoothed, centroid);
    debug("Centroids are: ", centroid[0], ", ", centroid[1], ", ", centroid[2]);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
    ne.compute(*cloud_normals);
    for (size_t i = 0; i < cloud_normals->size(); ++i)
    {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    PointCloud<PointXYZRGBNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointXYZRGBNormal>());
    concatenateFields(*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
    Poisson<PointXYZRGBNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    PolygonMesh mesh;
    poisson.reconstruct(mesh);

    time_t now = time(0);
    char *dt = ctime(&now);
    std::string filename(PCD_FILE_PATH);
    filename.append("POISSON_");
    filename.append(dt);
    filename.append(".ply");
    io::savePLYFile(filename, mesh);
    debug("Saved poisson reconstraction!");

    return cloud_smoothed;
}

void PicoZenseHandler::PolynomialReconstructionRGB(PointCloud<PointXYZRGB>::Ptr cloud_in)
{
    // Create a KD-Tree
    pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    PointCloud<PointXYZRGBNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    MovingLeastSquares<PointXYZRGB, PointXYZRGBNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud_in);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(mls_points);

    time_t now = time(0);
    char *dt = ctime(&now);
    std::string filename(PCD_FILE_PATH);
    filename.append("POLINOMIAL_");
    filename.append(dt);
    filename.append(".pcd");
    io::savePCDFile(filename, mls_points);
    debug("Saved polinomial reconstraction!");

    //Find a way to visualize it

}

void PicoZenseHandler::FastTriangolationRGB(PointCloud<PointXYZRGB>::Ptr cloud_in)
{
    // Normal estimation*
    NormalEstimation<PointXYZRGB, Normal> n;
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>);
    tree->setInputCloud(cloud_in);
    n.setInputCloud(cloud_in);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZRGB and normal fields*
    PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals(new PointCloud<PointXYZRGBNormal>);
    concatenateFields(*cloud_in, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<PointXYZRGBNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    GreedyProjectionTriangulation<PointXYZRGBNormal> gp3;
    PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();


    time_t now = time(0);
    char *dt = ctime(&now);
    std::string filename(PCD_FILE_PATH);
    filename.append("TRIANGLES_");
    filename.append(dt);
    filename.append(".ply");
    io::savePLYFile(filename, triangles);
    debug("Saved triangles reconstraction!");

    //Find a way to visualize it
}

void PicoZenseHandler::SendToVisualizer(PointCloud<PointXYZ>::Ptr cloud_in, int32_t dev_index, boost::barrier &p_barier)
{
    if (m_deviceCount == 2)
    {
        p_barier.wait();
    }
    else
    {
        if (!m_visualizer->updatePointCloud(cloud_in, "PointCloud"))
            m_visualizer->addPointCloud(cloud_in, "PointCloud");
        m_visualizer->spinOnce();
    }
}

void PicoZenseHandler::SendToVisualizer(PointCloud<PointXYZRGB>::Ptr cloud_in, int32_t dev_index, boost::barrier &p_barier)
{
    if (m_deviceCount == 2)
    {
        p_barier.wait();
    }
    else
    {
        if (!m_visualizer->updatePointCloud(cloud_in, "PointCloud"))
            m_visualizer->addPointCloud(cloud_in, "PointCloud");
        m_visualizer->spinOnce();
    }
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
        m_pause = !m_pause;
    }
    else if (event.getKeySym() == "Escape" && event.keyUp())
    {
        info("Shutting down ...");
        // add mutex here
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
        //First time clicking
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
    }
 }
