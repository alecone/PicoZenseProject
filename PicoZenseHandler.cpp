#include "PicoZenseHandler.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

static void keyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* pico);
static void mouseEventHandler(const pcl::visualization::MouseEvent &event, void* pico);
static void pointEventHandler(const pcl::visualization::PointPickingEvent &event, void *viewer);

PicoZenseHandler::PicoZenseHandler(int32_t devIndex)
{
    debug("PicoZense object created");
    m_visualizer = InitializeInterations();
    m_visualizer->setBackgroundColor(0.0, 0.0, 0.0);
    pointCloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
    pointCloudRGB = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>());
    rangeImage = pcl::RangeImage::Ptr(new pcl::RangeImage());

    m_devIndex = devIndex;
    m_depthRange = PsNearRange;
    m_dataMode = PsDepthAndRGB_30;
    m_pointCloudClassic = true;
    m_wdrDepth = false;
    m_pointCloudMappedRGB = false;
    m_detectorHarris = false;
    m_detectorNARF = false;
    m_detectorISS = false;

    info("Starting with:\n\tDepthRange: PsNearRange\n\tm_dataMode: PsDepthAndRGB_30\n\tPixelFormat: PsPixelFormatRGB888");
}


PicoZenseHandler::~PicoZenseHandler()
{
    if (pointCloud != nullptr)
    {
        pointCloud = nullptr;
        // delete pointCloud;
    }
    if (pointCloudRGB != nullptr)
    {
        pointCloudRGB = nullptr;
        // delete pointCloudRGB;
    }
    if (m_visualizer != nullptr)
    {
        m_visualizer = nullptr;
        // delete m_visualizer;
    }
    if (rangeImage != nullptr)
    {
        rangeImage = nullptr;
        // delete rangeImage;
    }

    debug("PicoZense destroyed");    
}


void PicoZenseHandler::Visualize()
{
    PsReturnStatus status;
    int32_t deviceCount = 0;
    uint32_t slope = 1450;
    uint32_t wdrSlope = 4400;

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
        error("OpenDevice failed!");
        system("pause");
        return;
    }

    //Set PixelFormat as PsPixelFormatBGR888 for opencv display
    PsSetColorPixelFormat(m_deviceIndex, PsPixelFormatRGB888);

    //Set to data mode
    status = PsSetDataMode(m_deviceIndex, (PsDataMode)m_dataMode);
    if (status != PsReturnStatus::PsRetOK)
    {
        std::cout << "Set DataMode Failed failed!\n";
    }

    //WDR STUFF
    //Set WDR Output Mode, three ranges Near/Middle/Far output from device every one frame
    // PsWDROutputMode wdrMode = { PsWDRTotalRange_Three, PsNearRange, 1, PsMidRange, 1, PsNearRange, 1 };
    //Set WDR fusion threshold
    // PsSetWDRFusionThreshold(m_deviceIndex, 1000, 2000);
    // PsSetWDROutputMode(m_deviceIndex, &wdrMode);
    // PsSetWDRStyle(m_deviceIndex, PsWDR_ALTERNATION);

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
    for (;;)
    {
        PsFrame depthFrame = {0};
        PsFrame wdrDepthFrame = {0};
        PsFrame mappedRGBFrame = {0};

        // Read one frame before call PsGetFrame
        status = PsReadNextFrame(m_deviceIndex);
        if (status != PsRetOK)
            warn("PsReadNextFrame gave ", PsStatusToString(status));

        //Get depth frame, depth frame only output in following data mode
        if (!m_pointCloudMappedRGB && !m_wdrDepth && (m_dataMode == PsDepthAndRGB_30 || m_dataMode == PsDepthAndIR_30 || m_dataMode == PsDepthAndIRAndRGB_30 || m_dataMode == PsDepthAndIR_15_RGB_30))
        {
            PsGetFrame(m_deviceIndex, PsDepthFrame, &depthFrame);

            if (depthFrame.pFrameData != NULL)
            {
                //Generate and display PointCloud
                PointCloudCreatorXYZ(depthFrame.height, depthFrame.width, imageMatrix, depthFrame.pFrameData, depthCameraParameters);
            }
        }

        //Get WDR depth frame(fusion or alternatively, determined by PsSetWDRStyle, default in fusion)
        //WDR depth frame only output in PsWDR_Depth data mode
        if (m_dataMode == PsWDR_Depth && !m_pointCloudMappedRGB)
        {
            PsGetFrame(m_deviceIndex, PsWDRDepthFrame, &wdrDepthFrame);
            if (wdrDepthFrame.pFrameData != NULL)
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
                PointCloudCreatorXYZRGB(mappedRGBFrame.height, mappedRGBFrame.width, imageMatrixRGB, imageMatrix, mappedRGBFrame.pFrameData, wdrDepthFrame.pFrameData, depthCameraParameters);
            }
        }
    }

    status = PsCloseDevice(m_deviceIndex);
    info("CloseDevice status: ", status);

    status = PsShutdown();
    info("Shutdown status: ", status);
    cv::destroyAllWindows();
}

void PicoZenseHandler::GetCameraParameters()
{
    PsCameraParameters cameraParameters;
    PsReturnStatus status = PsGetCameraParameters(m_deviceIndex, PsDepthSensor, &cameraParameters);
    info("Get PsGetCameraParameters PsDepthSensor status: ", PsStatusToString(status));
    info("Depth Camera Intinsic:\n", "Fx: ", std::to_string(cameraParameters.fx), "\n",
         "Cx: ", std::to_string(cameraParameters.cx), "\n",
         "Fy: ", std::to_string(cameraParameters.fy), "\n",
         "Cy: ", std::to_string(cameraParameters.cy), "\n",
         "Depth Distortion Coefficient: \n",
         "K1: ", std::to_string(cameraParameters.k1), "\n",
         "K2: ", std::to_string(cameraParameters.k2), "\n",
         "P1: ", std::to_string(cameraParameters.p1), "\n",
         "P2: ", std::to_string(cameraParameters.p2), "\n",
         "K3: ", std::to_string(cameraParameters.k3), "\n",
         "K4: ", std::to_string(cameraParameters.k4), "\n",
         "K5: ", std::to_string(cameraParameters.k5), "\n",
         "K6: ", std::to_string(cameraParameters.k6));

    status = PsGetCameraParameters(m_deviceIndex, PsRgbSensor, &cameraParameters);
    info("Get PsGetCameraParameters PsRgbSensor status: ", PsStatusToString(status));
    info("RGB Camera Intinsic: \n",
         "Fx: ", std::to_string(cameraParameters.fx), "\n",
         "Cx: ", std::to_string(cameraParameters.cx), "\n",
         "Fy: ", std::to_string(cameraParameters.fy), "\n",
         "Cy: ", std::to_string(cameraParameters.cy), "\n",
         "RGB Distortion Coefficient: \n",
         "K1: ", std::to_string(cameraParameters.k1), "\n",
         "K2: ", std::to_string(cameraParameters.k2), "\n",
         "P1: ", std::to_string(cameraParameters.p1), "\n",
         "P2: ", std::to_string(cameraParameters.p2), "\n",
         "K3: ", std::to_string(cameraParameters.k3), "\n");

    PsCameraExtrinsicParameters CameraExtrinsicParameters;
    status = PsGetCameraExtrinsicParameters(m_deviceIndex, &CameraExtrinsicParameters);

    info("Get PsGetCameraExtrinsicParameters status: ", PsStatusToString(status));
    info("Camera rotation: \n",
         std::to_string(CameraExtrinsicParameters.rotation[0]), " ",
         std::to_string(CameraExtrinsicParameters.rotation[4]), " ",
         std::to_string(CameraExtrinsicParameters.rotation[5]), " ",
         std::to_string(CameraExtrinsicParameters.rotation[6]), " ",
         std::to_string(CameraExtrinsicParameters.rotation[7]), " ",
         std::to_string(CameraExtrinsicParameters.rotation[8]), " ",
         "Camera transfer: \n",
         std::to_string(CameraExtrinsicParameters.translation[0]), " ",
         std::to_string(CameraExtrinsicParameters.translation[1]), " ",
         std::to_string(CameraExtrinsicParameters.translation[2]));
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
    PsReturnStatus status = PsSetDepthRange(m_devIndex, depthRange);
    if (status != PsRetOK)
        error("PsSetDepthRange failed with error ", PsStatusToString(status));
    else
    {
        info("SetDepthRange done");
        m_depthRange = depthRange;
    }
}

PsReturnStatus PicoZenseHandler::SetColoPixelFormat(PsPixelFormat pixelFormat)
{
    PsReturnStatus status = PsSetColorPixelFormat(m_devIndex, pixelFormat);
    if (status != PsRetOK)
        error("PsSetColorPixelFormat failed with error ", PsStatusToString(status));
    else
        info("SetColorPixelFormat done");
}

PsReturnStatus PicoZenseHandler::SetDataMode(PsDataMode dataMode)
{
    PsReturnStatus status = PsSetDataMode(m_devIndex, dataMode);
    if (status != PsRetOK)
        error("PsSetDataMode failed with error ", PsStatusToString(status));
    else
    {
        info("PsSetDataMode done");
        m_dataMode = dataMode;
    }
}

PsReturnStatus PicoZenseHandler::SetThreshold(uint16_t threshold)
{
    PsReturnStatus status = PsSetThreshold(m_devIndex, threshold);
    if (status != PsRetOK)
        error("PsSetThreshold failed with error ", PsStatusToString(status));
    else
        info("PsSetThreshold done");
}

PsReturnStatus PicoZenseHandler::SetFIlter(PsFilterType filterType, bool enable)
{
    PsReturnStatus status = PsSetFilter(m_devIndex, filterType, enable);
    if (status != PsRetOK)
        error("PsSetFilter failed with error ", PsStatusToString(status));
    else
        info("PsSetFilter done");
}

PsReturnStatus PicoZenseHandler::SetDepthDistortionCorrectionEnabled(bool enable)
{
    PsReturnStatus status = PsSetDepthDistortionCorrectionEnabled(m_devIndex, enable);
    if (status != PsRetOK)
        error("PsSetDepthDistortionCorrectionEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetDepthDistortionCorrectionEnabled done");
}

PsReturnStatus PicoZenseHandler::SetRGBDistortionCorrectionEnabled(bool enable)
{
    PsReturnStatus status = PsSetRGBDistortionCorrectionEnabled(m_devIndex, enable);
    if (status != PsRetOK)
        error("PsSetRGBDistortionCorrectionEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetRGBDistortionCorrectionEnabled done");
}

PsReturnStatus PicoZenseHandler::SetComputeRealDepthCorrectionEnabled(bool enable)
{
    PsReturnStatus status = PsSetComputeRealDepthCorrectionEnabled(m_devIndex, enable);
    if (status != PsRetOK)
        error("PsSetComputeRealDepthCorrectionEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetComputeRealDepthCorrectionEnabled done");
}

PsReturnStatus PicoZenseHandler::SetSmoothingFilterEnabled(bool enable)
{
    PsReturnStatus status = PsSetSmoothingFilterEnabled(m_devIndex, enable);
    if (status != PsRetOK)
        error("PsSetSmoothingFilterEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetSmoothingFilterEnabled done");
}

PsReturnStatus PicoZenseHandler::SetResolution(PsResolution resolution)
{
    PsReturnStatus status = PsSetResolution(m_devIndex, resolution);
    if (status != PsRetOK)
        error("PsSetResolution failed with error ", PsStatusToString(status));
    else
        info("PsSetResolution done");
}

PsReturnStatus PicoZenseHandler::SetSpatialFilterEnabled(bool enable)
{
    PsReturnStatus status = PsSetSpatialFilterEnabled(m_devIndex, enable);
    if (status != PsRetOK)
        error("PsSetSpatialFilterEnabled failed with error ", PsStatusToString(status));
    else
        info("PsSetSpatialFilterEnabled done");
}

void PicoZenseHandler::SetFeatureDetection(bool enable)
{
    m_detectorNARF = true;
}

void PicoZenseHandler::SetPointCloudClassic()
{
    // Disable RGB Mapped feature first
    PsReturnStatus status;
    if (m_pointCloudMappedRGB)
    {
        status = PsSetMapperEnabledDepthToRGB(m_deviceIndex, false);
        if (status != PsRetOK)
            error("PsSetMapperEnabledDepthToRGB failed with error ", PsStatusToString(status));
        m_pointCloudMappedRGB = false;
    }

    // TODO --> clear and delete the pointCloud classic and create the rgb

    m_pointCloudClassic = true;
}

void PicoZenseHandler::SetPointCloudRGB()
{
    // Disable Classic way
    m_pointCloudClassic = false;

    //Enable the Depth and RGB synchronize feature
    PsReturnStatus status = PsSetSynchronizeEnabled(m_deviceIndex, true);
    if (status != PsRetOK)
        error("PsSetSynchronizeEnabled failed with error ", PsStatusToString(status));
    status = PsSetMapperEnabledDepthToRGB(m_deviceIndex, true);
    if (status != PsRetOK)
        error("PsSetMapperEnabledDepthToRGB failed with error ", PsStatusToString(status));

    // TODO --> clear and delete the pointCloud classic and create the rgb

    m_pointCloudMappedRGB = true;
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
    viewer->registerPointPickingCallback(pointEventHandler, (void *)m_visualizer.get());

    return (viewer);
}

void PicoZenseHandler::PointCloudCreatorXYZRGB(int p_height, int p_width, Mat &p_imageRGB, cv::Mat &p_imageDepth, uint8_t *p_dataRGB, uint8_t *p_dataDepth, PsCameraParameters params)
{
    p_imageDepth = cv::Mat(p_height, p_width, CV_16UC1, p_dataDepth);
    p_imageRGB = cv::Mat(p_height, p_width, CV_8UC3, p_dataRGB);
    
    p_imageDepth.convertTo(p_imageDepth, CV_32F); // convert image data to float type

    if (p_imageDepth.cols != p_imageRGB.cols && p_imageDepth.rows != p_imageRGB.rows)
    {
        error("Images with different sizes!!!");
        return;
    }
    // m_visualizer->removeText3D("p");
    pointCloudRGB->clear();

    //From formula: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    // It has be done a rotation around the z azis
    // Quaternions are a really cool way to express 3D rotation and it doesn't have to use rotation matrix
    // since orientation is expressed by 1 + i + j + k. Using eulero fornula quaternion orientation goes like
    // q = exp[angle/2*(ax*i + ay*j + az*k)] = cos(angle/2) + (ax*i + ay*j + az*k)*sin(angle/2)
    // What will acually happen internally is that every point of the Point cloud acquired by the sensor will be sandwich multipy by q
    // in this following way --> p' = q * p * q-1
    Eigen::Quaternionf q;
    auto angle = M_PI;
    q.x() = 0 * sin(angle / 2);
    q.y() = 0 * sin(angle / 2);
    q.z() = 1 * sin(angle / 2);
    q.w() = 1 * std::cos(angle / 2);

    pointCloudRGB->sensor_orientation_ = q;
    PointXYZRGB pToVisualize;
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

            // Don't know why this now, consider to comment
            p.z = p.z / 1000;
            p.x = p.x / 1000;
            p.y = p.y / 1000;

            if (v == p_imageDepth.rows / 2 && u == p_imageDepth.cols / 2)
            {
                pToVisualize.x = p.x;
                pToVisualize.y = p.y;
                pToVisualize.z = p.z;
            }
            pointCloudRGB->push_back(p);
        }
    }

    pointCloudRGB->width = (uint32_t)pointCloudRGB->points.size();
    pointCloudRGB->height = 1;

    if (!m_visualizer->updatePointCloud(pointCloudRGB, "PointCloudRGB"))
        m_visualizer->addPointCloud(pointCloudRGB, "PointCloudRGB");
    
    // m_visualizer->addText3D(std::to_string(pToVisualize.z), pToVisualize, 0.01, 255.0, 100.0, 50.0, "p");
    m_visualizer->spinOnce();
}

void PicoZenseHandler::PointCloudCreatorXYZ(int p_height, int p_width, Mat &p_image, uint8_t *p_data, PsCameraParameters params)
{
    p_image = cv::Mat(p_height, p_width, CV_16UC1, p_data);
    p_image.convertTo(p_image, CV_32F); // convert image data to float type
    if (!imageMatrix.data)
        error("No depth data");

    //Dimension must be initialized to use 2-D indexing
    // m_visualizer->removeText3D("p");
    pointCloud->clear();

    //From formula: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    // It has be done a rotation around the z azis
    Eigen::Quaternionf q;
    auto angle = M_PI;
    q.x() = 0 * sin(angle / 2);
    q.y() = 0 * sin(angle / 2);
    q.z() = 1 * sin(angle / 2);
    q.w() = 1 * std::cos(angle / 2);
    pointCloud->sensor_orientation_ = q;
    PointXYZ pToVisualize;
    int zeroP = 0;
    int nonZeroP = 0;
    for (int v = 0; v < p_image.rows; v += 1)
    {
        for (int u = 0; u < p_image.cols; u += 1)
        {
            if (p_image.at<float>(v, u) == 0)
            {
                zeroP++;
                continue;
            }
            else
                nonZeroP++;
            PointXYZ p;
            p.z = p_image.at<float>(v, u);
            p.x = (u - (float)params.cx) * p.z / (float)params.fx;
            p.y = (v - (float)params.cy) * p.z / (float)params.fy;

            // Don't know why this now, consider to comment
            p.z = p.z / 1000;
            p.x = p.x / 1000;
            p.y = p.y / 1000;

            if (v == p_image.rows / 2 && u == p_image.cols / 2)
            {
                pToVisualize.x = p.x;
                pToVisualize.y = p.y;
                pToVisualize.z = p.z;
            }
            pointCloud->push_back(p);
        }
    }

    if (!m_visualizer->updatePointCloud(pointCloud, "PointCloud"))
        m_visualizer->addPointCloud(pointCloud, "PointCloud");

    pointCloud->width = (uint32_t)pointCloud->points.size();
    pointCloud->height = 1;

    // Invoke a corner detection method
    if (m_detectorHarris)
        Harris3DCornerDetection();
    else if (m_detectorNARF)
        NARFCorenerDetection();
    else if (m_detectorISS)
        ISSCornerDetection();

    // m_visualizer->addText3D(std::to_string(pToVisualize.z), pToVisualize, 0.01, 255.0, 100.0, 50.0, "p");
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
    debug("1");
    CreateRangeImage();
    // Show range image
    pcl::visualization::RangeImageVisualizer rangeImageWidget("Range Image");
    debug("4");
    rangeImageWidget.showRangeImage(*rangeImage);
    debug("5");
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
    debug("RangeImage ptr address is ", rangeImage);

    Eigen::Affine3f sceneSensorPose = Eigen::Affine3f(Eigen::Translation3f(pCloud.sensor_origin_[0], pCloud.sensor_origin_[1], pCloud.sensor_origin_[2])) * Eigen::Affine3f(pCloud.sensor_orientation_);
    rangeImage->createFromPointCloud(pCloud, pcl::deg2rad(0.5f), pcl::deg2rad(360.0f),
                pcl::deg2rad(180.0f), sceneSensorPose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
    debug("2");
    rangeImage->setUnseenToMaxRange();
}

static void mouseEventHandler(const pcl::visualization::MouseEvent &event, void* pico)
{
    // std::cout << "Mouse: button" << std::to_string(event.getButton()) << " type -> " << event.getType() << std::endl;
}

static void keyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void *pico)
{
    PicoZenseHandler* picoHandler = static_cast<PicoZenseHandler *>(pico);
    std::cout << "Keyboard: key -> " << event.getKeySym() << std::endl;
    if (event.getKeySym() == "p" && event.keyUp())
    {
        picoHandler->GetCameraParameters();
    }
}

static void pointEventHandler(const pcl::visualization::PointPickingEvent &event, void *viewer)
{
    pcl::visualization::PCLVisualizer *v = static_cast<pcl::visualization::PCLVisualizer *>(viewer);
    float x, y, z;
    event.getPoint(x, y, z);
    std::cout << "Point [" << std::to_string(x) << ";" << std::to_string(y) << ";" << std::to_string(z) << "]" << std::endl;
}
