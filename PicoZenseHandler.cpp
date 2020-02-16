#include "PicoZenseHandler.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace Eigen;

PicoZenseHandler::PicoZenseHandler(/* args */)
{
    debug("PicoZense object created");
    m_visualizer = new pcl::visualization::PCLVisualizer("PointCloud Viewrer");
    m_visualizer->setBackgroundColor(0.0, 0.0, 0.0);
    // m_visualizer->registerKeyboardCallback(&PicoZenseHandler::keyboardEventHandler, *m_visualizer);
    pointCloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
    pointCloudRGB = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>());
    rangeImage = pcl::RangeImage::Ptr(new pcl::RangeImage());
    debug("RangeImage ptr address is ", rangeImage);

    m_devIndex = 0;
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
        delete m_visualizer;
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
    int32_t deviceIndex = 0;
    int32_t deviceCount = 0;
    uint32_t slope = 1450;
    uint32_t wdrSlope = 4400;
    PsDepthRange depthRange = PsNearRange;
    int32_t dataMode = PsWDR_Depth;
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
        std::cout << "PsGetDeviceCount failed!\n";
        system("pause");
        return;
    }
    std::cout << "Get device count: " << std::to_string(deviceCount) << std::endl;

    //Set the Depth Range to Near through PsSetDepthRange interface
    status = PsSetDepthRange(deviceIndex, depthRange);
    if (status != PsReturnStatus::PsRetOK)
        std::cout << "PsSetDepthRange failed!\n";
    else
        std::cout << "Set Depth Range to Near\n";

    status = PsOpenDevice(deviceIndex);
    if (status != PsReturnStatus::PsRetOK)
    {
        std::cout << "OpenDevice failed!\n";
        system("pause");
        return;
    }

    //Set PixelFormat as PsPixelFormatBGR888 for opencv display
    PsSetColorPixelFormat(deviceIndex, PsPixelFormatRGB888);

    //Set to data mode
    status = PsSetDataMode(deviceIndex, (PsDataMode)dataMode);
    if (status != PsReturnStatus::PsRetOK)
    {
        std::cout << "Set DataMode Failed failed!\n";
    }

    //WDR STUFF
    //Set WDR Output Mode, three ranges Near/Middle/Far output from device every one frame
    // PsWDROutputMode wdrMode = { PsWDRTotalRange_Three, PsNearRange, 1, PsMidRange, 1, PsNearRange, 1 };
    //Set WDR fusion threshold
    // PsSetWDRFusionThreshold(deviceIndex, 1000, 2000);
    // PsSetWDROutputMode(deviceIndex, &wdrMode);
    // PsSetWDRStyle(deviceIndex, PsWDR_ALTERNATION);

    //Enable the Depth and RGB synchronize feature
    PsSetSynchronizeEnabled(deviceIndex, true);
    status = PsSetMapperEnabledDepthToRGB(deviceIndex, true);

    PsCameraParameters cameraParameters;
    status = PsGetCameraParameters(deviceIndex, PsDepthSensor, &cameraParameters);
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

    status = PsGetCameraParameters(deviceIndex, PsRgbSensor, &cameraParameters);
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
    status = PsGetCameraExtrinsicParameters(deviceIndex, &CameraExtrinsicParameters);

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

    cv::Mat imageMat;
    const string irImageWindow = "IR Image";
    const string rgbImageWindow = "RGB Image";
    const string depthImageWindow = "Depth Image";
    const string mappedDepthImageWindow = "MappedDepth Image";
    const string mappedRgbImageWindow = "MappedRGB Image";
    const string mappedIRWindow = "MappedIR Image";
    const string wdrDepthImageWindow = "WDR Depth Image";

    ofstream PointCloudWriter;

    bool f_bDistortionCorrection = false;
    bool f_bFilter = false;
    bool f_bMappedRGB = true;
    bool f_bMappedIR = true;
    bool f_bMappedDepth = true;
    bool f_bWDRMode = false;
    bool f_bInvalidDepth2Zero = false;
    bool f_bDustFilter = false;
    bool f_bSync = true;

    for (;;)
    {
        PsFrame depthFrame = {0};
        PsFrame wdrDepthFrame = {0};
        PsFrame mappedRGBFrame = {0};

        // Read one frame before call PsGetFrame
        status = PsReadNextFrame(deviceIndex);
        if (status != PsRetOK)
            warn("PsReadNextFrame gave ", PsStatusToString(status));

        //Get depth frame, depth frame only output in following data mode
        if (dataMode == PsDepthAndRGB_30 || dataMode == PsDepthAndIR_30 || dataMode == PsDepthAndIRAndRGB_30 || dataMode == PsDepthAndIR_15_RGB_30)
        {
            PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);

            if (depthFrame.pFrameData != NULL)
            {
                //Display the Depth Image

                //Generate and display PointCloud
                // PointCloudCreatorXYZ(depthFrame.height, depthFrame.width, imageMatrix, depthFrame.pFrameData, cameraParameters);
            }
        }

        //Get WDR depth frame(fusion or alternatively, determined by PsSetWDRStyle, default in fusion)
        //WDR depth frame only output in PsWDR_Depth data mode
        if (dataMode == PsWDR_Depth)
        {
            PsGetFrame(deviceIndex, PsWDRDepthFrame, &wdrDepthFrame);
            if (wdrDepthFrame.pFrameData != NULL)
            {
                //Display the WDR Depth Image
                // PointCloudCreatorXYZ(wdrDepthFrame.height, wdrDepthFrame.width, imageMatrix, wdrDepthFrame.pFrameData, cameraParameters);
            }
        }

        //Get mapped rgb frame which is mapped to depth camera space
        //Mapped rgb frame only output in following data mode
        //And can only get when the feature is enabled through api PsSetMapperEnabledDepthToRGB
        if (dataMode == PsDepthAndRGB_30 || dataMode == PsDepthAndIRAndRGB_30 || dataMode == PsWDR_Depth || dataMode == PsDepthAndIR_15_RGB_30)
        {
            PsGetFrame(deviceIndex, PsMappedRGBFrame, &mappedRGBFrame);

            if (mappedRGBFrame.pFrameData != NULL)
            {
                PointCloudCreatorXYZRGB(mappedRGBFrame.height, mappedRGBFrame.width, imageMatrixRGB, imageMatrix, mappedRGBFrame.pFrameData, wdrDepthFrame.pFrameData, cameraParameters);
            }
        }
    }

    status = PsCloseDevice(deviceIndex);
    info("CloseDevice status: ", status);

    status = PsShutdown();
    info("Shutdown status: ", status);
    cv::destroyAllWindows();
}

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

int PicoZenseHandler::SavePCD(const std::string &filename)
{
    pcl::PCDWriter w;
    debug("Going to write");
    int ret = w.write(filename, *pointCloud);
    return ret;
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
    m_visualizer->removeText3D("p");
    m_visualizer->removePointCloud("PointCloud");
    pointCloudRGB->clear();

    pointCloudRGB->width = p_imageDepth.cols;
    pointCloudRGB->height = p_imageDepth.rows;
    pointCloudRGB->resize(pointCloudRGB->width * pointCloudRGB->height);

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

    m_visualizer->addPointCloud(pointCloudRGB, "PointCloud");
    m_visualizer->addText3D(std::to_string(pToVisualize.z), pToVisualize, 0.01, 255.0, 100.0, 50.0, "p");
    m_visualizer->spinOnce();
}

void PicoZenseHandler::PointCloudCreatorXYZ(int p_height, int p_width, Mat &p_image, uint8_t *p_data, PsCameraParameters params)
{
    p_image = cv::Mat(p_height, p_width, CV_16UC1, p_data);
    p_image.convertTo(p_image, CV_32F); // convert image data to float type
    if (!imageMatrix.data)
        error("No depth data");

    //Dimension must be initialized to use 2-D indexing
    m_visualizer->removeText3D("p");
    m_visualizer->removeAllPointClouds();
    pointCloud->clear();


    // pointCloud->width = p_image.cols;
    // pointCloud->height = p_image.rows;
    // pointCloud->resize(pointCloud->width * pointCloud->height);

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
    for (int v = 0; v < p_image.rows; v += 4)
    {
        for (int u = 0; u < p_image.cols; u += 4)
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

    m_visualizer->addPointCloud(pointCloud, "PointCloud");

    // ISSCornerDetection();

    m_visualizer->addText3D(std::to_string(pToVisualize.z), pToVisualize, 0.01, 255.0, 100.0, 50.0, "p");
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

// void PicoZenseHandler::mouseEventHandler(const pcl::visualization::MouseEvent &event, void *viewer_void)
// {
//     if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
//         event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
//     {
//         std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

//         char str[512];
//         // sprintf(str, "text#%03d", text_id++);
//         // viewer->addText("clicked here", event.getX(), event.getY(), str);
//     }
// }

// void PicoZenseHandler::keyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
// {
//     if (event.getKeySym() == "r" && event.keyDown())
//     {
//         std::cout << "r was pressed => removing all text" << std::endl;

//         char str[512];
//         // for (unsigned int i = 0; i < text_id; ++i)
//         // {
//         //     sprintf(str, "text#%03d", i);
//         //     viewer->removeShape(str);
//         // }
//         // text_id = 0;
//     }
// }