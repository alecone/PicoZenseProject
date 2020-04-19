#include <iostream>
#include "PicoZenseHandler.h"
#include "MultiCamWorker.h"

typedef void *(*THREADFUNCPTR)(void *);

struct instanceHandlers
{
    PicoZenseHandler *pico1;
    PicoZenseHandler *pico2;
    MultiCamWorker *cams;
};

static bool stop;

void mainMenu()
{
    info("\n**************************************************");
    info("Menu. Choose an option");
    info("1. Tweak PicoZense camera parameters\n2. Get PicoZense camera parameters");
    info("3. Toggle RGB PointCloud\n4. Toggle 'Classic B&W' PointCloud\n5. Toggle WDR PointCloud");
    info("6. Set NARF Features detections\n7. Start ICP\n8. Set allign Point Clouds");
    info("9. Set manual trasform");
    info("80. Start test\n90. Save PointCloud as pcd file\n\n99. Shut Down");
    info("**************************************************\n");
}
void settersMenu()
{
    info("--------------------------------------------------");
    info("Choose action to perform:\n1. Set Depth Range\n2. Set Color Pixel Format\n3. Set Data Mode");
    info("4. Set Threshold\n5. Set Filter\n6. Set Depth Distortion Correction\n7. Set RGB Distortion Correction");
    info("8. Set Compute Real Depth Correction\n9. Set Smoothing Filter\n10. Set Spatial Feature");
    info("11. Apply Bilater Filter (PCL)\n12. Apply Bilateral Upsampling\n13. Set Normalized Box Filter (OpneCV)");
    info("14. Set Gaussian Filter (OpneCV)\n15. Set Bilater Filter (OpneCV)\n16. Set StatisticalOutlierRemoval FIlter (OpneCV)");
    info("17. Set RadiusOutlier Filter (OpneCV)\n18. Set Hole Filling (PCL)");
    info("19. Set Fast Triangolation recontraction (PCL)\n20. Set Polynomial recontraction (PCL)");
    info("--------------------------------------------------");
}

void *userAction(void *picoZenseHandlers)
{
    struct instanceHandlers *picos = (struct instanceHandlers *)picoZenseHandlers;
    if (picos->pico1 == NULL && picos->pico2 == NULL)
    {
        error("PicoZenseHandlers are null, Stop it!");
        return NULL;
    }
    debug("PicoZense 1: ", picos->pico1);
    debug("PicoZense 2: ", picos->pico2);
    debug("MultiCamWorker: ", picos->cams);

    int choice;
    std::string testName;
    PsReturnStatus status;
    while (!stop)
    {
        mainMenu();
        std::cin >> choice;
        switch (choice)
        {
        case 1:
            std::cin.clear();
            std::cin.ignore(1024,'\n');
            settersMenu();
            std::cin >> choice;
            switch (choice)
            {
            case 1:
                // Prompting user to choose depth
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                info("0. Near\n1. Mid\n2. Far\n3. XNear\n4. XMid\n5. XFar\n6. XXNear\n7. XXMid\n8. XXFar");
                std::cin >> choice;
                if (choice > -1 && choice < 9)
                {
                    status = picos->pico1->SetDepthRange((PsDepthRange) choice);
                    if(picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetDepthRange((PsDepthRange)choice);
                    }
                }
                break;
            case 2:
                // Prompting user to choose Pixel Format
                info("0. DepthMM16\n1. Gray16\n2. Gray8\n3. RGB888\n4. BGR888");
                std::cin >> choice;
                if (choice > -1 && choice < 5)
                {
                    status = picos->pico1->SetColoPixelFormat((PsPixelFormat) choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetColoPixelFormat((PsPixelFormat)choice);
                    }
                }
                break;
            case 3:
                // Prompting user to choose data mode
                info("0. DepthAndRGB_30\n7. DepthAndIRAndRGB_30\n11. WDR_Depth");
                std::cin >> choice;
                if (choice == 0 || choice == 7 || choice == 11)
                {
                    status = picos->pico1->SetDataMode((PsDataMode)choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetDataMode((PsDataMode)choice);
                    }
                }
                break;
            case 4:
                // Prompting user to set threshold
                info("Insert 0 < value < 100. Represents threshold confidence.");
                std::cin >> choice;
                if (choice > -1 && choice < 101)
                {
                    status = picos->pico1->SetThreshold((uint16_t)choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetThreshold((uint16_t)choice);
                    }
                }
                break;
            case 5:
                // Prompting user to set filter
                info("1. Enable ComputeRealDepthFilter\n2. Disable ComputeRealDepthFilter\n3. Enable SmoothingFilte\n4. Disable SmoothingFilter");
                std::cin >> choice;
                if (choice > 0 && choice < 5)
                {
                    bool enable = choice % 2;
                    PsFilterType type = (PsFilterType)(choice / 2);
                    status = picos->pico1->SetFIlter(type, enable);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetFIlter(type, enable);
                    }
                }
                break;
            case 6:
                // Prompting user to depth distortion correction
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = picos->pico1->SetDepthDistortionCorrectionEnabled((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetDepthDistortionCorrectionEnabled((bool)choice);
                    }
                }
                break;
            case 7:
                // Prompting user to rgb distortion correction
                info("0. Disable\n 1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = picos->pico1->SetRGBDistortionCorrectionEnabled((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetRGBDistortionCorrectionEnabled((bool)choice);
                    }
                }
                break;
            case 8:
                // Prompting user to Compute Real Depth Correction
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = picos->pico1->SetComputeRealDepthCorrectionEnabled((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetComputeRealDepthCorrectionEnabled((bool)choice);
                    }
                }
                break;
            case 9:
                // Prompting user to set smoothing filter
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = picos->pico1->SetSmoothingFilterEnabled((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetSmoothingFilterEnabled((bool)choice);
                    }
                }
                break;
            case 10:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = picos->pico1->SetSpatialFilterEnabled((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        status = picos->pico2->SetSpatialFilterEnabled((bool)choice);
                    }
                }
                break;
            case 11:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetBilateralNoiseFilter((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetBilateralNoiseFilter((bool)choice);
                    }
                }
                break;
            case 12:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetBilateralUpsampling((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetBilateralUpsampling((bool)choice);
                    }
                }
                break;
            case 13:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetNormalizedBoxFilter((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetNormalizedBoxFilter((bool)choice);
                    }
                }
                break;
            case 14:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetGaussinFilter((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetGaussinFilter((bool)choice);
                    }
                }
                break;
            case 15:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetBilateralFilter((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetBilateralFilter((bool)choice);
                    }
                }
                break;
            case 16:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetStatisticalOutlierRemoval((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetStatisticalOutlierRemoval((bool)choice);
                    }
                }
                break;
            case 17:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetRadialOutlierRemoval((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetRadialOutlierRemoval((bool)choice);
                    }
                }
                break;
            case 18:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetMLSUpsampling((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetMLSUpsampling((bool)choice);
                    }
                }
                break;
            case 19:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetFastTriangolation((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetFastTriangolation((bool)choice);
                    }
                }
                break;
            case 20:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    picos->pico1->SetPolynomialReconstruction((bool)choice);
                    if (picos->pico2 != NULL)
                    {
                        picos->pico2->SetPolynomialReconstruction((bool)choice);
                    }
                }
                break;
            default:
                break;
            }
            break;
        case 2:
            picos->pico1->GetImu();
            picos->pico1->GetCameraParameters();
            if (picos->pico2 != NULL)
            {
                picos->pico2->GetImu();
                picos->pico2->GetCameraParameters();
            }
            break;
        case 3:
            picos->pico1->SetPointCloudRGB();
            if (picos->pico2 != NULL)
            {
                picos->pico2->SetPointCloudRGB();
            }
            break;
        case 4:
            debug("Setting PointCloud in classic way");
            picos->pico1->SetPointCloudClassic();
            if (picos->pico2 != NULL)
            {
                picos->pico2->SetPointCloudClassic();
            }
            break;
        case 5:
            debug("Setting POintCloud with WDR feature");
            picos->pico1->SetWDRDataMode();
            if (picos->pico2 != NULL)
            {
                picos->pico2->SetWDRDataMode();
            }
            break;
        case 6:
            // Prompting user to set NAFT feature detection
            std::cin.clear();
            std::cin.ignore(1024, '\n');
            info("0. Disable\n1. Enable");
            std::cin >> choice;
            if (choice == 0 || choice == 1)
            {
                picos->pico1->SetFeatureDetection((bool)choice);
                if (picos->pico2 != NULL)
                {
                    picos->pico2->SetFeatureDetection((bool)choice);
                }
            }
            break;
        case 7:
            if (picos->cams != nullptr)
            {
                picos->cams->startComputeTransform();
            }
            break;
        case 8:
            if (picos->cams != nullptr)
            {
                std::cin.clear();
                std::cin.ignore(1024, '\n');
                info("0. Disallign\n1. Allign");
                std::cin >> choice;
                if (choice == 1)
                {
                    //Try to get the transform
                    Eigen::Matrix4f T;
                    bool readyToTransform = picos->cams->getTransform(T);
                    if (readyToTransform)
                    {
                        //Since the transform if from Cam#2 to Cam#1 set the transform properly
                        picos->pico2->SetTransform(T);
                    }
                    else
                        debug("Transform not conputed yet");
                }
                else if (choice == 0)
                {
                    picos->pico2->UnSetTranform();
                }
            }
            else
                warn("Nothing to do, MultiCamWorker does not exist!");
            break;
        case 9:
            if (picos->cams != nullptr)
            {
                // std::cin.clear();
                // std::cin.ignore(1024, '\n');
                // info("Set x traslation");
                // std::cin >> choice;
                //Try to get the transform
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                // From stereo fucking calibration
                T(0, 0) = 0.9994620161631419;
                T(0, 1) = -0.004643873026125718;
                T(0, 2) = -0.03246710166344729;
                T(1, 0) = 0.004781215055008394;
                T(1, 1) = 0.9999799426209818;
                T(0, 2) = 0.004153834172878227;
                T(2, 0) = 0.03244716058001317;
                T(2, 1) = -0.004306831672497989;
                T(2, 2) = 0.9994641729302955;
                T(0, 3) = 0.22129716386296153;
                T(1, 3) = 0.002012280755594414;
                T(2, 3) = 0.0026842127356713793;
                picos->pico2->SetTransform(T);
            }
            else
                warn("Nothing to do, MultiCamWorker does not exist!");
            break;
        case 80:
            std::cin.clear();
            std::cin.ignore(1024, '\n');
            info("Insert Test Name");
            std::cin >> testName;
            picos->pico1->SetTestName(testName);
            picos->pico1->PerformTest();
            if (picos->pico2 != NULL)
            {
                picos->pico2->SetTestName(testName);
                picos->pico2->PerformTest();
            }
            break;

        case 90:
            picos->pico1->SavePCD();
            if (picos->pico2 != NULL)
                picos->pico2->SavePCD();
            break;
        case 99:
            if (picos->pico1 != nullptr)
                picos->pico1->ShutDown();
            if (picos->pico2 != nullptr)
                picos->pico2->ShutDown();
            break;

        default:
            warn("Invalid option!", std::to_string(choice));
            break;
        }
        std::cin.clear();
        std::cin.ignore(1024,'\n');
    }
    info("Bye Bye");
    return NULL;
}

int main(int argc, char** argv) {
    int32_t devs;

    PsGetDeviceCount(&devs);
    while (devs < 1)
    {
        warn("No devices attached...Please start attaching the RIGHT device.");
        sleep(5);
        PsGetDeviceCount(&devs);
    }
    PsReturnStatus status = PsInitialize();
    if (status != PsReturnStatus::PsRetOK)
    {
        error("PsInitialize failed!");
        exit(1);
    }

    debug("Found ", std::to_string(devs), (devs > 1 ? " devices attached, going to run them" : " device attached, going to run it"));

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    boost::barrier myBarrier(devs+1);
    if (devs < 1)
    {
        error("There are no devices connected, connect it");
        return 1;
    }
    else if (devs > 1)
    {
        stop = false;
        PicoZenseHandler *pico1 = new PicoZenseHandler(0, viewer);
        PicoZenseHandler *pico2 = new PicoZenseHandler(1, viewer);
        pico1->init();
        pico2->init();
        struct instanceHandlers picos;
        picos.pico1 = pico1;
        picos.pico2 = pico2;
        MultiCamWorker *cams = new MultiCamWorker((void *)&picos, viewer);
        picos.cams = cams;
        boost::thread picoThread1(boost::bind(&PicoZenseHandler::Visualize, pico1, boost::ref(myBarrier)));
        boost::thread picoThread2(boost::bind(&PicoZenseHandler::Visualize, pico2, boost::ref(myBarrier)));
        boost::thread camsThread(boost::bind(&MultiCamWorker::worker, cams, boost::ref(myBarrier)));

        boost::thread menuThread(boost::bind(userAction, (void *)&picos));

        picoThread1.join();
        picoThread2.join();
        info("Shutdown status: ", PsShutdown());
        stop = true;
        menuThread.join();
        delete pico1;
        delete pico2;
        cams->ShutDown();
    }
    else
    {
        PicoZenseHandler *pico = new PicoZenseHandler(0, viewer);
        pico->init();
        stop = false;
        //Create and lunch threads
        boost::thread picoThread(boost::bind(&PicoZenseHandler::Visualize, pico, boost::ref(myBarrier)));

        struct instanceHandlers picos;
        picos.pico1 = pico;
        picos.pico2 = nullptr;
        picos.cams = nullptr;
        boost::thread menuThread(boost::bind(userAction, (void *)&picos));

        picoThread.join();
        info("Shutdown status: ", PsShutdown());
        stop = true;
        menuThread.join();
        delete pico;
    }
    return 0;
}
