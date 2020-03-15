#include <iostream>
#include "PicoZenseHandler.h"

typedef void *(*THREADFUNCPTR)(void *);

struct picoHandlers
{
    PicoZenseHandler *pico1;
    PicoZenseHandler *pico2;
};

static bool stop;

void mainMenu()
{
    info("\n**************************************************");
    info("Menu. Choose an option");
    info("1. Tweak PicoZense camera parameters\n2. Get PicoZense camera parameters");
    info("3. Toggle RGB PointCloud\n4. Toggle 'Classic B&W' PointCloud\n5. Toggle WDR PointCloud");
    info("6. Set NARF Features detections\n99. Save PointCloud as pcd file");
    info("**************************************************\n");
}
void settersMenu()
{
    info("--------------------------------------------------");
    info("Choose action to perform:\n1. Set Depth Range\n2. Set Color Pixel Format\n3. Set Data Mode");
    info("4. Set Threshold\n5. Set Filter\n6. Set Depth Distortion Correction\n7. Set RGB Distortion Correction");
    info("8. Set Compute Real Depth Correction\n9. Set Smoothing Filter\n10. Set Spatial Feature");
    info("11. Apply Bilater Filter (on PointCloud)\n12. Apply Bilateral Upsampling\n13. Set Normalized Box Filter (OpneCV)");
    info("14. Set Gaussian Filter (OpneCV)\n15. Set Bilater Filter (OpneCV)\n16. Set StatisticalOutlierRemoval FIlter (OpneCV)");
    info("17. Set RadiusOutlier Filter (OpneCV)");
    info("--------------------------------------------------");
}

void *userAction(void *picoZenseHandlers)
{
    struct picoHandlers *picos = (struct picoHandlers *)picoZenseHandlers;
    if (picos->pico1 == NULL && picos->pico2 == NULL)
    {
        error("PicoZenseHandlers are null, Stop it!");
        return NULL;
    }
    debug("PicoZense 1: ", picos->pico1);
    debug("PicoZense 2: ", picos->pico2);

    int choice;
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
            debug("Setting PointCloud RGB/Depth mapped");
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
        case 99:
            picos->pico1->SavePCD();
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
    debug("Found ", std::to_string(devs), (devs > 1 ? " devices attached, going to run them" : " device attached, going to run it"));
    if (devs < 1)
    {
        error("There are no devices connected, connect it");
        return 0;
    }
    else if (devs > 1)
    {
        // Introduced due to error
        //[xcb] Unknown sequence number while processing queue
        //[xcb] Most likely this is a multi - threaded client and XInitThreads has not been called
        //[xcb] Aborting, sorry about that.
        int status = XInitThreads();
        info("XInitThreads returned ", std::to_string(status));
        PicoZenseHandler *pico1 = new PicoZenseHandler(0);
        pico1->init();
        stop = false;
        //Create and lunch pthread
        pthread_t picoThread1;
        // TODO! if needed set scheduling parameters and whatever else may be needed.
        int err = pthread_create(&picoThread1, NULL, (THREADFUNCPTR)&PicoZenseHandler::Visualize, pico1);
        if (err)
            error("Thread creation failed: ", strerror(err));

        PicoZenseHandler *pico2 = new PicoZenseHandler(1);
        pico2->init();
        stop = false;
        //Create and lunch pthread
        pthread_t picoThread2;
        // TODO! if needed set scheduling parameters and whatever else may be needed.
        err = pthread_create(&picoThread2, NULL, (THREADFUNCPTR)&PicoZenseHandler::Visualize, pico2);
        if (err)
            error("Thread creation failed: ", strerror(err));
        
        pthread_t menuThread;
        struct picoHandlers picos;
        picos.pico1 = pico1;
        picos.pico2 = pico2;
        err = pthread_create(&menuThread, NULL, userAction, (void *)&picos);
        if (err)
            error("Thread creation failed: ", strerror(err));

        err = pthread_join(picoThread1, NULL);
        err = pthread_join(picoThread2, NULL);
        if (err)
            return err;
        else
            stop = true;
        err = pthread_join(menuThread, NULL);
        delete pico1;
    }
    else
    {
        PicoZenseHandler *pico = new PicoZenseHandler(0);
        pico->init();
        stop = false;
        //Create and lunch pthread
        pthread_t picoThread;
        // TODO! if needed set scheduling parameters and whatever else may be needed.
        int err = pthread_create(&picoThread, NULL, (THREADFUNCPTR)&PicoZenseHandler::Visualize, pico);
        if (err)
            error("Thread creation failed: ", strerror(err));
        
        pthread_t menuThread;
        struct picoHandlers picos;
        picos.pico1 = pico;
        picos.pico2 = nullptr;
        err = pthread_create(&menuThread, NULL, userAction, (void *)&picos);
        if (err)
            error("Thread creation failed: ", strerror(err));

        err = pthread_join(picoThread, NULL);
        if (err)
            return err;
        else
            stop = true;
        err = pthread_join(menuThread, NULL);
        delete pico;
    }
    return 0;
}
