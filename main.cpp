#include <iostream>
#include "PicoZenseHandler.h"

typedef void *(*THREADFUNCPTR)(void *);

static bool stop;

void mainMenu()
{
    info("\n**************************************************");
    info("Menu. Choose an option");
    info("1. Tweak PicoZense camera parameters\n2. Get PicoZense camera parameters");
    info("3. Toggle RGB PointCloud\n4. Toggle 'Classic B&W' PointCloud\n5. Toggle WDR PointCloud");
    info("**************************************************\n");
}
void settersMenu()
{
    info("--------------------------------------------------");
    info("Choose action to perform:\n1. Set Depth Range\n2. Set Color Pixel Format\n3. Set Data Mode");
    info("4. Set Threshold\n5. Set Filter\n6. Set Depth Distortion Correction\n7. Set RGB Distortion Correction");
    info("8. Set Compute Real Depth Correction\n9. Set Smoothing Filter\n10. Set Spatial Feature");
    info("--------------------------------------------------");
}

void * userAction(void *picoZenseHandler)
{
    PicoZenseHandler *pico = static_cast<PicoZenseHandler *>(picoZenseHandler);
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
                    status = pico->SetDepthRange((PsDepthRange) choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 2:
                // Prompting user to choose Pixel Format
                info("0. DepthMM16\n1. Gray16\n2. Gray8\n3. RGB888\n4. BGR888");
                std::cin >> choice;
                if (choice > -1 && choice < 5)
                {
                    status = pico->SetColoPixelFormat((PsPixelFormat) choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 3:
                // Prompting user to choose data mode
                info("0. DepthAndRGB_30\n7. DepthAndIRAndRGB_30\n11. WDR_Depth");
                std::cin >> choice;
                if (choice == 0 || choice == 7 || choice == 11)
                {
                    status = pico->SetDataMode((PsDataMode)choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 4:
                // Prompting user to set threshold
                info("Insert 0 < value < 100. Represents threshold confidence.");
                std::cin >> choice;
                if (choice > -1 && choice < 101)
                {
                    status = pico->SetThreshold((uint16_t)choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 5:
                // Prompting user to set filter
                info("1. Enable ComputeRealDepthFilter\n2. Disable ComputeRealDepthFilter\n3. Enable SmoothingFilte\n4. Disable SmoothingFilter");
                std::cin >> choice;
                if (choice > 0 && choice < 5)
                {
                    bool enable = choice % 2;
                    PsFilterType type = (PsFilterType)(choice / 2);
                    status = pico->SetFIlter(type, enable);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 6:
                // Prompting user to depth distortion correction
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = pico->SetDepthDistortionCorrectionEnabled((bool)choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 7:
                // Prompting user to rgb distortion correction
                info("0. Disable\n 1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = pico->SetRGBDistortionCorrectionEnabled((bool)choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 8:
                // Prompting user to Compute Real Depth Correction
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = pico->SetComputeRealDepthCorrectionEnabled((bool)choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 9:
                // Prompting user to set smoothing filter
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = pico->SetSmoothingFilterEnabled((bool)choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;
            case 10:
                // Prompting user to set spatial feature
                info("0. Disable\n1. Enable");
                std::cin >> choice;
                if (choice == 0 || choice == 1)
                {
                    status = pico->SetSpatialFilterEnabled((bool)choice);
                }
                std::cin.clear();
                std::cin.ignore(1024,'\n');
                break;            
            default:
                break;
            }
            break;
        case 2:
            pico->GetImu();
            pico->GetCameraParameters();
            break;
        case 3:
            debug("Setting PointCloud RGB/Depth mapped");
            pico->SetPointCloudRGB();
            break;
        case 4:
            debug("Setting PointCloud in classic way");
            pico->SetPointCloudClassic();
            break;
        case 5:
            debug("Setting POintCloud with WDR feature");
            pico->SetWDRDataMode();
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
    err = pthread_create(&menuThread, NULL, userAction, (void *)pico);
    if (err)
        error("Thread creation failed: ", strerror(err));

    err = pthread_join(picoThread, NULL);
    if (err)
        return err;
    else
        stop = true;
    err = pthread_join(menuThread, NULL);
    delete pico;
    return 0;
}
