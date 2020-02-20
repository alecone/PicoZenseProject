#include <iostream>
#include "PicoZenseHandler.h"

typedef void *(*THREADFUNCPTR)(void *);

static bool stop;

void mainMenu()
{
    info("Menu. Choose an option");
    info("1. Tweak PicoZense camera parameters\n2. Get PicoZense camera parameters");
    info("3. Toggle RGB PointCloud\n4. Toggle 'Classic B&W' PointCloud\n5. Toggle WDR PointCloud");
}
void settersMenu()
{
    info("Choose action to perform:\n1. Set Depth Range\n2. Set Color Pixel Format\n3. Set Data Mode\n");
    info("4. Set Threshold\n5. Set Filter\n6. Set Depth Distortion Correction\n7. Set RGB Distortion Correction");
    info("8. Set Compute Real Depth Correction\n9. Set Smoothin Filter\n10. Set Spatial Feature");
}

void getterMenu()
{
    info("Choose action to perform:\n1. Get Imu\n2. Get Camera parameters");
}

void * userAction(void *picoZenseHandler)
{
    PicoZenseHandler *pico = static_cast<PicoZenseHandler *>(picoZenseHandler);
    int choice;
    while (!stop)
    {
        mainMenu();
        std::cin >> choice;
        switch (choice)
        {
        case 1:
            settersMenu();
            // TODO! All the rest 
            break;
        case 2:
            getterMenu();
            // TODO! All teh rest
            break;
        case 3:
            info("Setting PointCloud RGB/Depth mapped");
            pico->SetPointCloudRGB();
            break;
        case 4:
            info("Setting PointCloud in classic way");
            pico->SetPointCloudClassic();
            break;
        case 5:
            info("Setting POintCloud with WDR feature");
            pico->SetWDRDataMode();
            break;

        default:
            warn("Invalid option!", std::to_string(choice));
            break;
        }
        std::cin.clear();
        std::cin.ignore();
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
