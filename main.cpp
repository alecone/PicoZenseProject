#include <iostream>
#include "PicoZenseHandler.h"

typedef void *(*THREADFUNCPTR)(void *);

int main(int argc, char** argv) {
    PicoZenseHandler *pico = new PicoZenseHandler(0);
    pico->init();
    //Create and lunch pthread
    pthread_t thread;
    // TODO! if needed set scheduling parameters and whatever else may be needed.
    int err =  pthread_create(&thread, NULL, (THREADFUNCPTR) &PicoZenseHandler::Visualize, pico);
    if (err)
    {
        error("Thread creation failed: ", strerror(err));
    }

    err = pthread_join(thread, NULL);
    if (err)
        return err;
    delete pico;
    return 0;
}
