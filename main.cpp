#include <iostream>
#include "PicoZenseHandler.h"

int main(int argc, char** argv) {
    PicoZenseHandler *pico = new PicoZenseHandler(0);
    pico->Visualize();
    delete pico;
    return 0;
}
