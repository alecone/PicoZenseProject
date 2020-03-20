#include "PicoZenseHandler.h"

void worker(boost::barrier &p_barier)
{
    p_barier.wait();
    debug("Ready to stitch");
}