// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"

#if defined(USBD_USE_HID_COMPOSITE)
    #include "Keyboard.h"
    #include "Mouse.h"
#endif

static void worker_thread(void* arg) {
    while(true) {
	// Do something every second
        os_delay(1000);
    }
}

void setup() {

    #if defined(USBD_USE_HID_COMPOSITE)
        Mouse.begin();
        Keyboard.begin();
    #endif
        
    // Init communication
    early_setup();
    
    // Launch program!
    create_threads(worker_thread);
};


void loop(){    
    __asm__ volatile("nop");

    #if defined(USBD_USE_CDC)
    //SerialUSB.print("Hi");
    #endif
};
