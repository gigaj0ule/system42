// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"

#if defined(USBD_USE_HID_COMPOSITE)
    #include "Keyboard.h"
    #include "Mouse.h"
#endif

//  Illustrates how to register a Bonjour service.

#include <SPI.h>

#include <Ethernet.h>




static void daughter_thread(void* arg) {
      

    while(true) {

        // You can use the "isDiscoveringService()" function to find out whether the
        // Bonjour library is currently discovering service instances.
        // If so, we skip this input, since we want our previous request to continue.
        /*if (!mdns.isDiscoveringService()) {
        
            char serviceName[] = "_http\0";

            SerialUSB.print("\nDiscovering services of type '");
            SerialUSB.print(serviceName);
            SerialUSB.println("' via Bonjour...");

            // Now we tell the mDNS library to discover the service. Below, I have
            // hardcoded the TCP protocol, but you can also specify to discover UDP
            // services.
            // The last argument is a duration (in milliseconds) for which we will
            // search (specify 0 to run the discovery indefinitely).
            // Note that the library will resend the discovery message every 10
            // seconds, so if you search for longer than that, you will receive
            // duplicate instances.

            mdns.startDiscoveringService(serviceName, MDNSServiceTCP, 5000);
        }*/

        // The code below is just taken from the "WebServer" example in the Ethernet
        // library. The only difference here is that this web server gets announced
        // over Bonjour, but this happens in setup(). This just displays something
        // in the browser when you connect.
    }
}

void setup() {

    #if defined(USBD_USE_HID_COMPOSITE)
        Mouse.begin();
        Keyboard.begin();
    #endif
        
    // Init communication
    early_setup();


    pntp_begin("my_computer_name");

    // Launch program!
    create_threads(daughter_thread);
};


void loop() {

    #if defined(USBD_USE_CDC)
    //SerialUSB.print("Hi");
    #endif
};


// This function is called when a name is resolved via mDNS/Bonjour. We set
// this up in the setup() function above. The name you give to this callback
// function does not matter at all, but it must take exactly these arguments
// as below.
// If a service is discovered, name, ipAddr, port and (if available) txtContent
// will be set.
// If your specified discovery timeout is reached, the function will be called
// with name (and all successive arguments) being set to NULL.
/*
void serviceFound(const char* type, MDNSServiceProtocol,
                  const char* name, IPAddress ip,
                  unsigned short port,
                  const char* txtContent)
{
    if (NULL == name) 
    {
        SerialUSB.print("Finished discovering services of type ");
        SerialUSB.println(type);
    } 
    else 
    {
        SerialUSB.print("Found: '");
        SerialUSB.print(name);
        SerialUSB.print("' at ");
        SerialUSB.print(ip);
        SerialUSB.print(":");
        SerialUSB.print(port);
        SerialUSB.println("");

        // Check out http://www.zeroconf.org/Rendezvous/txtrecords.html for a
        // primer on the structure of TXT records. Note that the Bonjour
        // library will always return the txt content as a zero-terminated
        // string, even if the specification does not require this.
        if (txtContent) {
            SerialUSB.print("\ttxt record: ");
            
            char buf[256];
            char len = *txtContent++;
            int i=0;

            while (len) {
                i = 0;
                while (len--)
                buf[i++] = *txtContent++;
                buf[i] = '\0';
                SerialUSB.print(buf);
                len = *txtContent++;
                
                if (len)
                SerialUSB.print(", ");
                else
                SerialUSB.println();
            }
        }
    }
}
*/