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
#include <EthernetUdp.h>
#include <MDNS_Generic.h>
//#include <ArduinoMDNS.h>

extern SPIClass SPI;

EthernetUDP udp;
MDNS mdns(udp);

void serviceFound(const char* type, MDNSServiceProtocol proto,
                  const char* name, IPAddress ip, unsigned short port,
                  const char* txtContent);


// you can find this written on the board of some Arduino Ethernets or shields
//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x0E, 0x9E };

// NOTE: Alternatively, you can assign a fixed IP to configure your
//       Ethernet shield.
//       byte ip[] = { 192, 168, 0, 154 };

EthernetServer server(80);

static void daughter_thread(void* arg) {

    // NOTE: Alternatively, you can assign a fixed IP to configure your
    //       Ethernet shield.
    //       Ethernet.begin(mac, ip);   4
    // 2k buffer
        
    SPI.setMOSI(PB15);
    SPI.setMISO(PB14);
    SPI.setSCLK(PB13);

    //pinMode(PB3, OUTPUT);

    Ethernet.init(PB8);
    //Ethernet.setCsPin(PB8);
    //Ethernet.setRstPin(PB9);
    //Ethernet.setHostname("bonjour");

    // Initialize Ethernet with DHCP

    if (Ethernet.begin(mac) == 0) {
        // Failed to configure Ethernet using DHCP
        __asm__ volatile ("nop");
    }

    /*if (Ethernet.link() == 0) {
      // Ethernet cable is not connected
    }*/
    
    server.begin();

    __asm__ volatile ("nop");

    os_delay(5000);

    SerialUSB.println("Starting MDNS");

    // Initialize the Bonjour/MDNS library. You can now reach or ping this
    // Arduino via the host name "arduino.local", provided that your operating
    // system is Bonjour-enabled (such as MacOS X).
    // Always call this before any other method!
    if(mdns.begin(Ethernet.localIP(), "arduino")) 
    {
        SerialUSB.println("MDNS Begin = 1");
    }
    else 
    {
        SerialUSB.println("MDNS Begin = 0");
    }

    mdns.setServiceFoundCallback(serviceFound);

    __asm__ volatile ("nop");

    // Now let's register the service we're offering (a web service) via Bonjour!
    // To do so, we call the addServiceRecord() method. The first argument is the
    // name of our service instance and its type, separated by a dot. In this
    // case, the service type is _http. There are many other service types, use
    // google to look up some common ones, but you can also invent your own
    // service type, like _mycoolservice - As long as your clients know what to
    // look for, you're good to go.
    // The second argument is the port on which the service is running. This is
    // port 80 here, the standard HTTP port.
    // The last argument is the protocol type of the service, either TCP or UDP.
    // Of course, our service is a TCP service.
    // With the service registered, it will show up in a Bonjour-enabled web
    // browser. As an example, if you are using Apple's Safari, you will now see
    // the service under Bookmarks -> Bonjour (Provided that you have enabled
    // Bonjour in the "Bookmarks" preferences in Safari).
    if(mdns.addServiceRecord("arduino._http", 80, MDNSServiceTCP, "hello\0") == 1) 
    {
        SerialUSB.println("MDNS addServiceRecord = 1");
    }
    else 
    {
        SerialUSB.println("MDNS addServiceRecord = 0");
    }

    __asm__ volatile ("nop");


    while(true) {
        // Whats our IP?
        IPAddress local_ip = Ethernet.localIP();
        sprintf(communicable.local_ip, "%d.%d.%d.%d", local_ip[0], local_ip[1], local_ip[2], local_ip[3]);

        // This actually runs the Bonjour module. YOU HAVE TO CALL THIS PERIODICALLY,
        // OR NOTHING WILL WORK! Preferably, call it once per loop().
        mdns.run();

        os_delay(1);

        // You can use the "isDiscoveringService()" function to find out whether the
        // Bonjour library is currently discovering service instances.
        // If so, we skip this input, since we want our previous request to continue.
        if (!mdns.isDiscoveringService()) {
        
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
        }


        // The code below is just taken from the "WebServer" example in the Ethernet
        // library. The only difference here is that this web server gets announced
        // over Bonjour, but this happens in setup(). This just displays something
        // in the browser when you connect.
        EthernetClient client = server.available();

        if (client) {

            // an http request ends with a blank line
            boolean current_line_is_blank = true;

            while (client.connected()) {
                if (client.available()) {
                    char c = client.read();
                    // if we've gotten to the end of the line (received a newline
                    // character) and the line is blank, the http request has ended,
                    // so we can send a reply
                    if (c == '\n' && current_line_is_blank) {
                        // send a standard http response header
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-Type: text/html");
                        client.println();
                        
                        client.println("This STM32 is advertising on your LAN with bonjour!");

                        break;
                    }
                    if (c == '\n') {
                        // we're starting a new line
                        current_line_is_blank = true;
                    } else if (c != '\r') {
                        // we've gotten a character on the current line
                        current_line_is_blank = false;
                    }
                }
            }
            // give the web browser time to receive the data
            os_delay(1);
            client.stop();
        }
        os_delay(1);
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
void serviceFound(const char* type, MDNSServiceProtocol /*proto*/,
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