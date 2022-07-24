#ifdef PNTP_USING_ETH0

    #include <stdlib.h>
    #include <Arduino.h>
    #include <Ethernet.h>
    #include <EthernetUdp.h>
    #include <ArduinoMDNS.h>
    #include <SPI.h>

    #include <tcp_transport.h>
    #include <protocol.hpp>

    #define PNTP_TCP_LISTEN_PORT 1337

    char mdns_name_[64] = {0};
    char local_ip_[17] = "0.0.0.0";

    // =========================================================================
    EthernetClient _tcp_server;
    char _tcpReadBuffer[64] = {0};

    EthernetServer tcp_server(PNTP_TCP_LISTEN_PORT);

    extern SPIClass SPI;

    EthernetUDP udp;
    MDNS mdns(udp);

    // Initialize Ethernet with DHCP
    byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x0E, 0x9E };

    // =========================================================================
    // To follow packet flow, start reading list from bottom
    //

    // 3. Then convert the packet to a stream and send it
    //

    class TCPSender : public PacketSink {
        public:
            TCPSender(uint8_t id) : sink_id_(id) {}

            int sink_the_bytestream(const uint8_t* buffer, uint32_t length) {
                
                if (length > TX_BUF_SIZE) {
                    //return -1;
                }

                if(_tcp_server.connected()) {
                    _tcp_server.write(buffer, length);
                }

                return 0;
            }

        private:
            uint8_t sink_id_;
    };

    TCPSender eth0_pntp_output_stream_sink(1);


    // 2. Proces the packet in the channel 
    //

    class TreatPacketSinkAsStreamSink : public StreamSink {
        public:
            TreatPacketSinkAsStreamSink(PacketSink& output) : output_(output) {}
            
            int process_bytes(const uint8_t* buffer, uint32_t length, uint32_t* processed_bytes) {

                if (output_.sink_the_bytestream(buffer, length) != 0) {
                    return -1;
                }

                if (processed_bytes) {
                    *processed_bytes += length;
                }
                return 0;
            }
            
            uint32_t get_free_space() { 
                return SIZE_MAX; 
            }

        private:
            PacketSink& output_;

    } eth0_pntp_depacketizer_sink(eth0_pntp_output_stream_sink);

    StreamBasedPacketSink           eth0_pntp_processor_channel(eth0_pntp_depacketizer_sink);

    // 1. Send the byte stream into the processor channel
    //

    BidirectionalPacketBasedChannel eth0_pntp_processor(eth0_pntp_processor_channel);

    // 0. Take the incoming byte stream and segment it to packets
    //

    StreamToPacketSegmenter         eth0_pntp_input_stream_sink(eth0_pntp_processor);


    // =========================================================================
    // The listener service begins here

    bool server_has_ip_ = 0;
    bool server_is_listening_ = 0;
    bool mdns_is_initialized_ = 0;
    bool mdns_is_advertised_ = 0;

    void pntp_eth0_reset(void) {
        
        server_has_ip_ = 0;
        server_is_listening_ = 0;
        mdns_is_initialized_ = 0;
        mdns_is_advertised_ = 0;

        mdns.removeServiceRecord(PNTP_TCP_LISTEN_PORT, MDNSServiceTCP);

        for(int i = 0; i < sizeof(mdns_name_); i++) {
            mdns_name_[i] = 0;
        }

        mdns_name_[0] = 'n';
        mdns_name_[1] = '/';
        mdns_name_[2] = 'a';
        mdns_name_[3] = 0;
    }

    // Monitor the server state and make sure its valid
    // 
    //
    int eth0_server(const char * host_name) {

        // Failed to configure Ethernet using DHCP
        if (server_has_ip_ == 0) {      
            
            // Set SPI Bus
            SPI.setMOSI(PB15);
            SPI.setMISO(PB14);
            SPI.setSCLK(PB13);
            
            // Init eth0
            Ethernet.init(PB12);

            // Do DHCP
            if(Ethernet.begin(mac, 100, 100) == 0) {
                pntp_eth0_reset();

                return 0;
            }
            else {
                server_has_ip_ = 1;

                return 0;
            }
        }

        // Catch broken link
        if (Ethernet.linkStatus() == LinkOFF) {
            pntp_eth0_reset();
            return 0;
        }

        // Init TCP
        if(server_is_listening_ == 0) {
            // Maintain DHCP lease
            Ethernet.maintain();

            // Begin TCP Server & Listen
            tcp_server.begin();
            server_is_listening_ = 1;

            return 0;
        }

        if(server_is_listening_ == 1 && mdns_is_initialized_ == 0) {

            // Initialize the Bonjour/MDNS library. You can now reach or ping this
            // Arduino via the host name "arduino.local", provided that your operating
            // system is Bonjour-enabled (such as MacOS X).
            // Always call this before any other method!
            mdns.begin(Ethernet.localIP(), "powernet");

            mdns_is_initialized_ = 1;

            return 0;
        }

        if(server_is_listening_ == 1 && mdns_is_initialized_ == 1 && mdns_is_advertised_ == 0) {
            
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

            sprintf(mdns_name_, "%s.%d.%s", host_name, micros(), "_pntp\0");

            mdns.addServiceRecord(mdns_name_, PNTP_TCP_LISTEN_PORT, MDNSServiceTCP, NULL);

            mdns_is_advertised_ = 1;

            return 0;
        }

        // Get IP
        IPAddress local_ip = Ethernet.localIP();
        sprintf(local_ip_, "%d.%d.%d.%d", local_ip[0], local_ip[1], local_ip[2], local_ip[3]);

        return 1;
    }

    // =========================================================================
    // Accept incoming bytes
    //
    void pntp_eth0_listener(const char * host_name) {

        int server_status = eth0_server(host_name);

        if(server_status == 0) {
            return;
        }

        if(server_status == 1) {
                
            // Send pending IRQ(s)
            interrupt_event_sender(PNTP_IRQ_TRANSPORT_TCP, &eth0_pntp_depacketizer_sink);

            // Check for message
            _tcp_server = tcp_server.available();

            // Do we have a packet?
            if (_tcp_server) {

                // While socket has data...
                while (_tcp_server.available()) {

                    // Get data len
                    uint16_t queue_len = _tcp_server.available();

                    if(queue_len > 64) {
                        queue_len = 64;
                    }

                    // Get data
                    _tcp_server.readBytes(_tcpReadBuffer, queue_len);

                    // Feed data into StreamToPacketSegmenter
                    eth0_pntp_input_stream_sink.process_bytes((uint8_t *)_tcpReadBuffer, queue_len, nullptr);
                }
            }

            // This actually runs the Bonjour module. YOU HAVE TO CALL THIS PERIODICALLY,
            // OR NOTHING WILL WORK! Preferably, call it once per loop().
            mdns.run();
        }
    };

#endif