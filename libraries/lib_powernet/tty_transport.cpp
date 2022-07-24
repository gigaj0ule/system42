#if defined(USBD_USE_CDC)

    #include <Arduino.h>
    #include <usbd_cdc.h>
    #include <usbd_cdc_if.h>

    #include <tty_transport.h>
    #include <protocol.hpp>

    char readBuffer_[CDC_RECEIVE_QUEUE_BUFFER_SIZE] = {0};

    // =========================================================================
    // To follow packet flow, start reading list from bottom
    //

    // 3. Then convert the packet to a stream and send it
    //
    class USBSender : public PacketSink {
        public:
            USBSender(uint8_t id) : sink_id_(id) {}

            int sink_the_bytestream(const uint8_t* buffer, uint32_t length) {
                
                if (length > TX_BUF_SIZE) {
                    //return -1;
                }

                SerialUSB.write(buffer, length);

                return 0;
            }

        private:
            uint8_t sink_id_;
    };

    USBSender pntp_output_stream_sink(0);


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

    } pntp_depacketizer_sink(pntp_output_stream_sink);

    StreamBasedPacketSink pntp_processor_channel(pntp_depacketizer_sink);


    // 1. Send the byte stream into the processor channel
    //


    BidirectionalPacketBasedChannel pntp_processor(pntp_processor_channel);

    // 0. Take the incoming byte stream and segment it to packets
    //

    StreamToPacketSegmenter         pntp_input_stream_sink(pntp_processor);


    // =========================================================================
    // The listener service begins here

    void pntp_tty_listener(){

        // Send pending IRQ(s)
        interrupt_event_sender(PNTP_IRQ_TRANSPORT_TTY, &pntp_depacketizer_sink);

        // Get message from TTY
        uint16_t queue_len = CDC_ReceiveQueue_ReadSize(&ReceiveQueue);

        // Process in finite chunks
        if(queue_len > 64) {
            queue_len = 64;
        }

        // Read from TTY buffer
        SerialUSB.readBytes(readBuffer_, queue_len);

        // Process these bytes we just got
        pntp_input_stream_sink.process_bytes((uint8_t *)readBuffer_, queue_len, nullptr);
    }


#endif