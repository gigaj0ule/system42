/* Includes ------------------------------------------------------------------*/
#pragma GCC optimize ("Og")

//#include <memory>
//#include <stdlib.h>
#include <protocol.hpp>
#include <crc.hpp>

/* Private defines -----------------------------------------------------------*/
#define INTERRUPT_QUEUE_SIZE 10
#define INTERRUPT_CHANNELS 2

#define PAYLOAD_CRC_LENGTH sizeof(CANONICAL_CRC32_POLYNOMIAL)
#define PAYLOAD_SEQUENCE_NUMBER_LENGTH 2

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/


// Data Packet (outgoing)
//typedef struct  __attribute__((packed, scalar_storage_order("little-endian"))){
typedef struct  __attribute__((packed)){
    uint8_t START_SENTINEL;
    uint8_t PROTOCOL_VERSION;
    uint16_t SEQUENCE_NO;

    uint32_t ORIGIN_PORT;

    uint64_t TIMESTAMP_64;

    uint32_t PAYLOAD_LENGTH;
    uint32_t ENDPOINT_ID;
    uint32_t PROCESSING_TIME;
    uint32_t HEADER_CRC32;
} response_data_packet_header_t;

// Interrupt packet (outgoing)
//typedef struct  __attribute__((packed, scalar_storage_order("little-endian"))){
typedef struct  __attribute__((packed)){
    uint8_t START_SENTINEL;
    uint8_t PROTOCOL_VERSION;

    uint16_t reserved_0;
    uint32_t reserved_1;

    uint64_t TIMESTAMP_64;

    uint32_t PAYLOAD_LENGTH;

    uint32_t IVT_CHANNEL;

    uint32_t reserved_2;

    uint32_t HEADER_CRC32;
} interrupt_event_packet_t;


/* Global constant data ------------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

Endpoint** endpoint_list_ = nullptr;       // initialized by calling fibre_publish
uint32_t n_endpoints_ = 0;                   // initialized by calling fibre_publish
uint32_t json_crc_;                        // initialized by calling fibre_publish
JSONDescriptorEndpoint json_file_endpoint_ = JSONDescriptorEndpoint();
EndpointProvider* application_endpoints_;

split_dword_64_t _micros;
split_dword_64_t _micros_offset;

/* Private constant data -----------------------------------------------------*/
constexpr uint8_t protocol_version_msb8 = static_cast<uint8_t>(PROTOCOL_VERSION >> 0);
constexpr uint8_t protocol_version_msb16 = static_cast<uint8_t>(PROTOCOL_VERSION >> 8);
constexpr uint8_t header_crc_index = TOTAL_HEADER_LEN - 4;

/* Private variables ---------------------------------------------------------*/
uint32_t host_interrupts_pending_[INTERRUPT_CHANNELS][INTERRUPT_QUEUE_SIZE] = {0};
bool host_interrupt_pending_[INTERRUPT_CHANNELS] = {0};

/* External variables --------------------------------------------------------*/
void micros_64(void);
uint32_t micros_32(void);
void set_micros_64(split_dword_64_t new_micros);
split_dword_64_t get_micros_64(void);
int32_t _latency;

/* Private function prototypes -----------------------------------------------*/
static inline int write_string(const char* str, StreamSink* output);

//#define NO_TIMESTAMPS

// =========================================================================
/* Function implementations ------------------------------------------------*/


// =========================================================================
// Process pending interrupts and send off to the host machine.
//
// This function is not thread-safe and must not be called during the 
// transmission of another packet on a StreamSink

void interrupt_event_sender(int transport, StreamSink* output) {

    if(host_interrupt_pending_[transport]) {
    
        host_interrupt_pending_[transport] = false;

        for (int i = INTERRUPT_QUEUE_SIZE - 1; i >= 0; i--) {

            if(host_interrupts_pending_[transport][i] != 0) {

                uint8_t payload_len = 0;

                // Create interrupt packet
                interrupt_event_packet_t interrupt_packet;

                interrupt_packet.START_SENTINEL = INTERRUPT_PACKET_PREFIX;
                interrupt_packet.PROTOCOL_VERSION = PROTOCOL_VERSION;
                interrupt_packet.IVT_CHANNEL = host_interrupts_pending_[transport][i];
                interrupt_packet.PAYLOAD_LENGTH = payload_len;

                // Calculate and append CRC32 into tail interrupt packet
                interrupt_packet.HEADER_CRC32 = calc_crc32<CANONICAL_CRC32_POLYNOMIAL>(CANONICAL_CRC32_INIT, (uint8_t *)&interrupt_packet, header_crc_index, HEADER_ENDIAN);
                
                // Create payload
                uint8_t payload_buffer[payload_len] = {0};
                
                // Calculate payload crc32
                uint32_t payload_crc32 = calc_crc32<CANONICAL_CRC32_POLYNOMIAL>
                    (CANONICAL_CRC32_INIT, (uint8_t *)&payload_buffer, sizeof(payload_buffer), HEADER_ENDIAN);
                
                // Send header
                output->process_bytes((uint8_t *)&interrupt_packet, sizeof(interrupt_packet), nullptr);

                // Send payload
                if(payload_len > 0) {
                    output->process_bytes(payload_buffer, sizeof(payload_buffer), nullptr);
                }

                // Send payload crc32
                output->process_bytes((const uint8_t *) &payload_crc32, PAYLOAD_CRC_LENGTH, nullptr);


                // Clear this interrupt
                host_interrupts_pending_[transport][i] = 0;
            }
        }
    }
}

// =========================================================================
// Time Stamp Functions 

void micros_64(void) {

    //uint32_t new_low = micros();
    uint32_t new_low = 0;

    if (new_low < _micros._word_32l){
        _micros._word_32h += 1;
    }

    _micros._word_32l = new_low;
}

void init_micros_64(void) {
    // Start us counter
    _micros._dword_64 = 0xFFFFFFF0;
    micros_64();
}

split_dword_64_t get_micros_64(void) {

    micros_64();

    uint64_t moo = {_micros._dword_64 +_micros_offset._dword_64};

    return { moo };
}


void set_micros_64(split_dword_64_t new_micros) {

    micros_64();

    _micros_offset._dword_64 = (new_micros._dword_64 - _micros._dword_64) + (_latency / 2);
}

// =========================================================================
// Integer to string
constexpr uint8_t INT_STR_SIZE = (sizeof(int)*CHAR_BIT/3 + 3);

void small_itoa(char *dest, uint32_t size, uint32_t x) {
    char buf[INT_STR_SIZE];
    char *p = &buf[INT_STR_SIZE - 1];
    
    *p = '\0';
    int i = x;

    do {
        *(--p) = abs(i%10) + '0';
        i /= 10;
    } while (i);

    if (x < 0) {
        *(--p) = '-';
    }

    uint32_t len = (uint32_t) (&buf[INT_STR_SIZE] - p);
    
    if (len > size) {
        // Not enough room
        return;
    }
    
    memcpy(dest, p, len);
}

// =========================================================================
// Converts serial stream to packets, if not complete packet, buffers for
// subsequent calls of function so not thread safe!
int StreamToPacketSegmenter::process_bytes(const uint8_t *buffer, uint32_t length, uint32_t* processed_bytes) {
    int result = 0;

    // Iterate bytes available
    while (length--) {

        // Process header
        if (header_index_ < TOTAL_HEADER_LEN) {

            // Process header byte
            header_buffer_[header_index_++] = *buffer;

            // What kind of header is this?
            if (header_index_ == 1) {
                if(header_buffer_[0] == TIMECODE_PACKET_PREFIX) {
                    // Timecode header, (test passed)
                }
                else if(header_buffer_[0] == DATA_PACKET_PREFIX) {
                    // Data header, (test passed)
                }
                // There are no device side interrupts yet
                else {
                    // Unrecognized header, go back to start
                    header_index_ = 0;
                }
            } 
            
            // Todo: Check Protocol Version
            else if (header_index_ == TOTAL_HEADER_LEN) {

                // Reconstruct length from header 
                // [PAYLOAD_LENGTH]
                packet_length_ = (
                    (uint32_t) *((uint32_t *)(header_buffer_ + 16))
                ) + PAYLOAD_CRC_LENGTH;

                // Check CRC
                uint32_t crc_remainder = calc_crc32<CANONICAL_CRC32_POLYNOMIAL>(CANONICAL_CRC32_INIT, header_buffer_, TOTAL_HEADER_LEN, HEADER_ENDIAN);
                
                if(crc_remainder) {
                    // If the packet header failed CRC then go back to start
                    header_index_ = 0;
                }

                else {
                    // Valid CRC

                    if(header_buffer_[0] == TIMECODE_PACKET_PREFIX){
                        
                        #ifndef NO_TIMESTAMPS                        
                            split_dword_64_t new_micros;

                            // Handle timecode here, since timecode has no payload
                            // [TIMESTAMP_64]
                            new_micros._dword_64 = (uint64_t) *((uint64_t *)(header_buffer_ + 8));
                            
                            set_micros_64(new_micros);
                        #endif

                        // Reset state
                        header_index_ = 0;
                        packet_index_ = 0;
                        packet_length_ = 0;
                    }
                }
            }

            // Clear payload receiver
            packet_index_ = 0;
        }

        // If header is processed, now try to process payload
        else if (header_index_ == TOTAL_HEADER_LEN) {
            
            if(packet_index_ < sizeof(packet_buffer_)) {
                // Copy incoming byte to the input buffer
                packet_buffer_[packet_index_++] = *buffer;
            }

            // Was the payload received?
            if (
                // Payload is fully received
                packet_index_ == packet_length_ ||
                // Or payload was too big and truncated... 
                packet_index_ >= sizeof(packet_buffer_)
            ) {

                // Calculate payload CRC remainder
                uint32_t payload_crc_remainder = calc_crc32<CANONICAL_CRC32_POLYNOMIAL>(CANONICAL_CRC32_INIT, packet_buffer_, packet_length_, HEADER_ENDIAN, USE_FAST_CRC32);

                // Check payload CRC32
                if (payload_crc_remainder == 0) {
            
                    // [SEQUENCE_NO]
                    uint16_t sequence_no_ = (
                        (uint16_t) *((uint16_t *)(header_buffer_ + 2))
                    );

                    // [ORIGIN_PORT]
                    origin_host_ = (
                        (uint32_t) *((uint32_t *)(header_buffer_ + 4))
                    );

                    // [ENDPOINT_ID]
                    endpoint_id_ = (
                        (uint32_t) *((uint32_t *)(header_buffer_ + 20))
                    );

                    // [EXPECTED_RESPONSE_LENGTH]
                    expected_response_length_ = (
                        (uint32_t) *((uint32_t *)(header_buffer_ + 24))
                    );

                    uint64_t origin_time = 0;//micros_32();

                    // If CRC is OK, hand payload off to the processor
                    result |= output_.process_endpoint(
                        packet_buffer_, 
                        packet_length_, 
                        origin_host_, 
                        sequence_no_, 
                        endpoint_id_, 
                        expected_response_length_,
                        origin_time
                    );
                }
                else { 
                    // If CRC is not OK, there was a problem...
                    // Todo: maybe return some fault code here so master 
                    // can resend the command?
                    __asm__ ("nop");
                }

                header_index_ = 0;
                packet_index_ = 0;
                packet_length_ = 0;
                sequence_no_ = 0;
                endpoint_id_ = 0;
                origin_host_ = 0;
            }
        }

        buffer++;
        
        // Increment process_bytes counter
        if (processed_bytes) {
            (*processed_bytes)++;
        }
    }

    return result;
}


// =========================================================================
// Process a packet we are sending to the master
int StreamBasedPacketSink::send_packet(
    const uint8_t *buffer, 
    uint32_t payload_length, 
    uint32_t origin_host,
    uint16_t sequence_no,
    uint64_t origin_time
    ) {

    // Cannot handle lengths greater than 2^31 due to fixed size header
    // payload count
    if (payload_length >= 2147483648) {
        return -1;
    }

    LOG_FIBRE("send header\r\n");
  
    //split_dword_64_t outgoing_time_stamp = get_micros_64();

    //uint32_t now_time = micros_32();
    //uint32_t processing_time = now_time - origin_time;

    response_data_packet_header_t outgoing_header;

    outgoing_header.START_SENTINEL      = DATA_PACKET_PREFIX;
    outgoing_header.PROTOCOL_VERSION    = PROTOCOL_VERSION;
    outgoing_header.ORIGIN_PORT         = (uint32_t) origin_host;
    outgoing_header.SEQUENCE_NO         = (uint16_t) sequence_no;
    outgoing_header.TIMESTAMP_64        = 0; //outgoing_time_stamp._dword_64;
    outgoing_header.PAYLOAD_LENGTH      = (uint32_t) payload_length;
    outgoing_header.PROCESSING_TIME     = 0; // processing_time;

    outgoing_header.HEADER_CRC32 = calc_crc32<CANONICAL_CRC32_POLYNOMIAL>(CANONICAL_CRC32_INIT, (uint8_t*)&outgoing_header, header_crc_index, HEADER_ENDIAN);    

    // Send packet header to host
    if (output_.process_bytes((uint8_t*)&outgoing_header, sizeof(outgoing_header), nullptr)) {
        return -1;
    }

    LOG_FIBRE("send payload:\r\n");
    //hexdump(buffer, length);
    
    // Now send payload to the host
    if (output_.process_bytes(buffer, payload_length, nullptr)) {
        return -1;
    }

    LOG_FIBRE("send crc32\r\n");
    
    // Calculate a CRC32 for the payload
    uint32_t payload_crc_remainder = calc_crc32<CANONICAL_CRC32_POLYNOMIAL>(CANONICAL_CRC32_INIT, buffer, payload_length, HEADER_ENDIAN, USE_FAST_CRC32);

    // Send CRC32
    if (output_.process_bytes((const uint8_t *) &payload_crc_remainder, PAYLOAD_CRC_LENGTH, nullptr)) {
        return -1;
    }

    LOG_FIBRE("sent!\r\n");
    return 0;
}

// =========================================================================
// Write JSON descriptor to the master
bool JSONDescriptorEndpoint::write_json(uint32_t id, StreamSink* output) {
    write_string("{\"n\":\"ep\",", output);

    // write endpoint ID
    write_string("\"id\":", output);
    char id_buf[10];
    
    small_itoa(id_buf, sizeof(id_buf), (unsigned)id);
    write_string(id_buf, output);

    write_string(",\"t\":\"json\",\"a\":\"r\"}", output);
    return true;
}

// =========================================================================
// Iterate endpoint list and register with master
void JSONDescriptorEndpoint::register_endpoints(Endpoint** list, uint32_t endpoint_id, uint32_t length) {
    if (endpoint_id < length) {
        list[endpoint_id] = this; 
    }
}

// =========================================================================
// Returns JSON manifest of all the endpoints and what they represent
//
void JSONDescriptorEndpoint::handle(const uint8_t* input, uint32_t input_length, StreamSink* output) {

    // The request must contain a 32 bit integer to specify an offset
    if (input_length < 4) {
        return;
    }

    // Read the 32 bit value, (little endian)
    uint32_t offset = 0;
    read_le<uint32_t>(&offset, input);

    // If the offset is special value 0xFFFFFFFF, send back the JSON crc instead
    // so we can compare it to the local cache and determine if we need to get a 
    // new object descriptor tree

    if (offset == 0xffffffff) {
        default_readwrite_endpoint_handler(&json_crc_, (uint32_t)0, (uint32_t)0, nullptr, 0, output);
    } 

    // If not a special offset, then see what was on this endpoint and execute
    else {
        NullStreamSink output_with_offset = NullStreamSink(offset, *output);

        uint32_t endpoint_id = 0;
        
        // Parent object
        write_string("{\"bitsnap\":[", &output_with_offset);
        
        // Write json
        json_file_endpoint_.write_json(endpoint_id, &output_with_offset);

        endpoint_id += decltype(json_file_endpoint_)::endpoint_count;

        write_string(",", &output_with_offset);

        application_endpoints_->write_json(endpoint_id, &output_with_offset);
        
        write_string("]}", &output_with_offset);
    }
}


// =========================================================================
// Process packets sent by the master to us, the Slave
int BidirectionalPacketBasedChannel::process_endpoint(
    const uint8_t* buffer, 
    uint32_t length, 
    uint32_t origin_host,
    uint16_t packet_sequence_number, 
    uint32_t endpoint_id,
    uint32_t expected_response_length,
    uint64_t origin_time
) {

    LOG_FIBRE("got packet of length %d: \r\n", length);
    //hexdump(buffer, length);
    if (length < 4)
        return -1;

    // TODO: think about some kind of ordering guarantees
    // currently the packet_sequence_number is just used to associate a response with a request
    
    // Is the master expecting a response?
    bool master_expects_response = endpoint_id & 0x80000000;

    // Bitmask to transform endpoint ID to integer
    endpoint_id &= 0x7fffffff;

    __asm__ volatile("nop");

    // Is this endpoint larger than the number of total endpoints?
    if (endpoint_id > n_endpoints_) {
        // Yes it is, return fail
        return -1;
    }

    // Does this endpoint ID exist for us?
    Endpoint* endpoint = endpoint_list_[endpoint_id];
    if (!endpoint) {

        __asm__ volatile("nop");

        // No, return fail
        LOG_FIBRE("critical: no endpoint at %d", endpoint_id);
        return -1;
    }

    __asm__ volatile("nop");

    // TODO: if more bytes than the MTU were requested, 
    // should we abort or just return as much as possible?

    // How many bytes does the master expect? (Limit is up to 2^31 - 4[crc32])
    //volatile uint32_t expected_response_length = read_le<uint32_t>(&buffer, &length);

    // Limit response length according to our local TX buffer size
    if (expected_response_length > sizeof(tx_buf_)) {
        expected_response_length = sizeof(tx_buf_);
    }

    __asm__ volatile("nop");

    // Create buffer sink of the size we need for our output buffer
    MemoryStreamSink output(tx_buf_, expected_response_length);

    // Handle / execute the endpoint! The endpoint returns its value to the 
    // MemoryStreamSink &output buffer
    // Note: length here is input length!
    endpoint->handle(buffer, length - PAYLOAD_CRC_LENGTH, &output);

    // Send response
    if (master_expects_response) {

        // What's the expected size of the response?
        volatile uint32_t actual_response_length = expected_response_length - output.get_free_space();

        LOG_FIBRE("send packet:\r\n");
        //hexdump(tx_buf_, actual_response_length);

        // Create response packet header and send off
        output_.send_packet(tx_buf_, actual_response_length, origin_host, packet_sequence_number, origin_time);
    }

    return 0;
}

// =========================================================================
// Interrupts are assigned to interrupt vectors 1 to 255. 0 is not a valid
// vector. 

// When send_event_to_host() is called, it buffers a vector into 
// the interrupt queue (size of 8). 

// The queue is normally processed immediately, but it is not processed
// if there is currently an endpoint operation going on. The firmware
// must first finish an endpoint operation before the vectors can be 
// processed.

// Interrupt order is preserved, but the queue drops interrupts silently 
// if too many are in the FILO stack
void send_event_to_host(event_vector_t interrupt_vector) {
    
    // Cycle the FIFOs
    for (int i = INTERRUPT_QUEUE_SIZE - 1; i > 0; i--) {
        for(int j = 0; j < INTERRUPT_CHANNELS; j ++) {
            host_interrupts_pending_[j][i] = host_interrupts_pending_[j][i-1];
        }
    }

    // Add new IRQ to FIFOs
    if(interrupt_vector.event_trigger_enabled & 0x80000000) {

        for(int j = 0; j < INTERRUPT_CHANNELS; j ++) {
            host_interrupts_pending_[j][0] = interrupt_vector.event_trigger_vector;
            host_interrupt_pending_[j] = true;
        }
    }
}


// =========================================================================
// Send a system envent vector
//
void send_system_interrupt(uint16_t sysint_vector) {
    event_vector_t user_button_irq = { (2147483647 - (uint32_t)sysint_vector) };
    user_button_irq.event_trigger_enabled |= 0x80000000;
    send_event_to_host(user_button_irq);
}