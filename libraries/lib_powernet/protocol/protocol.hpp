/*
see protocol.md for the protocol specification
*/
#pragma GCC optimize ("O1")
//#pragma once

#ifndef __PROTOCOL_HPP
#define __PROTOCOL_HPP

// TODO: resolve assert
#define assert(expr)

#include <functional>
#include <limits>
#include <float.h>
#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "crc.hpp"
#include "cpp_utils.hpp"
#include <unistd.h>
#include <kwargs.hpp>

/*******************************************************/
/* Important Parameters ********************************/
/*******************************************************/

constexpr uint16_t PROTOCOL_VERSION = 1;

// These values must not be larger than 2^31, or, more realistically, 
// however much RAM you want to allocate to the communications system.

// Transfers requests larger than this are truncated by this device. 
#ifdef __STM32F1__
constexpr uint16_t TX_BUF_SIZE = 512 + 32;
constexpr uint16_t RX_BUF_SIZE = 512 + 32;
#endif

#ifdef __STM32F4__
constexpr uint16_t TX_BUF_SIZE = 1024 + 32;
constexpr uint16_t RX_BUF_SIZE = 1024 + 32;
#endif

constexpr uint8_t DATA_PACKET_PREFIX       = 0xAA;
constexpr uint8_t TIMECODE_PACKET_PREFIX   = 0xAB;
constexpr uint8_t INTERRUPT_PACKET_PREFIX  = 0xAC;


/*******************************************************/
// System interrupt vectors, as defined in interaction.pyx
//
#define SYSINT_DEVICE_ALIVE        0
#define SYSINT_USER_BUTTON_FALLING 4
#define SYSINT_USER_BUTTON_RISING  5
#define SYSINT_REQUEST_TIMECODE    3

typedef union {
  
    uint64_t _dword_64;
    struct {
        uint32_t _word_32l;
        uint32_t _word_32h;
    };
} split_dword_64_t;


/*******************************************************/
void send_system_interrupt (uint16_t sysint_vector);
void small_itoa (char *dest, uint32_t size, uint32_t x);

// Note that this option cannot be used to debug UART because it prints on UART
//#define DEBUG_FIBRE
#ifdef DEBUG_FIBRE
#define LOG_FIBRE(...)  do { printf(__VA_ARGS__); } while (0)
#else
#define LOG_FIBRE(...)  //((void) 0)
#endif

// Default CRC-8 Polynomial: x^8 + x^5 + x^4 + x^2 + x + 1
// Can protect a 4 byte payload against toggling of up to 5 bits
//  source: https://users.ece.cmu.edu/~koopman/crc/index.html
constexpr uint8_t CANONICAL_CRC8_POLYNOMIAL = 0x37;
constexpr uint8_t CANONICAL_CRC8_INIT = 0x42;
constexpr uint32_t  CRC8_BLOCKSIZE = 4;

// Default CRC-16 Polynomial: 0x9eb2 x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1
// Can protect a 135 byte payload against toggling of up to 5 bits
//  source: https://users.ece.cmu.edu/~koopman/crc/index.html
// Also known as CRC-16-DNP
constexpr uint16_t CANONICAL_CRC16_POLYNOMIAL = 0x3d65;
constexpr uint16_t CANONICAL_CRC16_INIT = 0x1337;

// Default CRC-32 polynomial
constexpr uint32_t CANONICAL_CRC32_POLYNOMIAL = 0xc9d204f5;
constexpr uint32_t CANONICAL_CRC32_INIT = 0x1337C0DE;

#define TOTAL_HEADER_LEN 32

#define HEADER_IS_LITTLE_ENDIAN

#ifdef HEADER_IS_LITTLE_ENDIAN
    #define HEADER_ENDIAN false
#else
    #error "Big endian no longer supported"
#endif

#define USE_FAST_CRC32 false

typedef uint32_t endpoint_id_t;

struct ReceiverState {
    endpoint_id_t endpoint_id;
    uint32_t length;
    uint16_t seqno_thread;
    uint16_t seqno;
    bool expect_ack;
    bool expect_response;
    bool enforce_ordering;
};

enum string_type_t {
    NULL_TERMINATED_STRING,
    FIXED_SIZE_BUFFER
};

// Maximum time we allocate for processing and responding to a request
constexpr uint32_t PROTOCOL_SERVER_TIMEOUT_MS = 10;

typedef union {
    uint32_t event_trigger_vector;
    uint32_t event_trigger_enabled;
} event_vector_t;

void send_event_to_host(event_vector_t interrupt_vector);

#include <cstring>

template<typename T, typename = typename std::enable_if_t<!std::is_const<T>::value>>
inline uint32_t write_le(T value, uint8_t* buffer){
    //TODO: add static_assert that this is still a little endian machine
    memcpy(&buffer[0], &value, sizeof(value));
    return sizeof(value);
}

template<typename T>
typename std::enable_if_t<std::is_const<T>::value, uint32_t>
write_le(T value, uint8_t* buffer) {
    return write_le<std::remove_const_t<T>>(value, buffer);
}

template<>
inline uint32_t write_le<float>(float value, uint8_t* buffer) {
    static_assert(CHAR_BIT * sizeof(float) == 32, "32 bit floating point expected");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 floating point expected");
    const uint32_t * value_as_uint32 = reinterpret_cast<const uint32_t*>(&value);
    return write_le<uint32_t>(*value_as_uint32, buffer);
}

template<typename T>
inline uint32_t read_le(T* value, const uint8_t* buffer){
    // TODO: add static_assert that this is still a little endian machine
    memcpy(value, buffer, sizeof(*value));
    return sizeof(*value);
}

template<>
inline uint32_t read_le<float>(float* value, const uint8_t* buffer) {
    static_assert(CHAR_BIT * sizeof(float) == 32, "32 bit floating point expected");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 floating point expected");

    return read_le(reinterpret_cast<uint32_t*>(value), buffer);
}

// @brief Reads a value of type T from the buffer.
// @param buffer    Pointer to the buffer to be read. The pointer is updated by the number of bytes that were read.
// @param length    The number of available bytes in buffer. This value is updated to subtract the bytes that were read.
template<typename T>
static inline T read_le(const uint8_t** buffer, uint32_t* length) {
    T result;
    uint32_t cnt = read_le(&result, *buffer);
    *buffer += cnt;
    *length -= cnt;
    return result;
}

class PacketSink {
public:
    // @brief Get the maximum packet length (aka maximum transmission unit)
    // A packet size shall take no action and return an error code if the
    // caller attempts to send an oversized packet.
    //virtual uint32_t get_mtu() = 0;

    // @brief Processes a packet.
    // The blocking behavior shall depend on the thread-local deadline_ms variable.
    // @return: 0 on success, otherwise a non-zero error code
    // TODO: define what happens when the packet is larger than what the implementation can handle.
    virtual int sink_the_bytestream(const uint8_t* buffer, uint32_t length) {
        return 0;
    };
    virtual int process_endpoint(
        const uint8_t* buffer, 
        uint32_t length, 
        uint32_t origin_host,
        uint16_t sequence_no, 
        uint32_t endpoint_id, 
        uint32_t expected_response_length,
        uint64_t origin_time
    ) {
        return 0;
    };
    virtual int send_packet(
        const uint8_t *buffer, 
        uint32_t payload_length, 
        uint32_t origin_host,
        uint16_t sequence_no,
        uint64_t origin_time
    ) {
        return 0;
    };
};

class StreamSink {
public:
    // @brief Processes a chunk of bytes that is part of a continuous stream.
    // The blocking behavior shall depend on the thread-local deadline_ms variable.
    // @param processed_bytes: if not NULL, shall be incremented by the number of
    //        bytes that were consumed.
    // @return: 0 on success, otherwise a non-zero error code
    virtual int process_bytes(const uint8_t* buffer, uint32_t length, uint32_t* processed_bytes) = 0;

    // @brief Returns the number of bytes that can still be written to the stream.
    // Shall return SIZE_MAX if the stream has unlimited lenght.
    // TODO: deprecate
    virtual uint32_t get_free_space() = 0;

    /*int process_bytes(const uint8_t* buffer, uint32_t length) {
        uint32_t processed_bytes = 0;
        return process_bytes(buffer, length, &processed_bytes);
    }*/
};

#define PNTP_IRQ_TRANSPORT_TTY 0
#define PNTP_IRQ_TRANSPORT_TCP 1

void interrupt_event_sender(int channel, StreamSink* output);

class StreamSource {
public:
    // @brief Generate a chunk of bytes that are part of a continuous stream.
    // The blocking behavior shall depend on the thread-local deadline_ms variable.
    // @param generated_bytes: if not NULL, shall be incremented by the number of
    //        bytes that were written to buffer.
    // @return: 0 on success, otherwise a non-zero error code
    virtual int get_bytes(uint8_t* buffer, uint32_t length, uint32_t* generated_bytes) = 0;

    // @brief Returns the number of bytes that can still be written to the stream.
    // Shall return SIZE_MAX if the stream has unlimited lenght.
    // TODO: deprecate
    //virtual uint32_t get_free_space() = 0;
};

class StreamToPacketSegmenter : public StreamSink {
    public:
        StreamToPacketSegmenter(PacketSink& output) :
            output_(output)
        {
        };

        int process_bytes(const uint8_t *buffer, uint32_t length, uint32_t* processed_bytes);
        
        uint32_t get_free_space() { return SIZE_MAX; }

    private:
        uint8_t header_buffer_[TOTAL_HEADER_LEN];
        int16_t header_index_ = 0;
        uint8_t packet_buffer_[RX_BUF_SIZE];
        uint32_t packet_index_ = 0;
        uint32_t packet_length_ = 0;
        uint32_t origin_host_ = 0;
        uint32_t endpoint_id_ = 0;
        uint32_t expected_response_length_ = 0;
        uint16_t sequence_no_ = 0;
        PacketSink& output_;
};

class StreamBasedPacketSink : public PacketSink {
    public:
        StreamBasedPacketSink(StreamSink& output) :
            output_(output)
        {
        };
        
        //uint32_t get_mtu() { return SIZE_MAX; }
        //int process_packet(const uint8_t *buffer, uint32_t length);
        int send_packet(
            const uint8_t *buffer, 
            uint32_t payload_length, 
            uint32_t origin_host,
            uint16_t sequence_no,
            uint64_t origin_time
        );

    private:
        StreamSink& output_;
};

// @brief: Represents a stream sink that's based on an underlying packet sink.
// A single call to process_bytes may result in multiple packets being sent.
class PacketBasedStreamSink : public StreamSink {
public:
    PacketBasedStreamSink(PacketSink& packet_sink) : _packet_sink(packet_sink) {}
    ~PacketBasedStreamSink() {}

    int process_bytes(const uint8_t* buffer, uint32_t length, uint32_t* processed_bytes) {
        // Loop to ensure all bytes get sent
        while (length) {
            uint32_t chunk = length;
            // send chunk as packet
            if (_packet_sink.sink_the_bytestream(buffer, chunk))
                return -1;
            buffer += chunk;
            length -= chunk;
            if (processed_bytes)
                *processed_bytes += chunk;
        }
        return 0;
    }

    uint32_t get_free_space() { return SIZE_MAX; }

private:
    PacketSink& _packet_sink;
};

// Implements the StreamSink interface by writing into a fixed size
// memory buffer.
class MemoryStreamSink : public StreamSink {
public:
    MemoryStreamSink(uint8_t *buffer, uint32_t length) :
        buffer_(buffer),
        buffer_length_(length) {}

    // Returns 0 on success and -1 if the buffer could not accept everything because it became full
    int process_bytes(const uint8_t* input_buffer, uint32_t length, uint32_t* processed_bytes) {
        uint32_t chunk;

        if(length < buffer_length_) {
            chunk = length;
        }
        else {
            chunk = buffer_length_; 
        }
        
        memcpy(buffer_, input_buffer, chunk);

        buffer_ += chunk;
        buffer_length_ -= chunk;
        
        if (processed_bytes) {
            *processed_bytes += chunk;
        }

        if(chunk == length) {
            return 0;
        }
        else {
            return -1;
        }
    }

    uint32_t get_free_space() {
        return buffer_length_; 
    }

private:
    uint8_t * buffer_;
    uint32_t buffer_length_;
};

// Implements the StreamSink interface by discarding the first couple of bytes
// and then forwarding the rest to another stream.
class NullStreamSink : public StreamSink {
public:
    NullStreamSink(uint32_t skip, StreamSink& follow_up_stream) :
        skip_(skip),
        follow_up_stream_(follow_up_stream) {}

    // Returns 0 on success and -1 if the buffer could not accept everything because it became full
    int process_bytes(const uint8_t* buffer, uint32_t length, uint32_t* processed_bytes) {
        if (skip_ < length) {
            buffer += skip_;
            length -= skip_;
            if (processed_bytes)
                *processed_bytes += skip_;
            skip_ = 0;
            return follow_up_stream_.process_bytes(buffer, length, processed_bytes);
        } else {
            skip_ -= length;
            if (processed_bytes)
                *processed_bytes += length;
            return 0;
        }
    }

    uint32_t get_free_space() { return skip_ + follow_up_stream_.get_free_space(); }

private:
    uint32_t skip_;
    StreamSink& follow_up_stream_;
};


// Implements the StreamSink interface by calculating the CRC16 checksum
// on the data that is sent to it.
class CRC16Calculator : public StreamSink {
public:
    CRC16Calculator(uint16_t crc16_init) :
        crc16_(crc16_init) {}

    int process_bytes(const uint8_t* buffer, uint32_t length, uint32_t* processed_bytes) {
        crc16_ = calc_crc16<CANONICAL_CRC16_POLYNOMIAL>(crc16_, buffer, length);
        if (processed_bytes)
            *processed_bytes += length;
        return 0;
    }

    uint32_t get_free_space() { return SIZE_MAX; }

    uint16_t get_crc16() { return crc16_; }
private:
    uint16_t crc16_;
};


// Implements the StreamSink interface by calculating the CRC16 checksum
// on the data that is sent to it.
class CRC32Calculator : public StreamSink {
public:
    CRC32Calculator(uint32_t crc32_init, bool endian=1) :
        crc32_(crc32_init),
        endian_(endian) 
    {}

    int process_bytes(const uint8_t* buffer, uint32_t length, uint32_t* processed_bytes) {
        crc32_ = calc_crc32<CANONICAL_CRC32_POLYNOMIAL>(crc32_, buffer, length, endian_);
        if (processed_bytes)
            *processed_bytes += length;
        return 0;
    }

    uint32_t get_free_space() { return SIZE_MAX; }

    uint32_t get_crc32() { return crc32_; }

private:
    uint32_t crc32_;
    bool endian_;
};


// @brief Endpoint request handler
//
// When passed a valid endpoint context, implementing functions shall handle an
// endpoint read/write request by reading the provided input data and filling in
// output data. The exact semantics of this function depends on the corresponding
// endpoint's specification.
//
// @param input: pointer to the input data
// @param input_length: number of available input bytes
// @param output: The stream where to write the output to. Can be null.
//                The handler shall abort as soon as the stream returns
//                a non-zero error code on write.
typedef std::function<void(
    void * value, 
    uint32_t min_value, 
    uint32_t max_value, 
    const uint8_t* input, 
    uint32_t input_length, 
    StreamSink* output
    )> 
    EndpointHandler;

// @brief Default endpoint handler for const types
// @return: True if endpoint was written to, False otherwise
template<typename T>
    
    std::enable_if_t<
        std::is_const<T>::value, 
    bool>

    default_readwrite_endpoint_handler(
        T * value, 
        T min_value,
        T max_value,
        const uint8_t* input, 
        uint32_t input_length, 
        StreamSink* output        
    ) {
        
        // If the old value was requested, call the corresponding little endian serialization function
        if (output) {
        
            // Make buffer size dependent on the type
            uint8_t buffer[sizeof(T)];
            
            uint32_t cnt = write_le<T>(*value, buffer);
            
            if (cnt <= output->get_free_space()) {
                output->process_bytes(buffer, cnt, nullptr);
            }
        }

        // We don't ever write to const types
        return false;
    }



// @brief Default endpoint handler for non-const, non-endpoint-ref types
template<typename T>

    std::enable_if_t<
        !std::is_const<T>::value 
        //&& !std::is_same<T, event_vector_t>::value, 
    , bool >

    default_readwrite_endpoint_handler(
        T * value, 
        T min_value,
        T max_value,
        const uint8_t* input, 
        uint32_t input_length, 
        StreamSink* output
    ) {
        
        bool master_wrote_to_endpoint = false;

        // Make buffer to hold T    
        uint8_t buffer[sizeof(T)] = { 0 };

        // If a new value was passed, call the corresponding little endian deserialization function
        if (input_length >= sizeof(buffer)) {

            // Get plausible value from master
            T plausible_value;
            
            read_le<T>(&plausible_value, input);

            // Does the property have constraints? If so, enforce them.
            if constexpr(!std::is_same<T, event_vector_t>::value) {
                // Does not apply to event_vector_t
                if(min_value != max_value) {
                    if(plausible_value < min_value) {
                        plausible_value = min_value;
                    }
                    else if(plausible_value > max_value) {
                        plausible_value = max_value;
                    }
                }
            }

            // Is this an interrupt endpoint?
            if constexpr(std::is_same<T, event_vector_t>::value) {
                // Yes, we must protect event vector IDs from changing, we are only allowed 
                // to set the MSB and nothing else.
                value->event_trigger_vector = (value->event_trigger_vector & 0x7FFFFFFF) | (plausible_value.event_trigger_vector & 0x80000000);
            }
            else {
                // For all other types, set value entirely with buffer contents
                memcpy(value, &plausible_value, sizeof(T));
            }

            master_wrote_to_endpoint = true;
        }

        // Read the endpoint value into output
        default_readwrite_endpoint_handler<const T>(
            const_cast<const T*>(value), 
            (T)(min_value), 
            (T)(max_value), 
            input, 
            input_length, 
            output
        );

        return master_wrote_to_endpoint;
    }



// 

// @brief Default endpoint handler for non-const, non-endpoint-ref types
template<typename T>

    bool flexible_readwrite_endpoint_handler(
        T * value, 
        T min_value,
        T max_value,
        bool is_read_only,
        const uint8_t* input, 
        uint32_t input_length, 
        StreamSink* output
    ) {
        
        bool master_wrote_to_endpoint = false;

        // Make buffer to hold T    
        uint8_t buffer[sizeof(T)] = { 0 };

        // If a new value was passed, call the corresponding little endian deserialization function
        if (input_length >= sizeof(buffer) && !is_read_only) {

            // Get plausible value from master
            T plausible_value;
            
            read_le<T>(&plausible_value, input);

            // Does the property have constraints? If so, enforce them.
            if constexpr(!std::is_same<T, event_vector_t>::value) {
                // Does not apply to event_vector_t
                if(min_value != max_value) {
                    if(plausible_value < min_value) {
                        plausible_value = min_value;
                    }
                    else if(plausible_value > max_value) {
                        plausible_value = max_value;
                    }
                }
            }

            // Is this an interrupt endpoint?
            if constexpr(std::is_same<T, event_vector_t>::value) {
                // Yes, we must protect event vector IDs from changing, we are only allowed 
                // to set the MSB and nothing else.
                value->event_trigger_vector = (value->event_trigger_vector & 0x7FFFFFFF) | (plausible_value.event_trigger_vector & 0x80000000);
            }
            else {
                // For all other types, set value entirely with buffer contents
                memcpy(value, &plausible_value, sizeof(T));
            }

            master_wrote_to_endpoint = true;
        }

        // Read the endpoint value into output?
        // If the old value was requested...
        // call the corresponding little endian serialization function
        if (output) {
        
            // Make buffer size dependent on the type
            uint8_t buffer[sizeof(T)];
            
            uint32_t cnt = write_le<T>(*value, buffer);
            
            if (cnt <= output->get_free_space()) {
                output->process_bytes(buffer, cnt, nullptr);
            }
        }

        // Return if success
        return master_wrote_to_endpoint;
    }


// @brief Endpoint handler for non-const string or buffer types
// @return: True if endpoint was written to, False otherwise
template<typename T>

    bool string_readwrite_endpoint_handler(
        T* value, 
        uint32_t string_length,
        bool is_read_only,
        const uint8_t* input, 
        uint32_t input_length, 
        StreamSink* output
    ) {

        uint32_t t_size = string_length;//sizeof(T);

        // If a new value was passed, update the local string
        bool strings_are_not_the_same = 0;

        // IMPORTANT: you must send a null terminated zero-length string "\000" to "unset" 
        // any existing string, or we must assume it was a read request...
        if(input_length > 0 && !is_read_only) {

            // Update characters in string
            for(uint32_t i = 0; i < t_size && i < input_length; i++) {
                if(*(*(value) + i) != input[i]) {
                    *(*(value) + i) = input[i];
                    strings_are_not_the_same = 1;
                }
            }

            // Null-fill extra characters in string
            for(uint32_t i = input_length; i < t_size; i++) {
                *(*(value) + i) = 0;
            }
        }

        // Read the endpoint value into output?
        // If the old value was requested... 
        // call the corresponding little endian serialization function
        if (output) {

            //uint32_t t_size = sizeof(T);

            uint8_t buffer[t_size];
            memcpy(&buffer[0], value, t_size);

            // Prevent buffer overflows
            uint32_t free_space = output->get_free_space();        
            if (t_size > free_space) {
                t_size = free_space;
            }

            // Todo: Host should compare requested packet size to sent packet size, 
            // and warn if there was a truncation.

            output->process_bytes(buffer, t_size, nullptr);
        }

        return strings_are_not_the_same;
    }


template<typename T>
static inline const char* get_json_modifier();

template<>
inline constexpr const char* get_json_modifier<double>() {
    return "\"t\":\"d\"";
}
template<>
inline constexpr const char* get_json_modifier<float>() {
    return "\"t\":\"f\"";
}
template<>
inline constexpr const char* get_json_modifier<int64_t>() {
    return "\"t\":\"i64\"";
}
template<>
inline constexpr const char* get_json_modifier<int32_t>() {
    return "\"t\":\"i32\"";
}
template<>
inline constexpr const char* get_json_modifier<uint8_t>() {
    return "\"t\":\"u8\"";
}
template<>
inline constexpr const char* get_json_modifier<bool>() {
    return "\"t\":\"b\"";
}
template<>
inline constexpr const char* get_json_modifier<event_vector_t>() {
    return "\"t\":\"e\"";
}


class Endpoint {
public:
    //const char* const name_;
    virtual void handle(const uint8_t* input, uint32_t input_length, StreamSink* output) = 0;
    virtual bool get_string(char * output, uint32_t length) { return false; }
    virtual bool set_string(char * buffer, uint32_t length) { return false; }
    virtual bool set_from_float(float value) { return false; }
};

static inline int write_string(const char* str, StreamSink* output) {
    return output->process_bytes(reinterpret_cast<const uint8_t*>(str), strlen(str), nullptr);
}


/* @brief Handles the communication protocol on one channel.
*
* When instantiated with a list of endpoints and an output packet sink,
* objects of this class will handle packets passed into process_packet,
* pass the relevant data to the corresponding endpoints and dispatch response
* packets on the output.
*/
class BidirectionalPacketBasedChannel : public PacketSink {
public:
    BidirectionalPacketBasedChannel(PacketSink& output) :
        output_(output)
    { }

    //uint32_t get_mtu() {
    //    return SIZE_MAX;
    //}
    //int process_packet(const uint8_t* buffer, uint32_t length);
    int process_endpoint(
        const uint8_t* buffer, 
        uint32_t length, 
        uint32_t origin_host,
        uint16_t sequence_no, 
        uint32_t endpoint_id, 
        uint32_t expected_response_length,
        uint64_t origin_time
    );

private:
    PacketSink& output_;
    uint8_t tx_buf_[TX_BUF_SIZE];
};


/* ToString / FromString functions -------------------------------------------*/
/*
* These functions are currently not used by Fibre and only here to
* support the ODrive ASCII protocol.
* TODO: find a general way for client code to augment endpoints with custom
* functions
*/

#ifdef ENABLE_ASCII_PROTOCOL

template<typename T>
struct format_traits_t;

template<> struct format_traits_t<float> { using type = void;
     static constexpr const char * fmt = "%f";
     static constexpr const char * fmtp = "%f";
};
template<> struct format_traits_t<int64_t> { using type = void;
    static constexpr const char * fmt = "%lld";
    static constexpr const char * fmtp = "%lld";
};
/*
template<> struct format_traits_t<uint64_t> { using type = void;
    static constexpr const char * fmt = "%llu";
    static constexpr const char * fmtp = "%llu";
};*/
template<> struct format_traits_t<int32_t> { using type = void;
    static constexpr const char * fmt = "%ld";
    static constexpr const char * fmtp = "%ld";
};
/*
template<> struct format_traits_t<uint32_t> { using type = void;
    static constexpr const char * fmt = "%lu";
    static constexpr const char * fmtp = "%lu";
};*/
template<> struct format_traits_t<int16_t> { using type = void;
    static constexpr const char * fmt = "%hd";
    static constexpr const char * fmtp = "%hd";
};
/*template<> struct format_traits_t<uint16_t> { using type = void;
    static constexpr const char * fmt = "%hu";
    static constexpr const char * fmtp = "%hu";
};*/
template<> struct format_traits_t<int8_t> { using type = void;
    static constexpr const char * fmt = "%hhd";
    static constexpr const char * fmtp = "%d";
};
template<> struct format_traits_t<uint8_t> { using type = void;
    static constexpr const char * fmt = "%hhu";
    static constexpr const char * fmtp = "%u";
};

template<typename T, typename = typename format_traits_t<T>::type>
static bool to_string(const T& value, char * buffer, uint32_t length, int) {
    snprintf(buffer, length, format_traits_t<T>::fmtp, value);
    return true;
}
// Special case for float because printf promotes float to double, and we get warnings
template<typename T = float>
static bool to_string(const float& value, char * buffer, uint32_t length, int) {
    snprintf(buffer, length, "%f", (double)value);
    return true;
}
template<typename T = bool>
static bool to_string(const bool& value, char * buffer, uint32_t length, int) {
    buffer[0] = value ? '1' : '0';
    buffer[1] = 0;
    return true;
}
template<typename T>
static bool to_string(const T& value, char * buffer, uint32_t length, ...) {
    return false;
}

template<typename T, typename = typename format_traits_t<T>::type>
static bool from_string(const char * buffer, uint32_t length, T* property, int) {
    return sscanf(buffer, format_traits_t<T>::fmt, property) == 1;
}
template<typename T = bool>
static bool from_string(const char * buffer, uint32_t length, bool* property, int) {
    int val;
    if (sscanf(buffer, "%d", &val) != 1) {
        return false;
    }
    *property = val;
    return true;
}
template<typename T>
static bool from_string(const char * buffer, uint32_t length, T* property, ...) {
    return false;
}

#endif

// ====================================================================================================
// Object tree

template<typename ... TMembers>
struct MemberList;

template<>
struct MemberList<> {
    public:
        static constexpr size_t endpoint_count = 0;
        static constexpr bool is_empty = true;

        bool write_json(size_t id, StreamSink* output) {
            // no action
            //write_string("x", output);
            return false;
        }
        void register_endpoints(Endpoint** list, size_t id, size_t length) {
            // no actions
        }
        Endpoint* get_by_name(const char * name, size_t length) {
            return nullptr;
        }
        std::tuple<> get_names_as_tuple() const { return std::tuple<>(); }
};


// ====================================================================================================
template<typename TMember, typename ... TMembers>
struct MemberList<TMember, TMembers...> {
    public:

        static constexpr uint32_t endpoint_count = TMember::endpoint_count + MemberList<TMembers...>::endpoint_count;
        static constexpr bool is_empty = false;

        MemberList(TMember&& this_member, TMembers&&... subsequent_members) :
            this_member_(std::forward<TMember>(this_member)),
            subsequent_members_(std::forward<TMembers>(subsequent_members)...) {}

        MemberList(TMember&& this_member, MemberList<TMembers...>&& subsequent_members) :
            this_member_(std::forward<TMember>(this_member)),
            subsequent_members_(std::forward<MemberList<TMembers...>>(subsequent_members)) {}

        // @brief Move constructor
    /*    MemberList(MemberList&& other) :
            this_member_(std::move(other.this_member_)),
            subsequent_members_(std::move(other.subsequent_members_)) {}*/

        bool write_json(uint32_t id, StreamSink* output) /*final*/ {
            bool needs_comma = this_member_.write_json(id, output);

            // Is this the last entry?
            if (needs_comma == true && TMember::endpoint_count != endpoint_count) {
                // Nope, so add a comma!
                write_string(",", output);
            }
            
            subsequent_members_.write_json(id + TMember::endpoint_count, output);
            return true;
        }

        Endpoint* get_by_name(const char * name, uint32_t length) {
            Endpoint* result = this_member_.get_by_name(name, length);
            if (result) return result;
            else return subsequent_members_.get_by_name(name, length);
        }

        void register_endpoints(Endpoint** list, uint32_t id, uint32_t length) /*final*/ {
            this_member_.register_endpoints(list, id, length);
            subsequent_members_.register_endpoints(list, id + TMember::endpoint_count, length);
        }

        TMember this_member_;
        MemberList<TMembers...> subsequent_members_;
};

template<typename ... TMembers>
MemberList<TMembers...> make_protocol_member_list(TMembers&&... member_list) {

    //if (member_list...) {
        return MemberList<TMembers...>(std::forward<TMembers>(member_list)...);
    //}
}


// ====================================================================================================
template<typename ... TMembers>
class ProtocolObject {
    public:
        ProtocolObject(
            const char * name, 
            TMembers&&... member_list
        ) :
            name_(name),
            member_list_(std::forward<TMembers>(member_list)...) 
        {}

        static constexpr uint32_t endpoint_count = MemberList<TMembers...>::endpoint_count;

        bool write_json(uint32_t id, StreamSink* output) {
            write_string("{\"n\":\"", output);
            write_string(name_, output);
            write_string("\",\"t\":\"ob\",\"bitsnap\":[", output);
            member_list_.write_json(id, output),
            write_string("]}", output);
            return true;
        }

        Endpoint* get_by_name(const char * name, uint32_t length) {
            uint32_t segment_length = strlen(name);
            if (!strncmp(name, name_, length))
                return member_list_.get_by_name(name + segment_length + 1, length - segment_length - 1);
            else
                return nullptr;
        }

        void register_endpoints(Endpoint** list, uint32_t id, uint32_t length) {
            member_list_.register_endpoints(list, id, length);
        }
        
        const char * name_;
        MemberList<TMembers...> member_list_;
};

template<typename ... TMembers>
ProtocolObject<TMembers...> make_protocol_object(const char * name, TMembers&&... member_list) {
    return ProtocolObject<TMembers...>(name, std::forward<TMembers>(member_list)...);
}

namespace conversion {
    template<typename T>
    bool set_from_float_ex(float value, float* property, int) {
        return *property = value, true;
    }
    template<typename T>
    bool set_from_float_ex(float value, bool* property, int) {
        return *property = (value >= 0.0f), true;
    }
    template<typename T, typename = std::enable_if_t<std::is_integral<T>::value && !std::is_const<T>::value>>
    bool set_from_float_ex(float value, T* property, int) {
        return *property = static_cast<T>(std::round(value)), true;
    }
    template<typename T>
    bool set_from_float_ex(float value, T* property, ...) {
        return false;
    }
    template<typename T>
    bool set_from_float(float value, T* property) {
        return set_from_float_ex<T>(value, property, 0);
    }
}


// ====================================================================================================
// Endpoint to handle numeric property types
//
template<typename TProperty>
class ProtocolProperty2 : public Endpoint {
    public:
        static constexpr const char * json_modifier = get_json_modifier<TProperty>();
        static constexpr size_t endpoint_count = 1;

        // Todo: implement min and max value enforcement
        // Todo: what did I mean about this?

        ProtocolProperty2(
            const char * name, 
            TProperty * property, 
            bool is_read_only = false,
            bool is_non_volatile = false,
            const char * unit = nullptr,     
            void (*written_hook)(void *) = nullptr, 
            void * written_hook_args = nullptr,
            void (*read_hook)(void *) = nullptr, 
            void * read_hook_args = nullptr,
            TProperty min_value = 0,
            TProperty max_value = 0,
            char (*selection_values)[16] = nullptr
        ) : 
            name_(name),
            property_(property), 
            is_read_only_(is_read_only), 
            is_non_volatile_(is_non_volatile),
            unit_(unit),
            written_hook_(written_hook), 
            written_hook_args_(written_hook_args),
            read_hook_(read_hook), 
            read_hook_args_(read_hook_args),
            min_value_(min_value),
            max_value_(max_value),
            selection_values_(selection_values)
        {}

    /*  TODO: find out why the move constructor is not used when it could be
        ProtocolProperty(const ProtocolProperty&) = delete;
        // @brief Move constructor
        ProtocolProperty(ProtocolProperty&& other) :
            Endpoint(std::move(other)),
            name_(std::move(other.name_)),
            property_(other.property_)
        {}
        constexpr ProtocolProperty& operator=(const ProtocolProperty& other) = delete;
        constexpr ProtocolProperty& operator=(const ProtocolProperty& other) {
            //Endpoint(std::move(other)),
            //name_(std::move(other.name_)),
            //property_(other.property_)
            name_ = other.name_;
            property_ = other.property_;
            return *this;
        }
        ProtocolProperty& operator=(ProtocolProperty&& other)
            : name_(other.name_), property_(other.property_)
        {}
        ProtocolProperty& operator=(const ProtocolProperty& other)
            : name_(other.name_), property_(other.property_)
        {}*/

        bool write_json(uint32_t id, StreamSink* output) {
            // Write name
            write_string("{\"n\":\"", output);
            LOG_FIBRE("json: this at %x, name at %x is s\r\n", (uintptr_t)this, (uintptr_t)name_);
            //LOG_FIBRE("json\r\n");
            write_string(name_, output);
            write_string("\"", output);

            // Write unit (if any)
            if(unit_ != nullptr) {
                write_string(",\"u\":\"", output);
                char unit_buf[10] = {0};

                for(uint32_t i = 0; i < 8; i++) {
                    char unit_char = unit_[i];
                    if(unit_char == 0) break;
                    unit_buf[i] = unit_char;
                }
                
                write_string(unit_buf, output);
                write_string("\"", output);
            }

            // These never apply to event_vector_t
            if constexpr(!std::is_same<TProperty, event_vector_t>::value) {

                // Write enum hints if any
                if(selection_values_ != nullptr && min_value_ != max_value_) {
                    write_string(",\"s\":[", output);

                    for(int32_t i = 0; i <= max_value_; i++) {
                        write_string("\"", output);
                        char *enum_element = selection_values_[i];
                        write_string(enum_element, output);
                        write_string("\"", output);
                        if(i != max_value_) {
                            write_string(",", output);
                        }
                    }

                    write_string("]", output);
                }
          
                // Write constraints (if any)
                if(min_value_ != max_value_) {
                    write_string(",\"c\":[", output);
                    char lim_buf[24] = {0};
                    
                    if(min_value_ < 0) {
                        write_string("-", output);
                    }
                    
                    if constexpr(std::is_same<TProperty, float>::value) {
                        // Todo: we need a ftoa here
                        snprintf(lim_buf, sizeof(lim_buf), "%.5f",  (double) min_value_);
                    } else {
                        small_itoa(lim_buf, sizeof(lim_buf), (uint32_t) abs((TProperty)min_value_));
                    }
                    
                    write_string(lim_buf, output);
                    write_string(",", output);

                    if(max_value_ < 0) {
                        write_string("-", output);
                    }

                    if constexpr(std::is_same<TProperty, float>::value) {
                        // Todo: we need a ftoa here
                        snprintf(lim_buf, sizeof(lim_buf), "%.5f",  (double) max_value_);
                    }
                    else {
                        small_itoa(lim_buf, sizeof(lim_buf), (uint32_t) abs((TProperty)max_value_));
                    }

                    write_string(lim_buf, output);
                    write_string("]", output);
                }
            }            

            // Write endpoint ID
            write_string(",\"id\":", output);
            char id_buf[10];
            //snprintf(id_buf, sizeof(id_buf), "%u", (unsigned)id); // TODO: get rid of printf
            small_itoa(id_buf, sizeof(id_buf), (unsigned)id);        
            write_string(id_buf, output);

            // Write the type of the property
            if (json_modifier && json_modifier[0]) {
                write_string(",", output);
                write_string(json_modifier, output);
            }

            // Comma
            write_string(",", output);

            // Write permissions
            if(is_read_only_){
                write_string("\"a\":\"r\"", output);
            }
            else {
                write_string("\"a\":\"rw\"", output);
            }

            // Comma
            write_string(",", output);

            // Write volatility
            if(is_non_volatile_){
                write_string("\"v\":\"n\"", output);
            }
            else {
                write_string("\"v\":\"y\"", output);
            }

            write_string("}", output);
            return true;
        }

        // special-purpose function - to be moved
        Endpoint* get_by_name(const char * name, uint32_t length) {
            if (!strncmp(name, name_, length)) {
                return this;
            }
            else {
                return nullptr;
            }
        }

        #ifdef ENABLE_ASCII_PROTOCOL
        // special-purpose function - to be moved
        bool get_string(char * buffer, uint32_t length) final {
            return to_string(*property_, buffer, length, 0);
        }

        // special-purpose function - to be moved
        bool set_string(char * buffer, uint32_t length) final {
            return from_string(buffer, length, property_, 0);
        }

        bool set_from_float(float value) final {
            return conversion::set_from_float(value, property_);
        }
        #endif

        void register_endpoints(Endpoint** list, uint32_t id, uint32_t length) {
            if (id < length) {
                list[id] = this;
            }
        }
        void handle(const uint8_t* input, uint32_t input_length, StreamSink* output) final {

            bool wrote = flexible_readwrite_endpoint_handler<TProperty>(
                property_, 
                min_value_, 
                max_value_, 
                is_read_only_,
                input, 
                input_length, 
                output);
            
            // Request callback
            if (wrote && written_hook_ != nullptr) {
                // Was it a write request?
                written_hook_(written_hook_args_);
            }
            else if(read_hook_ != nullptr) {
                // No, was read request
                read_hook_(read_hook_args_);
            }
        }

        const char* name_;
        TProperty* property_;
        bool is_read_only_;
        bool is_non_volatile_;
        const char* unit_;
        void (*written_hook_)(void*);
        void* written_hook_args_;
        void (*read_hook_)(void*);
        void* read_hook_args_;
        TProperty min_value_;
        TProperty max_value_;
        char (*selection_values_)[16];
};
// ##############################



// ====================================================================================================
// Endpoint to handle string types
//
template<typename Tstring>
class ProtocolString : public Endpoint {
    public:
        static constexpr size_t endpoint_count = 1;

        ProtocolString(
            const char * name, 
            Tstring* string, 
            uint32_t string_length, 
            bool is_read_only,
            bool is_non_volatile,
            const char * unit, 
            void (*written_hook)(void*), 
            void* written_hook_args, 
            void (*read_hook)(void*), 
            void* read_hook_args,
            string_type_t string_type
        ) : 
            name_(name),
            string_(string),
            string_length_(string_length),
            is_read_only_(is_read_only),
            is_non_volatile_(is_non_volatile),
            unit_(unit),
            written_hook_(written_hook),
            written_hook_args_(written_hook_args),
            read_hook_(read_hook),
            read_hook_args_(read_hook_args),
            string_type_(string_type)
        {}

        bool write_json(uint32_t id, StreamSink* output) {

            // write name
            write_string("{\"n\":\"", output);
            LOG_FIBRE("json: this at %x, name at %x is s\r\n", (uintptr_t)this, (uintptr_t)name_);
            //LOG_FIBRE("json\r\n");
            write_string(name_, output);

            // Write unit (if any)
            if(unit_ != nullptr) {
                write_string("\",\"u\":\"", output);
                char unit_buf[10] = {0};

                for(uint32_t i = 0; i < 8; i++) {
                    char unit_char = unit_[i];
                    if(unit_char == 0) break;
                    unit_buf[i] = unit_char;
                }
                
                write_string(unit_buf, output);
            }

            // write endpoint ID
            write_string("\",\"id\":", output);
            char id_buf[10]; 
            small_itoa(id_buf, sizeof(id_buf), (unsigned)id);        
            write_string(id_buf, output);

            // Comma
            write_string(",", output);

            // Is this a string or a protocol buffer?
            if(string_type_ == NULL_TERMINATED_STRING) {
                // Is null-terminated string
                write_string("\"t\":\"s\"", output);
            }
            else {
                // Is fixed-size protocol buffer
                write_string("\"t\":\"c*\"", output);
            }

            // Comma
            write_string(",", output);

            // Write permissions
            if(is_read_only_){
                write_string("\"a\":\"r\"", output);
            }
            else {
                write_string("\"a\":\"rw\"", output);
            }

            // Comma
            write_string(",", output);

            // Write volatility
            if(is_non_volatile_){
                write_string("\"v\":\"n\"", output);
            }
            else {
                write_string("\"v\":\"y\"", output);
            }

            // Write string length
            write_string(",\"l\":", output);
            char len_buf[10]; // 2^32 = 10 decimal places
            small_itoa(len_buf, sizeof(len_buf), (unsigned) string_length_); 
            write_string(len_buf, output);

            write_string("}", output);
            return true;
        }

        // special-purpose function - to be moved
        Endpoint* get_by_name(const char * name, uint32_t length) {
            if (!strncmp(name, name_, length)) {
                return this;
            }
            else {
                return nullptr;
            }
        }

        #ifdef ENABLE_ASCII_PROTOCOL
        // special-purpose function - to be moved
        bool get_string(char * buffer, uint32_t length) final {
            //return to_string(*string_, buffer, length, 0);
            //return snprintf(buffer, length, "%s", *string_);
            return memcpy(buffer, string_, length);
        }

        // special-purpose function - to be moved
        bool set_string(char * buffer, uint32_t length) final {
            //return from_string(buffer, length, string_, 0);
            return sscanf(buffer, "%s", string_) == 1;
        }

        bool set_from_float(float value) final {
            return conversion::set_from_float(value, string_);
        }
        #endif

        void register_endpoints(Endpoint** list, uint32_t id, uint32_t length) {
            if (id < length)
                list[id] = this;
        }

        void handle(const uint8_t* input, uint32_t input_length, StreamSink* output) final {
            bool wrote = string_readwrite_endpoint_handler<Tstring>(
                string_,
                string_length_, 
                is_read_only_, 
                input, 
                input_length, 
                output
            );
            
            // Was this a write request?
            if (wrote && written_hook_ != nullptr) {
                // Yes, call write callback if any
                written_hook_(written_hook_args_);
            }
            else if(read_hook_ != nullptr) {
                // No, was read request
                read_hook_(read_hook_args_);
            }
        }

        const char* name_;
        Tstring* string_;
        uint32_t string_length_;
        bool is_read_only_;
        bool is_non_volatile_;
        const char * unit_;
        void (*written_hook_)(void*);
        void* written_hook_args_;
        void (*read_hook_)(void*);
        void* read_hook_args_;        
        string_type_t string_type_;
};

// ====================================================================================================
// These keys are used to define the keywordArgs we use in creating a protocol property
// The property name and enumerated choice must be a bijective map or stuff will break
// 
enum Keys {
  tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7,
  tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15
};

// global symbols used as keys in list of kwargs
static kw::Key<tag_0> property_name;
static kw::Key<tag_1> property_length;
static kw::Key<tag_2> property_is_read_only;
static kw::Key<tag_3> property_units;
static kw::Key<tag_4> property_min_value;
static kw::Key<tag_5> property_max_value;
static kw::Key<tag_6> after_written_callback; 
static kw::Key<tag_7> after_written_callback_args; 
static kw::Key<tag_8> after_read_callback; 
static kw::Key<tag_9> after_read_callback_args; 
static kw::Key<tag_10> property_option_strings; 
static kw::Key<tag_11> property_option_count; 
static kw::Key<tag_12> property_is_non_volatile;
//static kw::Key<tag_13> function_name; 
static kw::Key<tag_14> function_arguments; 


#define ONLY_KW_ARGS

// ====================================================================================================
// Event trigger types
//
static event_vector_t NULL_EVENT_VECTOR = {0};

/*
template<typename event_vector_t>

    ProtocolProperty<event_vector_t> make_event_trigger(
        const char * name, 
        event_vector_t * property, 
        void (*written_hook)(void*) = nullptr, 
        void* written_hook_args = nullptr,
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr
    ) {

        return ProtocolProperty<event_vector_t>(
            name, 
            property, 
            nullptr,
            written_hook, 
            written_hook_args,
            read_hook,
            read_hook_args,
            NULL_EVENT_VECTOR, 
            NULL_EVENT_VECTOR
        );
    };
*/ 

// KeywordArgs version -------------------------
template <typename event_vector_t, typename ...Args>

    ProtocolProperty2<event_vector_t> make_event_trigger(
        event_vector_t * property_kw,
        Args... kwargs
    ) {

        // First, we construct the parameter pack from the parameter pack
        kw::ParamPack<Args...> params(kwargs...);

        // Now attempt to extract the keyword arguments
        const char * property_name_kw   = kw::Get(params, property_name, "");
        void (*written_hook_kw)(void*)  = kw::Get(params, after_written_callback, nullptr);
        void* written_hook_args_kw      = kw::Get(params, after_written_callback_args, nullptr);
        void (*read_hook_kw)(void*)     = kw::Get(params, after_read_callback, nullptr);
        void* read_hook_args_kw         = kw::Get(params, after_read_callback_args, nullptr);
        bool is_non_volatile_kw         = kw::Get(params, property_is_non_volatile, false);

        // Now, feed parameters (or default values) into ProtocolProperty
        return ProtocolProperty2<event_vector_t>(
            property_name_kw, 
            property_kw, 
            false,
            is_non_volatile_kw,
            nullptr,
            written_hook_kw, 
            written_hook_args_kw,
            read_hook_kw,
            read_hook_args_kw,
            NULL_EVENT_VECTOR, 
            NULL_EVENT_VECTOR,
            nullptr
        );
    };


// ====================================================================================================
// Non-const non-enum types
//
#ifndef ONLY_KW_ARGS
    template<typename TProperty, 
        ENABLE_IF(
            !std::is_enum<TProperty>::value
        )>

        ProtocolProperty<TProperty> make_protocol_number(
            const char * property_name, 
            TProperty * property, 
            const char * unit = nullptr, 
            TProperty min_value = 0,
            TProperty max_value = 0,
            void (*written_hook)(void*) = nullptr, 
            void* written_hook_args = nullptr,
            void (*read_hook)(void*) = nullptr, 
            void* read_hook_args = nullptr
        ) {
    
            return ProtocolProperty<TProperty>(
                property_name, 
                property, 
                unit, 
                written_hook, 
                written_hook_args,
                read_hook,
                read_hook_args,
                min_value,
                max_value
            );
        };
#endif

// KeywordArgs version -------------------------
template <typename TProperty, typename ...Args,  
    ENABLE_IF(
        !std::is_enum<TProperty>::value
    )>

    ProtocolProperty2<TProperty>  make_protocol_number_kw(
        TProperty * property_kw,
        Args... kwargs
    ) {

    // First, we construct the parameter pack from the parameter pack
    kw::ParamPack<Args...> params(kwargs...);

    // Now attempt to extract the keyword arguments
    const char * property_name_kw   = kw::Get(params, property_name, "");
    const char * unit_kw            = kw::Get(params, property_units, nullptr);
    void (*written_hook_kw)(void*)  = kw::Get(params, after_written_callback, nullptr);
    void* written_hook_args_kw      = kw::Get(params, after_written_callback_args, nullptr);
    void (*read_hook_kw)(void*)     = kw::Get(params, after_read_callback, nullptr);
    void* read_hook_args_kw         = kw::Get(params, after_read_callback_args, nullptr);
    TProperty min_value_kw          = kw::Get(params, property_min_value, 0);
    TProperty max_value_kw          = kw::Get(params, property_max_value, 0);
    bool is_read_only_kw            = kw::Get(params, property_is_read_only, false);
    bool is_non_volatile_kw         = kw::Get(params, property_is_non_volatile, false);

    // Now, feed parameters (or default values) into ProtocolProperty
    return ProtocolProperty2<TProperty>(
        property_name_kw, 
        property_kw, 
        is_read_only_kw,
        is_non_volatile_kw,
        unit_kw, 
        written_hook_kw, 
        written_hook_args_kw,
        read_hook_kw,
        read_hook_args_kw,
        min_value_kw,
        max_value_kw,
        nullptr
    );
}

#ifndef ONLY_KW_ARGS
// Const non-enum types
template<typename TProperty, 
    ENABLE_IF(
        !std::is_enum<TProperty>::value
    )>
    ProtocolProperty<const TProperty> make_protocol_ro_number(
        const char * property_name, 
        TProperty* property, 
        const char * unit = nullptr, 
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr    
    ) {
        return ProtocolProperty<const TProperty>(
            property_name, 
            property, 
            unit, 
            nullptr, 
            nullptr,
            read_hook,
            read_hook_args,
            0, 0
        );
    };
#endif

// ====================================================================================================
// Selection types
//
#ifndef ONLY_KW_ARGS
template<typename TProperty>

    ProtocolProperty2<TProperty> make_protocol_selection(
        const char * property_name, 
        TProperty * property, 
        const char * unit = nullptr, 
        char (*selection_values)[16] = nullptr,
        uint8_t max_value = 0,
        void (*written_hook)(void*) = nullptr, 
        void* written_hook_args = nullptr,
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr
    ) {
 
        return ProtocolProperty2<TProperty>(
            property_name, 
            property, 
            false,
            unit,
            written_hook, 
            written_hook_args,
            read_hook,
            read_hook_args,
            0,
            max_value - 1, // Maximum selection index
            selection_values
        );
    };
#endif

// KeywordArgs version -------------------------
template<typename TProperty, typename ...Args>

    ProtocolProperty2<TProperty> make_protocol_selection_kw(
        TProperty * property_kw,
        Args... kwargs
    ) {

    // First, we construct the parameter pack from the parameter pack
    kw::ParamPack<Args...> params(kwargs...);

    // Now attempt to extract the keyword arguments
    const char * property_name_kw   = kw::Get(params, property_name, "");
    const char * unit_kw            = kw::Get(params, property_units, nullptr);
    void (*written_hook_kw)(void*)  = kw::Get(params, after_written_callback, nullptr);
    void* written_hook_args_kw      = kw::Get(params, after_written_callback_args, nullptr);
    void (*read_hook_kw)(void*)     = kw::Get(params, after_read_callback, nullptr);
    void* read_hook_args_kw         = kw::Get(params, after_read_callback_args, nullptr);
    uint16_t max_value_kw           = kw::Get(params, property_option_count, 0);
    char (*selection_values_kw)[16] = kw::Get(params, property_option_strings, 0);
    bool is_read_only_kw            = kw::Get(params, property_is_read_only, false);
    bool is_non_volatile_kw         = kw::Get(params, property_is_non_volatile, false);

    // Now, feed parameters (or default values) into ProtocolProperty
    return ProtocolProperty2<TProperty>(
        property_name_kw, 
        property_kw, 
        is_read_only_kw,
        is_non_volatile_kw,
        unit_kw,
        written_hook_kw, 
        written_hook_args_kw,
        read_hook_kw,
        read_hook_args_kw,
        0,
        max_value_kw - 1, // Maximum selection index
        selection_values_kw
    );
};

#ifndef ONLY_KW_ARGS
// Const selection types
template<typename TProperty>

    ProtocolProperty<const TProperty> make_protocol_ro_selection(
        const char * property_name, 
        TProperty* property, 
        const char * unit = nullptr, 
        char (*selection_values)[16] = nullptr,
        uint8_t max_value = 0,
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr 
    ) {
        return ProtocolProperty<const TProperty>(
            property_name, 
            property, 
            unit, 
            nullptr, 
            nullptr,
            read_hook,
            read_hook_args,
            0, 
            max_value - 1, // Maximum selection index
            selection_values
        );
    };
#endif 

// ====================================================================================================
// Non-const enum types
//
#ifndef ONLY_KW_ARGS
template<typename TProperty, 
    ENABLE_IF(
        std::is_enum<TProperty>::value)
    >
    ProtocolProperty<std::underlying_type_t<TProperty>> make_protocol_number(
        const char * property_name, 
        TProperty * property, 
        const char * unit = nullptr, 
        void (*written_hook)(void*) = nullptr, 
        void* written_hook_args = nullptr,
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr
    ) {

        return ProtocolProperty<std::underlying_type_t<TProperty>>(
            property_name, 
            reinterpret_cast<std::underlying_type_t<TProperty>*>(property), 
            unit, 
            written_hook, 
            written_hook_args,
            read_hook,
            read_hook_args
        );
    };
#endif

// KeywordArgs version -------------------------
template<typename TProperty, typename ...Args,
    ENABLE_IF(
        std::is_enum<TProperty>::value)
    >
    ProtocolProperty2<std::underlying_type_t<TProperty>> make_protocol_number_kw(
        TProperty * property_kw,
        Args... kwargs
    ) {

        // First, we construct the parameter pack from the parameter pack
        kw::ParamPack<Args...> params(kwargs...);

        // Now attempt to extract the keyword arguments
        const char * property_name_kw   = kw::Get(params, property_name, "");
        const char * unit_kw            = kw::Get(params, property_units, nullptr);
        void (*written_hook_kw)(void*)  = kw::Get(params, after_written_callback, nullptr);
        void* written_hook_args_kw      = kw::Get(params, after_written_callback_args, nullptr);
        void (*read_hook_kw)(void*)     = kw::Get(params, after_read_callback, nullptr);
        void* read_hook_args_kw         = kw::Get(params, after_read_callback_args, nullptr);
        bool is_read_only_kw            = kw::Get(params, property_is_read_only, false);
        bool is_non_volatile_kw         = kw::Get(params, property_is_non_volatile, false);

        // Now, feed parameters (or default values) into ProtocolProperty
        return ProtocolProperty2<std::underlying_type_t<TProperty>>(
            property_name_kw, 
            reinterpret_cast<std::underlying_type_t<TProperty>*>(property_kw), 
            is_read_only_kw,
            is_non_volatile_kw,
            unit_kw, 
            written_hook_kw, 
            written_hook_args_kw,
            read_hook_kw,
            read_hook_args_kw
        );
    };

#ifndef ONLY_KW_ARGS
// Const enum types
template<typename TProperty, 
    ENABLE_IF(
        std::is_enum<TProperty>::value
    )>
    ProtocolProperty<const std::underlying_type_t<TProperty>> make_protocol_ro_number(
        const char * property_name, 
        TProperty* property, 
        const char * unit = nullptr, 
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr    
    ) {
        return ProtocolProperty<const std::underlying_type_t<TProperty>>(
            property_name, 
            reinterpret_cast<const std::underlying_type_t<TProperty>*>(property), 
            unit, 
            nullptr, 
            nullptr,
            read_hook,
            read_hook_args
        );
    };
#endif 

// ====================================================================================================
// Strings
//
#ifndef ONLY_KW_ARGS
template<typename TString>

    ProtocolString<TString> make_protocol_string(
        const char * name, 
        TString * property, 
        uint32_t string_length,
        const char * unit = nullptr, 
        void (*written_hook)(void*) = nullptr, 
        void* written_hook_args = nullptr, 
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr
    ) {
        return ProtocolString<TString>(
            name, 
            property, 
            string_length, 
            false,
            unit, 
            written_hook, 
            written_hook_args, 
            read_hook, 
            read_hook_args, 
            NULL_TERMINATED_STRING
        );
    };
#endif

// KeywordArgs version -------------------------
template<typename TString, typename ...Args>

    ProtocolString<TString> make_protocol_string_kw(
        TString * property_kw,
        Args... kwargs
    ) {

        // First, we construct the parameter pack from the parameter pack
        kw::ParamPack<Args...> params(kwargs...);

        // Now attempt to extract the keyword arguments
        const char * property_name_kw   = kw::Get(params, property_name, "");
        uint32_t property_length_kw     = kw::Get(params, property_length, 0);
        const char * unit_kw            = kw::Get(params, property_units, nullptr);
        void (*written_hook_kw)(void*)  = kw::Get(params, after_written_callback, nullptr);
        void* written_hook_args_kw      = kw::Get(params, after_written_callback_args, nullptr);
        void (*read_hook_kw)(void*)     = kw::Get(params, after_read_callback, nullptr);
        void* read_hook_args_kw         = kw::Get(params, after_read_callback_args, nullptr);
        bool is_read_only_kw            = kw::Get(params, property_is_read_only, false);
        bool is_non_volatile_kw         = kw::Get(params, property_is_non_volatile, false);

        // Now, feed parameters (or default values) into ProtocolString
        return ProtocolString<TString>(
            property_name_kw, 
            property_kw,
            property_length_kw, 
            is_read_only_kw,
            is_non_volatile_kw,
            unit_kw, 
            written_hook_kw, 
            written_hook_args_kw,
            read_hook_kw,
            read_hook_args_kw,
            NULL_TERMINATED_STRING
        );
    };

#ifndef ONLY_KW_ARGS
// Const string types
template<typename TString>

    ProtocolString<TString> make_protocol_ro_string(
        const char * property_name, 
        TString * property, 
        uint32_t string_length, 
        const char * unit = nullptr, 
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr
    ) {
        return ProtocolString<TString>(
            property_name, 
            property, 
            string_length, 
            true,
            unit, 
            nullptr, 
            nullptr, 
            read_hook, 
            read_hook_args, 
            NULL_TERMINATED_STRING
        );
    };
#endif

// ====================================================================================================
// String Buffer (non-null-terminated, fixed length string)
//
#ifndef ONLY_KW_ARGS
template<typename TString>
    ProtocolString<TString> make_protocol_buffer(
        const char * name, 
        TString * property, 
        uint32_t string_length, 
        const char * unit = nullptr, 
        void (*written_hook)(void*) = nullptr, 
        void* written_hook_args = nullptr, 
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr
    ) {
        return ProtocolString<TString>(
            name, 
            property, 
            string_length, 
            false,
            unit, 
            written_hook, 
            written_hook_args, 
            read_hook, 
            read_hook_args, 
            FIXED_SIZE_BUFFER
        );
    };
#endif

// KeywordArgs version -------------------------
template<typename TString, typename ...Args>

    ProtocolString<TString> make_protocol_buffer_kw(
        TString * property_kw, 
        Args... kwargs
    ) {

        // First, we construct the parameter pack from the parameter pack
        kw::ParamPack<Args...> params(kwargs...);

        // Now attempt to extract the keyword arguments
        const char * property_name_kw   = kw::Get(params, property_name, "");
        uint32_t property_length_kw     = kw::Get(params, property_length, nullptr);
        const char * unit_kw            = kw::Get(params, property_units, nullptr);
        void (*written_hook_kw)(void*)  = kw::Get(params, after_written_callback, nullptr);
        void* written_hook_args_kw      = kw::Get(params, after_written_callback_args, nullptr);
        void (*read_hook_kw)(void*)     = kw::Get(params, after_read_callback, nullptr);
        void* read_hook_args_kw         = kw::Get(params, after_read_callback_args, nullptr);
        bool is_read_only_kw            = kw::Get(params, property_is_read_only, false);
        bool is_non_volatile_kw         = kw::Get(params, property_is_non_volatile, false);

        // Now, feed parameters (or default values) into ProtocolString
        return ProtocolString<TString>(
            property_name_kw, 
            property_kw,
            property_length_kw, 
            is_read_only_kw,
            is_non_volatile_kw,
            unit_kw, 
            written_hook_kw, 
            written_hook_args_kw,
            read_hook_kw,
            read_hook_args_kw,
            FIXED_SIZE_BUFFER
        );
    };

#ifndef ONLY_KW_ARGS
// Const buffer types
template<typename TString>

    ProtocolString<const TString> make_protocol_ro_buffer(
        const char * name, 
        TString * property, 
        uint32_t string_length, 
        const char * unit = nullptr, 
        void (*written_hook)(void*) = nullptr, 
        void* written_hook_args = nullptr, 
        void (*read_hook)(void*) = nullptr, 
        void* read_hook_args = nullptr
    ) {
        return ProtocolString<const TString>(
            name, 
            property, 
            string_length, 
            true,
            unit, 
            written_hook, 
            written_hook_args, 
            read_hook, 
            read_hook_args, 
            FIXED_SIZE_BUFFER
        );
    };
#endif

// ====================================================================================================
// Function input generators
//
template<typename ... TArgs>
struct PropertyListFactory;

template<>
struct PropertyListFactory<> {
    template<unsigned IPos, typename ... TAllProperties>
    static MemberList<> make_property_list(
        std::array<const char *, 
        sizeof...(TAllProperties)> names, 
        std::tuple<TAllProperties...>& values
    ) {
        return MemberList<>();
    }
};


#ifndef ONLY_KW_ARGS
template<typename TProperty, typename ... TProperties>
struct PropertyListFactory<TProperty, TProperties...> {
    template<unsigned IPos, typename ... TAllProperties>
    static MemberList<ProtocolProperty<TProperty>, ProtocolProperty<TProperties>...>
    make_property_list(
        std::array<const char *, 
        sizeof...(TAllProperties)> names, 
        std::tuple<TAllProperties...>& values
        ) {
        return MemberList<ProtocolProperty<TProperty>, ProtocolProperty<TProperties>...>(
            make_protocol_number(std::get<IPos>(names), 
            &std::get<IPos>(values)),
            PropertyListFactory<TProperties...>::template make_property_list<IPos+1>(names, values)
        );
    }
};
#else
template<typename TProperty, typename ... TProperties>
struct PropertyListFactory<TProperty, TProperties...> {
    template<unsigned IPos, typename ... TAllProperties>
    static MemberList<ProtocolProperty2<TProperty>, ProtocolProperty2<TProperties>...>
    make_property_list(
        std::array<const char *, 
        sizeof...(TAllProperties)> names, 
        std::tuple<TAllProperties...>& values
        ) {
        return MemberList<ProtocolProperty2<TProperty>, ProtocolProperty2<TProperties>...>(
           
            make_protocol_number_kw(&std::get<IPos>(values), property_name = std::get<IPos>(names)), 
            PropertyListFactory<TProperties...>::template make_property_list<IPos+1>(names, values)
        );
    }
};
#endif

// ===============================================================
// Iterator for tuples of arbitrary length
// https://codereview.stackexchange.com/questions/51407/stdtuple-foreach-implementation

template <typename Tuple, typename F, std::size_t ...Indices>
void for_each_impl(Tuple && tuple, F&& f, std::index_sequence<Indices...>) {
    using swallow = int[];

    (void)swallow{ 1,
        (f(std::get<Indices>(std::forward<Tuple>(tuple))), void(), int{})...
    };
}
template<typename F, typename... Args>
void tuple_for_each(std::tuple<Args...>& tuple, F&& f)
{
    for_each_impl(tuple, std::forward<F>(f), std::index_sequence_for<Args...>{});
}


// ===============================================================
// Iterator for tuples of arbitrary length, with associated array
// of names
//
template <typename Tuple, typename F, std::size_t ...Indices>
void for_each_impl_names(Tuple && tuple, F&& f, std::array<const char *, sizeof...(Indices)> array, std::index_sequence<Indices...>) {
    using swallow = int[];

    (void)swallow{ 1,
        (f(std::get<Indices>(std::forward<Tuple>(tuple)), std::get<Indices>(array)), void(), int{})...
    };
}

template<typename F, typename... Args>
void tuple_for_each_with_names(std::tuple<Args...>& tuple, std::array<const char *, sizeof...(Args)> array, F&& f)
{
    for_each_impl_names(tuple, std::forward<F>(f), array, std::index_sequence_for<Args...>{});
}


/* @brief return_type<TypeList>::type represents the true return type
* of a function returning 0 or 1 arguments.
*
* For an empty TypeList, the return type is void. For a list with
* one type, the return type is equal to that type.
*/
template<typename ... Types>
struct return_type;

template<>
struct return_type<> { typedef void type; };

template<typename T>
struct return_type<T> { typedef T type; };

template<typename TObj, typename TReturnType, typename ... TInputsAndOutputs>
class ProtocolFunction;


// If you see this comment in the compiler that means you passed
// an unsupported return type to ProtocolFunction
template <auto A, typename...> auto function_retval_assert_helper = A;

template<typename TObj, typename TReturnType, typename ... TInputs>

class ProtocolFunction<TObj, TReturnType, std::tuple<TInputs...>> : Endpoint {
    public:
        // @brief The return type of the function as written by a C++ programmer

        static constexpr size_t endpoint_count = 1 + MemberList<ProtocolProperty2<TInputs>...>::endpoint_count;

        ProtocolFunction(
            const char * name, 
            TObj& obj, 
            TReturnType(TObj::*func_ptr)(TInputs...),
            std::array<const char *, sizeof...(TInputs)> input_names
        ) :
            name_(name), 
            obj_(&obj), 
            func_ptr_(func_ptr),
            input_names_{input_names}, 
            input_properties_(PropertyListFactory<TInputs...>::template make_property_list<0>(input_names_, in_args_))
        {
            LOG_FIBRE("my tuple is at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
        }

        // The custom copy constructor is needed because otherwise the
        // input_properties_ and output_properties_ would point to memory
        // locations of the old object.
        ProtocolFunction(const ProtocolFunction& other) :
            name_(other.name_), 
            obj_(other.obj_), 
            func_ptr_(other.func_ptr_),
            input_names_{other.input_names_}, 
            input_properties_(PropertyListFactory<TInputs...>::template make_property_list<0>(input_names_, in_args_))
        {
            LOG_FIBRE("COPIED! my tuple is at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
        }

        bool write_json(uint32_t id, StreamSink* output) {
            // write name
            write_string("{\"n\":\"", output);
            write_string(name_, output);

            // write endpoint ID
            write_string("\",\"id\":", output);
            char id_buf[10];
            //snprintf(id_buf, sizeof(id_buf), "%u", (unsigned)id); // TODO: get rid of printf
            small_itoa(id_buf, sizeof(id_buf), (unsigned)id);        
            write_string(id_buf, output);
            
            // Write attribute type (function)
            write_string(",\"t\":\"fn\"", output);

            // Write input arguments (v2)
            write_string(",\"i\":[", output);

            // Iterate input arguments tuple and print JSON that describes the 
            // type of each argument
            uint8_t index = 0;

            tuple_for_each_with_names(in_args_, input_names_, [&output, &index](auto in_arg, auto input_name) {

                // Open input dict
                write_string("{\"t\":\"", output);

                // Print type
                if constexpr(std::is_same<decltype(in_arg), uint8_t>::value) {
                    write_string("u8", output);
                }
                else if constexpr(std::is_same<decltype(in_arg), int32_t>::value) {
                    write_string("i32", output);
                }
                else if constexpr(std::is_same<decltype(in_arg), int64_t>::value) {
                    write_string("i64", output);
                }
                else if constexpr(std::is_same<decltype(in_arg), float>::value) {
                    write_string("f", output);
                }
                else if constexpr(std::is_same<decltype(in_arg), double>::value) {
                    write_string("d", output);
                }

                write_string("\"", output);

                write_string(",\"n\":\"", output);
                //LOG_FIBRE("json: this at %x, name at %x is s\r\n", (uintptr_t)this, (uintptr_t)name_);
                write_string(input_name, output);
                write_string("\"", output);

                // Close input dict
                write_string("}", output);

                // Increment tuple index
                index++;

                // Trailing comma if not the last member
                if (index < std::tuple_size<decltype(in_args_)>::value) {
                    write_string(",", output);
                }
            });

            // Close off input type list
            write_string("]", output);

            // Write output arguments
            
            // Any return types at all?
            if constexpr(!std::is_same<TReturnType, void>::value) {

                // Return data type
                write_string(",\"o\":[{\"t\":\"", output);
                
                // Supported function return types
                if constexpr(std::is_same<TReturnType, int32_t>::value) {
                    write_string("i32", output);
                }
                else if constexpr(std::is_same<TReturnType, int64_t>::value) {
                    write_string("i64", output);
                }
                else if constexpr(std::is_same<TReturnType, float>::value) {
                    write_string("f", output);
                }
                else if constexpr(std::is_same<TReturnType, double>::value) {
                    write_string("d", output);
                }
                else {
                    static_assert(function_retval_assert_helper<false, TReturnType>, "Function return type is unsupported");
                }
            
                write_string("\"}]", output);
            }

            write_string("}", output);

            return true;
        }

        // special-purpose function - to be moved
        Endpoint* get_by_name(const char * name, uint32_t length) {
            return nullptr; // can't address functions by name
        }

        void register_endpoints(Endpoint** list, uint32_t id, uint32_t length) {
            if (id < length)
                list[id] = this;
            input_properties_.register_endpoints(list, id + 1, length);
        }

        void handle(const uint8_t* input, uint32_t input_length, StreamSink* output) final {
            (void) input;
            (void) input_length;
            (void) output;
            
            LOG_FIBRE("tuple still at %x and of size %u\r\n", (uintptr_t)&in_args_, sizeof(in_args_));
            LOG_FIBRE("invoke function using %d and %.3f\r\n", std::get<0>(in_args_), std::get<1>(in_args_));
            
            // Iterate over the input arguments and deserialize them
            uint32_t offset = 0;

            tuple_for_each(in_args_, [&output, &offset, &input, &input_length](auto & in_arg) {

                // What is the type of this value?
                using TEType = typename std::remove_reference<decltype(in_arg)>::type;

                // Get plausible value from buffer at specified offset 
                TEType value;
                read_le<TEType>(&value, (input + offset));

                // Assign value to input
                in_arg = value;

                // What is the new buffer index?
                offset += sizeof(TEType);
            });
            
            // Supress compiler warnings if offset is not used
            (void) offset;

            // Does this function have a return value?
            if constexpr(std::is_same<TReturnType, void>::value) {
                // No it does not, invoke it, then forget it
                invoke_function_with_tuple(*obj_, func_ptr_, in_args_);
            }
            else {
                // Yes it does! Invoke it and save its return value.
                TReturnType out_args = invoke_function_with_tuple(*obj_, func_ptr_, in_args_);

                // Was a response expected?
                if (output) {
                
                    // Make buffer size dependent on the type
                    uint8_t buffer[sizeof(TReturnType)];
                    uint32_t cnt = write_le<TReturnType>(out_args, buffer);
                    
                    // Send return value
                    if (cnt <= output->get_free_space()) {
                        output->process_bytes(buffer, cnt, nullptr);
                    }
                }
            }
        }

        const char * name_;
        TObj* obj_;
        TReturnType(TObj::*func_ptr_)(TInputs...);
        std::array<const char *, sizeof...(TInputs)> input_names_; // TODO: remove
        std::tuple<TInputs...> in_args_;
        MemberList<ProtocolProperty2<TInputs>...> input_properties_;
};


// ====================================================================================================
// Functions returning void
//
#ifndef ONLY_KW_ARGS
template<typename TObj, typename ... TArgs, typename ... TNames,
    typename = std::enable_if_t<
        sizeof...(TArgs) == sizeof...(TNames)>
    >
    ProtocolFunction<TObj, void, std::tuple<TArgs...>> 
    
    make_protocol_function_explicit(
        const char * name, 
        TObj& obj, 
        void(TObj::*func_ptr)(TArgs...), 
        TNames ... names
        ) {
        return ProtocolFunction<TObj, void, std::tuple<TArgs...>>(
            name, 
            obj, 
            func_ptr, 
            {names...}
        );
    }
#endif

// KeywordArgs version -------------------------
template<typename TObj, typename ... TArgs, typename... Args>

    ProtocolFunction<TObj, void, std::tuple<TArgs...>> 
    
    make_protocol_function_kw(
            TObj& function_obj, 
            void(TObj::*func_ptr)(TArgs...), 
            Args ... kwargs
        ) {

        // First, we construct the parameter pack from the parameter pack
        kw::ParamPack<Args...> params(kwargs...);

        // Now attempt to extract the keyword arguments
        const char * function_name_kw   = kw::Get(params, property_name, "");
        auto argument_names             = kw::Get(params, function_arguments, std::array<const char *, 0>{});
        
        // Now, feed parameters (or default values) into ProtocolProperty
        return ProtocolFunction<TObj, void, std::tuple<TArgs...>>(
            function_name_kw, 
            function_obj, 
            func_ptr, 
            argument_names
        );
    }

// ====================================================================================================
// Functions returning TReturnType
//
#ifndef ONLY_KW_ARGS
template<typename TObj, typename TReturnType, typename ... TArgs, typename ... TNames, 
    typename = std::enable_if_t<
        sizeof...(TArgs) == sizeof...(TNames) 
        && !std::is_void<TReturnType>::value>
    >
    
    ProtocolFunction<TObj, TReturnType, std::tuple<TArgs...>> 
    
    make_protocol_function_explicit(
        const char * name, 
        TObj& obj, 
        TReturnType(TObj::*func_ptr)(TArgs...), 
        TNames ... names
        ) {
        return ProtocolFunction<TObj, TReturnType, std::tuple<TArgs...>>(
            name, 
            obj, 
            func_ptr, 
            {names...}//, 
            //{"result"}
        );
    }
#endif

// KeywordArgs version -------------------------
template<typename TObj, typename TReturnType, typename ... TArgs, typename... Args,
    typename = std::enable_if_t<
        !std::is_void<TReturnType>::value>
    >

    ProtocolFunction<TObj, TReturnType, std::tuple<TArgs...>> 
    
    make_protocol_function_kw(
            TObj& function_obj, 
            TReturnType(TObj::*func_ptr)(TArgs...), 
            Args ... kwargs
        ) {

        // First, we construct the parameter pack from the parameter pack
        kw::ParamPack<Args...> params(kwargs...);

        // Now attempt to extract the keyword arguments
        const char * function_name_kw   = kw::Get(params, property_name, "");
        auto argument_names = kw::Get(params, function_arguments, std::array<const char *, 0>{});
        
        // Now, feed parameters (or default values) into ProtocolProperty
        return ProtocolFunction<TObj, TReturnType, std::tuple<TArgs...>>(
            function_name_kw, 
            function_obj, 
            func_ptr, 
            argument_names
        );
    }


// ====================================================================================================
//
class EndpointProvider {
    public:
        virtual size_t get_endpoint_count() = 0;
        virtual bool write_json(uint32_t id, StreamSink* output) = 0;
        virtual Endpoint* get_by_name(char * name, uint32_t length) = 0;
        virtual void register_endpoints(Endpoint** list, uint32_t id, uint32_t length) = 0;
};

template<typename T>
class EndpointProvider_from_MemberList : public EndpointProvider {
    public:
        EndpointProvider_from_MemberList(T& member_list) : member_list_(member_list) {}
        size_t get_endpoint_count() final {
            return T::endpoint_count;
        }
        bool write_json(uint32_t id, StreamSink* output) final {
            return member_list_.write_json(id, output);
        }
        void register_endpoints(Endpoint** list, uint32_t id, uint32_t length) final {
            return member_list_.register_endpoints(list, id, length);
        }
        Endpoint* get_by_name(char * name, uint32_t length) final {
            for (uint32_t i = 0; i < length; i++) {
                if (name[i] == '.')
                    name[i] = 0;
            }
            name[length-1] = 0;
            return member_list_.get_by_name(name, length);
        }
        T& member_list_;
};

class JSONDescriptorEndpoint : Endpoint {
    public:
        static constexpr uint32_t endpoint_count = 1;
        bool write_json(uint32_t id, StreamSink* output);
        void register_endpoints(Endpoint** list, uint32_t id, uint32_t length);
        void handle(const uint8_t* input, uint32_t input_length, StreamSink* output);
};

// defined in protocol.cpp
extern Endpoint** endpoint_list_;
extern uint32_t n_endpoints_;
extern uint32_t json_crc_;
extern JSONDescriptorEndpoint json_file_endpoint_;
extern EndpointProvider* application_endpoints_;

// @brief Registers the specified application object list using the provided endpoint table.
// This function should only be called once during the lifetime of the application. TODO: fix this.
// @param application_objects The application objects to be registred.
template<typename T>
    int fibre_publish(T& application_objects) {
        static constexpr uint32_t endpoint_list_size = 1 + T::endpoint_count;
        static Endpoint* endpoint_list[endpoint_list_size];
        static auto endpoint_provider = EndpointProvider_from_MemberList<T>(application_objects);

        json_file_endpoint_.register_endpoints(endpoint_list, 0, endpoint_list_size);
        application_objects.register_endpoints(endpoint_list, 1, endpoint_list_size);

        // Update the global endpoint table
        endpoint_list_ = endpoint_list;
        n_endpoints_ = endpoint_list_size;
        application_endpoints_ = &endpoint_provider;
        
        // Calculate the CRC32 of the JSON file (little endian)
        // The init value is the protocol version.
        CRC32Calculator crc32_calculator(PROTOCOL_VERSION, false);

        uint8_t offset[4] = { 0 };

        json_file_endpoint_.handle(offset, sizeof(offset), &crc32_calculator);
        json_crc_ = crc32_calculator.get_crc32();

        return 0;
    }
#endif