// =========================================================================
// PNTP Server
//

#include <Arduino.h>

#include "powernet.h"
#include "protocol.hpp"
#include "nvm_config.hpp"

// For custom definitions
#include "sketch.hpp"

// For reboot function
#include <cmsis_os.h>

// TTY Transport
#if defined(USBD_USE_CDC)
    #include "usbd_cdc_if.h"

    #if defined(PNTP_USING_TTY)
        #include "tty_transport.h"
    #endif
#endif

// TCP Transport
#if defined(PNTP_USING_ETH0)
    #include "tcp_transport.h"
    #include "Ethernet.h"
#endif

// =========================================================================
// Options for status LED colors
//


// User defined properties
extern PowernetNamespace pntp;

// Globals
int64_t serial_number_;

char build_date_[] = __DATE__;
char build_time_[] = __TIME__;

char _host_name[32] = "powernet server";


// =========================================================================
// Function prototypes
//
void pntp_generate_descriptor(const char* host_name);
void pntp_generate_descriptor(const char* host_name);


// =========================================================================
// This procedure of building a USB serial number should be identical
// to the way the STM's built-in USB bootloader does it. This means
// that the device will have the same serial number in normal and DFU mode.
//
static inline uint64_t get_serial_number() {

    uint32_t uuid0 = *(uint32_t *)(UID_BASE + 0);
    uint32_t uuid1 = *(uint32_t *)(UID_BASE + 4);
    uint32_t uuid2 = *(uint32_t *)(UID_BASE + 8);
    uint32_t uuid_mixed_part = uuid0 + uuid2;

    return ((uint64_t)uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);
}


// =========================================================================
// Non volatile memory functions
//
struct NvmBuiltins_t {
    char hostname[32] = "";
};

NvmBuiltins_t NvmBuiltins;

typedef Config<
    NvmBuiltins_t,
    PowernetNamespace::NvmProperties_t
> ConfigFormat;

// Save configuration "ConfigFormat" to NVM
//
void save_configuration(void *) {
    if (ConfigFormat::safe_store_config(&NvmBuiltins, &pntp.disk0)) {
        // Failed... we should catch the error or something
    } 
}


// Load configuration "ConfigFormat" to NVM
//
int32_t load_configuration(void) {

    int32_t retval = 0;

    volatile int32_t init_failed = NVM_init();
    volatile int32_t load_failed = ConfigFormat::safe_load_config(&NvmBuiltins, &pntp.disk0);

    // Try to load configs
    if ((init_failed | load_failed) != 0) {
        // If loading failed, restore defaults
        pntp.disk0 = PowernetNamespace::NvmProperties_t();
        NvmBuiltins = NvmBuiltins_t();
        retval = load_failed;
    } 
    else {
        // Loaded config from NVM succesfully
    }

    return retval;
}


// =========================================================================
// System functions
//
// Helper class because the protocol library doesn't yet
// support non-member functions
// TODO: make this go away
//
class SystemFunctions {
public:

    // Reboot
    void reboot_system(void) {
        NVIC_SystemReset();
    }

    // Save the non volatile memory
    void save_configuration_static(void) {
        save_configuration(0);
    }

    // Read the non volatile memory
    int64_t load_configuration_static(void) {
        return load_configuration();
    }
    
    // Erase the non volatile memory
    void erase_configuration_static(void) {
        // This reboots, has no retval ever...
        int32_t retval = NVM_erase();
        
        // Silence compiler warning
        (void) retval;

        reboot_system();
    }

    // Enter usb device firmware upgrade mode
    void enter_dfu_static(void) {
        uint32_t * user_code_boot_token = (uint32_t *)(0x20005000 - 4);
        user_code_boot_token[0] = 0xB105F00D;
        reboot_system();
    }

    // Reconnect TTY port
    void reconnect_tty0(void) {
        #if defined(ARDUINO_ARCH_STM32)
            #if defined(PNTP_USING_TTY)
                CDC_deInit();
                pinMode(PA12, OUTPUT);
                digitalWrite(PA12, LOW);
                delay(1);
                CDC_init();
            #endif
        #endif
    }    

    // Reconnect TCP port
    void reconnect_eth0(){
        #if defined(PNTP_USING_ETH0)
            pntp_eth0_reset();
        #endif
    }

} system_functions;


// =========================================================================
// Options for status LED colors
//
void was_assigned_new_hostname(void *){

    // Save hostname to flash
    save_configuration(0);

    // Reconnect transports
    system_functions.reconnect_tty0();
    system_functions.reconnect_eth0();
}


// =========================================================================
// When adding new functions/variables to the protocol, be careful not to
// blow the PowernetNamespace stack. You can check comm_stack_info to see
// how much headroom you have.
//

// Todo: Move
#define HAS_HID_LED
#if defined(HAS_HID_LED)
    // Led colors
    char led_color_options[][16] = {
        "OFF", "RED", "GREEN", "YELLOW", "BLUE", "MAGENTA", "CYAN", "WHITE"
    };

    // LED state
    extern uint8_t LED_color_;
#endif


static inline auto make_obj_tree() {

    return make_protocol_member_list(
        
        // Volatile Objects
        make_protocol_object("ram", 
            pntp.volatile_properties()
        ),

        // Event handlers (if any)
        make_protocol_object("irq", 
            pntp.interrupt_properties()
        ),

        // System utilities
        make_protocol_object("dev",

            // Eth Device
            #if defined(PNTP_USING_ETH0)

                make_protocol_object("eth0",
                    make_protocol_string_kw(
                        &local_ip_, 
                        property_name = "ip",
                        property_length = sizeof(local_ip_),
                        property_is_read_only = true
                    ),

                    make_protocol_string_kw(
                        &mdns_name_, 
                        property_name = "mdns",
                        property_length = sizeof(mdns_name_),
                        property_is_read_only = true
                    ),

                    make_protocol_function_kw(
                        system_functions, 
                        &SystemFunctions::reconnect_eth0, 
                        property_name = "reset"
                    )
                ),
            #endif

            // TTY Device
            #if defined(PNTP_USING_TTY)
                make_protocol_object("tty0",

                    make_protocol_function_kw(
                        system_functions, 
                        &SystemFunctions::reconnect_tty0, 
                        property_name = "reset"
                    )
                ),
            #endif

            // HID LED
            #if defined(HAS_HID_LED)
                make_protocol_object("led0",
                    make_protocol_selection_kw(
                        &LED_color_, 
                        property_name = "color", 
                        property_option_strings = led_color_options, 
                        property_option_count = 8
                    )
                ),
            #endif

            // Non Volatile Memory
            make_protocol_object("nvm0",

                pntp.non_volatile_properties(),

                make_protocol_function_kw(
                    system_functions, 
                    &SystemFunctions::save_configuration_static, 
                    property_name = "save"
                ),
                make_protocol_function_kw(
                    system_functions, 
                    &SystemFunctions::load_configuration_static, 
                    property_name = "load"
                ),
                make_protocol_function_kw(
                    system_functions, 
                    &SystemFunctions::erase_configuration_static, 
                    property_name = "erase"
                )
            )
        ),

        // System
        make_protocol_object("sys",

            // inaccurate...
            make_protocol_string_kw(
                &NvmBuiltins.hostname, 
                property_name = "hostname", 
                property_length = sizeof(NvmBuiltins.hostname),
                property_is_non_volatile = true,
                after_written_callback = was_assigned_new_hostname
            ),
        
            make_protocol_object("info", 
                make_protocol_number_kw(
                    &serial_number_, 
                    property_name = "serial", 
                    property_is_read_only = true
                ),

                make_protocol_string_kw(
                    &build_date_, 
                    property_name = "build_date", 
                    property_length = sizeof(build_date_), 
                    property_is_read_only = true
                ),

                make_protocol_string_kw(
                    &build_time_, 
                    property_name = "buil_time", 
                    property_length = sizeof(build_time_), 
                    property_is_read_only = true
                )
            ),

            make_protocol_function_kw(
                system_functions, 
                &SystemFunctions::reboot_system, 
                property_name = "reboot"
            ),

            make_protocol_function_kw(
                system_functions, 
                &SystemFunctions::enter_dfu_static, 
                property_name = "dfu"
            )
        )
    );
}


// =========================================================================
// Create namespace
//
bool _pntp_namespace_created = false;

void pntp_create_namespace() {

    // Load nonvolatile
    load_configuration();

    // Was there a nonvolatile name?
    if(strlen(NvmBuiltins.hostname) == 0) {

        // Nope, use default instead

        // Zero fill name
        for(uint8_t i = 0; i < sizeof(NvmBuiltins.hostname); i++) {
            NvmBuiltins.hostname[i] = 0;
        }

        // Use default name
        if (strlen(_host_name) < 32) {
            memcpy(NvmBuiltins.hostname, _host_name, strlen(_host_name));
        }
        else {
            memcpy(NvmBuiltins.hostname, _host_name, 32);
        }
    }
        
    // Get our S/N
    serial_number_ = get_serial_number();
    
    // Build time
    uint8_t last_build_time_char_index = sizeof(build_time_) - 2;
    uint8_t last_build_time_dec = build_time_[last_build_time_char_index] - 48;
    //uint8_t last_build_time_dec_is_even = (last_build_time_dec % 2 == 0);
    
    build_time_[last_build_time_char_index] = 48 + last_build_time_dec; 

    _pntp_namespace_created = true;
}


// =========================================================================
// Create communicable variables in namespace
//
bool _pntp_properties_created = false;

using tree_type = decltype(make_obj_tree());
uint8_t tree_buffer[sizeof(tree_type)];

void pntp_create_properties(void) {
    // TODO: this is supposed to use the move constructor, but currently
    // the compiler uses the copy-constructor instead. Thus the make_obj_tree
    // ends up with a stupid stack size of around 8000 bytes. Fix this.
    auto tree_ptr = new (tree_buffer) (tree_type)(make_obj_tree());
    fibre_publish(*tree_ptr);

    _pntp_properties_created = true;
}



// =========================================================================
// PNTP Service itself
//
void pntp_begin(const char * host_name) {

    // Assign name
    if(host_name != NULL) {

        // Zero fill name
        for(uint8_t i = 0; i < sizeof(_host_name); i++) {
            _host_name[i] = 0;
        }

        // Overrun guard
        uint8_t host_name_length = strlen(host_name);
        if(host_name_length > sizeof(_host_name)) {
            host_name_length = sizeof(_host_name);
        }

        // Host name
        if (strlen(host_name) <= 32) {
            memcpy(_host_name, host_name, host_name_length);
        }
    }

    // Create namespace
    pntp_create_namespace();

    // Populate namespace with properties
    pntp_create_properties();    
}

// =========================================================================
//
int pntp_listen_eth0() {

    int had_activity = 0;

    // Now Listen for connections...
    #if defined(PNTP_USING_ETH0)
        // Using TCP
        pntp_eth0_listener(NvmBuiltins.hostname);
    #endif

    return had_activity;
}


// =========================================================================
//
int pntp_listen_tty0() {

    int had_activity = 0;

    #if defined(PNTP_USING_TTY)
        // Using TTY
        if (SerialUSB.available()) { 
            had_activity = 1;
            pntp_tty_listener();
        }
    #endif

    return had_activity;
}

// =========================================================================
//
int pntp_listen() {

    int had_activity = 0;

    had_activity |= pntp_listen_eth0();
    had_activity |= pntp_listen_tty0();

    return had_activity;
}
