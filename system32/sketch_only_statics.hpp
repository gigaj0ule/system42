#if defined(__SKETCH_ONLY_STATICS)
    #error __base_file__ should be included in sketch.ino only, after sketch.hpp
#else
    #define __SKETCH_ONLY_STATICS
#endif

#include "STM32FreeRTOSConfig.h"
#include "STM32FreeRTOS.h"

#include "powernet.h"


// =========================================================================
#if defined(ARDUINO_ARCH_STM32)
    #define RED_LED_PIN PB6
    #define GREEN_LED_PIN PB7
    #define BLUE_LED_PIN PA1
    #define USER_BUTTON PB2
#endif

#define LED_COLOR_OFF 0b000
#define LED_COLOR_RED 0b001
#define LED_COLOR_GREEN 0b010
#define LED_COLOR_BLUE 0b100

uint8_t LED_color_ = LED_COLOR_OFF;
uint8_t LED_color_active_ = LED_color_;

bool user_button_state_  = 0;
float user_button_value_ = 0;

void set_led_color(uint8_t led_color);
void user_button_handler(void);

// =========================================================================
// Delay function that will ensure the scheduler doesn't bind up
//
static void os_delay(uint32_t milliseconds) {
    vTaskDelay((milliseconds * configTICK_RATE_HZ) / 1000L);
}


// =========================================================================
// This thread listens on the USB port for communication requests and will
// respond to said requests
// 
static void communication_thread(void* arg) {
  
  while(true) {

    #if defined(INCLUDE_PNTP)
      pntp_listen();
    #endif

    vTaskDelay(1);
  }
}


// =========================================================================
// If we got here then something really bad went wrong, or the MCU had a
// single-upset event.
//
static void catch_kernel_fault() {
    while(1) {

        #if defined(USE_USB_CDC)
          SerialUSB.println("Kernel fault");
        #endif

        delay(1000);
    }
}

// =========================================================================
// If we got here something went wrong when launching the threads. This
// most often means there was a stack overflow
//
static void catch_thread_fault() {
    while(1) {

      #if defined(USE_USB_CDC)
        SerialUSB.println("Thread allocation problem");
      #endif

      delay(2000);
    }
}

// =========================================================================
// This is a thread that listens for user button presses
// 
static void ui_thread(void* arg) {
  while(true) {

    bool user_button_reading = digitalRead(USER_BUTTON);

    user_button_value_ += 0.3f * (user_button_reading - user_button_value_);

    bool falling_edge = false;
    bool rising_edge = false;

    if(user_button_value_ > 0.6f && user_button_state_ == false) {
      user_button_state_ = true;
      rising_edge = true;
    }
    else if(user_button_value_ < 0.4f && user_button_state_ == true){
      user_button_state_ = false; 
      falling_edge = true;
    }

    if(rising_edge) {
      #ifdef INCLUDE_PNTP
      send_system_interrupt(SYSINT_USER_BUTTON_RISING);
      #endif

      user_button_value_ = 1;
    }
    else if(falling_edge) {
      #ifdef INCLUDE_PNTP
      send_system_interrupt(SYSINT_USER_BUTTON_FALLING);
      #endif

      user_button_value_ = 0;
    }
    
    if(LED_color_ != LED_color_active_) {
      set_led_color(LED_color_);
    }

    os_delay(20);
  }
}


// =========================================================================
// This initializes things that are run-once operations
// done with all projects
//
inline void ui_setup(void) {

  // Set-up GPIOs
  pinMode(USER_BUTTON, INPUT);

  set_led_color(LED_color_active_);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
}


// =========================================================================
// This boots up the operating system and launches the 
// worker thread
//

static void worker_thread(void* arg) {
	//SerialUSB.println("hmm");
    
	while(true) {
	    // Do something every second
        os_delay(100);
    	__asm__ volatile("nop");
		//find_i2c_devices();
		//while(SerialUSB.available()){
		//	SerialUSB.print(SerialUSB.read());
		//}
    }
}


inline void create_threads(
  void (*worker_thread)(void *), 
  uint8_t worker_thread_stack_size = 5, 
  uint8_t communication_thread_stack_size = 5
) {

    // Create worker task at priority 1 (low)
    portBASE_TYPE worker_thread_handle
      = xTaskCreate(worker_thread, NULL, worker_thread_stack_size * configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Create communication task at priority 2 (medium)
    // Increase stack size if communication does not work
    #ifdef INCLUDE_PNTP
    portBASE_TYPE communication_thread_handle
      = xTaskCreate(communication_thread, NULL, communication_thread_stack_size * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    #endif

    // Create UI task at priority 2 (medium)
    // Increase stack size if does not work
    portBASE_TYPE ui_thread_handle
      = xTaskCreate(ui_thread, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    // Check for thread creation errors
    if (worker_thread_handle != pdPASS || ui_thread_handle != pdPASS) {
        catch_thread_fault();
    }

    #ifdef INCLUDE_PNTP 
    if(communication_thread_handle != pdPASS) {
        catch_thread_fault();
    }
    #endif 

    // Start scheduler
    vTaskStartScheduler();

    // We should not reach this point unless there was an error
    catch_kernel_fault();
}


// =========================================================================
// Sets the color of the status LED
//
void set_led_color(uint8_t led_color) {

  uint8_t led_state = (uint8_t) led_color;

  digitalWrite(RED_LED_PIN, !(led_state & 0b001));
  digitalWrite(GREEN_LED_PIN, !(led_state & 0b010));
  digitalWrite(BLUE_LED_PIN, !(led_state & 0b100));

  LED_color_active_ = LED_color_;
}
