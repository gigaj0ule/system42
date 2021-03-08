#if !defined(__SKETCH_HPP_) || defined(__SKETCH_ONLY_STATICS)
    #error __base_file__ should be included in sketch.ino only, after sketch.hpp
#else
    #define __SKETCH_ONLY_STATICS
#endif

#include "STM32FreeRTOSConfig.h"
#include "STM32FreeRTOS.h"

#if defined(STM32F1)
    #define RED_LED_PIN PC13
    #define GREEN_LED_PIN PC14
    #define BLUE_LED_PIN PC15
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
static void os_delay(uint32_t milliseconds) {
    vTaskDelay((milliseconds * configTICK_RATE_HZ) / 1000L);
}


// =========================================================================
static void communication_thread(void* arg) {
  while(true) {

    #if defined(USBD_USE_CDC)
      if (serialEventUSB && SerialUSB.available()) {
        serialEventUSB();
      }

    #if defined(USE_BITSNAP)
    host_interrupt_event_handler();
    #endif

    #endif
    vTaskDelay(1);
  }
}


// =========================================================================
static void catch_kernel_fault() {
    while(1) {

        #if defined(USE_USB_CDC)
          SerialUSB.println("Kernel fault");
        #endif

        delay(1000);
    }
}

// =========================================================================
static void catch_thread_fault() {
    while(1) {

      #if defined(USE_USB_CDC)
        SerialUSB.println("Thread allocation problem");
      #endif

      delay(1000);
    }
}

// =========================================================================
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
      #ifdef USE_BITSNAP
      send_system_interrupt(SYSINT_USER_BUTTON_RISING);
      #endif

      user_button_value_ = 1;
    }
    else if(falling_edge) {
      #ifdef USE_BITSNAP
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
inline void early_setup(void) {
  #ifdef USE_BITSNAP
  init_communication();
  #endif

  // Set-up GPIOs
  pinMode(USER_BUTTON, INPUT);

  set_led_color(LED_color_active_);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
}


// =========================================================================
inline void create_threads(void (*worker_thread)(void *), uint8_t worker_thread_stack_size = 5, uint8_t communication_thread_stack_size = 4) {

    // Create worker task at priority 1 (low)
    portBASE_TYPE worker_thread_handle
      = xTaskCreate(worker_thread, NULL, worker_thread_stack_size * configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Create communication task at priority 2 (high)
    // Increase stack size if communication does not work
    #ifdef USE_BITSNAP
    portBASE_TYPE communication_thread_handle
      = xTaskCreate(communication_thread, NULL, communication_thread_stack_size * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    #endif

    // Create UI task at priority 2 (medium)
    // Increase stack size if communication does not work
    portBASE_TYPE ui_thread_handle
      = xTaskCreate(ui_thread, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    // Check for thread creation errors
    if (worker_thread_handle != pdPASS || ui_thread_handle != pdPASS) {
        __asm__ volatile("nop");
        catch_thread_fault();
    }

    #ifdef USE_BITSNAP 
    if(communication_thread_handle != pdPASS) {
        catch_thread_fault();
    }
    #endif 

    // Start scheduler
    vTaskStartScheduler();

    // We should not reach this point unless there was an error
    catch_kernel_fault();
}

#ifdef USE_BITSNAP
communication communicable;
#endif


// =========================================================================
void set_led_color(uint8_t led_color) {

  uint8_t led_state = (uint8_t) led_color;

  digitalWrite(RED_LED_PIN, !(led_state & 0b001));
  digitalWrite(GREEN_LED_PIN, !(led_state & 0b010));
  digitalWrite(BLUE_LED_PIN, !(led_state & 0b100));

  LED_color_active_ = LED_color_;
}
