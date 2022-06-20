#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/usb/usbstd.h>

#if defined(STM32F0) || defined(STM32F1)
	#include <libopencm3/usb/usbd.h>
	#include <libopencm3/stm32/st_usbfs.h>
#elif defined(STM32F4)
	
#endif

#include "bootloader_config.h"
#include "bootloader_usb_descriptor.h"
#include "bootloader_dfuse.h"
#include "bootloader_flash_functions.h"
#include "bootloader_utils.h"
#include "bootloader_tests.h"

#ifdef BOOT_KEY_FILE
	#include "AES/aes.h"
#endif

// USB control data buffer
#define MAX_FLASH_PAGE_SIZE_PLACEHOLDER 1024
uint8_t usbd_control_buffer[MAX_FLASH_PAGE_SIZE_PLACEHOLDER]  __attribute__((aligned(4)));

// Flash information
//uint16_t flash_page_size_ = 0;
uint32_t writable_flash_end_address_ = 0;
uint16_t wTransferSize_ = 0;
uint32_t application_start_address_ = FLASH_BASE_ADDR + (FLASH_BOOTLDR_SIZE_KB_INT * 1024);
uint32_t flash_end_address_ = 0;
uint8_t last_flash_sector_erased = 0;

// DFU state
enum dfu_state dfu_state_ = STATE_DFU_IDLE;

#ifdef BOOT_KEY_FILE
	uint8_t aes_key[] = BOOT_KEY;
	struct AES_ctx ctx;
#endif

// =========================================================================
#if defined(STM32F0) || defined(STM32F1)
static void check_do_erase() {
	// For protection reasons, we do not allow reading the flash using DFU
	// and also we make sure to wipe the entire flash on an ERASE/WRITE command
	// just to guarantee that nobody is able to extract the data by flashing a
	// stub and executing it.

	static bool erased = 0;

	if (erased) {
		return;
	}
	
	// All pages on STM32F0/1 are the same size
	uint32_t flash_page_size = getFlashSectorSize(0);

	// Iterate and erase all pages which are writable
	for (uint32_t addr = application_start_address_; addr < writable_flash_end_address_; addr += flash_page_size) {
		flash_erase_page(addr);
	}

	erased = 1;
}
#endif


// =========================================================================
#ifdef STM32F4
	static void erase_sector_if_not_erased_already(uint32_t address) {

		uint32_t sector_cumulative_sizes[12];
		sector_cumulative_sizes[0] = 0;
		for(int i = 1; i < 12; i++) {
			sector_cumulative_sizes[i] = sector_cumulative_sizes[i-1] + getFlashSectorSize(i-1);
		}

		#ifndef PROTECT_LAST_TWO_FLASH_PAGES
			uint8_t last_writable_sector = 12;
		#else
			uint8_t last_writable_sector = 10;
		#endif

		address -= FLASH_BASE_ADDR;

		for(int i = 1; i < last_writable_sector; i++) {
			if(address == sector_cumulative_sizes[i]) {
				flash_erase_sector(i, 2);
				last_flash_sector_erased = i;
				break;
			}		
		}
	}

	void erase_remaining_sectors() {
		
		#ifndef PROTECT_LAST_TWO_FLASH_PAGES
			uint8_t last_writable_sector = 12;
		#else
			uint8_t last_writable_sector = 10;
		#endif

		for (int i = last_flash_sector_erased; i < last_writable_sector; i++) {
			flash_erase_sector(i, 2);
			last_flash_sector_erased = i;
		}
	}
#endif 


// =========================================================================
static struct {
	uint8_t buf[sizeof(usbd_control_buffer)]  __attribute__((aligned(4)));
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog  __attribute__((aligned(4)));


// =========================================================================
uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout) {

	switch (dfu_state_) {
		case STATE_DFU_DNLOAD_SYNC:
			dfu_state_ = STATE_DFU_DNBUSY;
			*bwPollTimeout = 10;
			return DFU_STATUS_OK;

		case STATE_DFU_MANIFEST_SYNC:
			// Device will reset when read is complete.
			dfu_state_ = STATE_DFU_MANIFEST;
			return DFU_STATUS_OK;

		case STATE_DFU_ERROR:
			return DFU_STATUS_ERR_VENDOR;

		default:
			return DFU_STATUS_OK;
	}
}


// =========================================================================	
void usbdfu_getstatus_complete(struct usb_setup_data *req) {
	(void)req;

	switch (dfu_state_) {
            
        case STATE_DFU_DNBUSY:
            flash_unlock();

            if (prog.blocknum == 0) {

                switch (prog.buf[0]) {
					
					// DFU Erase Operations.............................................
                	case CMD_ERASE: {

						// Clear this page here.
						uint32_t erase_address = 0;
						memcpy(&erase_address, &(prog.buf[1]), sizeof(int32_t));
	
						// Protect the bootloader by only writing to the valid flash area
						if ( erase_address >= application_start_address_ &&
							(erase_address + wTransferSize_) <= writable_flash_end_address_
						) {
							#if defined(STM32F0) || defined(STM32F1)
								// All pages on STM32F0/1 are the same size
								uint32_t sector_size = getFlashSectorSize(0);

								// Only erase page if this address is the start of a page
								if(erase_address % sector_size == 0) {
									flash_erase_page(erase_address);
								}
							#elif defined(STM32F4)
								#warning "fix fl erase sector"
								__asm__ ("nop");
							#endif
						}
					}
					break;

					// DFU Set Address Operations.............................................
					case CMD_SETADDR:
						// Assuming little endian here.
						memcpy(&(prog.addr), &(prog.buf[1]), sizeof(int32_t));
						break;
                }
            } 
            
            else {
				// DFU Download Operations.............................................
				#if defined(STM32F0) || defined(STM32F1)
                	check_do_erase();
				#endif

                // From formula Address_Pointer + ((wBlockNum - 2)*wTransferSize)
                volatile uint32_t write_address = prog.addr + ((prog.blocknum - 2) * wTransferSize_);

				#ifdef BOOT_KEY_FILE
					// If this request is for the start of the program flash
					// then we should have our initialization vector at the 
					// beginning 16 bytes of the firmware update file 
					// (per our snappack.py program). 

					if(write_address == FLASH_BASE_ADDR) {
						// Get first 16 bytes of data as our init_vector
						uint8_t init_vector[16];
						for(uint8_t i = 0; i < 16; i++) {
							init_vector[i] = prog.buf[i];
						}
						AES_init_ctx_iv(&ctx, aes_key, init_vector);
					}
				
					// Decrypt the application one block at a time
					for(int j = 0; j < prog.len; j += AES_BLOCKLEN) {
						// Iterate over blocks...
					
						uint8_t ciphertext_buffer[AES_BLOCKLEN];

						for(int i = 0; i < sizeof(ciphertext_buffer); i++) {
							ciphertext_buffer[i] = prog.buf[i+j];
						}
						
						AES_CBC_decrypt_buffer(&ctx, ciphertext_buffer, AES_BLOCKLEN);
						
						for(int i = 0; i < AES_BLOCKLEN; i++) {
							prog.buf[i+j] = ciphertext_buffer[i];
						}
					}
				#endif

				// Protect the bootloader by only writing to the valid flash area
                if (write_address >= application_start_address_) {

					if ((write_address + prog.len) <= writable_flash_end_address_) {

						#if defined(STM32F4)
							erase_sector_if_not_erased_already(write_address);
						#endif

						// Since the flash should be erased by now we can write to it.
						// But if it has not been erased this may cause a hard fault.
						_flash_program_buffer(write_address, prog.buf, prog.len);
					}
					else {
						// No more flash to write...
						dfu_state_ = STATE_DFU_ERROR;
						return;
					}
				}
			}

			// Make sure flash is ready before continuing
			flash_wait_for_last_operation();
            flash_lock();

            /* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
            dfu_state_ = STATE_DFU_DNLOAD_IDLE;
            return;

        case STATE_DFU_MANIFEST:
		    // Reset placed in main loop.
            return;  
        
        default:
            return;
    }
}

// =========================================================================
// 24-bit integer in DFU class spec
uint32_t bwPollTimeout = 0; 

enum usbd_request_return_codes usbdfu_control_request(
	usbd_device *usbd_devicezz,
	struct usb_setup_data *req, 
	uint8_t **buf, 
	uint16_t *len,
	usbd_control_complete_callback *complete
) {

	switch (req->bRequest) {

		case DFU_DNLOAD:
			if ((len == NULL) || (*len == 0)) {
				// wLength = 0 means leave DFU
				dfu_state_ = STATE_DFU_MANIFEST_SYNC;
				return USBD_REQ_HANDLED;
			} 
            else {
				// Copy download data for use on GET_STATUS.
				prog.blocknum = req->wValue;
				
                // Beware overflows!
				prog.len = *len;
				
                if (prog.len > sizeof(prog.buf)) {
					prog.len = sizeof(prog.buf);
                }
				
                memcpy(prog.buf, usbd_control_buffer, prog.len);
				
                dfu_state_ = STATE_DFU_DNLOAD_SYNC;
				
                return USBD_REQ_HANDLED;
			}

		case DFU_CLRSTATUS:
			// Just clears errors.
			if (dfu_state_ == STATE_DFU_ERROR) {
				dfu_state_ = STATE_DFU_IDLE;
            }
			return USBD_REQ_HANDLED;

		case DFU_ABORT:
			// Abort just returns to IDLE state.
			dfu_state_ = STATE_DFU_IDLE;
			return USBD_REQ_HANDLED;

		case DFU_DETACH:
			// @todo: implement clear dfu memory token, reset, enter application
			dfu_state_ = STATE_DFU_MANIFEST;
			return USBD_REQ_HANDLED;

		case DFU_UPLOAD:
			// Send data back to host by reading the image.
			dfu_state_ = STATE_DFU_UPLOAD_IDLE;

			if (!req->wValue) {
				// Send back supported commands.
				usbd_control_buffer[0] = 0x00;
				usbd_control_buffer[1] = CMD_SETADDR;
				usbd_control_buffer[2] = CMD_ERASE;
				*len = 3;
				return USBD_REQ_HANDLED;
			} 
            else {
				// Send back data if only if we enabled that.
				#ifndef ENABLE_FIRMWARE_UPLOAD
				dfu_state_ = STATE_DFU_ERROR;
				*len = 0;
				#else

				// From formula Address_Pointer + ((wBlockNum - 2) * wTransferSize)
				uint32_t request_address = prog.addr + ((req->wValue - 2) * wTransferSize_);
				
				// Check if this is a valid address
				if (request_address >= application_start_address_ && request_address + wTransferSize_ <= flash_end_address_) {
					// Yes it is valid
					memcpy(usbd_control_buffer, (void*)request_address, wTransferSize_);
					*len = wTransferSize_;
				} 
				else {
					// No it is not valid, return FFs
					for(int i = 0; i < wTransferSize_; i+=8) {
						usbd_control_buffer[i+0] = 'B';
						usbd_control_buffer[i+1] = '1';
						usbd_control_buffer[i+2] = '0';
						usbd_control_buffer[i+3] = '5';
						usbd_control_buffer[i+4] = 'C';
						usbd_control_buffer[i+5] = '0';
						usbd_control_buffer[i+6] = 'D';
						usbd_control_buffer[i+7] = 'E';
					}
					//memcpy(usbd_control_buffer, (void*)request_address, wTransferSize_);
					//dfu_state_ = STATE_DFU_ERROR;
					//*len = 0;
				}

				#endif
			}

			return USBD_REQ_HANDLED;

		case DFU_GETSTATUS: 
        	// Perfom the action and register complete callback.

			usbd_control_buffer[0] = usbdfu_getstatus(&bwPollTimeout);
			usbd_control_buffer[1] = bwPollTimeout & 0xFF;
			usbd_control_buffer[2] = (bwPollTimeout >> 8) & 0xFF;
			usbd_control_buffer[3] = (bwPollTimeout >> 16) & 0xFF;
			usbd_control_buffer[4] = dfu_state_;
            
            // iString not used here
			usbd_control_buffer[5] = 0;
			*len = 6;
			*complete = (usbd_control_complete_callback *) usbdfu_getstatus_complete;

			return USBD_REQ_HANDLED;

		case DFU_GETSTATE:
			// Return state with no state transision.
			usbd_control_buffer[0] = dfu_state_;
			*len = 1;
			return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NEXT_CALLBACK;
}
