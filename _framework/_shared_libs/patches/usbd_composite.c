/**
 * @file                usbd_composite.c
 * @author            Weyne
 * @version         V01
 * @date                2016.10.28
 * @brief             MSC + CDC 复合设备
 * @note
 * @attention     COYPRIGHT WEYNE
 * @attention     COYPRIGHT ADAM MUNICH
 */

/*
On instantiation of this composite device driver class...

	USBD_RegisterClass(&hUSBD_Device_COMPOSITE, USBD_COMPOSITE_CLASS)

hUSBD_Device_COMPOSITE is a USBD_HandleTypeDef that has function pointers:

    USBD_Composite_Init,
    USBD_Composite_DeInit,

...which sequentially instantiate child drivers using static memory 
of the form  (extern USBD_HandleTypeDef hUSBD_Device_CDC;) which is 
supplied by the class_if.c

    USBD_Composite_Setup,
    USBD_Composite_EP0_RxReady,
    USBD_Composite_DataIn,
    USBD_Composite_DataOut,

Then dispatch the incoming *pdev, req, and epnum to the appropriate
drivers as described by the configuration descriptor generators

    USBD_Composite_GetHSCfgDesc,
    USBD_Composite_GetFSCfgDesc,
    USBD_Composite_GetOtherSpeedCfgDesc,
    USBD_Composite_GetDeviceQualifierDescriptor,

Which themselves are programmed through macros in usbd_ep_conf.h
and associated PMA address tables in usbd_ep_conf.c
*/


#include "usbd_composite.h"
#include "usbd_composite_descriptors.h"
#include "usbd_ep_conf.h"

/** @defgroup USBD_HID_Private_Defines
  * @{
  */

#ifdef USBD_USE_CDC
    #include "usbd_cdc.h"
    #include "usbd_cdc_if.h"
	// Load static device driver memory
    extern USBD_HandleTypeDef    hUSBD_Device_CDC;
#endif

#ifdef USBD_USE_HID_COMPOSITE
    #include "usbd_hid_composite.h"
    #include "usbd_hid_composite_if.h"
	// Load static device driver memory
    extern USBD_HandleTypeDef    hUSBD_Device_HID;
#endif

#ifdef USBD_USE_AUDIO
    #include "usbd_audio.h"
    #include "usbd_audio_if.h"
	// Load static device driver memory
    extern USBD_HandleTypeDef    hUSBD_Device_AUDIO;
#endif

/**
  * @}
  */

/** @defgroup USBD_HID_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_Composite_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_Composite_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_Composite_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

// Transfer callbacks

static uint8_t USBD_Composite_EP0_TxReady(USBD_HandleTypeDef *pdev);

static uint8_t USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev);

static uint8_t USBD_Composite_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_Composite_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_Composite_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_Composite_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_Composite_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

// Configuration descriptor generators

static uint8_t *USBD_Composite_GetFSCfgDesc (uint16_t *length);

static uint8_t *USBD_Composite_GetHSCfgDesc(uint16_t *length);

static uint8_t *USBD_Composite_GetOtherSpeedCfgDesc(uint16_t *length);

static uint8_t *USBD_Composite_GetDeviceQualifierDescriptor (uint16_t *length);

/**
  * @}
  */


/** @defgroup USBD_HID_Private_TypesDefinitions
  * @{
  */

USBD_ClassTypeDef USBD_COMPOSITE =
{
    USBD_Composite_Init,
    USBD_Composite_DeInit,
    USBD_Composite_Setup,
    USBD_Composite_EP0_TxReady,
    USBD_Composite_EP0_RxReady,
    USBD_Composite_DataIn,
    USBD_Composite_DataOut,
    USBD_Composite_SOF,
    USBD_Composite_IsoINIncomplete,
    USBD_Composite_IsoOutIncomplete,
    USBD_Composite_GetHSCfgDesc,
    USBD_Composite_GetFSCfgDesc,
    USBD_Composite_GetOtherSpeedCfgDesc,
    USBD_Composite_GetDeviceQualifierDescriptor,
};
/**
  * @}
  */


/**
* @brief 	USBD_Composite_Init
*			Initialize the Composite interface
* @param    pdev: device instance
* @param    cfgidx: Configuration index
* @retval 	status
*/
static uint8_t USBD_Composite_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    uint8_t result = USBD_OK;

	// ------------------------------------------
	// Initialize static memory of the drivers by copying
	// the contents of pdev to them. 
	// @TODO: Make sure this is a safe copy operation
	
    #ifdef USBD_USE_CDC
		hUSBD_Device_CDC = *pdev;
	#endif

    #ifdef USBD_USE_HID_COMPOSITE
		hUSBD_Device_HID = *pdev;
	#endif

	#ifdef USBD_USE_AUDIO
		hUSBD_Device_AUDIO = *pdev;
	#endif

	// ------------------------------------------

    #ifdef USBD_USE_CDC
		// Link function ops to the driver
		hUSBD_Device_CDC.pUserData = &USBD_CDC_fops;

		// Some kind of circular reference... delete this line 
		// and ST's HAL driver breaks. IDK
		hUSBD_Device_CDC.pClassData = &hUSBD_Device_CDC;

		// Run Init() for device driver
		result += USBD_CDC.Init(&hUSBD_Device_CDC, cfgidx);
    #endif

    #ifdef USBD_USE_HID_COMPOSITE
		// HID has no fops
		hUSBD_Device_HID.pUserData = NULL;

		// Some kind of circular reference... delete this line 
		// and ST's HAL driver breaks. IDK
		hUSBD_Device_HID.pClassData = &hUSBD_Device_HID;

		// Run Init() for device driver
		result += USBD_COMPOSITE_HID.Init(&hUSBD_Device_HID, cfgidx);
    #endif

	#ifdef USBD_USE_AUDIO
		// Link function ops to the driver
		hUSBD_Device_AUDIO.pUserData = &USBD_AUDIO_fops;

		// Some kind of circular reference... delete this line 
		// and ST's HAL driver breaks. IDK
		hUSBD_Device_AUDIO.pClassData = &hUSBD_Device_AUDIO;

		// Run Init() for device driver
		result += USBD_AUDIO.Init(&hUSBD_Device_AUDIO, cfgidx);
	#endif

    return result;
}

/**
* @brief    USBD_Composite_DeInit
*                 DeInitilaize    the Composite configuration
* @param    pdev: device instance
* @param    cfgidx: configuration index
* @retval 	status
*/
static uint8_t USBD_Composite_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	uint8_t result = USBD_OK;

	#ifdef USBD_USE_CDC
		// @TODO: Make sure free() on static doesn't cause
		// problems...
		result += USBD_CDC.DeInit(&hUSBD_Device_CDC, cfgidx);
	#endif

	#ifdef USBD_USE_HID_COMPOSITE
		// @TODO: Make sure free() on static doesn't cause
		// problems...
		result += USBD_COMPOSITE_HID.DeInit(&hUSBD_Device_HID, cfgidx);
	#endif

	#ifdef USBD_USE_AUDIO
		result += USBD_AUDIO.DeInit(&hUSBD_Device_AUDIO, cfgidx);
	#endif

    return result;
}


/**
* @brief    USBD_Composite_Setup
*                 Handle the Composite requests
* @param    pdev: device instance
* @param    req: USB request
* @retval 	status
*/
static uint8_t USBD_Composite_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    uint8_t result = USBD_OK;

	uint16_t masked_request = (req->bmRequest & USB_REQ_RECIPIENT_MASK);

    if (masked_request == USB_REQ_RECIPIENT_INTERFACE) {

		USBD_HandleTypeDef utilized_static_device = *pdev;

		switch((req->wIndex) & 0xFF)
		{
			#ifdef USBD_USE_CDC
				case USBD_CDC_DATA_INTERFACE:
				case USBD_CDC_CMD_INTERFACE:

					__asm__ volatile ("nop");

					// Upon inspection, ep0_data_len and request were
					// different in pdev from hUSBD_Device_CDC. So uh, 
					// let's copy them.

					// @TODO: Create a function that copies only elements
					// from pdev to the static memory we want to copy and 
					// preserve the other values as-is.
					//hUSBD_Device_CDC = *pdev;
					
					//hUSBD_Device_CDC.ep0_state = pdev->ep0_state;
					//hUSBD_Device_CDC.ep0_data_len = pdev->ep0_data_len;
					//hUSBD_Device_CDC.request = pdev->request;
					hUSBD_Device_CDC.dev_state = pdev->dev_state;

					result += USBD_CDC.Setup(&hUSBD_Device_CDC, req);
					utilized_static_device = hUSBD_Device_CDC;
					break;
			#endif
			
			#ifdef USBD_USE_HID_COMPOSITE
				case HID_MOUSE_INTERFACE:
				case HID_KEYBOARD_INTERFACE:

					__asm__ volatile ("nop");

					// Upon inspection, ep0_data_len and request were
					// different in pdev from hUSBD_Device_HID. So uh, 
					// let's copy them.

					// @TODO: Create a function that copies only elements
					// from pdev to the static memory we want to copy and 
					// preserve the other values as-is.

					//hUSBD_Device_HID.ep0_state = pdev->ep0_state;
					//hUSBD_Device_HID.ep0_data_len = pdev->ep0_data_len;
					hUSBD_Device_HID.dev_state = pdev->dev_state;

					result += USBD_COMPOSITE_HID.Setup(&hUSBD_Device_HID, req);
					utilized_static_device = hUSBD_Device_HID;

					__asm__ volatile ("nop");

					break;
			#endif

			#ifdef USBD_USE_AUDIO
				case USBD_AUDIO_INTERFACE_NUMBER:
				case USBD_AUDIO_STREAM_IF_NUMBER:

					__asm__ volatile ("nop");

					// Upon inspection, ep0_data_len and request were
					// different in pdev from hUSBD_Device_HID. So uh, 
					// let's copy them.

					// @TODO: Create a function that copies only elements
					// from pdev to the static memory we want to copy and 
					// preserve the other values as-is.
					//hUSBD_Device_AUDIO.ep0_state = pdev->ep0_state;
					//hUSBD_Device_AUDIO.ep0_data_len = pdev->ep0_data_len;
					//hUSBD_Device_AUDIO.request = pdev->request;
					hUSBD_Device_AUDIO.dev_state = pdev->dev_state;

					result += USBD_AUDIO.Setup(&hUSBD_Device_AUDIO, req);
					utilized_static_device = hUSBD_Device_AUDIO;
					break;
			#endif

			default:
				break;
		}

		__asm__ volatile ("nop");

		// If you don't have this line here linux will screw 
		// around for 10 seconds without explanation before
		// creating the /dev/ttyACMx file descriptor		
		pdev->ep0_state = utilized_static_device.ep0_state;

		// Following lines are experimental and maybe not needed
		pdev->dev_state = utilized_static_device.dev_state;
	}

	else if (masked_request == USB_REQ_RECIPIENT_ENDPOINT) { 

		__asm__ volatile ("nop");

		switch(req->wIndex & 0xFF)
		{
		#ifdef USBD_USE_CDC
			case CDC_IN_EP:
			case CDC_OUT_EP:
			case CDC_CMD_EP:

				// @TODO: Are these 2 lines needed?
				//hUSBD_Device_CDC.ep0_data_len = pdev->ep0_data_len;
				//hUSBD_Device_CDC.request = pdev->request;
				//hUSBD_Device_CDC.pClassData = pdev->pClassData;
				result += USBD_CDC.Setup(&hUSBD_Device_CDC, req);
				break;
		#endif

		#ifdef USBD_USE_HID_COMPOSITE
			case HID_MOUSE_EPIN_ADDR:
			case HID_KEYBOARD_EPIN_ADDR:

				result += USBD_COMPOSITE_HID.Setup(&hUSBD_Device_HID, req);
				break;
		#endif

		#ifdef USBD_USE_AUDIO
			case AUDIO_OUT_EP:

				__asm__ volatile ("nop");
				result += USBD_AUDIO.Setup(&hUSBD_Device_AUDIO, req);
				break;
		#endif

			default:
				break;
		}
    }

    return result;
}

/**
* @brief    USBD_Composite_DataIn
*           	This function dispatches the IN endpoint
*				data from the appropriate device driver.	
* @param    pdev: device instance
* @param    epnum: endpoint index
* @retval 	status
*/
uint8_t USBD_Composite_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    uint8_t result = USBD_OK;

	#ifdef USBD_USE_CDC
		if(epnum == (CDC_IN_EP & (~0x80))) {
			// This seems to work with screen on /dev/ttyACMx so
			// let's keep it!
			result += USBD_CDC.DataIn(&hUSBD_Device_CDC, epnum);
		}
	#endif

	#ifdef USBD_USE_HID_COMPOSITE
		if(
			(epnum == (HID_MOUSE_EPIN_ADDR & (~0x80)))
			|| (epnum == (HID_KEYBOARD_EPIN_ADDR & (~0x80)))
		) {
			result += USBD_COMPOSITE_HID.DataIn(&hUSBD_Device_HID, epnum);
		}
	#endif

    return result;
}

/**
* @brief    USBD_Composite_DataOut
*           	This function dispatches the OUT endpoint
*				data to the appropriate device driver.	
* @param    pdev: device instance
* @param    epnum: endpoint index
* @retval 	status
*/
uint8_t USBD_Composite_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    uint8_t result = USBD_OK;

	#ifdef USBD_USE_CDC
		if(epnum == CDC_OUT_EP || epnum == CDC_CMD_EP) {
			// This seems to work with screen on /dev/ttyACMx so
			// let's keep it!
			result += USBD_CDC.DataOut(&hUSBD_Device_CDC, epnum);
		}
	#endif

	#ifdef USBD_USE_AUDIO
		if (epnum == AUDIO_OUT_EP) {
			result += USBD_AUDIO.DataOut(&hUSBD_Device_AUDIO, epnum);
		}	
	#endif

    return result;
}


/**
* @brief    USBD_Composite_EP0_RxReady
*                 handle EP0 rxReady event
* @param    pdev: device instance
* @retval 	status
*/
static uint8_t USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
	#ifdef USBD_USE_CDC
    	return USBD_CDC.EP0_RxReady(pdev);
	#endif

	#ifdef USBD_USE_AUDIO
		//hUSBD_Device_AUDIO.pClassData = pdev->pClassData;
		return USBD_AUDIO.EP0_RxReady(&hUSBD_Device_AUDIO);
	#endif

	return USBD_OK;
}


/**
* @brief    USBD_Composite_EP0_TxReady
*                 handle EP0 txReady event
* @param    pdev: device instance
* @retval 	status
*/
static uint8_t USBD_Composite_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
	return USBD_OK;
}

/**
  * @brief  USBD_Composite_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_Composite_SOF(USBD_HandleTypeDef *pdev)
{
  return USBD_OK;
}

/**
  * @brief  USBD_Composite_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_Composite_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_OK;
}

/**
  * @brief  USBD_Composite_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_Composite_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_OK;
}

// -----------------------------------------------------------------------------
// Device & Interface Desriptor Generator Functions
// -----------------------------------------------------------------------------

/**
* @brief    USBD_COMPOSITE_GetHSCfgDesc
*                 Return configuration descriptor
* @param    speed : current device speed
* @param    length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t *USBD_Composite_GetFSCfgDesc(uint16_t *length)
{
	*length = calculate_descriptor_length();

	// If CDC port is enabled then we must over-write the 
	// values of bInterval and wMaxPacketSize with the 
	// appropriate speed values.
	#if defined(USBD_USE_CDC)
		COMPOSITE_DESCRIPTOR.CDC_IF_CMD_EP_DESC_bInterval = CDC_FS_BINTERVAL;
		COMPOSITE_DESCRIPTOR.CDC_IF_OUT_EP_DESC_wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
		COMPOSITE_DESCRIPTOR.CDC_IF_IN_EP_DESC_wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
	#endif

	#if defined(USBD_USE_HID_COMPOSITE)
		COMPOSITE_DESCRIPTOR.HID_MOUSE_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL;
		COMPOSITE_DESCRIPTOR.HID_KB_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL;
	#endif

	return (uint8_t*) &COMPOSITE_DESCRIPTOR;
}

/**
* @brief    USBD_Composite_GetHSCfgDesc
*                 Return configuration descriptor
* @param    speed : current device speed
* @param    length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t *USBD_Composite_GetHSCfgDesc(uint16_t *length)
{
	*length = calculate_descriptor_length();

	// If CDC port is enabled then we must over-write the 
	// values of bInterval and wMaxPacketSize with the 
	// appropriate speed values.
	#if defined(USBD_USE_CDC)
		COMPOSITE_DESCRIPTOR.CDC_IF_CMD_EP_DESC_bInterval = CDC_HS_BINTERVAL;
		COMPOSITE_DESCRIPTOR.CDC_IF_OUT_EP_DESC_wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;
		COMPOSITE_DESCRIPTOR.CDC_IF_IN_EP_DESC_wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;
	#endif

	#if defined(USBD_USE_HID_COMPOSITE)
		COMPOSITE_DESCRIPTOR.HID_MOUSE_IF_IN_EP_DESC_bInterval = HID_HS_BINTERVAL;
		COMPOSITE_DESCRIPTOR.HID_KB_IF_IN_EP_DESC_bInterval = HID_HS_BINTERVAL;
	#endif

	return (uint8_t*) &COMPOSITE_DESCRIPTOR;
}

/**
* @brief    USBD_Composite_GetOtherSpeedCfgDesc
*                 Return configuration descriptor
* @param    speed : current device speed
* @param    length : pointer data length
* @retval 	pointer to descriptor buffer
*/
static uint8_t *USBD_Composite_GetOtherSpeedCfgDesc(uint16_t *length)
{
    *length = calculate_descriptor_length();

	COMPOSITE_DESCRIPTOR.cfgDesc_bDescriptorType = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;

	// If CDC port is enabled then we must over-write the 
	// values of bInterval and wMaxPacketSize with the 
	// appropriate speed values.
	#if defined(USBD_USE_CDC)
		COMPOSITE_DESCRIPTOR.CDC_IF_CMD_EP_DESC_bInterval = CDC_FS_BINTERVAL;
		COMPOSITE_DESCRIPTOR.CDC_IF_OUT_EP_DESC_wMaxPacketSize = 64;
		COMPOSITE_DESCRIPTOR.CDC_IF_IN_EP_DESC_wMaxPacketSize = 64;
	#endif

	#if defined(USBD_USE_HID_COMPOSITE)
		COMPOSITE_DESCRIPTOR.HID_MOUSE_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL;
		COMPOSITE_DESCRIPTOR.HID_KB_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL;
	#endif

	return (uint8_t*) &COMPOSITE_DESCRIPTOR;
}

/**
* @brief    DeviceQualifierDescriptor
*                 return Device Qualifier descriptor
* @param    length : pointer data length
* @retval 	pointer to descriptor buffer
*/
static uint8_t *USBD_Composite_GetDeviceQualifierDescriptor(uint16_t *length)
{
    *length = sizeof(USBD_COMPOSITE_DeviceQualifierDesc);
    return USBD_COMPOSITE_DeviceQualifierDesc;
}

/**
* @brief    USBD_Composite_RegisterInterface
* @param    pdev: device instance
* @param    fops: CD Interface callback
* @retval 	status
*/
uint8_t USBD_Composite_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_Composite_ItfTypeDef *fops)
{
    uint8_t ret = USBD_FAIL;

    if (fops != NULL)
    {
        pdev->pUserData = fops;
        ret = USBD_OK;
    }

    return ret;
}


/**
    * @}
    */


/**
    * @}
    */


/**
    * @}
    */

/************************ (C) COPYRIGHT WEYNE *****END OF FILE****/