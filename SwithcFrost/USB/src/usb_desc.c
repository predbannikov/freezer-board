/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptors for Custom HID Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "HidDev_Config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize40*/
//    0x83,                       /*idVendor (0x0483)*/
//    0x04,
//    0x50,                       /*idProduct = 0x5750*/
//    0x57,
//    0x02,                       /*bcdDevice rel. 2.00*/
//    0x02,

    ((uint8_t)(((uint16_t)(USB_VID))&255)),
    ((uint8_t)((((uint16_t)(USB_VID))>>8)&255)),
    ((uint8_t)(((uint16_t)(USB_PID))&255)),
    ((uint8_t)((((uint16_t)(USB_PID))>>8)&255)),
    ((uint8_t)(((uint16_t)(USB_NUM))&255)),
    ((uint8_t)((((uint16_t)(USB_NUM))>>8)&255)),

    1,                          /*Index of string descriptor describing
                                              manufacturer */
    2,                          /*Index of string descriptor describing
                                             product*/
    3,                          /*Index of string descriptor describing the
                                             device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* CustomHID_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    CUSTOMHID_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface */
    0x01,         /* bConfigurationValue: Configuration value */
    0x00,         /* iConfiguration: Index of string descriptor describing
                                 the configuration*/
    0xC0,         /* bmAttributes: Self powered */
    0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */

    /************** Descriptor of Custom HID interface ****************/
    /* 09 */
    0x09,         /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type */
    0x00,         /* bInterfaceNumber: Number of Interface */
    0x00,         /* bAlternateSetting: Alternate setting */
    0x02,         /* bNumEndpoints */
    0x03,         /* bInterfaceClass: HID */
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,            /* iInterface: Index of string descriptor */
    /******************** Descriptor of Custom HID HID ********************/
    /* 18 */
    0x09,         /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
    0x10,         /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,         /* bCountryCode: Hardware target country */
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         /* bDescriptorType */
    CUSTOMHID_SIZ_REPORT_DESC,/* wItemLength: Total length of Report descriptor */
    0x00,
    /******************** Descriptor of Custom HID endpoints ******************/
    /* 27 */
    0x07,          /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: */

    0x81,          /* bEndpointAddress: Endpoint Address (IN) */
    0x03,          /* bmAttributes: Interrupt endpoint */
    ENDP_IN_SIZE,  /* wMaxPacketSize: 64 Bytes max */
    0x00,
    ENDP_IN_INTERVAL, /* bInterval: Polling Interval (10 ms) */
    /* 34 */

    0x07,	/* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: */
			/*	Endpoint descriptor type */
    0x01,	/* bEndpointAddress: */
			/*	Endpoint Address (OUT) */
    0x03,	/* bmAttributes: Interrupt endpoint */
    ENDP_OUT_SIZE,  /* wMaxPacketSize: 64 Bytes max  */
    0x00,
    ENDP_OUT_INTERVAL, /* bInterval: Polling Interval (10 ms) */
    /* 41 */
  }
  ; /* CustomHID_ConfigDescriptor */
const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] =
  {
    0x06, 0x00 , 0xFF,          // Usage_page (vendor Defined Page 1)
    0x09, 0x01,                 // Usage (vendor Usage 1)
    0xA1, 0x01,                 // Collection (Application)

#if (((ENDP_IN_SIZE)>0) && ((ENDP_IN_SIZE)<=64))
// Описание конечной точки, типа Input.
    0x09, 0x00,                 // Usage (undefined)
    0x15, 0x00,                 // Logical_minimum (0)
    0x25, 0xFF,                 // Logical_maximum (255)
    0x75, 0x08,                 // Report_size (8)
    0x95, ENDP_IN_SIZE,         // Report_count
    0x81, 0x02,                 // Input(data , Var , Abs)
#endif

#if (((ENDP_OUT_SIZE)>0) && ((ENDP_OUT_SIZE)<=64))
// Описание конечной точки, типа Output.
    0x09, 0x00,                 // Usage (undefined)
    0x15, 0x00,                 // Logical_minimum (0)
    0x25, 0xFF,                 // Logical_maximum (255)
    0x75, 0x08,                 // Report_size (8)
    0x95, ENDP_OUT_SIZE,        // Report_count
    0x91, 0x02,                 // Output(data , Var , Abs)
 #endif

 #if (((ENDP_FEATURE_SIZE)>0) && ((ENDP_FEATURE_SIZE)<=64))
// Описание конечной точки, типа Feature.
    0x09, 0x00,                 // Usage (undefined)
    0x15, 0x00,                 // Logical_minimum (0)
    0x25, 0xFF,                 // Logical_maximum (255)
    0x75, 0x08,                 // Report_size (8)
    0x95, ENDP_FEATURE_SIZE,    // Report_count
    0xB1, 0x02,                 // Feature(data , Var , Abs)
#endif

    0xc0                         // End Collection


  }; /* CustomHID_ReportDescriptor */

/* USB String Descriptors (optional) */
const uint8_t CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] =
{   // pure-basic@yandex.ru
    CUSTOMHID_SIZ_STRING_VENDOR,
    USB_STRING_DESCRIPTOR_TYPE,
    0x70, 0x00, 0x75, 0x00, 0x72, 0x00, 0x65, 0x00, 0x2D, 0x00,
    0x62, 0x00, 0x61, 0x00, 0x73, 0x00, 0x69, 0x00, 0x63, 0x00,
    0x40, 0x00, 0x79, 0x00, 0x61, 0x00, 0x6E, 0x00, 0x64, 0x00,
    0x65, 0x00, 0x78, 0x00, 0x2E, 0x00, 0x72, 0x00, 0x75, 0x00
};

const uint8_t CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] =
{   // USB HID устройство
    CUSTOMHID_SIZ_STRING_PRODUCT,
    USB_STRING_DESCRIPTOR_TYPE,
    0x55, 0x00, 0x53, 0x00, 0x42, 0x00, 0x20, 0x00, 0x48, 0x00,
    0x49, 0x00, 0x44, 0x00, 0x20, 0x00, 0x43, 0x04, 0x41, 0x04,
    0x42, 0x04, 0x40, 0x04, 0x3E, 0x04, 0x39, 0x04, 0x41, 0x04,
    0x42, 0x04, 0x32, 0x04, 0x3E, 0x04
};

uint8_t CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0,'3', 0,'2', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

