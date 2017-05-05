#include "usbd_dap_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

/* USBD_DAP_CDC_Private_FunctionPrototypes */

static uint8_t  USBD_DAP_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_DAP_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_DAP_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  USBD_DAP_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);
static uint8_t  USBD_DAP_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_DAP_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  *USBD_DAP_CDC_GetCfgDesc (uint16_t *length);


/* USBD_DAP_CDC_Private_Variables */ 
USBD_DAP_CDC_HandleTypeDef hdapcdc;

/* DAP_CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_DAP_CDC = 
{
  USBD_DAP_CDC_Init,
  USBD_DAP_CDC_DeInit,
  USBD_DAP_CDC_Setup,
  NULL, /*EP0_TxSent*/  
  USBD_DAP_CDC_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
  USBD_DAP_CDC_DataIn, /*DataIn*/
  USBD_DAP_CDC_DataOut,
  NULL, /*SOF */
  NULL,
  NULL,      
  NULL,
  USBD_DAP_CDC_GetCfgDesc, 
  NULL,
  NULL,
};

/* USB DAP_CDC device Configuration Descriptor */

static uint8_t USBD_DAP_CDC_CfgDesc[USB_DAP_CDC_CONFIG_DESC_SIZ] =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,   /* bDescriptorType: Configuration */
  USB_DAP_CDC_CONFIG_DESC_SIZ,   /* wTotalLength: Bytes returned */
  0x00,
  0x03,         /*bNumInterfaces: CDC:2 HID:1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x04,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */

#if 1
  /************** Descriptor of DAP(HID) interface ****************/
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0x05,         /*iInterface: Index of string descriptor*/
  /******************** Descriptor of DAP(HID) *************************/
  /* 18 */

  0x09,         /*bLength: DAP(HID) Descriptor size*/
  DAP_DESCRIPTOR_TYPE, /*bDescriptorType: HID */
  0x11,         /*Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_DAP_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of DAP endpoints ********************/
  /* 27 */

  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
  
  DAP_EPIN_ADDR, /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  DAP_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  0x01,          /*bInterval: Polling Interval (1 ms)*/
  /* 34 */
  
  0x07,          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  DAP_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03, /* bmAttributes: Interrupt endpoint */
  DAP_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  0x01, /* bInterval: Polling Interval (1 ms) */
  /* 41 */
#endif

#if 1
  /******************** Descriptor of Interface Association ********************/
  0x08,          /* bLength: InterfaceAssociation Descriptor size */
  0x0B,          /* bDescriptorType: Interface Assocation */
  0x01,          /* bFirstInterface */
  0x02,          /* bInterfaceCount */
  0x02,          /* bInterfaceClass: Communication Interface Class */
  0x02,          /* bInterfaceSubClass: Abstract Control Model */
  0x01,          /* bInterfaceProtocol: Common AT commands */
  0x00,          /* iInterface: */
  /* 49 */

  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  /* 58 */

  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  /* 63 */
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x02,   /* bDataInterface: 1 */
  /* 68 */

  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  /* 72 */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x01,   /* bMasterInterface: Communication class interface */
  0x02,   /* bSlaveInterface0: Data Class Interface */
  /* 77 */
  
  /*Endpoint Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */ 
  /* 84 */

  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x02,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  /* 93 */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  /* 100 */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  /* 107 */
#endif
} ;

/* USB DAP device Configuration Descriptor */
static uint8_t USBD_DAP_Desc[USB_DAP_DESC_SIZ] =
{
  0x09,         /*bLength: DAP Descriptor size*/
  DAP_DESCRIPTOR_TYPE, /*bDescriptorType: DAP(HID)*/
  0x11,         /*Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_DAP_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
};
/**
  * @brief  USBD_DAP_CDC_Init
  *         Initialize the CMSIS-DAP interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_DAP_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                 DAP_EPIN_ADDR,
                 USBD_EP_TYPE_INTR,
                 DAP_EPIN_SIZE);  
  
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
                 DAP_EPOUT_ADDR,
                 USBD_EP_TYPE_INTR,
                 DAP_EPOUT_SIZE);
  
  /* Open EP IN */
   USBD_LL_OpenEP(pdev,
                 CDC_IN_EP,
                 USBD_EP_TYPE_BULK,
                 CDC_DATA_FS_IN_PACKET_SIZE);
    
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
                 CDC_OUT_EP,
                 USBD_EP_TYPE_BULK,
                 CDC_DATA_FS_OUT_PACKET_SIZE);

  /* Open Command IN EP */
  USBD_LL_OpenEP(pdev,
                 CDC_CMD_EP,
                 USBD_EP_TYPE_INTR,
                 CDC_CMD_PACKET_SIZE);
  
  hdapcdc.state = DAP_IDLE;

  ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->Init();

  /* Init Xfer states */
  hdapcdc.TxState =0;
  hdapcdc.RxState =0;

  /* Prepare Out endpoint to receive 1st packet */ 
  USBD_LL_PrepareReceive(pdev, DAP_EPOUT_ADDR, hdapcdc.Report_buf, 
                           USBD_DAP_OUTREPORT_BUF_SIZE);

  USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, hdapcdc.RxBuffer,
                         CDC_DATA_FS_OUT_PACKET_SIZE);

  return USBD_OK;
}

/**
  * @brief  USBD_DAP_CDC_DeInit
  *         DeInitialize the CMSIS-DAP layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_DAP_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Close CUSTOM_HID EP IN */
  USBD_LL_CloseEP(pdev,
                  DAP_EPIN_ADDR);
  
  /* Close CUSTOM_HID EP OUT */
  USBD_LL_CloseEP(pdev,
                  DAP_EPOUT_ADDR);

  /* Close EP IN */
  USBD_LL_CloseEP(pdev,
              CDC_IN_EP);
  
  /* Close EP OUT */
  USBD_LL_CloseEP(pdev,
              CDC_OUT_EP);
  
  /* Close Command IN EP */
  USBD_LL_CloseEP(pdev,
              CDC_CMD_EP);

  /* DeInit  physical Interface components */
  ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->DeInit();

  return USBD_OK;
}

/**
  * @brief  USBD_DAP_Setup
  *         Handle the DAP specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_DAP_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  switch (req->bRequest)
  {
  case DAP_REQ_SET_PROTOCOL:
    hdapcdc.Protocol = (uint8_t)(req->wValue);
    break;
    
  case DAP_REQ_GET_PROTOCOL:
    USBD_CtlSendData (pdev, 
                      (uint8_t *)&hdapcdc.Protocol,
                      1);    
    break;
    
  case DAP_REQ_SET_IDLE:
    hdapcdc.IdleState = (uint8_t)(req->wValue >> 8);
    break;
    
  case DAP_REQ_GET_IDLE:
    USBD_CtlSendData (pdev, 
                      (uint8_t *)&hdapcdc.IdleState,
                      1);        
    break;      
  
  case DAP_REQ_SET_REPORT:
    hdapcdc.IsReportAvailable = 1;
    USBD_CtlPrepareRx (pdev, hdapcdc.Report_buf, (uint8_t)(req->wLength));
    
    break;

  default:
    USBD_CtlError (pdev, req);
    return USBD_FAIL; 
  }

  return USBD_OK;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_CDC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  if (req->wLength)
  {
    if (req->bmRequest & 0x80)
    {
      ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->Control(req->bRequest,
                                            (uint8_t *)hdapcdc.data,
                                            req->wLength);
        USBD_CtlSendData (pdev, 
                          (uint8_t *)hdapcdc.data,
                          req->wLength);
    }
    else
    {
      hdapcdc.CmdOpCode = req->bRequest;
      hdapcdc.CmdLength = req->wLength;
      
      USBD_CtlPrepareRx (pdev, 
                         (uint8_t *)hdapcdc.data,
                         req->wLength);
    }
      
  }
  else
  {
    ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->Control(req->bRequest,
                                          (uint8_t*)req,
                                          0);
  }

  return USBD_OK;
}

/**
  * @brief  USBD_DAP_CDC_Setup
  *         Handle the CMSIS-DAP specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_DAP_CDC_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  uint16_t len = 0;
  uint8_t  *pbuf = NULL;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
    {
      case USB_REQ_RECIPIENT_INTERFACE:
        if (req->wIndex == 0) {
          return USBD_DAP_Setup(pdev, req);
        }
        else
        if (req->wIndex <= 2) {
          return USBD_CDC_Setup(pdev, req);
        }
        break;

      default:
        break;
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( req->wValue >> 8 == DAP_REPORT_DESC)
      {
        len = MIN(USBD_DAP_REPORT_DESC_SIZE , req->wLength);
        pbuf =  ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->pReport;
      }
      else if( req->wValue >> 8 == DAP_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_DAP_Desc;   
        len = MIN(USB_DAP_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&hdapcdc.AltSetting,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      hdapcdc.AltSetting = (uint8_t)(req->wValue);
      break;
    }
 
  default: 
    break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_DAP_CDC_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_DAP_CDC_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  switch (epnum) {
    case 1:
      /* Ensure that the FIFO is empty before a new transfer, this condition could 
         be caused by  a new transfer before the end of the previous transfer */
      hdapcdc.state = DAP_IDLE;
      return USBD_OK;

    case 2:
    case 3:
      hdapcdc.TxState = 0;
      return USBD_OK;

    default:
      break;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_DAP_CDC_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_DAP_CDC_DataOut (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  switch (epnum) {
    case 1:
      ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->OutEvent(hdapcdc.Report_buf);
      USBD_LL_PrepareReceive(pdev, DAP_EPOUT_ADDR , hdapcdc.Report_buf, 
                             USBD_DAP_OUTREPORT_BUF_SIZE);
      return USBD_OK;

    case 2:
    case 3:
      /* Get the received data length */
      hdapcdc.RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
  
      /* USB data will be immediately processed, this allow next USB traffic being 
        NAKed till the end of the application Xfer */
      ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->Receive(hdapcdc.RxBuffer, &hdapcdc.RxLength);

      return USBD_OK;

    default:
      break;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_DAP_CDC_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_DAP_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  switch (pdev->request.bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (pdev->request.bmRequest & USB_REQ_RECIPIENT_MASK)
      {
        case USB_REQ_RECIPIENT_INTERFACE:
          if (pdev->request.wIndex == 0) {
            if (hdapcdc.IsReportAvailable == 1)
            {
              ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->OutEvent(hdapcdc.Report_buf);
              hdapcdc.IsReportAvailable = 0;      
            }
            return USBD_OK;
          }
          else
          if (pdev->request.wIndex <= 2) {
            if((pdev->pUserData != NULL) && (hdapcdc.CmdOpCode != 0xFF))
            {
              ((USBD_DAP_CDC_ItfTypeDef*)pdev->pUserData)->Control(hdapcdc.CmdOpCode,
                                          (uint8_t *)hdapcdc.data,
                                          hdapcdc.CmdLength);
              hdapcdc.CmdOpCode = 0xFF;
            }
          }
          break;

        default:
          break;
      }
  }
  return 0;
}

/**
  * @brief  USBD_DAP_SendReport 
  *         Send DAP HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_DAP_SendReport     (USBD_HandleTypeDef  *pdev, 
                                 uint8_t *report,
                                 uint16_t len)
{
  if (pdev->dev_state == USBD_STATE_CONFIGURED )
  {
    if(hdapcdc.state == DAP_IDLE)
    {
      hdapcdc.state = DAP_BUSY;
      USBD_LL_Transmit (pdev, 
                        DAP_EPIN_ADDR,                                      
                        report,
                        len);
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  hdapcdc.TxBuffer = pbuff;
  hdapcdc.TxLength = length;  
  
  return USBD_OK;  
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
  hdapcdc.RxBuffer = pbuff;
  
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  if(hdapcdc.TxState == 0)
  {
    /* Tx Transfer in progress */
    hdapcdc.TxState = 1;
    
    /* Transmit next packet */
    USBD_LL_Transmit(pdev,
                     CDC_IN_EP,
                     hdapcdc.TxBuffer,
                     hdapcdc.TxLength);
    
    return USBD_OK;
  }
  else
  {
    return USBD_BUSY;
  }
}

/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  /* Suspend or Resume USB Out process */
  
  /* Prepare Out endpoint to receive next packet */
  USBD_LL_PrepareReceive(pdev,
                         CDC_OUT_EP,
                         hdapcdc.RxBuffer,
                         CDC_DATA_FS_OUT_PACKET_SIZE);
  
  return USBD_OK;
}

/**
  * @brief  USBD_DAP_CDC_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_DAP_CDC_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_DAP_CDC_CfgDesc);
  return USBD_DAP_CDC_CfgDesc;
}

/**
* @brief  USBD_DAP_CDC_RegisterInterface
  * @param  pdev: DAP_CDC device instance
  * @param  fops: DAP_CDC Interface callback
  * @retval status
  */
uint8_t  USBD_DAP_CDC_RegisterInterface  (USBD_HandleTypeDef *pdev,
                                             USBD_DAP_CDC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData= fops;

    ret = USBD_OK;    
  }
  
  return ret;
}
