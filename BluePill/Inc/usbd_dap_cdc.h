/* Define to prevent recursive inclusion -------------------------------------*/ 
#ifndef __USB_DAP_CDC_H
#define __USB_DAP_CDC_H

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

#define USB_DAP_CDC_CONFIG_DESC_SIZ   107
/* DAP + CDC(IA) : 107 */
/* DAP:41 */
/* CDC:67 */

#define DAP_EPIN_ADDR                 0x81  /* EP1 for data IN */
#define DAP_EPOUT_ADDR                0x01  /* EP1 for data OUT */
#define CDC_IN_EP                     0x82  /* EP2 for data IN */
#define CDC_OUT_EP                    0x02  /* EP2 for data OUT */
#define CDC_CMD_EP                    0x83  /* EP3 for CDC commands */

#define DAP_EPIN_SIZE                 0x40
#define DAP_EPOUT_SIZE                0x40

#define CDC_DATA_FS_MAX_PACKET_SIZE   64  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE            8  /* Control Endpoint Packet size */ 

#define CDC_DATA_FS_IN_PACKET_SIZE    CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE   CDC_DATA_FS_MAX_PACKET_SIZE


#define USB_DAP_DESC_SIZ              9

#define DAP_DESCRIPTOR_TYPE           0x21
#define DAP_REPORT_DESC               0x22


#define DAP_REQ_SET_PROTOCOL          0x0B
#define DAP_REQ_GET_PROTOCOL          0x03

#define DAP_REQ_SET_IDLE              0x0A
#define DAP_REQ_GET_IDLE              0x02

#define DAP_REQ_SET_REPORT            0x09
#define DAP_REQ_GET_REPORT            0x01


#define CDC_DATA_MAX_PACKET_SIZE      64  /* Endpoint IN & OUT Packet size */

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23


typedef enum
{
  DAP_IDLE = 0,
  DAP_BUSY,
}
DAP_StateTypeDef; 

typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}USBD_CDC_LineCodingTypeDef;

typedef struct _USBD_DAP_CDC_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* OutEvent)      (uint8_t*);

  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);
  int8_t (* Receive)       (uint8_t *, uint32_t *);
}USBD_DAP_CDC_ItfTypeDef;

typedef struct
{
  uint8_t           Report_buf[USBD_DAP_OUTREPORT_BUF_SIZE];
  uint32_t          Protocol;
  uint32_t          IdleState;
  uint32_t          AltSetting;
  uint32_t          IsReportAvailable;
  DAP_StateTypeDef  state;

  uint32_t          data[CDC_DATA_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t           CmdOpCode;
  uint8_t           CmdLength;    
  uint8_t           *RxBuffer;  
  uint8_t           *TxBuffer;   
  uint32_t          RxLength;
  uint32_t          TxLength;    
  
  __IO uint32_t     TxState;     
  __IO uint32_t     RxState;  
}
USBD_DAP_CDC_HandleTypeDef; 

extern USBD_ClassTypeDef  USBD_DAP_CDC;

uint8_t USBD_DAP_CDC_RegisterInterface  (USBD_HandleTypeDef *pdev,
                                          USBD_DAP_CDC_ItfTypeDef *fops);

uint8_t USBD_DAP_SendReport (USBD_HandleTypeDef *pdev, 
                              uint8_t *report,
                              uint16_t len);

uint8_t  USBD_CDC_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_CDC_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_CDC_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_CDC_TransmitPacket     (USBD_HandleTypeDef *pdev);

#endif  /* __USB_DAP_CDC_H */
