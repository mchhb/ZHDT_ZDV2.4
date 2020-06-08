#ifndef  _W5500_H_
#define  _W5500_H_

#include "stdint.h"
#include "main.h"
extern void setkeepalive(uint8_t s);
extern uint8_t txsize[];
extern uint8_t rxsize[];
extern uint8_t socket_flag;
extern uint32_t sevbuffer_len;
extern uint8_t w5500_buffer[12];
extern uint8_t Cache_buffer[2048];
extern uint32_t w5500_connect_error;
extern uint16_t u8len;
extern uint32_t sev_data;
extern uint32_t cumulative_add;
extern uint8_t sev_crc;
extern uint32_t sevbuffer_len;
extern uint8_t sev_time_ok;
extern uint32_t test_sev_send;
extern uint32_t sev_text;
extern uint8_t dhcp_ok;
#define SOCKET	0

#define	MAX_SOCK_NUM		1
#define FW_VER_HIGH   	1
#define FW_VER_LOW    	0

#define ADC_CONVERTED_DATA_BUFFER_SIZE ((uint32_t)  6)
#define WIZ_RESET_HIGH	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET)
#define WIZ_RESET_LOW		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET)
#define WIZ_CS_HIGH		 	 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET)
#define WIZ_CS_LOW		 	 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET)
/**
 @brief Mode Register address
 * W5500 SPI Frame consists of 16bits Offset Address in Address Phase, 
 * 8bits Control Phase and N bytes Data Phase.
 * 0                8                16               24                   ~
 * |----------------|----------------|----------------|----------------------
 * |        16bit offset Address     | Control Bits   |  Data Phase
 *
 * The 8bits Control Phase is reconfigured with Block Select bits (BSB[4:0]), 
 * Read/Write Access Mode bit (RWB) and SPI Operation Mode (OM[1:0]). 
 * Block Select bits select a block as like common register, socket register, tx buffer and tx buffer.
 * Address value is defined as 16bit offset Address, BSB[4:0] and the three bits of zero-padding.(The RWB and OM [1:0] are '0 'padding)
 * Please, refer to W5500 datasheet for more detail about Memory Map.
 *
 */

/**
 @brief Mode Register address
 */
#define MR                          (0x000000)

/**
 @brief Gateway IP Register address
 */
#define GAR0                        (0x000100)
#define GAR1                        (0x000200)
#define GAR2                        (0x000300)
#define GAR3                        (0x000400)
/**
 @brief Subnet mask Register address
 */
#define SUBR0                       (0x000500)
#define SUBR1                       (0x000600)
#define SUBR2                       (0x000700)
#define SUBR3                       (0x000800)

/**
 @brief Source MAC Register address
 */
#define SHAR0                       (0x000900)
#define SHAR1                       (0x000A00)
#define SHAR2                       (0x000B00)
#define SHAR3                       (0x000C00)
#define SHAR4                       (0x000D00)
#define SHAR5                       (0x000E00)
/**
 @brief Source IP Register address
 */
#define SIPR0                       (0x000F00)
#define SIPR1                       (0x001000)
#define SIPR2                       (0x001100)
#define SIPR3                       (0x001200)
/**
 @brief set Interrupt low level timer register address
 */
#define INTLEVEL0                   (0x001300)
#define INTLEVEL1                   (0x001400)
/**
 @brief Interrupt Register
 */
#define IR                          (0x001500)
/**
 @brief Interrupt mask register
 */
#define IMR                         (0x001600)
/**
 @brief Socket Interrupt Register
 */
#define SIR                         (0x001700) 
/**
 @brief Socket Interrupt Mask Register
 */
#define SIMR                        (0x001800)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define RTR0                        (0x001900)
#define RTR1                        (0x001A00)
/**
 @brief Retry count reigster
 */
#define WIZ_RCR                         (0x001B00)
/**
 @briefPPP LCP Request Timer register  in PPPoE mode
 */
#define PTIMER                      (0x001C00)
/**
 @brief PPP LCP Magic number register  in PPPoE mode
 */
#define PMAGIC                      (0x001D00)
/**
 @brief PPP Destination MAC Register address
 */
#define PDHAR0                      (0x001E00)
#define PDHAR1                      (0x001F00)
#define PDHAR2                      (0x002000)
#define PDHAR3                      (0x002100)
#define PDHAR4                      (0x002200)
#define PDHAR5                      (0x002300)
/**
 @brief PPP Session Identification Register
 */
#define PSID0                       (0x002400)
#define PSID1                       (0x002500)
/**
 @brief PPP Maximum Segment Size(MSS) register
 */
#define PMR0                        (0x002600)
#define PMR1                        (0x002700)
/**
 @brief Unreachable IP register address in UDP mode
 */
#define UIPR0                       (0x002800)
#define UIPR1                       (0x002900)
#define UIPR2                       (0x002A00)
#define UIPR3                       (0x002B00)
/**
 @brief Unreachable Port register address in UDP mode
 */
#define UPORT0                      (0x002C00)
#define UPORT1                      (0x002D00)
/**
 @brief PHY Configuration Register
 */
#define PHYCFGR                      (0x002E00)
/**
 @brief chip version register address
 */
#define VERSIONR                    (0x003900)   


/*"0x00XXXX"��"0000 0000 XXXX XXXX XXXX XXXX"����16λΪ��ַ�Σ�ƫ�Ƶ�ַ���������˿��ƶ���ָ���Ĵ�����ƫ�Ƶ�ַ��
��8λΪ���ƶΣ�����ǰ5λΪ����ѡ��λBSB����1λΪR/W����2λΪOMλ����ָ��Ҫ���ĸ��Ĵ���������
"ch"����ΪSocket�Ĵ����ı�ţ���0~7��
		BSB��	00000 ͨ�üĴ���   			
					00001	Socket0�Ĵ���			ch=0
					00101	Socket1�Ĵ���			ch=1
					01001	Socket2�Ĵ���			ch=2
					01101 Socket3�Ĵ���			ch=3
					10001 Socket4�Ĵ���			ch=4
					10101 Socket5�Ĵ���			ch=5
					11001 Socket6�Ĵ���			ch=6
					11101 Socket7�Ĵ���			ch=7		*/


/**
 @brief socket Mode register
 */
#define Sn_MR(ch)                       (0x000008 + (ch<<5))

/**
 @brief channel Sn_CR register
 */
#define Sn_CR(ch)                       (0x000108 + (ch<<5))
/**
 @brief channel interrupt register
 */
#define Sn_IR(ch)                       (0x000208 + (ch<<5))
/**
 @brief channel status register
 */
#define Sn_SR(ch)                       (0x000308 + (ch<<5))
/**
 @brief source port register
 */
#define Sn_PORT0(ch)                    (0x000408 + (ch<<5))
#define Sn_PORT1(ch)                    (0x000508 + (ch<<5))
/**
 @brief Peer MAC register address
 */
#define Sn_DHAR0(ch)                    (0x000608 + (ch<<5))
#define Sn_DHAR1(ch)                    (0x000708 + (ch<<5))
#define Sn_DHAR2(ch)                    (0x000808 + (ch<<5))
#define Sn_DHAR3(ch)                    (0x000908 + (ch<<5))
#define Sn_DHAR4(ch)                    (0x000A08 + (ch<<5))
#define Sn_DHAR5(ch)                    (0x000B08 + (ch<<5))
/**
 @brief Peer IP register address
 */
#define Sn_DIPR0(ch)                    (0x000C08 + (ch<<5))
#define Sn_DIPR1(ch)                    (0x000D08 + (ch<<5))
#define Sn_DIPR2(ch)                    (0x000E08 + (ch<<5))
#define Sn_DIPR3(ch)                    (0x000F08 + (ch<<5))
/**
 @brief Peer port register address
 */
#define Sn_DPORT0(ch)                   (0x001008 + (ch<<5))
#define Sn_DPORT1(ch)                   (0x001108 + (ch<<5))
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define Sn_MSSR0(ch)                    (0x001208 + (ch<<5))
#define Sn_MSSR1(ch)                    (0x001308 + (ch<<5))
/** 
 @brief IP Type of Service(TOS) Register 
 */
#define Sn_TOS(ch)                      (0x001508 + (ch<<5))
/**
 @brief IP Time to live(TTL) Register 
 */
#define Sn_TTL(ch)                      (0x001608 + (ch<<5))
/**
 @brief Receive memory size reigster
 */
#define Sn_RXMEM_SIZE(ch)               (0x001E08 + (ch<<5))
/**
 @brief Transmit memory size reigster
 */
#define Sn_TXMEM_SIZE(ch)               (0x001F08 + (ch<<5))
/**
 @brief Transmit free memory size register
 */
#define Sn_TX_FSR0(ch)                  (0x002008 + (ch<<5))
#define Sn_TX_FSR1(ch)                  (0x002108 + (ch<<5))
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0(ch)                   (0x002208 + (ch<<5))
#define Sn_TX_RD1(ch)                   (0x002308 + (ch<<5))
/**
 @brief Transmit memory write pointer register address
 */
#define Sn_TX_WR0(ch)                   (0x002408 + (ch<<5))
#define Sn_TX_WR1(ch)                   (0x002508 + (ch<<5))
/**
 @brief Received data size register
 */
#define Sn_RX_RSR0(ch)                  (0x002608 + (ch<<5))
#define Sn_RX_RSR1(ch)                  (0x002708 + (ch<<5))
/**
 @brief Read point of Receive memory
 */
#define Sn_RX_RD0(ch)                   (0x002808 + (ch<<5))
#define Sn_RX_RD1(ch)                   (0x002908 + (ch<<5))
/**
 @brief Write point of Receive memory
 */
#define Sn_RX_WR0(ch)                   (0x002A08 + (ch<<5))
#define Sn_RX_WR1(ch)                   (0x002B08 + (ch<<5))
/**
 @brief socket interrupt mask register
 */
#define Sn_IMR(ch)                      (0x002C08 + (ch<<5))
/**
 @brief frag field value in IP header register
 */
#define Sn_FRAG(ch)                     (0x002D08 + (ch<<5))
/**
 @brief Keep Timer register
 */
#define Sn_KPALVTR(ch)                  (0x002F08 + (ch<<5))

/* MODE register values */
#define MR_RST                       0x80 /**< reset */
#define MR_WOL                       0x20 /**< Wake on Lan */
#define MR_PB                        0x10 /**< ping block */
#define MR_PPPOE                     0x08 /**< enable pppoe */
#define MR_UDP_FARP                  0x02 /**< enbale FORCE ARP */


/* IR register values */
#define IR_CONFLICT                  0x80 /**< check ip confict */
#define IR_UNREACH                   0x40 /**< get the destination unreachable message in UDP sending */
#define IR_PPPoE                     0x20 /**< get the PPPoE close message */
#define IR_MAGIC                     0x10 /**< get the magic packet interrupt */

/* Sn_MR values */
#define Sn_MR_CLOSE                  0x00     /**< unused socket */
#define Sn_MR_TCP                    0x01     /**< TCP */
#define Sn_MR_UDP                    0x02     /**< UDP */
#define Sn_MR_IPRAW                  0x03      /**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW                 0x04      /**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE                  0x05     /**< PPPoE */
#define Sn_MR_UCASTB                 0x10     /**< Unicast Block in UDP Multicating*/
#define Sn_MR_ND                     0x20     /**< No Delayed Ack(TCP) flag */
#define Sn_MR_MC                     0x20     /**< Multicast IGMP (UDP) flag */
#define Sn_MR_BCASTB                 0x40     /**< Broadcast blcok in UDP Multicating */
#define Sn_MR_MULTI                  0x80     /**< support UDP Multicating */

 /* Sn_MR values on MACRAW MODE */
#define Sn_MR_MIP6N                  0x10     /**< IPv6 packet Block */
#define Sn_MR_MMB                    0x20     /**< IPv4 Multicasting Block */
//#define Sn_MR_BCASTB                 0x40     /**< Broadcast blcok */
#define Sn_MR_MFEN                   0x80     /**< support MAC filter enable */


/* Sn_CR values */
#define Sn_CR_OPEN                   0x01     /**< initialize or open socket */
#define Sn_CR_LISTEN                 0x02     /**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT                0x04     /**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON                 0x08     /**< send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE                  0x10     /**< close socket */
#define Sn_CR_SEND                   0x20     /**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC               0x21     /**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP              0x22     /**<  send keep alive message */
#define Sn_CR_RECV                   0x40     /**< update rxbuf pointer, recv data */

#ifdef __DEF_IINCHIP_PPP__
   #define Sn_CR_PCON                0x23      
   #define Sn_CR_PDISCON             0x24      
   #define Sn_CR_PCR                 0x25      
   #define Sn_CR_PCN                 0x26     
   #define Sn_CR_PCJ                 0x27     
#endif

/* Sn_IR values */
#ifdef __DEF_IINCHIP_PPP__
   #define Sn_IR_PRECV               0x80     
   #define Sn_IR_PFAIL               0x40     
   #define Sn_IR_PNEXT               0x20     
#endif

#define Sn_IR_SEND_OK                0x10     /**< complete sending */
#define Sn_IR_TIMEOUT                0x08     /**< assert timeout */
#define Sn_IR_RECV                   0x04     /**< receiving data */
#define Sn_IR_DISCON                 0x02     /**< closed socket */
#define Sn_IR_CON                    0x01     /**< established connection */

/* Sn_SR values */
#define SOCK_CLOSED                  0x00     /**< closed */
#define SOCK_INIT                    0x13     /**< init state */
#define SOCK_LISTEN                  0x14     /**< listen state */
#define SOCK_SYNSENT                 0x15     /**< connection state */
#define SOCK_SYNRECV                 0x16     /**< connection state */
#define SOCK_ESTABLISHED             0x17     /**< success to connect */
#define SOCK_FIN_WAIT                0x18     /**< closing state */
#define SOCK_CLOSING                 0x1A     /**< closing state */
#define SOCK_TIME_WAIT               0x1B     /**< closing state */
#define SOCK_CLOSE_WAIT              0x1C     /**< closing state */
#define SOCK_LAST_ACK                0x1D     /**< closing state */
#define SOCK_UDP                     0x22     /**< udp socket */
#define SOCK_IPRAW                   0x32     /**< ip raw mode socket */
#define SOCK_MACRAW                  0x42     /**< mac raw mode socket */
#define SOCK_PPPOE                   0x5F     /**< pppoe socket */

/* IP PROTOCOL */
#define IPPROTO_IP                   0        /**< Dummy for IP */
#define IPPROTO_ICMP                 1        /**< Control message protocol */
#define IPPROTO_IGMP                 2        /**< Internet group management protocol */
#define IPPROTO_GGP                  3        /**< Gateway^2 (deprecated) */
#define IPPROTO_TCP                  6        /**< TCP */
#define IPPROTO_PUP                  12       /**< PUP */
#define IPPROTO_UDP                  17       /**< UDP */
#define IPPROTO_IDP                  22       /**< XNS idp */
#define IPPROTO_ND                   77       /**< UNOFFICIAL net disk protocol */
#define IPPROTO_RAW                  255      /**< Raw IP packet */

/*********************************************************
* iinchip access function
*********************************************************/
void IINCHIP_WRITE( uint32_t addrbsb,  uint8_t data);
uint8_t IINCHIP_READ(uint32_t addrbsb);
uint16_t wiz_write_buf(uint32_t addrbsb,uint8_t* buf,uint16_t len);
uint16_t wiz_read_buf(uint32_t addrbsb, uint8_t* buf,uint16_t len);


void iinchip_init(void); // reset iinchip
void sysinit(uint8_t * tx_size, uint8_t * rx_size); // setting tx/rx buf size
uint8_t getISR(uint8_t s);
void putISR(uint8_t s, uint8_t val);
uint16_t getIINCHIP_RxMAX(uint8_t s);
uint16_t getIINCHIP_TxMAX(uint8_t s);
void setMR(uint8_t val);
void setRTR(uint16_t timeout); // set retry duration for data transmission, connection, closing ...
void setRCR(uint8_t retry); // set retry count (above the value, assert timeout interrupt)
void clearIR(uint8_t mask); // clear interrupt
uint8_t getIR( void );
void setSn_MSS(uint8_t s, uint16_t Sn_MSSR); // set maximum segment size
uint8_t getSn_IR(uint8_t s); // get socket interrupt status
uint8_t getSn_SR(uint8_t s); // get uint8_t status
uint16_t getSn_TX_FSR(uint8_t s); // get socket TX free buf size
uint16_t getSn_RX_RSR(uint8_t s); // get socket RX recv buf size
uint8_t getSn_SR(uint8_t s);
void setSn_TTL(uint8_t s, uint8_t ttl);
void send_data_processing(uint8_t s, uint8_t *wizdata, uint16_t len);
void recv_data_processing(uint8_t s, uint8_t *wizdata, uint16_t len);

void setGAR(uint8_t * addr); // set gateway address
void setSUBR(uint8_t * addr); // set subnet mask address
void setSHAR(uint8_t * addr); // set local MAC address
void setSIPR(uint8_t * addr); // set local IP address
void getGAR(uint8_t * addr);
void getSUBR(uint8_t * addr);
void getSHAR(uint8_t * addr);
void getSIPR(uint8_t * addr);
void w5500_check(void);
void setSn_IR(uint8_t s, uint8_t val);
uint8_t SPI2_SendByte(uint8_t SPI2_txbuffer);
void Reset_W5500(void);
void set_w5500_mac(void);
void set_w5500_ip(void);
void do_tcp_client(void);
void u32_to_u8(uint32_t *u32data,uint8_t *u8data, uint32_t u16len);
void GetLockCode(void);
/**
 @brief WIZCHIP_OFFSET_INC on IINCHIP_READ/WRITE
 * case1.
 *  IINCHIP_WRITE(RTR0,val);
 *  IINCHIP_WRITE(RTR1,val);
 * case1. 
 *  IINCHIP_WRITE(RTR0,val);
 *  IINCHIP_WRITE(WIZCHIP_OFFSET_INC(RTR0,1)); 
 */
//#define WIZCHIP_OFFSET_INC(ADDR, N)    (ADDR + (N<<8)) //< Increase offset address


#endif
