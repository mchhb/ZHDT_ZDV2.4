#include <stdio.h>
#include <string.h>
#include "w5500.h"
#include "socket.h"
#include "dhcp.h"
#include "w5500_config.h"

uint16_t revbuffer_len;
uint8_t w5500_buffer[12];
uint8_t Cache_buffer[2048];
uint32_t sevbuffer_len = ADC_CONVERTED_DATA_BUFFER_SIZE;
uint16_t u8len = 9;
uint32_t test_sev_send = 0;
uint8_t sev_time_ok = 0;
uint8_t socket_flag = 1;
uint8_t sev_crc = 0;
uint8_t dhcp_ok = 0;
CONFIG_MSG  ConfigMsg, RecvMsg;
unsigned char id_name[4];
//uint8_t  BD_TG_server_ip[4] = {172,16,40,39}; //{172,16,40,144};       //BD_TG服务器IP地址
uint8_t  BD_TG_server_ip[4] = {222,32,108,88};//22.32.108.88
//uint8_t  BD_TG_server_ip[4] = {10,10,10,10};
//uint16_t BD_TG_server_port  = 8080;//8888; 
//uint16_t BD_TG_server_port  = 61619;
uint16_t BD_TG_server_port  = 61619;
//uint8_t  BD_TG_server_ip[4] = {172,16,40,144}; //{172,16,40,144};       //BD_TG服务器IP地址
//uint16_t BD_TG_server_port  = 8888;//8888; 
uint8_t ip_from = IP_FROM_DEFINE;
uint8_t mac[6];
uint8_t test_ip[4]={0};
uint8_t local_ip[4]={192,168,8,141};								 /*定义W5500默认IP地址*/
uint8_t subnet[4]={255,255,255,0};									 /*定义W5500默认子网掩码*/
uint8_t gateway[4]={192,168,8,1};										 /*定义W5500默认网关*/
uint8_t dns_server[4]={103,44,168,6};								 /*定义W5500默认DNS*/
uint16_t local_port = 6000;	                    		 /*定义本地端口*/
//uint8_t dns_server[4]={5,5,5,5};
uint32_t CpuID[3];
uint32_t Lock_Code;
uint32_t cumulative_add = 0;
static uint8_t I_STATUS[MAX_SOCK_NUM];
static uint16_t SSIZE[MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static uint16_t RSIZE[MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */
uint8_t txsize[MAX_SOCK_NUM] = {16};		// 选择8个Socket每个Socket发送缓存的大小，在w5500.c的void sysinit()有设置过程
uint8_t rxsize[MAX_SOCK_NUM] = {16};		// 选择8个Socket每个Socket接收缓存的大小，在w5500.c的void sysinit()有设置过程
uint64_t w5500_connect_error = 0;
uint32_t sev_data = 0;
uint32_t sev_text = 0;
uint8_t getISR(uint8_t s)
{
  return I_STATUS[s];
}
void putISR(uint8_t s, uint8_t val)
{
   I_STATUS[s] = val;
}

uint16_t getIINCHIP_RxMAX(uint8_t s)
{
   return RSIZE[s];
}
uint16_t getIINCHIP_TxMAX(uint8_t s)
{
   return SSIZE[s];
}
void IINCHIP_CSoff(void)
{
		WIZ_CS_LOW;
}
void IINCHIP_CSon(void)
{
		WIZ_CS_HIGH;
}
uint8_t  IINCHIP_SpiSendData(uint8_t dat)
{
   return(SPI2_SendByte(dat));
}

void IINCHIP_WRITE( uint32_t addrbsb,  uint8_t data)
{
  // IINCHIP_ISR_DISABLE();                        // Interrupt Service Routine Disable
   IINCHIP_CSoff();                              // CS=0, SPI start
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);// Address byte 1
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);// Address byte 2
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8) + 4);    // Data write command and Write data length 1
   IINCHIP_SpiSendData(data);                    // Data write (write 1byte data)
   IINCHIP_CSon();                               // CS=1,  SPI end
 //  IINCHIP_ISR_ENABLE();                         // Interrupt Service Routine Enable
}

uint8_t IINCHIP_READ(uint32_t addrbsb)
{
   uint8_t data = 0;
  // IINCHIP_ISR_DISABLE();                        //  Service Routine Disable
   IINCHIP_CSoff();                              // CS=0, SPI start
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);// Address byte 1
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);// Address byte 2
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8))    ;// Data read command and Read data length 1
   data = IINCHIP_SpiSendData(0x00);             // Data read (read 1byte data)
//	printf("data is %d\r\n",data);
   IINCHIP_CSon();                               // CS=1,  SPI end
  // IINCHIP_ISR_ENABLE();                         // Interrupt Service Routine Enable
   return data;    
}
void set_w5500_ip(void)
{	
	/*使用DHCP获取IP参数，需调用DHCP子函数*/		
	if(ip_from==IP_FROM_DEFINE)	
	{
		printf(" 使用定义的IP信息配置W5500:\r\n");
		//memcpy(ConfigMsg.mac, mac, 6);
		local_ip[3] = id_name[3];
		memcpy(ConfigMsg.lip,local_ip,4);
		memcpy(ConfigMsg.sub,subnet,4);
		memcpy(ConfigMsg.gw,gateway,4);
		memcpy(ConfigMsg.dns,dns_server,4);
	}
		
	/*以下配置信息，根据需要选用*/	
	ConfigMsg.sw_ver[0]=FW_VER_HIGH;
	ConfigMsg.sw_ver[1]=FW_VER_LOW;	

	/*将IP配置信息写入W5500相应寄存器*/	
	setSUBR(ConfigMsg.sub);
	setGAR(ConfigMsg.gw);
	setSIPR(ConfigMsg.lip);
	
	getSIPR (local_ip);			
	printf(" W5500 IP地址   : %d.%d.%d.%d\r\n", local_ip[0],local_ip[1],local_ip[2],local_ip[3]);
	getSUBR(subnet);
	printf(" W5500 子网掩码 : %d.%d.%d.%d\r\n", subnet[0],subnet[1],subnet[2],subnet[3]);
	getGAR(gateway);
	printf(" W5500 网关     : %d.%d.%d.%d\r\n", gateway[0],gateway[1],gateway[2],gateway[3]);
	dhcp_ok = 1;
}
uint16_t wiz_write_buf(uint32_t addrbsb,uint8_t* buf,uint16_t len)
{
   uint16_t idx = 0;
   if(len == 0) printf("Unexpected2 length 0\r\n");

//   IINCHIP_ISR_DISABLE();
   IINCHIP_CSoff();                              // CS=0, SPI start
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);// Address byte 1
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);// Address byte 2
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8) + 4);    // Data write command and Write data length 1
   for(idx = 0; idx < len; idx++)                // Write data in loop
   {
     IINCHIP_SpiSendData(buf[idx]);
   }
   IINCHIP_CSon();                               // CS=1, SPI end
//   IINCHIP_ISR_ENABLE();                         // Interrupt Service Routine Enable    

   return len;  
}

uint16_t wiz_read_buf(uint32_t addrbsb, uint8_t* buf,uint16_t len)
{
  uint16_t idx = 0;
  if(len == 0)
  {
    printf("Unexpected2 length 0\r\n");
  }

//  IINCHIP_ISR_DISABLE();
  //SPI MODE I/F
  IINCHIP_CSoff();                                  // CS=0, SPI start
  IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);// Address byte 1
  IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);// Address byte 2
  IINCHIP_SpiSendData( (addrbsb & 0x000000F8));    // Data write command and Write data length 1
  for(idx = 0; idx < len; idx++)                    // Write data in loop
  {
    buf[idx] = IINCHIP_SpiSendData(0x00);
  }
  IINCHIP_CSon();                                   // CS=1, SPI end
 // IINCHIP_ISR_ENABLE();                             // Interrupt Service Routine Enable
  
  return len;
}


/**
@brief  This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/
void iinchip_init(void)
{
  setMR( MR_RST );
#ifdef __DEF_IINCHIP_DBG__
  printf("MR value is %02x \r\n",IINCHIP_READ_COMMON(MR));
#endif
}

/**
@brief  This function set the transmit & receive buffer size as per the channels is used
Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n
bit 9-8 : memory size of channel #4 \n
bit 11-10 : memory size of channel #5 \n
bit 12-12 : memory size of channel #6 \n
bit 15-14 : memory size of channel #7 \n
W5500的Tx, Rx的最大寄存器宽度是16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and receiption from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to centain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/
void sysinit( uint8_t * tx_size, uint8_t * rx_size  )
{
  int16_t i;
  int16_t ssum,rsum;
#ifdef __DEF_IINCHIP_DBG__
  printf("sysinit()\r\n");
#endif
  ssum = 0;
  rsum = 0;

  for (i = 0 ; i < MAX_SOCK_NUM; i++)       // Set the size, masking and base address of Tx & Rx memory by each channel
  {
          IINCHIP_WRITE( (Sn_TXMEM_SIZE(i)), tx_size[i]);		// MCU记录每个Socket发送缓存的大小到数组tx_size[]对应的元素值
          IINCHIP_WRITE( (Sn_RXMEM_SIZE(i)), rx_size[i]);		// MCU记录每个Socket接收缓存的大小到数组rx_size[]对应的元素值
          
#ifdef __DEF_IINCHIP_DBG__
         printf("tx_size[%d]: %d, Sn_TXMEM_SIZE = %d\r\n",i, tx_size[i], IINCHIP_READ(Sn_TXMEM_SIZE(i)));
         printf("rx_size[%d]: %d, Sn_RXMEM_SIZE = %d\r\n",i, rx_size[i], IINCHIP_READ(Sn_RXMEM_SIZE(i)));
#endif
    SSIZE[i] = (int16_t)(0);
    RSIZE[i] = (int16_t)(0);


    if (ssum <= 16384)
    {
         switch( tx_size[i] )
      {
      case 1:
        SSIZE[i] = (int16_t)(1024);
        break;
      case 2:
        SSIZE[i] = (int16_t)(2048);
        break;
      case 4:
        SSIZE[i] = (int16_t)(4096);
        break;
      case 8:
        SSIZE[i] = (int16_t)(8192);
        break;
      case 16:
        SSIZE[i] = (int16_t)(16384);
      break;
      default :
        RSIZE[i] = (int16_t)(2048);
        break;
      }
    }

   if (rsum <= 16384)
    {
         switch( rx_size[i] )
      {
      case 1:
        RSIZE[i] = (int16_t)(1024);
        break;
      case 2:
        RSIZE[i] = (int16_t)(2048);
        break;
      case 4:
        RSIZE[i] = (int16_t)(4096);
        break;
      case 8:
        RSIZE[i] = (int16_t)(8192);
        break;
      case 16:
        RSIZE[i] = (int16_t)(16384);
        break;
      default :
        RSIZE[i] = (int16_t)(2048);
        break;
      }
    }
    ssum += SSIZE[i];
    rsum += RSIZE[i];

  }
}

// added

/*.
*/
void setGAR(
  uint8_t * addr  /**< a pointer to a 4 -byte array responsible to set the Gateway IP address. */
  )
{
    wiz_write_buf(GAR0, addr, 4);
}
void getGWIP(uint8_t * addr)
{
    wiz_read_buf(GAR0, addr, 4);
}

/**
@brief  It sets up SubnetMask address
*/
void setSUBR(uint8_t * addr)
{   
    wiz_write_buf(SUBR0, addr, 4);
}
/**
@brief  This function sets up MAC address.
*/
void setSHAR(
  uint8_t * addr  /**< a pointer to a 6 -byte array responsible to set the MAC address. */
  )
{
  wiz_write_buf(SHAR0, addr, 6);  
}

/**
@brief  This function sets up Source IP address.
*/
void setSIPR(
  uint8_t * addr  /**< a pointer to a 4 -byte array responsible to set the Source IP address. */
  )
{
    wiz_write_buf(SIPR0, addr, 4);  
}

/**
@brief  W5500心跳检测程序，设置Socket在线时间寄存器Sn_KPALVTR，单位为5s
*/
void setkeepalive(uint8_t s)
{ 
  IINCHIP_WRITE(Sn_KPALVTR(s),0x02);
}

/**
@brief  This function sets up Source IP address.
*/
void getGAR(uint8_t * addr)
{
    wiz_read_buf(GAR0, addr, 4);
}
void getSUBR(uint8_t * addr)
{
    wiz_read_buf(SUBR0, addr, 4);
}
void getSHAR(uint8_t * addr)
{
    wiz_read_buf(SHAR0, addr, 6);
}
void getSIPR(uint8_t * addr)
{
    wiz_read_buf(SIPR0, addr, 4);
}

void setMR(uint8_t val)
{
  IINCHIP_WRITE(MR,val);
}

/**
@brief  This function gets Interrupt register in common register.
 */
uint8_t getIR( void )
{
   return IINCHIP_READ(IR);
}

/**
@brief  This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission
will be there as per RTR (Retry Time-value Register)setting
*/
void setRTR(uint16_t timeout)
{
  IINCHIP_WRITE(RTR0,(uint8_t)((timeout & 0xff00) >> 8));
  IINCHIP_WRITE(RTR1,(uint8_t)(timeout & 0x00ff));
}

/**
@brief  This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time
as per RTR & RCR register seeting then time out will occur.
*/
void setRCR(uint8_t retry)
{
  IINCHIP_WRITE(WIZ_RCR,retry);
}

/**
@brief  This function set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)

If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
set in IR register.
*/
void clearIR(uint8_t mask)
{
  IINCHIP_WRITE(IR, ~mask | getIR() ); // must be setted 0x10.
}

/**
@brief  This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(uint8_t s, uint16_t Sn_MSSR)
{
  IINCHIP_WRITE( Sn_MSSR0(s), (uint8_t)((Sn_MSSR & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_MSSR1(s), (uint8_t)(Sn_MSSR & 0x00ff));
}

void setSn_TTL(uint8_t s, uint8_t ttl)
{    
   IINCHIP_WRITE( Sn_TTL(s) , ttl);
}



/**
@brief  get socket interrupt status

These below functions are used to read the Interrupt & Soket Status register
*/
uint8_t getSn_IR(uint8_t s)
{
   return IINCHIP_READ(Sn_IR(s));
}


/**
@brief   get uint8_t status
*/
uint8_t getSn_SR(uint8_t s)
{
   return IINCHIP_READ(Sn_SR(s)); // MCU读Sn_SR对应地址里面的数值并返回
}


/**
@brief  get socket TX free buf size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User shuold check this value first and control the size of transmitting data
*/
uint16_t getSn_TX_FSR(uint8_t s)
{
  uint16_t val=0,val1=0;
  do
  {
    val1 = IINCHIP_READ(Sn_TX_FSR0(s));
    val1 = (val1 << 8) + IINCHIP_READ(Sn_TX_FSR1(s));
      if (val1 != 0)
    {
        val = IINCHIP_READ(Sn_TX_FSR0(s));
        val = (val << 8) + IINCHIP_READ(Sn_TX_FSR1(s));
    }
  } while (val != val1);
   return val;
}


/**
@brief   get socket RX recv buf size

This gives size of received data in receive buffer.
*/
uint16_t getSn_RX_RSR(uint8_t s)		// 获取Sn_RX_RSR空闲接收缓存寄存器的值并返回,Sn_RX_RSR中保存的是接收缓存中已接收和保存的数据大小
{
  uint16_t val=0,val1=0;
  do
  {
    val1 = IINCHIP_READ(Sn_RX_RSR0(s));		
    val1 = (val1 << 8) + IINCHIP_READ(Sn_RX_RSR1(s));		// MCU读出Sn_RX_RSR的值赋给val1
    if(val1 != 0)																				// 如果Sn_RX_RSR的值不为0，说明接收缓存中收到了数据
    {
        val = IINCHIP_READ(Sn_RX_RSR0(s));				
        val = (val << 8) + IINCHIP_READ(Sn_RX_RSR1(s));	// MCU读出Sn_RX_RSR的值赋给val
    }
  } 
	while (val != val1);																	// 此时应该val=val1,表达式为假，跳出do while循环
  return val;																						// 返回val
}


/**
@brief   This function is being called by send() and sendto() function also.

This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
*/
void send_data_processing(uint8_t s, uint8_t *data, uint16_t len)		// MCU把数据发送给W5500的过程，W5500将数据写入buf，并更新数据的写指针的地址
{
  uint16_t ptr =0;
  uint32_t addrbsb =0;
//	printf("send_data_processing = %d\r\n",len);
  if(len == 0)
  {
    //printf("CH: %d Unexpected1 length 0\r\n", s);
    return;
  }

// Sn_RX_WR是发送写指针寄存器，用来保存发送缓存中将要传输数据的首地址 
  ptr = IINCHIP_READ( Sn_TX_WR0(s) );
  ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_TX_WR1(s));			// MCU读取Sn_RX_WR寄存器的值

  addrbsb = (uint32_t)(ptr<<8) + (s<<5) + 0x10;										// 将发送缓存中将要传输数据的首地址转成32位
  wiz_write_buf(addrbsb, data, len);														// W5500从该首地址开始写入数据，数据长度为len
  
  ptr += len;																										// 首地址的值+len变为数据新的首地址
  IINCHIP_WRITE( Sn_TX_WR0(s) ,(uint8_t)((ptr & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_TX_WR1(s),(uint8_t)(ptr & 0x00ff));						// 将新的首地址保存在Sn_RX_WR中
}


/**
@brief  This function is being called by recv() also.

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
void recv_data_processing(uint8_t s, uint8_t *data, uint16_t len)
{
  uint16_t ptr = 0;
  uint32_t addrbsb = 0;
  
  if(len == 0)
  {
    printf("CH: %d Unexpected2 length 0\r\n", s);
    return;
  }
// Sn_RX_RD是发送读指针寄存器
  ptr = IINCHIP_READ( Sn_RX_RD0(s) );			
  ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ( Sn_RX_RD1(s) );			// MCU读取Sn_RX_RD寄存器的值

  addrbsb = (uint32_t)(ptr<<8) + (s<<5) + 0x18;					// 将发送缓存中将要传输数据的首地址转成32位
  wiz_read_buf(addrbsb, data, len);										// W5500从该首地址开始读取数据，数据长度为len
  ptr += len;																					// 首地址的值+len变为数据新的首地址

  IINCHIP_WRITE( Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));		
  IINCHIP_WRITE( Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));		// 将新的首地址保存在Sn_RX_RD中
}

void setSn_IR(uint8_t s, uint8_t val)
{
    IINCHIP_WRITE(Sn_IR(s), val);
}
uint8_t getPHYStatus( void )
{
 return IINCHIP_READ(PHYCFGR);
}
/**
*@brief		检测物理层连接
*@param		无
*@return	无
*/
extern uint8_t rxd_buffer_locked;
void w5500_check(void)
{
	uint8_t cot = 0;
	uint8_t PHY_connect=0;  
    PHY_connect=getPHYStatus();
	PHY_connect=0x01&getPHYStatus();
	if(PHY_connect==0)
	{
		LED0_OFF;
		LED1_ON;
		printf(" \r\n 请检查网线是否连接?\r\n");
		while(cot < 8)
		{
			close(cot++);
		}
		while(PHY_connect == 0) { 
			PHY_connect=0x01&getPHYStatus();	
			printf(" .");
			HAL_Delay(500);
		}
		//dhcp_ok = 0;
		printf(" \r\n");	
		printf("Cable Connect Success.\r\n");
	}
}

void Reset_W5500(void)
{
  WIZ_RESET_LOW;
  HAL_Delay(500);  
  WIZ_RESET_HIGH;
  HAL_Delay(500);
}
uint8_t SPI2_SendByte(uint8_t SPI2_txbuffer)
{
		uint8_t SPI2_rxbuffer;
		HAL_SPI_TransmitReceive(&hspi2,&SPI2_txbuffer,&SPI2_rxbuffer,1,1000);
		return SPI2_rxbuffer;
}
void GetLockCode(void)
{
	CpuID[0]=HAL_GetUIDw0();
	CpuID[1]=HAL_GetUIDw1();
	CpuID[2]=HAL_GetUIDw2();
	Lock_Code=(CpuID[0]>>1)+(CpuID[1]>>2)+(CpuID[2]>>3);
	id_name[0] = Lock_Code >> 24;
	id_name[1] = Lock_Code >> 16;
	id_name[2] = Lock_Code >> 8;
	id_name[3] = Lock_Code &0xff;
	local_port = Lock_Code & 0xffff;
	printf("设备id：%d,端口: %d\r\n",Lock_Code,local_port);
}
void set_w5500_mac(void)
{
	mac[2] = id_name[0];
	mac[3] = id_name[1];
	mac[4] = id_name[2];
	mac[5] = id_name[3];
	memcpy(ConfigMsg.mac, mac, 6);
	setSHAR(ConfigMsg.mac);	/**/
}
/**
*@brief		配置W5500的IP地址
*@param		无
*@return	无
*/
void do_tcp_client(void)
{	
			switch(getSn_SR(0))										// 获取socket0的状态
			{
				case SOCK_INIT:											// Socket处于初始化完成(打开)状态
					//	listen(0);											//    监听刚刚打开的本地端口，等待客户端连接
					//socket_flag = 1;
					if(socket_flag == 1)
					{
						socket_flag++;
						printf("W5500 To Connected. ip:%d.%d.%d.%d,port:%d\r\n",BD_TG_server_ip[0],BD_TG_server_ip[1],BD_TG_server_ip[2],BD_TG_server_ip[3],BD_TG_server_port);
					}	
					connect(0,BD_TG_server_ip,BD_TG_server_port);
					HAL_Delay(1000);
				break;
				case SOCK_ESTABLISHED:							// Socket处于连接建立状态
						w5500_connect_error = 0;
						if(getSn_IR(0) & Sn_IR_CON)			
						{
							setSn_IR(0, Sn_IR_CON);				// Sn_IR的CON位置1，通知W5500连接已建立
							if(socket_flag == 2)
							{
								socket_flag = 0;
								reset_cmd = 0;
								printf("W5500 Connect Success.\r\n");
								LED0_ON;
								LED1_OFF;
							}
						}
						// 数据回环测试程序：数据从上位机客户端发给W5500，W5500接收到数据后再回给客户端
						revbuffer_len=getSn_RX_RSR(0);						// 读取W5500空闲接收缓存寄存器的值并赋给len，Sn_RX_RSR表示接收缓存中已接收和保存的数据大小
						if(revbuffer_len>0)
						{
							recv(0,w5500_buffer,revbuffer_len);						// W5500接收来自客户端的数据，并通过SPI发送给MCU
							printf("接收到的数据：%d\r\n",revbuffer_len);			// 串口打印接收到的数据
							send(0,w5500_buffer,revbuffer_len);						// 接收到数据后再回给客户端，完成数据回�
						}
						// W5500从串口发数据给客户端程序，数据需以回车结束
//						if(USART_RX_STA & 0x8000)				// 判断串口数据是否接收完成
//						{					   
//							len=USART_RX_STA & 0x3fff;		// 获取串口接收到数据的长度
//							send(0,USART_RX_BUF,len);			// W5500向客户端发送数据
//							USART_RX_STA=0;								// 串口接收状态标志位清0
//							memset(USART_RX_BUF,0,len+1);	// 串口接收缓存清0
//						}
				break;
				case SOCK_CLOSE_WAIT:								// Socket处于等待关闭状态
						close(0);												// 关闭Socket0
				break;
				case SOCK_CLOSED:										// Socket处于关闭状态
						socket_flag = 1;
						socket(0,Sn_MR_TCP,local_port,Sn_MR_ND);		// 打开Socket0，并配置为TCP无延时模式，打开一个本地端口
				break;
				default:
					close(0);
				break;
			}
}
void u32_to_u8(uint32_t *u32data,uint8_t *u8data, uint32_t u16len)
{
//	u8data[4] = ++sev_crc;
//	u8data[5] = cumulative_add >> 54;
//	u8data[6] = cumulative_add >> 48;
//	u8data[7] = cumulative_add >> 40;
//	u8data[8] = cumulative_add >> 32;
//	u8data[9] = cumulative_add >> 24;
//	u8data[10] = cumulative_add >> 16;
//	u8data[11] = cumulative_add >> 8;
//	u8data[12] = cumulative_add & 0x00000000000000ff;
//	u8len = 13;
	uint8_t data_buffer_len = 0;
	for(uint16_t u32len = 0;u32len<u16len;u32len++)
	{
		//u8data[u8len++] = u32data[u32len] >> 24;
		//u8data[u8len++] = u32data[u32len] >> 16;
		u8data[data_buffer_len++] = u32data[u32len] >> 8;
		u8data[data_buffer_len++] = u32data[u32len] & 0x000000ff;
	}
}


