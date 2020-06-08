#include "w5500_config.h"
#include "Types.h"
#include "w5500.h"
#include "config.h"
#include "usart2.h"
#include <stdio.h> 
#include "w5500_config.h"
#include <string.h>
#include "dhcp.h"
#include "Tim2.h"
#include "24c16.h"
#include "LOOP.h"

CONFIG_MSG  ConfigMsg, RecvMsg;
EEPROM_MSG_STR EEPROM_MSG;	
void dhcp_config_w5500(void);
uint8_t mac[6]={0x00,0x08,0xdc,0x22,0x22,0x22};
uint8_t local_ip[4]={172,16,40,141};											/*����W5500Ĭ��IP��ַ*/
uint8_t test_ip[4]={0};
uint8_t subnet[4]={255,255,255,0};												/*����W5500Ĭ����������*/
uint8_t gateway[4]={172,16,40,1};													/*����W5500Ĭ������*/
uint8_t dns_server[4]={103,44,168,8};								/*����W5500Ĭ��DNS*/
uint16_t local_port=6000;	                    						/*���屾�ض˿�*/
/*IP���÷���ѡ��������ѡ��*/
uint8_t  ip_from=IP_FROM_DEFINE;		

uint32_t BD_TG_ping_time =0;  
uint8_t  dhcp_ok=0;																				/*dhcp�ɹ���ȡIP*/
uint32_t ms=0;																						/*�������*/
vu8    ntptimer = 0;																		/*NPT�����*/
uint8_t txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};		// ѡ��8��Socketÿ��Socket���ͻ���Ĵ�С����w5500.c��void sysinit()�����ù���
uint8_t rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};		// ѡ��8��Socketÿ��Socket���ջ���Ĵ�С����w5500.c��void sysinit()�����ù���
/**
*@brief		dhcp�õ��Ķ�ʱ����ʼ��
*@param		��
*@return	��
*/
void dhcp_timer_init(void)
{
  //timer2_init();	
  /*******TIM2 ��ʱ����*********/
	TIM2_Configuration();
	/******* �ж����ȼ� **********/
	TIM2_NVIC_Configuration();
  /* TIM2 ���¿�ʱ�ӣ���ʼ��ʱ */	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
}

/**
*@brief		����W5500��IP��ַ
*@param		��
*@return	��
*/
void set_w5500_ip(void)
{	
	/*ʹ��DHCP��ȡIP�����������DHCP�Ӻ���*/	
	//if(at24c16_read(EE_ADDR_W5500_CONFIG) == EE_ADDR_W5500_PC_CONFIG_SUCCESS)
	if(ip_from==IP_FROM_DEFINE)	
	{
		
		printf(" ʹ�ö����IP��Ϣ����W5500:\r\n");
		memcpy(ConfigMsg.mac, mac, 6);
		memcpy(ConfigMsg.lip,local_ip,4);
		memcpy(ConfigMsg.sub,subnet,4);
		memcpy(ConfigMsg.gw,gateway,4);
		memcpy(ConfigMsg.dns,dns_server,4);
	}
//		/*ʹ��EEPROM�洢��IP����*/	
//	if(at24c16_read(EE_ADDR_W5500_CONFIG_DHCP_IP) == EE_ADDR_W5500_PC_CONFIG_SUCCESS)
//	{	
//		/*�����ȡEEPROM��MAC��Ϣ,��������ã����ʹ��*/		
//			printf(" IP from EEPROM\r\n");
//			/*����EEPROM������Ϣ�����õĽṹ�����*/
//			EEPROM_MSG.lip[0] = at24c16_read(EE_ADDR_DHCP_IP_ONE);
//			EEPROM_MSG.lip[1] = at24c16_read(EE_ADDR_DHCP_IP_TWO);
//			EEPROM_MSG.lip[2] = at24c16_read(EE_ADDR_DHCP_IP_THREE);
//			EEPROM_MSG.lip[3] = at24c16_read(EE_ADDR_DHCO_IP_FOUR);
//			memcpy(ConfigMsg.lip,EEPROM_MSG.lip, 4);				
//	}
//	else
//	{
//		printf(" EEPROMδ����,ʹ��DHCP��IP��Ϣ����W5500,��д��EEPROM\r\n");
//		//write_config_to_eeprom();	/*ʹ��Ĭ�ϵ�IP��Ϣ������ʼ��EEPROM������*/
//		at24c16_write(EE_ADDR_DHCP_IP_ONE, DHCP_GET.lip[0]);
//		at24c16_write(EE_ADDR_DHCP_IP_TWO, DHCP_GET.lip[1]);
//		at24c16_write(EE_ADDR_DHCP_IP_THREE, DHCP_GET.lip[2]);
//		at24c16_write(EE_ADDR_DHCO_IP_FOUR, DHCP_GET.lip[3]);		
//		memcpy(ConfigMsg.lip,DHCP_GET.lip, 4);
//	}	

	/*ʹ��DHCP��ȡIP�����������DHCP�Ӻ���*/		
	if(ip_from==IP_FROM_DHCP)								
	{
		/*����DHCP��ȡ��������Ϣ�����ýṹ��*/
		if(dhcp_ok==1)
		{
			printf(" IP from DHCP\r\n");		 
			memcpy(ConfigMsg.lip,DHCP_GET.lip, 4);
			memcpy(ConfigMsg.sub,DHCP_GET.sub, 4);
			memcpy(ConfigMsg.gw,DHCP_GET.gw, 4);
			memcpy(ConfigMsg.dns,DHCP_GET.dns,4);
		}
		else
		{
			printf(" DHCP�ӳ���δ����,���߲��ɹ�\r\n");
			printf(" ʹ�ö����IP��Ϣ����W5500\r\n");
		}
	}
		
	/*����������Ϣ��������Ҫѡ��*/	
	ConfigMsg.sw_ver[0]=FW_VER_HIGH;
	ConfigMsg.sw_ver[1]=FW_VER_LOW;	

	/*��IP������Ϣд��W5500��Ӧ�Ĵ���*/	
	setSUBR(ConfigMsg.sub);
	setGAR(ConfigMsg.gw);
	setSIPR(ConfigMsg.lip);
												// ��ʼ��8��socket
	
	//setRTR(2000);																		// ���ó�ʱʱ��
 // setRCR(3);																			// ����������·��ʹ���
	
	getSIPR(test_ip);			
	printf(" W5500 IP��ַ   : %d.%d.%d.%d\r\n", test_ip[0],test_ip[1],test_ip[2],test_ip[3]);
	getSUBR(test_ip);
	printf(" W5500 �������� : %d.%d.%d.%d\r\n", test_ip[0],test_ip[1],test_ip[2],test_ip[3]);
	getGAR(test_ip);
	printf(" W5500 ����     : %d.%d.%d.%d\r\n", test_ip[0],test_ip[1],test_ip[2],test_ip[3]);
}

void set_w5500_mac(void)
{
	if(at24c16_read(EE_ADDR_W5500_CONFIG_MAC) == EE_ADDR_W5500_PC_CONFIG_SUCCESS)
	{
		printf("Start read pc w5500 config ! \r\n");
		mac[0] = at24c16_read(EE_ADDR_MAC_ONE);
		mac[1] = at24c16_read(EE_ADDR_MAC_TWO);
		mac[2] = at24c16_read(EE_ADDR_MAC_THREE);
		mac[3] = at24c16_read(EE_ADDR_MAC_FOUR);
		mac[4] = at24c16_read(EE_ADDR_MAC_FIVE);
		mac[5] = at24c16_read(EE_ADDR_MAC_SIX);
		printf("Read pc w5500 config success ! \r\n");
	}
	else
	{
		printf(" First start ram write at24c16 ! \r\n");
		mac[2] = (CpuID[2]>>24)&0xff;
		mac[3] = (CpuID[2]>>16)&0xff;
		mac[4] = (CpuID[2]>>8)&0xff;
		mac[5] =  CpuID[2]&0xff;
		printf("CpuID[2] = %d,mac[2] = %d,mac[3] = %d,mac[4] = %d,mac[5] = %d.\r\n",CpuID[2],mac[2],mac[3],mac[4],mac[5]);
		at24c16_write(EE_ADDR_MAC_ONE, mac[0]);
		at24c16_write(EE_ADDR_MAC_TWO, mac[1]);
		at24c16_write(EE_ADDR_MAC_THREE, mac[2]);
		at24c16_write(EE_ADDR_MAC_FOUR, mac[3]);	//mcu id number  
		at24c16_write(EE_ADDR_MAC_FIVE, mac[4]);	//mcu id number 
		at24c16_write(EE_ADDR_MAC_SIX, mac[5]);		//mcu id number 
		printf(" First start ram write at24c16 ! \r\n");
	}
	memcpy(ConfigMsg.mac, mac, 6);
	setSHAR(ConfigMsg.mac);	/**/
	memcpy(DHCP_GET.mac, mac, 6);
}

void dhcp_config_w5500(void)
{
		printf(" First start ram write at24c16 ! \r\n");
		at24c16_write(EE_ADDR_SUBNET_ONE, subnet[0]);
		at24c16_write(EE_ADDR_SUBNET_TWO, subnet[1]);
		at24c16_write(EE_ADDR_SUBNET_THREE, subnet[2]);
		at24c16_write(EE_ADDR_SUBNET_FOUR, subnet[3]);	
		at24c16_write(EE_ADDR_GATEWAY_ONE, gateway[0]);
		at24c16_write(EE_ADDR_GATEWAY_TWO, gateway[1]);
		at24c16_write(EE_ADDR_GATEWAY_THREE, gateway[2]);
		at24c16_write(EE_ADDR_GATEWAY_FOUR, gateway[3]);
		at24c16_write(EE_ADDR_DNS_ONE, dns_server[0]);
		at24c16_write(EE_ADDR_DNS_TWO, dns_server[1]);
		at24c16_write(EE_ADDR_DNS_THREE, dns_server[2]);
		at24c16_write(EE_ADDR_DNS_FOUR, dns_server[3]);
		printf(" Ram writed w55000 config at24c16 success ! \r\n");
}




