#ifndef __W5500_CONFIG_H_
#define __W5500_CONFIG_H_

#include "stdint.h"

#define IP_FROM_DEFINE	        0                          /*使用初始定义的IP信息*/
#define IP_FROM_DHCP	          1                          /*使用DHCP获取IP信息*/
#define IP_FROM_EEPROM	        2							             /*使用EEPROM定义的IP信息*/
extern unsigned char id_name[4];
typedef struct
{
  uint8_t op[4];//header: FIND;SETT;FACT...
  uint8_t mac[6];
  uint8_t sw_ver[2];
  uint8_t lip[4];
  uint8_t sub[4];
  uint8_t gw[4];
  uint8_t dns[4];	
  uint8_t dhcp;
  uint8_t debug;

  uint16_t fw_len;
  uint8_t state;
  
}CONFIG_MSG;
typedef struct	                    
{
	uint8_t mac[6];																							/*MAC地址*/
  uint8_t lip[4];																							/*local IP本地IP地址*/
  uint8_t sub[4];																							/*子网掩码*/
  uint8_t gw[4];																							/*网关*/
}EEPROM_MSG_STR;
extern uint32_t BD_TG_ping_time;                                    /*与BD_TG平台心跳包响应秒计数*/
extern CONFIG_MSG  ConfigMsg, RecvMsg;
extern uint8_t txsize[];
extern uint8_t rxsize[];
void dhcp_config_w5500(void);
void set_network(void);
void set_default(void);
void set_w5500_mac(void);
void set_w5500_ip(void);
void dhcp_timer_init(void);
extern uint16_t local_port;
#endif


