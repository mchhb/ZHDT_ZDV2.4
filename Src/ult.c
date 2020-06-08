#include "ult.h"
#include "w5500_config.h"
#include "w5500.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
//#include <ctype.h>

uint16_t ATOI(char* str,uint16_t base	)
{
  unsigned int num = 0;
  while (*str !=0)
          num = num * base + C2D(*str++);
  return num;
}

uint32_t ATOI32(char* str,uint16_t base	)
{
  uint32_t num = 0;
  while (*str !=0)
          num = num * base + C2D(*str++);
  return num;
}


void itoa(uint16_t n,uint8_t str[5], uint8_t len)
{
  
  uint8_t i=len-1;

  memset(str,0x20,len);
  do{
  str[i--]=n%10+'0';
  
 }while((n/=10)>0);

 return;
}

int ValidATOI(char* str, int base,int* ret)
{
  int c;
  char* tstr = str;
  if(str == 0 || *str == '\0') return 0;
  while(*tstr != '\0')
  {
    c = C2D(*tstr);
    if( c >= 0 && c < base) tstr++;
    else    return 0;
  }
  
  *ret = ATOI(str,base);
  return 1;
}
 
void replacetochar(char * str,	char oldchar,char newchar	)
{
  int x;
  for (x = 0; str[x]; x++) 
    if (str[x] == oldchar) str[x] = newchar;	
}

char C2D(uint8_t c	)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return 10 + c -'a';
	if (c >= 'A' && c <= 'F')
		return 10 + c -'A';

	return (char)c;
}

uint16_t swaps(uint16_t i)
{
  uint16_t ret=0;
  ret = (i & 0xFF) << 8;
  ret |= ((i >> 8)& 0xFF);
  return ret;	
}

uint32_t swapl(uint32_t l)
{
  uint32_t ret=0;
  ret = (l & 0xFF) << 24;
  ret |= ((l >> 8) & 0xFF) << 16;
  ret |= ((l >> 16) & 0xFF) << 8;
  ret |= ((l >> 24) & 0xFF);
  return ret;
}

//get mid str
void mid(char* src, char* s1, char* s2, char* sub)
{
	char* sub1;
	char* sub2;
	uint16_t n;

  sub1=strstr(src,s1);
  sub1+=strlen(s1);
  sub2=strstr(sub1,s2);
  n=sub2-sub1;
  strncpy(sub,sub1,n);
  sub[n]=0;
}
void inet_addr_(unsigned char* addr,unsigned char *ip)
{
	int i;
//	uint32_t inetaddr = 0;
	char taddr[30];
	char * nexttok;
	char num;
	strcpy(taddr,(char *)addr);
	
	nexttok = taddr;
	for(i = 0; i < 4 ; i++)
	{
		nexttok = strtok(nexttok,".");
		if(nexttok[0] == '0' && nexttok[1] == 'x') num = ATOI(nexttok+2,0x10);
		else num = ATOI(nexttok,10);
		
		ip[i] = num;
		nexttok = NULL;
	}
}	
/**
@brief	Convert 32bit Address(Host Ordering) into Dotted Decimal Format
@return 	a char pointer to a static buffer containing the text address in standard ".'' notation. Otherwise, it returns NULL. 
*/  
char* inet_ntoa(
	unsigned long addr	/**< Pointer variable to store converted value(INPUT) */
	)
{
	static char addr_str[32];
	memset(addr_str,0,32);
	sprintf(addr_str,"%d.%d.%d.%d",(int)(addr>>24 & 0xFF),(int)(addr>>16 & 0xFF),(int)(addr>>8 & 0xFF),(int)(addr & 0xFF));
	return addr_str;
}

char* inet_ntoa_pad(unsigned long addr)
{
	static char addr_str[16];
	memset(addr_str,0,16);
	printf(addr_str,"%03d.%03d.%03d.%03d",(int)(addr>>24 & 0xFF),(int)(addr>>16 & 0xFF),(int)(addr>>8 & 0xFF),(int)(addr & 0xFF));
	return addr_str;
}


/**
@brief	Verify dotted notation IP address string
@return 	success - 1, fail - 0
*/ 
char VerifyIPAddress_orig(
	char* src	/**< pointer to IP address string */
	)
{
	int i;
	int tnum;
	char tsrc[50];
	char* tok = tsrc;
	
	strcpy(tsrc,src);
	
	for(i = 0; i < 4; i++)
	{
		tok = strtok(tok,".");
		if ( !tok ) return 0;
		if(tok[0] == '0' && tok[1] == 'x')
		{
			if(!ValidATOI(tok+2,0x10,&tnum)) return 0;
		}
		else if(!ValidATOI(tok,10,&tnum)) return 0;
		
		if(tnum < 0 || tnum > 255) return 0;
		tok = NULL;
	}
	return 1;	
}

char VerifyIPAddress(char* src, uint8_t * ip)
{
	int i;
	int tnum;
	char tsrc[50];
	char* tok = tsrc;
	
	strcpy(tsrc,src);
	
	for(i = 0; i < 4; i++)
	{
		tok = strtok(tok,".");
		if ( !tok ) return 0;
		if(tok[0] == '0' && tok[1] == 'x')
		{
			if(!ValidATOI(tok+2,0x10,&tnum)) return 0;
		}
		else if(!ValidATOI(tok,10,&tnum)) return 0;

		ip[i] = tnum;
		
		if(tnum < 0 || tnum > 255) return 0;
		tok = NULL;
	}
	return 1;	
}

/**
@brief	Output destination IP address of appropriate channel
@return 	32bit destination address (Host Ordering)
*/ 
unsigned long GetDestAddr(
	uint8_t s	/**< Channel number which try to get destination IP Address */
	)
{
	uint32_t addr=0;
	int i = 0;
	for(i=0; i < 4; i++)
	{
		addr <<=8;
		addr += IINCHIP_READ(Sn_DIPR0(s)+i);
	}
	return addr;
}

/**
@brief	Output destination port number of appropriate channel
@return 	16bit destination port number
*/ 
unsigned int GetDestPort(
	uint8_t s	/**< Channel number which try to get destination port */
	)
{
	uint16_t port;
	port = ((uint16_t) IINCHIP_READ(Sn_DPORT0(s))) & 0x00FF;
	port <<= 8;
	port += ((uint16_t) IINCHIP_READ(Sn_DPORT0(s)+1)) & 0x00FF;
	return port;
}


/**
@brief	htons function converts a unsigned short from host to TCP/IP network byte order (which is big-endian).
@return 	the value in TCP/IP network byte order
*/ 
uint16_t htons( 
	uint16_t hostshort	/**< A 16-bit number in host byte order.  */
	)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )
	return swaps(hostshort);
#else
	return hostshort;
#endif		
}


/**
@brief	htonl function converts a unsigned long from host to TCP/IP network byte order (which is big-endian).
@return 	the value in TCP/IP network byte order
*/ 
unsigned long htonl(
	unsigned long hostlong		/**< hostshort  - A 32-bit number in host byte order.  */
	)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )
	return swapl(hostlong);
#else
	return hostlong;
#endif	
}


/**
@brief	ntohs function converts a unsigned short from TCP/IP network byte order to host byte order (which is little-endian on Intel processors).
@return 	a 16-bit number in host byte order
*/ 
unsigned long ntohs(
	unsigned short netshort	/**< netshort - network odering 16bit value */
	)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )	
	return htons(netshort);
#else
	return netshort;
#endif		
}


/**
@brief	converts a unsigned long from TCP/IP network byte order to host byte order (which is little-endian on Intel processors).
@return 	a 16-bit number in host byte order
*/ 
unsigned long ntohl(unsigned long netlong)
{
#if ( SYSTEM_ENDIAN == _ENDIAN_LITTLE_ )
	return htonl(netlong);
#else
	return netlong;
#endif		
}

// destip : BigEndian
uint8_t CheckDestInLocal(uint32_t destip)
{
	int i = 0;
	uint8_t * pdestip = (uint8_t *)&destip;
	for(i =0; i < 4; i++)
	{
		if((pdestip[i] & IINCHIP_READ(SUBR0+i)) != (IINCHIP_READ(SIPR0+i) & IINCHIP_READ(SUBR0+i)))
			return 1;	// Remote
	}
	return 0;
}


/**
@brief	Get handle of uint8_t which status is same to 'status'
@return 	uint8_t number
*/ 
uint8_t getuint8_t(
	unsigned char status, 	/**< uint8_t's status to be found */
	uint8_t start			/**< base of uint8_t to be found */
	)
{
	uint8_t i;
	if(start > 3) start = 0;

	for(i = start; i < MAX_SOCK_NUM ; i++) if( getSn_SR(i)==status ) return i;
	return MAX_SOCK_NUM;	
}


/**
@brief	Calculate checksum of a stream
@return 	checksum
*/ 
unsigned short checksum(
	unsigned char * src, 	/**< pointer to stream  */
	unsigned int len		/**< size of stream */
	)
{
	uint16_t sum, tsum, i, j;
	uint32_t lsum;

	j = len >> 1;

	lsum = 0;

	for (i = 0; i < j; i++) 
	{
		tsum = src[i * 2];
		tsum = tsum << 8;
		tsum += src[i * 2 + 1];
		lsum += tsum;
	}

	if (len % 2) 
	{
		tsum = src[i * 2];
		lsum += (tsum << 8);
	}


	sum = lsum;
	sum = ~(sum + (lsum >> 16));
	return (uint16_t) sum;	
}





