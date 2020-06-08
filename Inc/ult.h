#ifndef _ULT_H
#define _ULT_H
#include "stdio.h"
#include "stdint.h"
#include "w5500.h"


uint16_t ATOI(char* str,uint16_t base); 			/* Convert a string to integer number */
uint32_t ATOI32(char* str,uint16_t base); 			/* Convert a string to integer number */
void itoa(uint16_t n,uint8_t* str, uint8_t len);
int ValidATOI(char* str, int base, int* ret); 		/* Verify character string and Convert it to (hexa-)decimal. */
char C2D(uint8_t c); 					/* Convert a character to HEX */

uint16_t swaps(uint16_t i);
uint32_t swapl(uint32_t l);

void replacetochar(char * str, char oldchar, char newchar);

void mid(char* src, char* s1, char* s2, char* sub);
void inet_addr_(unsigned char* addr,unsigned char *ip);

char* inet_ntoa(unsigned long addr);			/* Convert 32bit Address into Dotted Decimal Format */
char* inet_ntoa_pad(unsigned long addr);

//unsigned long inet_addr(unsigned char* addr);		/* Converts a string containing an (Ipv4) Internet Protocol decimal dotted address into a 32bit address */

char VerifyIPAddress_orig(char* src);			/* Verify dotted notation IP address string */
char VerifyIPAddress(char* src, uint8_t * ip);
unsigned long GetDestAddr(uint8_t s);			/* Output destination IP address of appropriate channel */
unsigned int GetDestPort(uint8_t s);			/* Output destination port number of appropriate channel */
unsigned short htons( unsigned short hostshort);	/* htons function converts a unsigned short from host to TCP/IP network byte order (which is big-endian).*/
unsigned long htonl(unsigned long hostlong);		/* htonl function converts a unsigned long from host to TCP/IP network byte order (which is big-endian). */
unsigned long ntohs(unsigned short netshort);		/* ntohs function converts a unsigned short from TCP/IP network byte order to host byte order (which is little-endian on Intel processors). */
unsigned long ntohl(unsigned long netlong);		/* ntohl function converts a u_long from TCP/IP network order to host byte order (which is little-endian on Intel processors). */
uint8_t CheckDestInLocal(uint32_t destip);			/* Check Destination in local or remote */

uint8_t getuint8_t(unsigned char status, uint8_t start); 	/* Get handle of uint8_t which status is same to 'status' */
unsigned short checksum(unsigned char * src, unsigned int len);		/* Calculate checksum of a stream */

#endif
