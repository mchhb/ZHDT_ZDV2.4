
#ifndef	_SOCKET_H_
#define	_SOCKET_H_

#include "main.h"

#define SOCK_OK 1

extern uint8_t socket(uint8_t s, uint8_t protocol, uint16_t port, uint8_t flag); // Opens a socket(TCP or UDP or IP_RAW mode)
extern void close(uint8_t s); // Close socket
extern uint8_t connect(uint8_t s, uint8_t * addr, uint16_t port); // Establish TCP connection (Active connection)
extern void disconnect(uint8_t s); // disconnect the connection
extern uint8_t listen(uint8_t s);	// Establish TCP connection (Passive connection)
extern uint16_t send(uint8_t s, const uint8_t * buf, uint16_t len); // Send data (TCP)
extern uint16_t recv(uint8_t s, uint8_t * buf, uint16_t len);	// Receive data (TCP)
extern uint16_t sendto(uint8_t s, const uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port); // Send data (UDP/IP RAW)
extern uint16_t recvfrom(uint8_t s, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t  *port); // Receive data (UDP/IP RAW)
void send_ka(uint8_t s);

#ifdef __MACRAW__
void macraw_open(void);
uint16_t macraw_send( const uint8_t * buf, uint16_t len ); //Send data (MACRAW)
uint16_t macraw_recv( uint8_t * buf, uint16_t len ); //Recv data (MACRAW)
#endif

#endif
/* _SOCKET_H_ */

