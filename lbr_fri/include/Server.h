// A C++ socket server class
//
// Keith Vertanen 11/98, updated 12/08

#ifndef _SERVER_H__
#define _SERVER_H__

static const int SERVER_BUFF_SIZE = 64000;

// Adds in the send/recv acks after each message.
#define DEBUG_ACK

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#ifdef _WIN32
#include <winsock2.h>
#else
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>

// Duplicated from winsock2.h
#define SD_RECEIVE      0x00
#define SD_SEND         0x01
#define SD_BOTH         0x02

#endif

class Server
{
    public:
	Server(int iPort, int iPortDatagram, bool* pResult);
	~Server();

	bool				Connect();										// Accept a new connection
	bool				Close();										// Close the socket

	bool				SendString(char* pStr);							// Send a string to socket
	bool				SendInts(int* pVals, int iLen);					// Send some integers
	bool				SendBytes(char* pVals, int iLen);				// Send some bytes
	bool				SendFloats(float* pVals, int iLen);				// Send some floats
	bool				SendDoubles(double* pVals, int iLen);			// Send some doubles

	int					RecvString(char* pStr, int iMax, char chTerm);	// Receive a string
	int					RecvInts(int* pVals, int iLen);  				// Receive some ints
	int					RecvFloats(float* pVals, int iLen);  			// Receive some floats
	int					RecvDoubles(double* pVals, int iLen);  			// Receive some doubles
	int					RecvBytes(char* pVals, int iLen);  				// Receive some bytes

	// NOTE: these are not currently implemented!
	bool				SendDatagram(char* pVals, int iLen);			// Send a datagram
	int					RecvDatagram(char* pVals, int iLen);  			// Receive a datagram

    protected:		
	bool				m_bReverse;							// Am I reversing byte order or not?
	int					m_iPort;							// The port I'm listening on
	int					m_iPortDatagram;					// Port I listen for datagrams on (can be same or different from port)
	int					m_iListen;							// Descriptor we are listening on
	int					m_iSock;							// Descriptor for the socket
	struct sockaddr_in	m_addrRemote;						// Connector's address information
	struct sockaddr_in	m_addrMe;							// My address information
	double*				m_pBuffer;							// Reuse the same memory for buffer
	double*				m_pBuffer2;

    private:
	bool				RecvAck();
	bool				SendAck();

};

#endif



