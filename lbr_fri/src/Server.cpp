
#include "Server.h"

#define BACKLOG 10      // How many pending connections queue will hold 
#define VERBOSE 1       // Turn on or off debugging output

Server::Server(int iPort, int iPortDatagram, bool* pResult)
{
	m_iPort			= iPort;
	m_iPortDatagram	= iPortDatagram;
	m_iListen		= -1;
	m_pBuffer		= NULL;
	m_pBuffer2		= NULL;

	if (pResult)
		*pResult = false;

	// Allocate our temporary buffers that are used to convert data types
	m_pBuffer = (double *) malloc(sizeof(double) * SERVER_BUFF_SIZE);
	if (!m_pBuffer)
	{
		perror("Server::Server, failed to malloc buffer!");
		return;
	}

	m_pBuffer2 = (double *) malloc(sizeof(double) * SERVER_BUFF_SIZE);
	if (!m_pBuffer2)
	{
		free(m_pBuffer);
		m_pBuffer = NULL;
		perror("Server::Server, failed to malloc buffer2!");
		return;
	}

#ifdef _WIN32
	// For Windows, we need to fire up the Winsock DLL before we can do socket stuff.
	WORD wVersionRequested;
    WSADATA wsaData;
    int err;

	// Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h 
    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) 
	{
        // Tell the user that we could not find a usable Winsock DLL
        perror("Server::Server, WSAStartup failed with error");
        return;
    }
#endif

	if (VERBOSE) 
		printf("Server: opening socket on port = %d\n", m_iPort);

	if ((m_iListen = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
	{
		perror("Server::Server, socket");
		return;
	}

	m_addrMe.sin_family			= AF_INET;          // Host byte order 
	m_addrMe.sin_port			= htons(m_iPort);	// Short, network byte order 
	m_addrMe.sin_addr.s_addr	= INADDR_ANY;		// Auto-fill with my IP 
	memset(&(m_addrMe.sin_zero), 0, 8);				// Zero the rest of the struct 

	if (bind(m_iListen, (struct sockaddr *) &m_addrMe, sizeof(struct sockaddr)) == -1) 
	{
		// Note, this can fail if the server has just been shutdown and not enough time has elapsed.
		// See: http://www.developerweb.net/forum/showthread.php?t=2977 
		perror("Server::Server, bind");
		return;
	}

	if (listen(m_iListen, BACKLOG) == -1) 
	{
		perror("Server::Server, listen");
		return;
	}

	// See if we need to prepare for datagrams
	if (m_iPortDatagram != -1)
	{
		// TBD
	}

	if (pResult)
		*pResult = true;
}

Server::~Server()
{
#ifdef _WIN32
	// Windows specific socket shutdown code
	WSACleanup();
#endif

	if (m_pBuffer)
	{
		free(m_pBuffer);
		m_pBuffer = NULL;
	}
	if (m_pBuffer2)
	{
		free(m_pBuffer2);
		m_pBuffer2 = NULL;
	}
}


// Wait for somebody to connect to us on our port.
bool Server::Connect()
{
#ifdef _WIN32
	int iSinSize = sizeof(struct sockaddr_in);
#else
	socklen_t iSinSize = (socklen_t) sizeof(struct sockaddr_in);
#endif


	if ((m_iSock = accept(m_iListen, (struct sockaddr *) &m_addrRemote, &iSinSize)) == -1) 
	{
		perror("Server::Connect, accept");
		return false;
	}
	if (VERBOSE) 
		printf("Server: got connection from %s\n", inet_ntoa(m_addrRemote.sin_addr));

	// The client sends us an int to indicate if we should
	// be reversing byte order on this connection.  The client 
	// is sending 0 or 1, so a reversed 0 still looks
	// like a 0, no worries mate!

	char temp[1];
	int iTotal = 0;
	int iResult = 0;
	while ((iTotal < 1) && (iResult != -1))
	{
		iResult = recv(m_iSock, temp, 1, 0);
		iTotal += iResult;
	}
	if (iResult == -1)
	{
		perror("Server::Connect, recv");
		return false;
	}

	int iVal = temp[0];

	if (iVal == 0) 
		m_bReverse = false;
	else 
		m_bReverse = true;

	if (VERBOSE) 
	{
		if (iVal == 0)
			printf("Client requested normal byte order.\n");
		else
			printf("Client requested reversed byte order.\n");
	}
	return true;
}

// send a string to the socket
bool Server::SendString(char* pStr)
{
	if (send(m_iSock, (char *) pStr, strlen(pStr), 0) == -1)
	{
		perror("Server::SendString, send");              
		return false;
	}

	if (VERBOSE) 
		printf("Server: sending string '%s'\n", pStr);

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;
#endif

	return true;
}

// Send some bytes over the wire
bool Server::SendBytes(char* pVals, int iLen)
{
	int i = 0;

	if (send(m_iSock, (char *) pVals, iLen, 0) == -1)
	{
		perror("Server::SendBytes, send bytes");              
		return false;
	}

	if (VERBOSE)
	{ 
		printf("Server: sending %d bytes - ", iLen);                       
		for (i = 0; i < iLen; i++)
			printf("%d ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;
#endif

	return true;
}

// Send some integers over the wire
bool Server::SendInts(int* pVals, int iLen)
{
	int*	buff	= NULL;
	char*	ptr		= NULL;
	char*	valptr	= NULL;
	int		i		= 0;
	int		j		= 0;

	// We may need to reverse the byte order, oh joy
	if (m_bReverse)
	{
		// Set the buff pointer to our previously allocated spot
		buff	= (int *)  m_pBuffer;
		ptr		= (char *) buff;
		valptr	= (char *) pVals;

		// We need to reverse the order of each set of bytes
		for (i = 0; i < iLen; i++)
		{
			for (j=0; j < sizeof(int); j++)
				ptr[i * sizeof(int) + j] = (char) valptr[(i + 1) * sizeof(int) - j - 1];
		}
		if (send(m_iSock, (char *) buff, sizeof(int) * iLen, 0) == -1)
		{
			perror("Server::SendInts, send ints");              		
			return false;
		}
	}
	else
	{
		if (send(m_iSock, (char *) pVals, sizeof(int) * iLen, 0) == -1)
		{
			perror("send ints");
			return false;
		}
	}

	if (VERBOSE)
	{ 
		printf("Server: sending %d ints - ", iLen);                       
		for (i = 0; i < iLen; i++)
			printf("%d ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;
#endif

	return true;
}

// Send some floats over the wire
bool Server::SendFloats(float* pVals, int iLen)
{
	float*	buff	= NULL;
	char*	ptr		= NULL;
	char*	valptr	= NULL;
	int		i		= 0;
	int		j		= 0;

	// We may need to reverse the byte order, oh joy
	if (m_bReverse)
	{
		// Set the buff pointer to our previously allocated spot
		buff	= (float *) m_pBuffer;
		ptr		= (char *)  buff;
		valptr	= (char *)  pVals;

		// We need to reverse the order of each set of bytes
		for (i = 0; i < iLen; i++)
		{
			for (j=0; j < sizeof(float); j++)
				ptr[i * sizeof(float) + j] = (char) valptr[(i + 1) * sizeof(float) - j - 1];
		}
		if (send(m_iSock, (char *) buff, sizeof(float) * iLen, 0) == -1)
		{
			perror("Server::SendFloats, send floats");              		
			return false;
		}
	}
	else
	{
		if (send(m_iSock, (char *) pVals, sizeof(float) * iLen, 0) == -1)
		{
			perror("send floats");
			return false;
		}
	}

	if (VERBOSE)
	{ 
		printf("Server: sending %d floats - ", iLen);                       
		for (i = 0; i < iLen; i++)
			printf("%0.3f ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;
#endif

	return true;
}

// Send some floats over the wire
bool Server::SendDoubles(double* pVals, int iLen)
{
	double*	buff	= NULL;
	char*	ptr		= NULL;
	char*	valptr	= NULL;
	int		i		= 0;
	int		j		= 0;

	// We may need to reverse the byte order, oh joy
	if (m_bReverse)
	{
		// Set the buff pointer to our previously allocated spot
		buff	= (double *) m_pBuffer;
		ptr		= (char *)  buff;
		valptr	= (char *)  pVals;

		// We need to reverse the order of each set of bytes
		for (i = 0; i < iLen; i++)
		{
			for (j=0; j < sizeof(double); j++)
				ptr[i * sizeof(double) + j] = (char) valptr[(i + 1) * sizeof(double) - j - 1];
		}
		if (send(m_iSock, (char *) buff, sizeof(double) * iLen, 0) == -1)
		{
			perror("Server::SendDoubles, send double");              		
			return false;
		}
	}
	else
	{
		if (send(m_iSock, (char *) pVals, sizeof(double) * iLen, 0) == -1)
		{
			perror("Server::SendDoubles, send double");
			return false;
		}
	}

	if (VERBOSE)
	{ 
		printf("Server: sending %d doubles - ", iLen);                       
		for (i = 0; i < iLen; i++)
			printf("%0.3f ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;
#endif

	return true;
}

// Receive a string, returns num of bytes received.
int Server::RecvString(char* pStr, int iMax, char chTerm)
{
	int		iNumBytes	= 0;
	bool	bEnd		= false;
	int		i			= 0;
	int		j			= 0;
	int		iLastRead	= 0;

	// Set the temp buffer to our already allocated spot
	char* pTemp = (char *) m_pBuffer;

	// This is annoying, but the java end is sending a char
	// at a time, so we recv some chars (probably 1), append
	// it to our str string, then carry on until we see
	// the terminal character.
	while (!bEnd)
	{
		if ((iLastRead = recv(m_iSock, pTemp, SERVER_BUFF_SIZE, 0)) == -1)
		{
			perror("Server::RecvString, recv");
			return -1;
		}
		for (i = 0; i < iLastRead; i++)
		{
			pStr[j] = pTemp[i];	
			j++;
		}
		if ((pTemp[i - 1] == chTerm) || (j == (iMax - 1)))
			bEnd = true;

		iNumBytes += iLastRead;
	}

	pStr[j] = '\0';

	if (VERBOSE) 
		printf("Server: received '%s'\n", pStr);                       	

#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif

	return iNumBytes;
}


// Receive some bytes, returns number of bytes received.
int Server::RecvBytes(char* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp = (char *) m_pBuffer;

	// We receiving the incoming ints one byte at a time.
	while (!bEnd)
	{
		if ((iNumBytes = recv(m_iSock, pTemp, SERVER_BUFF_SIZE, 0)) == -1)
		{
			perror("Server::RecvBytes, recv");
			return -1;
		}
		for (i = 0; i < iNumBytes; i++)
		{
			pVals[j] = pTemp[i];
			j++;
		}

		iTotalBytes += iNumBytes;
		if (iTotalBytes == iLen)
			bEnd = true;
	}

	if (VERBOSE)
	{
		printf("Server: received %d bytes - ", iTotalBytes);
		for (i = 0; i < iTotalBytes; i++)
			printf("%d ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif

	return iTotalBytes;
}


// Receive some ints, returns num of ints received.
int Server::RecvInts(int* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	char*		pResult			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp	= (char *) m_pBuffer;
	pResult	= (char *) m_pBuffer2;

	// We receiving the incoming ints one byte at a time.
	while (!bEnd)
	{
		if ((iNumBytes = recv(m_iSock, pTemp, SERVER_BUFF_SIZE, 0)) == -1)
		{
			perror("Server::RecvInts, recv");
			return -1;
		}
		for (i = 0; i < iNumBytes; i++)
		{
			pResult[j] = pTemp[i];	
			j++;
		}

		iTotalBytes += iNumBytes;
		if (iTotalBytes == (iLen * sizeof(int)))
			bEnd = true;
	}

	// Now we need to put the array of bytes into the array of ints
	char*	ptr		= (char *) pVals;
	int		iNum	= j / sizeof(int);

	// The significance order depends on the platform
	if (m_bReverse)
	{
		// we need to reverse the order of each set of bytes
		for (i = 0; i < iNum; i++)
		{
			for (j = 0; j < sizeof(int); j++)
				ptr[i * sizeof(int) + j] = (char) pResult[(i + 1) * sizeof(int) - j - 1];
		}
	}
	else
	{
		// Leave the byte order as is
		for (i = 0; i < j; i++)
		{
			ptr[i] = pResult[i];
		}
	}

	if (VERBOSE) 
	{
		printf("Server: received %d ints - ", iNum);             	
		for (i = 0; i < iNum; i++)
			printf("%d ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif

	return iNum;
}

// Receive some floats, returns num of floats received.
int Server::RecvFloats(float* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	char*		pResult			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp	= (char *) m_pBuffer;
	pResult	= (char *) m_pBuffer2;

	// We receiving the incoming ints one byte at a time.
	while (!bEnd)
	{
		if ((iNumBytes = recv(m_iSock, pTemp, SERVER_BUFF_SIZE, 0)) == -1)
		{
			perror("Server::RecvFloats, recv");
			return -1;
		}
		for (i = 0; i < iNumBytes; i++)
		{
			pResult[j] = pTemp[i];	
			j++;
		}

		iTotalBytes += iNumBytes;
		if (iTotalBytes == (iLen * sizeof(float)))
			bEnd = true;
	}

	// Now we need to put the array of bytes into the array of ints
	char*	ptr		= (char *) pVals;
	int		iNum	= j / sizeof(float);

	// The significance order depends on the platform
	if (m_bReverse)
	{
		// we need to reverse the order of each set of bytes
		for (i = 0; i < iNum; i++)
		{
			for (j = 0; j < sizeof(float); j++)
				ptr[i * sizeof(float) + j] = (char) pResult[(i + 1) * sizeof(float) - j - 1];
		}
	}
	else
	{
		// Leave the byte order as is
		for (i = 0; i < j; i++)
		{
			ptr[i] = pResult[i];
		}
	}

	if (VERBOSE) 
	{
		printf("Server: received %d floats - ", iNum);             	
		for (i = 0; i < iNum; i++)
			printf("%0.3f ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif

	return iNum;
}

// Receive some doubles, returns num of doubles received.
int Server::RecvDoubles(double* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	char*		pResult			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp	= (char *) m_pBuffer;
	pResult	= (char *) m_pBuffer2;

	// We receiving the incoming ints one byte at a time.
	while (!bEnd)
	{
		if ((iNumBytes = recv(m_iSock, pTemp, SERVER_BUFF_SIZE, 0)) == -1)
		{
			perror("Server::RecvDoubles, recv");
			return -1;
		}
		for (i = 0; i < iNumBytes; i++)
		{
			pResult[j] = pTemp[i];	
			j++;
		}

		iTotalBytes += iNumBytes;
		if (iTotalBytes == (iLen * sizeof(double)))
			bEnd = true;
	}

	// Now we need to put the array of bytes into the array of ints
	char*	ptr		= (char *) pVals;
	int		iNum	= j / sizeof(double);

	// The significance order depends on the platform
	if (m_bReverse)
	{
		// we need to reverse the order of each set of bytes
		for (i = 0; i < iNum; i++)
		{
			for (j = 0; j < sizeof(double); j++)
				ptr[i * sizeof(double) + j] = (char) pResult[(i + 1) * sizeof(double) - j - 1];
		}
	}
	else
	{
		// Leave the byte order as is
		for (i = 0; i < j; i++)
		{
			ptr[i] = pResult[i];
		}
	}

	if (VERBOSE) 
	{
		printf("Server: received %d doubles - ", iNum);             	
		for (i = 0; i < iNum; i++)
			printf("%0.3f ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif

	return iNum;
}


// Shut down the socket
bool Server::Close()
{
	if (shutdown(m_iSock, SD_BOTH) == -1)
	{
		perror("Server::Close, shutdown");
		return false;
	}
#ifndef _WIN32
	close(m_iSock);
#endif

	// See if we have datagram resources to shutdown
	if (m_iPortDatagram != -1)
	{
		// TBD
	}

	return true;
}

// Receive a short ack from the client 
bool Server::RecvAck()
{
	char temp[1];
	int iTotal = 0;

	if (VERBOSE)
		printf("Waiting for ack...\n");

	int iResult = 0;
	while ((iTotal < 1) && (iResult != -1))
	{
		iResult = recv(m_iSock, temp, 1, 0);	
		iTotal += iResult;
	}
	if (iResult == -1)
	{
		perror("Server::RecvAck, recv");
		return false;
	}

	if (VERBOSE)
		printf("Ack recieved.\n");
	return true;
}	

// Send a short ack to the client 
bool Server::SendAck()
{
	char temp[1];
	temp[0] = 42;

	if (VERBOSE)
		printf("Sending ack...\n");

	if (send(m_iSock, temp, 1, 0) == -1)
		return false;
	return true;
}

// Send a packet of bytes using a datagram
bool Server::SendDatagram(char* pVals, int iLen)
{
	// TBD
	return false;
}

int Server::RecvDatagram(char* pVals, int iLen)
{
	// TBD
	return 0;
}
