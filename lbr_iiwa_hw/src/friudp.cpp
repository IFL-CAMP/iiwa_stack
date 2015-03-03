/*{{[PH]
****************************************************************************
 Project:  FRI

  This material is the exclusive property of KUKA Roboter GmbH 
  and must be returned to KUKA Roboter GmbH immediately upon 
  request.  This material and the information illustrated or 
  contained herein may not be used, reproduced, stored in a 
  retrieval system, or transmitted in whole or in part in any 
  way - electronic, mechanical, photocopying, recording, or 
  otherwise, without the prior written consent of KUKA Roboter GmbH.  
  
                 All Rights Reserved
                 Copyright (C)  2009
                  KUKA Roboter GmbH
                  Augsburg, Germany
  
[PH]}}
*/

/*
{{[FH]
****************************************************************************
  friUdp.cpp
       
      NOTE: This sample, as the corresponding FRI (Fast Research inteface) is subject to radical change
	  
      
[FH]}}
*/ 
/**
	\addtogroup friRemoteLib
	\brief Library for FRI (FastResearchInterface) 
*/
/* @{ */

/** *************************************************************************
 \author (Guenter Schreiber)
\file
\brief Implementation of a UDP socket
 *                                                                         *
 ***************************************************************************/
#include "friudp.h"

#ifdef WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib") 
#endif // WIN32


friUdp::friUdp(int port, char * remoteHost) : serverPort(port)
{
	   /* check struct sizes */
    if (!FRI_CHECK_SIZES_OK)
    {
        printf("data structure size error!\n");
        exit(1);
    }

	m_timestamp=0;
	// Make shure, that e.g. simulink uses no stupid standard definition - e.g. 0
	if ( serverPort < 10 )
	{
		serverPort=FRI_DEFAULT_SERVER_PORT;	
	}

#ifdef WIN32
	StartWinsock();
#endif
  Init(remoteHost);
}


friUdp::~friUdp()
{
  Close();

}

#ifdef WIN32
int friUdp::StartWinsock(void)
{
	WSADATA WSAData;
	return WSAStartup(MAKEWORD(2,0), &WSAData);
}
#endif// WIN32

void friUdp::Init(char * remoteHost)
{
	struct sockaddr_in servAddr;
	m_timestamp = 0;
	memset(&servAddr, 0, sizeof(servAddr));
	memset(&krcAddr, 0, sizeof(krcAddr));

	/* socket creation */
	udpSock = socket(PF_INET, SOCK_DGRAM, 0);
	if (udpSock < 0)
	{
		printf("opening socket failed!\n");
		exit(1);
	}
#ifdef HAVE_TIME_STAMP_RECEIVE
	{
		int temp = 1;
		if (setsockopt(udpSock, SOL_SOCKET, SO_TIMESTAMP, &temp, sizeof(int)) < 0)
		{
			printf("failed to enable receive time stamps\n");
			exit(1);
		}
	}
#endif
	/* bind local server port */
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons(serverPort);

	if (bind(udpSock, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0)
	{
		printf("binding port number %d failed!\n", serverPort);
		Close();
		exit(1);
	}
	// if the remote host is specified, 
	// preinitialize the socket properly
	if ( remoteHost ) 
	{
		printf("preinitialized remote host to %s\n",remoteHost);
		krcAddr.sin_addr.s_addr = inet_addr(remoteHost);
		krcAddr.sin_family = AF_INET;
		krcAddr.sin_port=htons(serverPort);		
	}


#ifdef HAVE_GETHOSTNAME
	/* get IP(s) and port number and display them (just for
	 convenience, so debugging friOpen is easier) */
	{
		char hostname[100];
		struct hostent * host;
		int i;

		gethostname(hostname, sizeof(hostname));
		host = gethostbyname(hostname);
		for (i=0; host->h_addr_list[i]!=0; i++)
		{
			struct in_addr addr;
			memcpy(&addr, host->h_addr_list[i], sizeof(addr));
			printf("IP %s - Port %d\n", inet_ntoa(addr), serverPort);
		}
	}
#endif // 


}



/* reveive one packet from KRC (blocking!) */
int friUdp::Recv(tFriMsrData *packet)
{
    if (udpSock >= 0)
    {
        int received;
        struct timeval ts;
        
        received = RecvPacket(udpSock, packet, &ts, &krcAddr);

        if (received == sizeof(tFriMsrData))
        {
#ifdef HAVE_TIME_STAMP_RECEIVE

            /* FIXME: need another #ifdef for VxWorks */
#ifdef QNX
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            m_timestamp = (double)ts.tv_sec + (double)ts.tv_nsec/1.0e9;
#else
            m_timestamp = (double)ts.tv_sec + (double)ts.tv_usec/1.0e6;
#endif // QNX
#endif // HAVE_TIME_STAMP
            return 0;
        }
        else
        {
            printf("received something, but wrong size %d (expected %d)...\n",received, sizeof(tFriMsrData));
            fflush(stdout);
        }
    }
    memset(packet, 0, sizeof(tFriMsrData));
    return -1;
}



/* send one answer packet to KRC */
int friUdp::Send(tFriCmdData *data)
{
    krcAddr.sin_family = AF_INET;
#ifdef KRC_IP_ADDRESS
    krcAddr.sin_addr.s_addr = inet_addr(KRC_IP_ADDRESS);
#endif
#ifdef KRC_RECEIVE_PORT
    krcAddr.sin_port = htons(KRC_RECEIVE_PORT);
#endif

    if ((udpSock >= 0) && (ntohs(krcAddr.sin_port) != 0))
    {
        int sent;
        sent = sendto(udpSock, (char *) data, sizeof(tFriCmdData), 0,
                      (struct sockaddr *)&krcAddr, sizeof(krcAddr));
        if (sent == sizeof(tFriCmdData))
        {
            return 0;
        }
    }
    return -1;
}




/* close the socket */
void friUdp::Close(void)
{
	if (udpSock >= 0)
	{
#ifdef WIN32
		closesocket(udpSock); 
		WSACleanup(); 
#else
		close(udpSock);
#endif
	}
	udpSock = -1;
}


#ifdef HAVE_TIME_STAMP_RECEIVE
// Socket option SO_TIMESTAMP is supported 
/* receive with timestamp  */
int friUdp::RecvPacket(int fd, tFriMsrData* p, struct timeval* ts, struct sockaddr_in* client) 
{
        struct msghdr msg;
        struct iovec vec[1];
        union {
                struct cmsghdr cm;
                char control[20];
        } cmsg_un;
        struct cmsghdr *cmsg;
        struct timeval *tv = NULL;
        int n;
        
        vec[0].iov_base = p;
        vec[0].iov_len = sizeof(*p);
        
        memset(&msg, 0, sizeof(msg));
        memset(&cmsg_un, 0, sizeof(cmsg_un));
        
        msg.msg_name = (caddr_t)client;
        if(client)
                msg.msg_namelen = sizeof(*client);
        else
                msg.msg_namelen = 0;
        msg.msg_iov = vec;
        msg.msg_iovlen = 1;
        msg.msg_control = cmsg_un.control;
        msg.msg_controllen = sizeof(cmsg_un.control);
        msg.msg_flags = 0;
        
        n = recvmsg(fd, &msg, 0); // MSG_DONTWAIT
        if(n < 0) {
                perror("recvmsg");
                return -1;
        }
        if(msg.msg_flags & MSG_TRUNC) {
                printf("received truncated message\n");
                return -1;
        }
        if(!ts)
                return n;

        /* get time stamp of packet */
        if(msg.msg_flags & MSG_CTRUNC) {
                printf("received truncated ancillary data\n");
                return -1;
        }
        if(msg.msg_controllen < sizeof(cmsg_un.control)) {
                printf("received short ancillary data (%d/%d)\n", msg.msg_controllen, (int)sizeof(cmsg_un.control));
                return -1;
        }
        for(cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
                if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMP)
                        tv = (struct timeval *)CMSG_DATA(cmsg);
        }
        if(tv) {
            ts->tv_sec = tv->tv_sec;
            ts->tv_usec = tv->tv_usec;
        }
        return n;
}

#else
/* receive with timestamp  */
int friUdp::RecvPacket(int udpSock, tFriMsrData* data, struct timeval* ts, struct sockaddr_in* client) 
{

	  if (udpSock >= 0)
    {
/** HAVE_SOCKLEN_T
	Yes - unbelieavble: There are differences in standard calling parameters (types) to recvfrom 
	Windows winsock, VxWorks and QNX use int
	newer Posix (most Linuxes) use socklen_t
	*/
#ifdef HAVE_SOCKLEN_T
		socklen_t sockAddrSize;
#else
        int sockAddrSize;
#endif
        int received;
        
		sockAddrSize = sizeof(struct sockaddr_in);

        received = recvfrom(udpSock, (char *) data, sizeof(tFriMsrData), 0,
                            (struct sockaddr *)&krcAddr, &sockAddrSize);

	return received;
	  }
	  return -1;
}

#endif // HAVE_TIME_STAMP_RECEIVE


#ifdef VXWORKS //USE_BERKELEY_PACKAGE_FILTER_VXWORKS
#define DEBUG_BPF_READ

#include "vxworks.h" 
#include "bpfDrv.h" 
#include "ioLib.h"
#include <logLib.h> 
#include <sys/ioctl.h>
//#include "drv/netif/smNetLib.h"
#include <wrn/coreip/net/ethernet.h> 
#include <wrn/coreip/net/if.h>
#include <wrn/coreip/netinet/ip.h>
#include <wrn/coreip/netinet/udp.h>

#ifdef DEBUG_BPF_READ
#include <iostream>
#include "friremote.h"

#endif
 

/*
 * Packet filter program...
 *
 * XXX: Changes to the filter program may require changes to the
 * constant offsets used in if_register_send to patch the BPF program!
 */
struct bpf_insn friUpdSock_bpf_filter[] = {
	/* Make sure this is an IP packet... */
	BPF_STMT(BPF_LD + BPF_H + BPF_ABS, 12),
	BPF_JUMP(BPF_JMP + BPF_JEQ + BPF_K, ETHERTYPE_IP, 0, 8),

	/* Make sure it's a UDP packet... */
	BPF_STMT(BPF_LD + BPF_B + BPF_ABS, 23),
	BPF_JUMP(BPF_JMP + BPF_JEQ + BPF_K, IPPROTO_UDP, 0, 6),

	/* Make sure this isn't a fragment... */
	BPF_STMT(BPF_LD + BPF_H + BPF_ABS, 20),
	BPF_JUMP(BPF_JMP + BPF_JSET + BPF_K, 0x1fff, 4, 0),

	/* Get the IP header length... */
	BPF_STMT(BPF_LDX + BPF_B + BPF_MSH, 14),

	/* Make sure it's to the right port... */
	BPF_STMT(BPF_LD + BPF_H + BPF_IND, 16),
	BPF_JUMP(BPF_JMP + BPF_JEQ + BPF_K, 67, 0, 1),		/* patch */

	/* If we passed all the tests, ask for the whole packet. */
	BPF_STMT(BPF_RET+BPF_K, (u_int)-1),

	/* Otherwise, drop it. */
	BPF_STMT(BPF_RET+BPF_K, 0),
};

int friUpdSock_bpf_filter_len = sizeof(friUpdSock_bpf_filter) / sizeof(struct bpf_insn);
struct bpf_program  mybpf;


void testBPF1(int socketPort, char * devName) 
{ 

  int bpffd = 0; 
  struct bpf_hdr * buf = NULL; 
  char * pbuffer = NULL; 
  int buflen; 
  int len,i; 
  ///int j=10; 
  char dev[8] = "gei0"; 
  struct ifreq ifr; 
  int trueValue=1; 
  int Rcvlen; 
 
	if ( socketPort <= 10) socketPort = 12345;
	if ( devName != NULL )
	{
		strncpy(dev,devName,8);
		dev[8]=0;
	}
  mybpf.bf_len = friUpdSock_bpf_filter_len; 
  mybpf.bf_insns = friUpdSock_bpf_filter; 
 

  /* Patch the server port into the BPF program...
	   *
	   * XXX: changes to filter program may require changes to the
	   * insn number(s) used below!
	   */
  friUpdSock_bpf_filter[8].k = socketPort;

  bpfDrv();
 
  if ( bpfDevCreate("/dev/bpf",2,4096) == ERROR)
  {
	printf("bpfDevCreate failed \n");
	return;
  }
  bpffd = open( "/dev/bpf0",0,0);
  if ( bpffd <= 0)
	{
		printf("open /dev/bpf0 failed\n");
		return;
	}
 
  memset(&ifr, sizeof(struct ifreq), 0); 
  strncpy(ifr.ifr_name, dev, sizeof(ifr.ifr_name)); 
 #define IOCTRL_CAST_THIRD_ARG (int)
 
 
 
	if (ioctl(bpffd,BIOCIMMEDIATE, IOCTRL_CAST_THIRD_ARG &trueValue) < 0) 
   { 
	printf("Error BIOCIMMEDIATE \n");
 
  }    
 
 
 
  if (ioctl(bpffd,(BIOCSETF),IOCTRL_CAST_THIRD_ARG (caddr_t)(&mybpf)) < 0) 
  { 
	perror("Error BIOCSETF \n");
	goto errorMark;
  }

  if (ioctl(bpffd,(BIOCSETIF),IOCTRL_CAST_THIRD_ARG (caddr_t)&ifr) < 0) 
   { 
	 printf("ERROR BIOCSETIF %s \n",dev);
	 goto errorMark;
	
 
   } 
   if (ioctl(bpffd,BIOCGBLEN, IOCTRL_CAST_THIRD_ARG &buflen) < 0) 
	{ 
	 printf("Error BIOCGBLEN \n");
 
   } 

 
	if (buflen > 4096) buflen=4096;
  buf = (struct bpf_hdr *)malloc(buflen); 
  //bzero(buf,buflen);  
  memset(buf,0x0,buflen);
  while ((len = read(bpffd,(char *)buf,buflen)) != 0) 
  {
	// im bpf header steht noch ein Timestamp -- waere gut fuer Timing thematik
	//
	 
//	Empfangene Rohdaten ohne bpf Header	
	pbuffer = (char *)buf + buf->bh_hdrlen; 
 
// Empfangene Rohdatenlaenge ohne bpf Header
	 Rcvlen = len - (buf->bh_hdrlen); 
//
// Wie trennt man nun die "Nutzdaten" von den Verwaltungsdaten??
//

  struct ip * iph = (struct ip *) ((char *) buf + buf->bh_hdrlen + sizeof(struct ether_header));
  struct udphdr * udph = (struct udphdr *) ((char *) iph + sizeof(struct ip));
  char * userData = ((char *) udph) + sizeof( struct udphdr); 

  tFriCmdData * cmd = ( tFriCmdData * ) userData;
  #ifdef DEBUG_BPF_READ
	printf("recvLen %d\n",Rcvlen);
  printf("IP SRC:\t\t%s\n", inet_ntoa(iph->ip_src));
  printf("IP DST:\t\t%s\n", inet_ntoa(iph->ip_dst));
  printf("UDP SRC:\t%u\n", ntohs(udph->uh_sport));
  printf("UDP DST:\t%u\n", ntohs(udph->uh_dport));
  printf("Len #:\t\t%u\n", ntohs(udph->uh_ulen));
  #endif

	 logMsg("BUF LEN = 0x%x\n",Rcvlen,0,0,0,0,0); 
 
	std::cout << (*cmd) << std::endl;
	 for (i=0;i< ntohs(udph->uh_ulen);i++) 
	 {
		 printf(" %2x", userData[i]);
		 if ((i % 10) == 9)
		 {
			 printf("\n");//,0,0,0,0,0,0);
		 }
	 }
	printf("\nFull packet \n");
	 for (i=0;i< Rcvlen;i++) 
		 {
			 printf(" %2x(%4d)", pbuffer[i],pbuffer[i]);
			 if ((i % 10) == 9)
			 {
				 printf("\n");//,0,0,0,0,0,0);
			 }
		 }
  } 
errorMark:
	printf("leaving %s\n",__PRETTY_FUNCTION__);
	if ( buf) 
		free(buf);
  if ( bpffd > 0 )   
		close (bpffd);
	bpfDevDelete("/dev/bpf");
 
} 



#endif


/*****************************************************************************
 $Log: $
 *****************************************************************************/



/* @} */
