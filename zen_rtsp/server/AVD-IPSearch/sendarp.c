#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#include <netpacket/packet.h>
#include <net/if.h>
#include <unistd.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <assert.h>

#include <ctype.h>

//���ֻ�����öԷ��������ǾͰ�macԴ�����MAC_TRICK��
//��ػ����ݰ��Ǿ���MAC_SOURCE
#define MAC_TRICK {0x00, 0x00, 0x00, 0x00, 0x00, 0x01}
#define MAC_SOURCE {0x00, 0x0c, 0x29, 0xc7, 0x16, 0x33}
//ð���IP
#define IP_TRICK "192.168.1.109"
//Ŀ�������MAC
#define MAC_TARGET {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
//Ŀ�������IP
#define IP_TARGET "192.168.1.204"
#define IP_TARGET "192.168.1.204"

#define DEVCONFIG              "/www/config/config.ini"  //anger

#define udp_send_brodcast 0 //���͹㲥����
#define udp_reciv_brodcast 1 //���չ㲥����
struct arp_packet
{
	//DLC Header
	//���շ�mac
	unsigned char mac_target[ETH_ALEN];
	//���ͷ�mac
	unsigned char mac_source[ETH_ALEN];
	//Ethertype - 0x0806��ARP֡������ֵ
	unsigned short ethertype;

	//ARP Frame
	//Ӳ������ - ��̫������ֵ0x1
	unsigned short hw_type;
	//�ϲ�Э������ - IPЭ��(0x0800)
	unsigned short proto_type;
	//MAC��ַ����
	unsigned char mac_addr_len;
	//IP��ַ����
	unsigned char ip_addr_len;
	//������ - 0x1��ʾARP�����,0x2��ʾӦ���
	unsigned short operation_code;
	//���ͷ�mac
	unsigned char mac_sender[ETH_ALEN];
	//���ͷ�ip
	unsigned char ip_sender[4];
	//���շ�mac
	unsigned char mac_receiver[ETH_ALEN];
	//���շ�ip
	unsigned char ip_receiver[4];
	//�������
	unsigned char padding[42];
};

void die(const char *pre);
int netinfo(char (*parray)[20]);	//��ȡ����������Ϣ������MAC��ַ��IP��ַ�����������Ĭ������
int netInfoMacaddr(char *pstr);		//ֻ��ȡ����MAC
int netInfoIpaddr(char *pstr);		//ֻ��ȡ����IP
int netInfoNetmask(char *pstr);		//ֻ��ȡ������������
int netInfoGateway(char *pstr);		//ֻ��ȡĬ������
unsigned char a2x(unsigned char c);	//��MAC��ַ�ַ����е��ַ�ת����ֵ
void mac_str2array(unsigned char *mac_array, char *mac_str);	//���ַ���MACת�������飨6�飩




int main(void)
{

	static int so_broadcast = 1;
	int iSocket_Send = 0;
	int iSocket_Send_brodcast = 0;
	int iRet = 0;
	uint16_t port = 54321;
	char buffer[256] = {0};
	struct sockaddr_in addr_server;
	struct sockaddr_in addr_local;
	struct sockaddr_in addr_brodcast;
	socklen_t addr_len = 0;
	int ret=0;
	 char pair[20];
	 //char equals[20];
	 char pp[5][10];
	 
	 	char ipaddr[16];
		unsigned short port_r;

	iSocket_Send = socket(AF_INET, SOCK_DGRAM, 0);
	if(iSocket_Send == -1)
	{
		printf("%s\n","socket fault");
		return -1;
	}

	
	
	iRet = setsockopt(iSocket_Send, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast));
	if(iRet == -1)
	{
		perror("setsockopt()");
		return -1;
	}
     
	#if udp_reciv_brodcast
	memset(&addr_local, 0, sizeof(struct sockaddr_in));
	addr_local.sin_family = AF_INET;
	addr_local.sin_port = htons(port);
	addr_local.sin_addr.s_addr = htonl(INADDR_ANY);
	 if(bind(iSocket_Send, (struct sockaddr *)&addr_local, sizeof(struct sockaddr)) == -1)
    {
        return -1;
    }
	#if 1
		memset(&addr_brodcast, 0, sizeof(struct sockaddr_in));
		addr_brodcast.sin_family = AF_INET;
		addr_brodcast.sin_port = htons(port);
		addr_brodcast.sin_addr.s_addr = inet_addr("255.255.255.255");// htonl(INADDR_ANY);//inet_add
		
		iSocket_Send_brodcast = socket(AF_INET, SOCK_DGRAM, 0);
		if(iSocket_Send_brodcast == -1)
		{
			printf("%s\n","socket iSocket_Send_brodcast fault");
			return -1;
		}
		iRet = setsockopt(iSocket_Send_brodcast, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast));
		if(iRet == -1)
		{
			perror("setsockopt()");
			return -1;
		}
    
	#endif
	#endif
	while(1)
	{
		int sfd, len, i;
		struct arp_packet ap;
		struct in_addr inaddr_sender, inaddr_receiver;
		struct sockaddr_ll sl;
		char local_mac[20] = {'\0'};
		char local_ip[20] = {'\0'};
		char local_netmask[20] = {'\0'};
		char local_gateway[20] = {'\0'};
		
	
	
		#if udp_reciv_brodcast
		
		
		memset(buffer, 0, 256);
		memset(&addr_server, 0, sizeof(struct sockaddr_in));
		addr_len = sizeof(struct sockaddr_in);
		printf("start udp rec\n");
		iRet = recvfrom(iSocket_Send, buffer, 256, 0, (struct sockaddr *)&addr_server, &addr_len);
		if(iRet == -1)
		{
		printf("recvfrom()\n");
		continue;
		}
		else
		{
			printf("\nsucessfMessage:[%s]\n", buffer);
			netInfoMacaddr(local_mac);
			netInfoIpaddr(local_ip);
			netInfoNetmask(local_netmask);
			netInfoGateway(local_gateway);
			
			char routernetwork[150] = "zenith,03,01,mac,"; 
			strcat(routernetwork, local_mac);
			strcat(routernetwork, ",ip,");
			strcat(routernetwork, local_ip);
			strcat(routernetwork, ",netmask,");
			strcat(routernetwork, local_netmask);
			strcat(routernetwork, ",gateway,");
			strcat(routernetwork, local_gateway);
			if(strncmp(buffer,"zenith-request",14) == 0)
			{
				
				//addr_server.sin_port =htons(port);
				//ret=sendto(iSocket_Send,routernetwork,strlen(routernetwork),0, (const struct sockaddr *)&addr_server,sizeof(struct sockaddr_in));//��㲥��ַ������Ϣ
				
				strcpy(ipaddr,(char *)inet_ntoa(addr_server.sin_addr));
			port_r =htons(addr_server.sin_port);
			printf("recvfrom:ipaddr=%s,port=%d\r\n",ipaddr,port_r);
			
				printf("I an is router,I send brodcast\n");
				ret=sendto(iSocket_Send_brodcast,routernetwork,strlen(routernetwork),0, (const struct sockaddr *)&addr_brodcast,sizeof(struct sockaddr_in));//��㲥��ַ������Ϣ
		
				//ARP�㲥
				#if 0
				sfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
				if(-1 == sfd)
				{
					perror("socket");
				}
				
				memset(&ap, 0, sizeof(ap));
				ap.mac_target[0] = 0xff;
				ap.mac_target[1] = 0xff;
				ap.mac_target[2] = 0xff;
				ap.mac_target[3] = 0xff;
				ap.mac_target[4] = 0xff;
				ap.mac_target[5] = 0xff;
				mac_str2array(ap.mac_source, local_mac);
	
				ap.ethertype = htons(0x0806);
				ap.hw_type = htons(0x1);
				ap.proto_type = htons(0x0800);
				ap.mac_addr_len = ETH_ALEN;
				ap.ip_addr_len = 4;
				ap.operation_code = htons(0x1);
				
				for(i = 0; i < sizeof(ap.mac_source); i++)
					ap.mac_sender[i] =  ap.mac_source[i];
				inet_aton(local_ip, &inaddr_sender);
				memcpy(&ap.ip_sender, &inaddr_sender, sizeof(inaddr_sender));
				
				for(i = 0; i < sizeof(ap.mac_target); i++)
					ap.mac_receiver[i] =  ap.mac_target[i];
				
				inet_aton(local_gateway, &inaddr_receiver);
				memcpy(&ap.ip_receiver, &inaddr_receiver, sizeof(inaddr_receiver));
				
				memset(&sl, 0, sizeof(sl));
				sl.sll_family = AF_PACKET;
				//sl.sll_addr = MAC_SOURCE;
				//sl.sll_halen = ETH_ALEN;
				sl.sll_ifindex = IFF_BROADCAST;//�ǳ���Ҫ
				
				//�豸���ͣ�00:�����豸;01:ץ�Ļ���02��ʶ�����03������������04�������������05��ץ��-ʶ��
				//���ݰ����ͣ�01�����ߣ�02������IP
				char ip_str[42] = "zenith,03,01,ip,";
				strcat(ip_str, local_ip);
				char netmask_str[42] = "zenith,03,01,netmask,";
				strcat(netmask_str, local_netmask);
				char gateway_str[42] = "zenith,03,01,gateway,";
				strcat(gateway_str, local_gateway);
				i = 0; 
				{
					memset(ap.padding, '\0', sizeof(ap.padding));
					strcpy(ap.padding, ip_str);
					len = sendto(sfd, &ap, sizeof(ap), 0, (struct sockaddr*)&sl, sizeof(sl));
					printf("i = %d: %s\n", i, ap.padding);
					if(-1 == len)
					{
						die("sendto");
					}
					
					
					memset(ap.padding, '\0', sizeof(ap.padding));
					strcpy(ap.padding, netmask_str);
					len = sendto(sfd, &ap, sizeof(ap), 0, (struct sockaddr*)&sl, sizeof(sl));
					printf("i = %d: %s\n", i, ap.padding);
					
					if(-1 == len)
					{
						die("sendto");
					}
					
				
					
					memset(ap.padding, '\0', sizeof(ap.padding));
					strcpy(ap.padding, gateway_str);
					len = sendto(sfd, &ap, sizeof(ap), 0, (struct sockaddr*)&sl, sizeof(sl));
					printf("i = %d: %s\n", i, ap.padding);
					if(-1 == len)
					{
						die("sendto");
					}
					
					
					i++;
					close(sfd);
				}
				#endif
				fflush(stdout);
			}
			//printf("\nReceive Message from : [%s:%d].\n", inet_ntoa(addr_server.sin_addr), ntohs(addr_server.sin_port));
			//printf("\nMessage:[%s]. length:[%d]\n", buffer, iRet);
		}
		#endif
		
		#if udp_send_brodcast
		
		netInfoMacaddr(local_mac);
		netInfoIpaddr(local_ip);
		netInfoNetmask(local_netmask);
		netInfoGateway(local_gateway);	
		
		char routernetwork[150] = "zenith,03,01,mac,"; 
		strcat(routernetwork, local_mac);
		strcat(routernetwork, ",ip,");
		strcat(routernetwork, local_ip);
		strcat(routernetwork, ",netmask,");
		strcat(routernetwork, local_netmask);
		strcat(routernetwork, ",gateway,");
		strcat(routernetwork, local_gateway);
		
		
		//ָ�����õ�iP��������
		memset(&addr_local, 0, sizeof(struct sockaddr_in));
		addr_local.sin_family = AF_INET;
		addr_local.sin_port = htons(port);
		addr_local.sin_addr.s_addr = inet_addr("255.255.255.255");// htonl(INADDR_ANY);//inet_addr("192.168.1.35");

		printf("I an is router\n");
		ret=sendto(iSocket_Send,routernetwork,strlen(routernetwork),0, (const struct sockaddr *)&addr_local,sizeof(struct sockaddr_in));//��㲥��ַ������Ϣ
		//printf("routernetwork = %d\n",ret);
		printf("I an is router\n");
			#endif
			
		sleep(1);
	}
	
	return 0;
}


void die(const char *pre)
{
	perror(pre);
	exit(1);
}
int netinfo(char (*parray)[20])
{
	char netDetail[1024*2] = {'\0'};
	char netBuffer[512*3] = {'\0'};
	
	system("ifconfig enp1s0 > /tmp/nettmp");
	
	FILE * fp;
	fp = fopen("/tmp/nettmp","r");
	if(fp == NULL)
		return -1;
	while(!feof(fp))
	{
		memset(netBuffer, '\0', sizeof(netBuffer));
		fgets(netBuffer,sizeof(netBuffer),fp);
		strcat(netDetail, netBuffer);
	}
	fclose(fp);
	
	char *p, *line1, *line2, *pmac, *pip, *pbroadcast, *pnetmask, *pgateway;
	line1 = strtok(netDetail,"\n");
	line2 = strtok(NULL,"\n");	
	
	line2 = strstr(line2,"addr");
	
	pmac = strstr(line1,"HWadd");
	pip = strtok(line2," ");
	pbroadcast = strtok(NULL," ");
	pnetmask = strtok(NULL," ");
	
	strtok(pmac," ");
	pmac = strtok(NULL," ");	//MAC��ַ
	strtok(pip,":");
	pip = strtok(NULL,":");		//IP��ַ
	strtok(pbroadcast,":");
	pbroadcast = strtok(NULL,":");		//�㲥��ַ
	strtok(pnetmask,":");
	pnetmask = strtok(NULL,":");		//��������
	
	strcat(parray[0], pmac);
	strcat(parray[1], pip);
	strcat(parray[2], pnetmask);
	
	system("ip route show > /tmp/nettmp");
	fp = fopen("/tmp/nettmp","r");
	memset(netDetail, '\0', sizeof(netDetail));
	while(!feof(fp))
	{
		memset(netBuffer, '\0', sizeof(netBuffer));
		fgets(netBuffer,sizeof(netBuffer),fp);
		strcat(netDetail, netBuffer);
	}
	fclose(fp);
	
	if(pgateway = strstr(netDetail,"default"))
	{
		pgateway = strstr(pgateway,"via");
		pgateway = strtok(pgateway," ");
		pgateway = strtok(NULL," ");	//Ĭ������
		strcat(parray[3], pgateway);
	}
	
	system("rm /tmp/nettmp");
	/**/
	return 1;
}

int netInfoMacaddr(char *pstr)
{
	printf("\netInfoMacaddr:[%d]\n",88);
	char netInfo[4][20];
	memset(netInfo, '\0', sizeof(netInfo));
	int re = netinfo(netInfo);
	printf("\netInfoMacaddr:[%d]\n", re);
	if(!re)
		return -1;
	memset(pstr, '\0', 20);
	strcat(pstr, netInfo[0]);
	
	return 1;	
}
int netInfoIpaddr(char *pstr)
{
	
	char netInfo[4][20];
	memset(netInfo, '\0', sizeof(netInfo));
	int re = netinfo(netInfo);
	if(!re)
		return -1;
	memset(pstr, '\0', 20);
	strcat(pstr, netInfo[1]);
	return 1;	
}
int netInfoNetmask(char *pstr)
{
	char netInfo[4][20];
	memset(netInfo, '\0', sizeof(netInfo));
	int re = netinfo(netInfo);
	if(!re)
		return -1;
	memset(pstr, '\0', 20);
	strcat(pstr, netInfo[2]);
	return 1;	
}
int netInfoGateway(char *pstr)
{
	char netInfo[4][20];
	memset(netInfo, '\0', sizeof(netInfo));
	int re = netinfo(netInfo);
	if(!re)
		return -1;
	memset(pstr, '\0', 20);
	strcat(pstr, netInfo[3]);
	return 1;	
}
unsigned char a2x(unsigned char c)
{
	switch(c) {
		case '0'...'9':
			return (unsigned char)atoi(&c);
		case 'a'...'f':
			return 0xa + (c-'a');
		case 'A'...'F':
			return 0xa + (c-'A');
		default:
			exit(0);
	}
}
void mac_str2array(unsigned char *mac_array, char *mac_str)
{
	int i;
	for(i = 0; i <6; i++)
	{
		mac_array[i] = (a2x(mac_str[i*3]) << 4) + a2x(mac_str[i*3+1]);
		//printf("%d\n", mac_array[i]);
	}
}

