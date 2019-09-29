#ifndef __ZEN_IPSEARCH_H_
#define __ZEN_IPSEARCH_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#include <net/if_arp.h>
#include <netpacket/packet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>
#include "zenlog.h"

//���ֻ�����öԷ��������ǾͰ�macԴ�����MAC_TRICK��
//��ػ����ݰ��Ǿ���MAC_SOURCE
//#define MAC_TRICK {0x00, 0x00, 0x00, 0x00, 0x00, 0x01}
//#define MAC_SOURCE {0x00, 0x0c, 0x29, 0xc7, 0x16, 0x33}
//ð���IP
//#define IP_TRICK "192.168.1.109"
//Ŀ�������MAC
//#define MAC_TARGET {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
//Ŀ�������IP
//#define IP_TARGET "192.168.1.204"
//#define IP_TARGET "192.168.1.204"

#define BUFSIZEMAX    500
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

//device_type�豸���ͣ�00:�����豸;01:ץ�Ļ���02��ʶ�����03������������04�������������05��ץ��-ʶ��
//operation���ݰ����ͣ�01�����ߣ�02������IP
//pack_type�������ͣ�ip, netmask, gateway
struct device_arp
{
	char device_type[3];
	char operation[18];
	char pack_type[16];
	char data[16];
};
struct device_node
{
	char device_type[3];
	char mac[18];
	char ip[16];
	char netmask[16];
	char gateway[16];
};

void  Catcharp();
void Sendarp(void);
int GetNetInfo(void);

#endif
