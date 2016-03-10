/**********************************************************************
 * file:  sr_router.c
 * date:  Mon Feb 18 12:50:42 PST 2002
 * Contact: casado@stanford.edu
 *
 * Description:
 *
 * This file contains all the functions that interact directly
 * with the routing table, as well as the main entry method
 * for routing.
 *
 **********************************************************************/

#include <stdio.h>
#include <assert.h>


#include "sr_if.h"
#include "sr_rt.h"
#include "sr_router.h"
#include "sr_protocol.h"
#include "sr_arpcache.h"
#include "sr_utils.h"

/*---------------------------------------------------------------------
 * Method: sr_init(void)
 * Scope:  Global
 *
 * Initialize the routing subsystem
 *
 *---------------------------------------------------------------------*/

uint8_t * createICMPPacket(uint8_t sourceMAC[6],
		uint8_t destMAC[6],
		uint32_t sourceIP,
		uint32_t destIP,
		uint8_t icmpCode,
		uint8_t icmpType);


typedef struct QueueNode{
	struct QueueNode* front;
	struct QueueNode* next;
	sr_ip_hdr_t *packet;
}queue_node;


/* push ip packet *p onto queue q */
void enqueue(queue_node * q, sr_ip_hdr_t *p){
	queue_node * curr = q->front;
	while(curr->next){
		curr = curr->next;
	}
	curr->next = NULL;
	curr->packet = p;

	return;
}
/* pop the first element of q */
queue_node* dequeue(queue_node * q){
	queue_node * curr = q->front;
	q->front = curr->next;
	return curr;

}

uint32_t* longest_prefix(struct sr_instance* sr, uint32_t dst){
	struct sr_rt *rt = sr->routing_table;
	struct sr_rt *best;
	struct sr_rt *curr = rt;
	while(curr){
		/* uint32_t entry = curr->dest; */
	}
}


void sr_init(struct sr_instance* sr)
{
    /* REQUIRES */
    assert(sr);

    /* Initialize cache and cache cleanup thread */
    sr_arpcache_init(&(sr->cache));

    pthread_attr_init(&(sr->attr));
    pthread_attr_setdetachstate(&(sr->attr), PTHREAD_CREATE_JOINABLE);
    pthread_attr_setscope(&(sr->attr), PTHREAD_SCOPE_SYSTEM);
    pthread_attr_setscope(&(sr->attr), PTHREAD_SCOPE_SYSTEM);
    pthread_t thread;

    pthread_create(&thread, &(sr->attr), sr_arpcache_timeout, sr);
    
    /* Add initialization code here! */


} /* -- sr_init -- */

/*---------------------------------------------------------------------
 * Method: sr_handlepacket(uint8_t* p,char* interface)
 * Scope:  Global
 *
 * This method is called each time the router receives a packet on the
 * interface.  The packet buffer, the packet length and the receiving
 * interface are passed in as parameters. The packet is complete with
 * ethernet headers.
 *
 * Note: Both the packet buffer and the character's memory are handled
 * by sr_vns_comm.c that means do NOT delete either.  Make a copy of the
 * packet instead if you intend to keep it around beyond the scope of
 * the method call.
 *
 *---------------------------------------------------------------------*/

uint8_t * createICMPPacket(uint8_t sourceMAC[6],
		uint8_t destMAC[6],
		uint32_t sourceIP,
		uint32_t destIP,
		uint8_t icmpCode,
		uint8_t icmpType)
{
	uint8_t * newPacket;

	newPacket = malloc(
			sizeof(sr_ethernet_hdr_t) +
			sizeof(sr_ip_hdr_t) +
			sizeof(sr_icmp_hdr_t)
			);

	sr_ethernet_hdr_t * ethernetHeader = newPacket;
	sr_ip_hdr_t * ipHeader = newPacket + sizeof(sr_ethernet_hdr_t);
	sr_icmp_hdr_t * icmpHeader = newPacket + sizeof(sr_ethernet_hdr_t) + sizeof(sr_ip_hdr_t);

	/* Ethernet Info */
	/* Ethernet type says the packet is IP */
	ethernetHeader -> ether_type =  (uint16_t) 2048;

	ethernetHeader -> ether_shost[0] = (uint8_t) sourceMAC[0]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[1] = (uint8_t) sourceMAC[1]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[2] = (uint8_t) sourceMAC[2]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[3] = (uint8_t) sourceMAC[3]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[4] = (uint8_t) sourceMAC[4]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[5] = (uint8_t) sourceMAC[5]; /* SOURCE MAC */

	ethernetHeader -> ether_dhost[0] = (uint8_t) destMAC[0]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[1] = (uint8_t) destMAC[1]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[2] = (uint8_t) destMAC[2]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[3] = (uint8_t) destMAC[3]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[4] = (uint8_t) destMAC[4]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[5] = (uint8_t) destMAC[5]; /* DESTINATION MAC */


	/* IP info */
	ipHeader -> ip_src = (uint32_t) sourceIP; /* SOURCE IP */
	ipHeader -> ip_dst = (uint32_t) destIP; /* DESTINATION IP */
	ipHeader -> ip_tos = (uint8_t) 0;  /* Type of Service >> NOT SURE ABOUT THIS */
	ipHeader -> ip_len = (uint16_t) ((int)icmpHeader + (int)sizeof(sr_icmp_hdr_t) - (int)ipHeader ); /*  Length */
	ipHeader -> ip_id = (uint16_t) 0;  /* First packet being sent back...? */
	ipHeader -> ip_off = (uint16_t) 0; /* Offset:Don't think the Datagram is split due to ICMP being one message */
	ipHeader -> ip_ttl = (uint8_t) 250;	/* TTL >> NOT SURE ABOUT THIS */
    ipHeader -> ip_p = (uint8_t) 1; 	/* Protocol: ICMP */
    ipHeader -> ip_sum = (uint16_t) cksum(ipHeader, sizeof(sr_ip_hdr_t));

    /* ICMP info */
    icmpHeader -> icmp_code = (uint8_t) icmpCode;
    icmpHeader -> icmp_type = (uint8_t) icmpType;
    icmpHeader -> icmp_sum = (uint16_t) cksum(icmpHeader, sizeof(sr_icmp_hdr_t));

    return newPacket;
}

/* Preps a type 3 ICMP packet */
uint8_t * createICMP_type3Packet(
		uint8_t sourceMAC[6],
		uint8_t destMAC[6],
		uint32_t sourceIP,
		uint32_t destIP,
		uint8_t icmpCode,
		uint16_t nextMTU
		)
{

	uint8_t * newPacket;

	newPacket = malloc(
			sizeof(sr_ethernet_hdr_t) +
			sizeof(sr_ip_hdr_t) +
			sizeof(sr_icmp_t3_hdr_t)
			);

	sr_ethernet_hdr_t * ethernetHeader = newPacket;
	sr_ip_hdr_t * ipHeader = newPacket + sizeof(sr_ethernet_hdr_t);
	sr_icmp_t3_hdr_t * icmpHeader = newPacket + sizeof(sr_ethernet_hdr_t) + sizeof(sr_ip_hdr_t);

	/* Ethernet Info */
	/* Ethernet type says the packet is IP */
	ethernetHeader -> ether_type =  (uint16_t) 2048;

	ethernetHeader -> ether_shost[0] = (uint8_t) sourceMAC[0]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[1] = (uint8_t) sourceMAC[1]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[2] = (uint8_t) sourceMAC[2]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[3] = (uint8_t) sourceMAC[3]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[4] = (uint8_t) sourceMAC[4]; /* SOURCE MAC */
	ethernetHeader -> ether_shost[5] = (uint8_t) sourceMAC[5]; /* SOURCE MAC */

	ethernetHeader -> ether_dhost[0] = (uint8_t) destMAC[0]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[1] = (uint8_t) destMAC[1]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[2] = (uint8_t) destMAC[2]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[3] = (uint8_t) destMAC[3]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[4] = (uint8_t) destMAC[4]; /* DESTINATION MAC */
	ethernetHeader -> ether_dhost[5] = (uint8_t) destMAC[5]; /* DESTINATION MAC */


	/* IP info */
	ipHeader -> ip_src = (uint32_t) sourceIP; /* SOURCE IP */
	ipHeader -> ip_dst = (uint32_t) destIP; /* DESTINATION IP */
	ipHeader -> ip_tos = (uint8_t) 0;  /* Type of Service >> NOT SURE ABOUT THIS */
	ipHeader -> ip_len = (uint16_t) ((int)icmpHeader + (int)sizeof(sr_icmp_t3_hdr_t) - (int)ipHeader ); /*  Length */
	ipHeader -> ip_id = (uint16_t) 0;  /* First packet being sent back...? */
	ipHeader -> ip_off = (uint16_t) 0; /* Offset:Don't think the Datagram is split due to ICMP being one message */
	ipHeader -> ip_ttl = (uint8_t) 250;	/* TTL >> NOT SURE ABOUT THIS */
    ipHeader -> ip_p = (uint8_t) 1; 	/* Protocol: ICMP */
    ipHeader -> ip_sum = (uint16_t) cksum(ipHeader, sizeof(sr_ip_hdr_t));

    /* ICMP info */
    icmpHeader -> icmp_code = (uint8_t) icmpCode;
    icmpHeader -> icmp_type = (uint8_t) 3;
    icmpHeader -> icmp_sum = (uint16_t) cksum(icmpHeader, sizeof(sr_icmp_hdr_t));
    icmpHeader -> next_mtu = nextMTU;
    icmpHeader -> unused = 0;

    return newPacket;
}

uint8_t * createARPreply(
		uint8_t sourceMAC[6],
		uint8_t destMAC[6],
		uint32_t sourceIP,
		uint32_t destIP
		)
{
	uint8_t * newPacket;

	newPacket = malloc( sizeof(sr_ethernet_hdr_t) + sizeof(sr_arp_hdr_t) );

	sr_ethernet_hdr_t * ethernetHeader = newPacket;
	sr_arp_hdr_t * arpHeader = newPacket + sizeof( sr_ethernet_hdr_t );

	ethernetHeader -> ether_type =  htons(0x0806);

	arpHeader -> ar_hrd = 0;	/* Hardware */
	arpHeader -> ar_pro = htons(0x0800);	/* Protocol */
	arpHeader -> ar_hln = 6;
	arpHeader -> ar_pln = 4;
	arpHeader -> ar_op = htons(2);	/* Op code, reply */

	int i;
	for ( i = 0; i < 6; i++ )
	{
		ethernetHeader -> ether_shost[i] = (uint8_t) sourceMAC[i];
		ethernetHeader -> ether_dhost[i] = (uint8_t) destMAC[i];
		arpHeader -> ar_sha[i] = (uint8_t) sourceMAC[i];
		arpHeader -> ar_tha[i] = (uint8_t) destMAC[i];
	}

	arpHeader -> ar_sip = sourceIP;
	arpHeader -> ar_tip = destIP;

	return newPacket;
}

/* Searches the router's Interfaces for a given IP. If found, return it.
 * If not, return null.
 */
struct sr_if * findInterfaceInRouter(struct sr_if * currentInterface, uint32_t ip)
{
	/* Goes through the router's interface list */
	while ( currentInterface != NULL )
	{
		if ( ip == currentInterface -> ip )
			break;

		currentInterface = currentInterface -> next;
	}

	return currentInterface;
}

struct sr_if * findInterfaceWithName(struct sr_if * currentInterface, char * name)
{
	/* Goes through the router's interface list */
	while ( currentInterface != NULL )
	{
		if ( memcmp(name, currentInterface -> name, 4 ) )
			break;

		currentInterface = currentInterface -> next;
	}

	return currentInterface;
}

void sr_handlepacket(struct sr_instance* sr,
		uint8_t * packet/* lent */,
		unsigned int len,
		char* interface/* lent */)
{
	/* REQUIRES */
	assert(sr);
	assert(packet);
	assert(interface);

	printf("*** -> Received packet of length %d \n",len);

	print_hdr_eth(packet);
	print_hdr_arp(packet + sizeof(sr_ethernet_hdr_t));
	/* figure out what kind of packet it is */
	sr_ethernet_hdr_t *header = packet;
	int type = ntohs(header -> ether_type);
	printf("TEST: %d\n\n", type);
	if(type == 2048)
	{ /* ip packet (not sure if it's supposed to be a hex value or what */
		sr_ip_hdr_t *ip_packet = packet+21;

		if(cksum(&ip_packet,len-4)==ip_packet->ip_sum)
		{ /* I _think_ len is in terms of bytes */
			printf("Checksum OK!\n");
			/* Check if the packet is destined to one of the router's IPs */
			struct sr_if * destInterface = findInterfaceInRouter(sr -> if_list, ip_packet -> ip_dst);

			if ( destInterface != NULL )
				/* The packet is directed to one of the router's interfaces! */
			{
				printf("Found an interface to whom it was destined!\n");
				/* If it contains a TCP/UDP payload */
				if ( ( ip_packet -> ip_p == 6 ) || ( ip_packet -> ip_p ==  17 ) )
				{
					/* Make Port Unreachable packet (type 3, code 3) */
					uint8_t * newFrame = createICMP_type3Packet(header -> ether_dhost,
							header -> ether_shost, destInterface -> ip, ip_packet ->ip_src,
							3, 0);
					/* Sends new Packet from interface that the packet was destined to */
					sr_send_packet(sr, newFrame, (uint) sizeof(sr_ethernet_hdr_t) + (uint) sizeof(sr_ip_hdr_t)
							+ (uint) sizeof(sr_icmp_t3_hdr_t), destInterface -> name);

					free(newFrame);
				}

				if ( ip_packet -> ip_p == 1 )	/* It's and ICMP packet */
				{
					printf("It's an ICMP!\n");
					struct sr_icmp_hdr * icmp_packet = ip_packet + sizeof(sr_ip_hdr_t);

					if ( icmp_packet -> icmp_type == 8 ) /* It's an echo request */
					{
						printf("It's an echo!\n");
						/* Creates ICMP echo response */
						uint8_t * newFrame = createICMPPacket(destInterface -> addr,
								header -> ether_shost, destInterface -> ip, ip_packet ->ip_src,
								0, 0);
						/* Sends new Packet from interface that the packet was destined to */
						sr_send_packet(sr, newFrame,
								(uint) sizeof(sr_ethernet_hdr_t) + (uint)sizeof(sr_arp_hdr_t),
								destInterface -> name);

						free(newFrame);
					}
				}
			}

			ip_packet->ip_ttl--;
			ip_packet->ip_sum = cksum(&ip_packet,len-4);
			struct sr_rt *rt = sr->routing_table;

			while(rt->next){
				/* longest prefix match */
				uint32_t dest = ip_packet->ip_dst;

			}

		}
	}
	/* If it's an ARP packet */
	else if ( type ==  2054)
	{
		printf("----> Found and ARP Packet!\n");
		sr_arp_hdr_t * arpHeader = packet + sizeof(sr_ethernet_hdr_t);

		/* Check if it's a request */
		if ( ntohs(arpHeader -> ar_op) == 1 )
		{
			printf("----> It's a request!\n");
			struct sr_if * interface = findInterfaceInRouter(sr -> if_list, arpHeader -> ar_tip);
			if ( interface == NULL )
			{
				/* It was meant for some other IP! */
			}
			else
			{
				printf("----> Received an ARP request to find router's IP:\n");
				print_addr_ip_int(ntohl(interface -> ip));
				/* It's a request to find the router's IP */
				/* So, the router must send an ARP reply to the sender */
				struct sr_if * intByName = findInterfaceWithName(sr -> if_list, interface);
				uint8_t * newFrame = createARPreply(
						intByName -> addr,
						arpHeader -> ar_sha,
						intByName -> ip,
						arpHeader -> ar_sip
				);
				/* Sends new Packet from interface that the packet was destined to */
				printf("----> Attempting to send packet:\n");
				print_hdrs(newFrame,
						(uint) sizeof(sr_ethernet_hdr_t) + (uint) sizeof(sr_arp_hdr_t));
				sr_send_packet(sr, newFrame,
						(uint) sizeof(sr_ethernet_hdr_t) + (uint) sizeof(sr_arp_hdr_t),
						intByName -> name);

				free(newFrame);
			}
		}
	}
}/* end sr_ForwardPacket */


