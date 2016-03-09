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

typedef struct QueueNode{
	struct QueueNode* front;
	struct QueueNode* next;
	sr_ip_hdr_t *packet;
}queue_node;


//push ip packet *p onto queue q
void enqueue(queue_node q, sr_ip_hdr_t *p){
	queue_node curr = q->front;
	while(curr->next){
		curr = curr->next;
	}
	curr->next = NULL;
	curr->packet = p;
}
//pop the first element of q
queue_node* dequeue(queue_node q){
	queue_node curr = q->front;
	q->front = curr->next;
	return curr;

}

uint32_t* longest_prefix(struct sr_instance* sr, uint32_t dst){
	struct sr_rt *rt = sr->routing_table;
	struct sr_rt *best;
	struct sr_rt *curr = rt;
	while(curr){
		uint32_t entry = curr->dest;
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

  /* fill in code here */

  //figure out what kind of packet it is
  sr_ethernet_hdr_t *header = packet;
  uint16_t type = header->ether_type;
  if(type==2048){//ip packet (not sure if it's supposed to be a hex value or what
	  sr_ip_hdr_t *ip_packet = packet+21;

	  if(cksum(&ip_packet,len-4)==ip_packet->ip_sum){//I _think_ len is in terms of bytes

		  // Check if the packet is destined to one of the router's IPs
		  struct sr_if * currentInterface = sr -> if_list; // Goes through the router's interface list
		  while ( currentInterface != NULL )
		  {
			  if ( ip_packet -> ip_dst == currentInterface -> ip )
			  {
				  break;
			  }

		  }

		  if ( currentInterface != NULL )
		  {
			  if ( ip_packet -> ip_p == 1 )	// It's and ICMP packet
			  {
				  struct sr_icmp_hdr * icmp_packet = ip_packet + sizeof(sr_ip_hdr_t);

				  if ( icmp_packet -> icmp_type == 8 ) // It's an echo request
				  {
					  // Make packet to reply
				  }
			  }
		  }


		  ip_packet->ip_ttl--;
		  ip_packet->ip_sum = cksum(&ip_packet,len-4);
		  struct sr_rt *rt = sr->routing_table;

		  while(rt->next){
			  //longest prefix match
			  uint32_t dest = ip_packet->ip_dst;

		  }
	  }
  }
}/* end sr_ForwardPacket */
