#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"
#include <string.h>


/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

#define WINDOW_SIZE 6             /* SR window size */
#define BUFFER_SIZE 50            /* Total buffer size */

struct pkt send_buffer[BUFFER_SIZE];   /* Sending buffer */
int ack_status[BUFFER_SIZE];           /* ACK status: 0 (not acknowledged), 1 (acknowledged) */

int base = 0;            /* The base sequence number of the sending window */
int nextseqnum = 0;      /* The next sequence number to use */


/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message) {
  /* Check if the sending window is full */
  if (nextseqnum < base + WINDOW_SIZE) {
      struct pkt packet;

      /* Fill the packet fields */
      packet.seqnum = nextseqnum;
      packet.acknum = -1;  /* Not used for data packets */
      memcpy(packet.payload, message.data, 20);

      /* Compute checksum */
      packet.checksum = 0;
      int i;
      for (i = 0; i < 20; i++) {
          packet.checksum += packet.payload[i];
      }
      packet.checksum += packet.seqnum + packet.acknum;

      /* Store the packet in the sending buffer */
      send_buffer[nextseqnum % BUFFER_SIZE] = packet;
      ack_status[nextseqnum % BUFFER_SIZE] = 0;  /* Not acknowledged yet */

      /* Send the packet to layer 3 */
      tolayer3(0, packet);

      /* Start the timer if this is the base packet */
      if (base == nextseqnum) {
          starttimer(0, 16.0);
      }

      nextseqnum++;
  } else {
      /* Window is full, discard the message */
      printf("A_output: window is full, message discarded\n");
  }
}



/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet) {
  /*Check for corruption*/
  if (IsCorrupted(packet)) {
      if (TRACE > 0)
          printf("----A: corrupted ACK %d is received, ignored.\n", packet.acknum);
      return;
  }

  if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);

  int acknum = packet.acknum;

  /*Check if ACK is within the sending window*/
  if (acknum >= base && acknum < base + WINDOW_SIZE) {
      if (ack_status[acknum % BUFFER_SIZE] == 0) {  // Not a duplicate
          ack_status[acknum % BUFFER_SIZE] = 1;
          if (TRACE > 0)
              printf("----A: ACK %d is not a duplicate\n", acknum);

          /*Slide window forward if ACK is for the base*/
          if (acknum == base) {
              while (ack_status[base % BUFFER_SIZE] == 1 && base < nextseqnum) {
                  base++;
              }

              /*Timer management*/ 
              stoptimer(0);
              if (base != nextseqnum) {
                  starttimer(0, 16.0);
              }
          }
      } else {
          if (TRACE > 0)
              printf("----A: duplicate ACK %d received, ignored.\n", acknum);
      }
  } else {
      if (TRACE > 0)
          printf("----A: ACK %d not in window, ignored.\n", acknum);
  }
}


/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  for(i=0; i<windowcount; i++) {

    if (TRACE > 0)
      printf ("---A: resending packet %d\n", (buffer[(windowfirst+i) % WINDOWSIZE]).seqnum);

    tolayer3(A,buffer[(windowfirst+i) % WINDOWSIZE]);
    packets_resent++;
    if (i==0) starttimer(A,RTT);
  }
}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;

  /* if not corrupted and received packet is in order */
  if  ( (!IsCorrupted(packet))  && (packet.seqnum == expectedseqnum) ) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
    packets_received++;

    /* deliver to receiving application */
    tolayer5(B, packet.payload);

    /* send an ACK for the received packet */
    sendpkt.acknum = expectedseqnum;

    /* update state variables */
    expectedseqnum = (expectedseqnum + 1) % SEQSPACE;        
  }
  else {
    /* packet is corrupted or out of order resend last ACK */
    if (TRACE > 0) 
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    if (expectedseqnum == 0)
      sendpkt.acknum = SEQSPACE - 1;
    else
      sendpkt.acknum = expectedseqnum - 1;
  }

  /* create packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;
    
  /* we don't have any data to send.  fill payload with 0's */
  for ( i=0; i<20 ; i++ ) 
    sendpkt.payload[i] = '0';  

  /* computer checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt); 

  /* send out packet */
  tolayer3 (B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

