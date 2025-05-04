#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"


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
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */


bool isInWindow(int seq, int base, int size) {
  if (base + size < SEQSPACE)
      return seq >= base && seq < base + size;
  else
      return (seq >= base && seq < SEQSPACE) || (seq < (base + size) % SEQSPACE);
}




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

static struct pkt buffer[SEQSPACE];  /* array for storing packets waiting for ACK */
static int windowfirst;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int acked[SEQSPACE];          /* SR: track which packets are ACKed */


/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    buffer[A_nextseqnum] = sendpkt;
    acked[A_nextseqnum] = 0;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* Only start timer if this is the first unACKed packet */
    if (windowcount == 1) {
      starttimer(A, RTT);
    }

    /* no cumulative ACK logic here; SR handles ACK per packet */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int acknum = packet.acknum;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* check if new ACK or duplicate */
    if (windowcount != 0) {
      int seqfirst = buffer[windowfirst].seqnum;
      int seqlast = buffer[(windowfirst + windowcount - 1) % SEQSPACE].seqnum;

      /* check case when seqnum has and hasn't wrapped */
      if (((seqfirst <= seqlast) && (acknum >= seqfirst && acknum <= seqlast)) ||
          ((seqfirst > seqlast) && (acknum >= seqfirst || acknum <= seqlast))) {

        /* packet is a new ACK */
        if (acked[acknum] == 0) {
          acked[acknum] = 1;

          /*if (TRACE > 2)
            printf("[DEBUG] A received and processed ACK %d\n", acknum);*/



          if (TRACE > 0)
            printf("----A: ACK %d is not a duplicate\n", packet.acknum);
          new_ACKs++;

          /* slide window base forward only if ACK is for the windowfirst */
          while (windowcount > 0 && acked[buffer[windowfirst].seqnum]) {
            windowfirst = (windowfirst + 1) % SEQSPACE;
            windowcount--;
          }

          stoptimer(A);
          if (windowcount > 0){
            starttimer(A, RTT);  /* restart for earliest unacked packet */
          }
          
        }
        else {
          if (TRACE > 0)
            printf("----A: duplicate ACK received, do nothing!\n");
        }
      }
    }
    else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}


/* called when A's timer goes off*/
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* Selective Repeat: retransmit the earliest unACKed packet */
  for (i = 0; i < SEQSPACE; i++) {
    if (acked[i] == 0 && buffer[i].seqnum != NOTINUSE) {
      tolayer3(A, buffer[i]);


      /*if (TRACE > 2)
        printf("[DEBUG] A retransmitting packet seq %d\n", buffer[i].seqnum);*/
      if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[i].seqnum);
      
      packets_resent++;
    }
  }

  /* Restart timer after retransmission */
  starttimer(A, RTT);
}


   



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  A_nextseqnum = 0;
  windowfirst = 0;
  windowcount = 0;

  for (i = 0; i < SEQSPACE; i++) {
    acked[i] = 0;
    buffer[i].seqnum = NOTINUSE;
  }
}





/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum;                     /*SR: base of receiver window*/ 
static struct pkt B_buffer[SEQSPACE];        /*buffer for out-of-order packets*/
static int B_received[SEQSPACE];             /*flags: whether a seqnum has been received*/



/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  int seq = packet.seqnum;
  int i;
  struct pkt ack;
  struct msg message;

  if (IsCorrupted(packet)) {
    /* packet is corrupted or out of order resend last ACK */
    printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    ack.seqnum = 0;
    ack.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    for (i = 0; i < 20; i++) ack.payload[i] = 0;
    ack.checksum = ComputeChecksum(ack);
    tolayer3(1, ack);
    return;
  }

  /* check if seq is within receiving window */
  if (isInWindow(seq, expectedseqnum, WINDOWSIZE)) {

    if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seq);


    if (!B_received[seq]) {
      B_received[seq] = 1;
      B_buffer[seq] = packet;

      packets_received++;

      /*if (TRACE > 2)
        printf("[DEBUG] B received packet %d\n", seq);*/

    }

    /* send ACK */
    ack.seqnum = 0;
    ack.acknum = seq;
    for (i = 0; i < 20; i++) ack.payload[i] = 0;
    ack.checksum = ComputeChecksum(ack);
    tolayer3(1, ack);


    /* deliver in-order packets to layer5 */
    while (B_received[expectedseqnum]) {
      for (i = 0; i < 20; i++) {
        message.data[i] = B_buffer[expectedseqnum].payload[i];
      }

      tolayer5(B, message.data);
      B_received[expectedseqnum] = 0;
      B_buffer[expectedseqnum].seqnum = NOTINUSE;
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }
  } else {
    /* packet is outside window: resend last ACK */
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", seq);
    ack.seqnum = 0;
    ack.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    for (i = 0; i < 20; i++) ack.payload[i] = 0;
    ack.checksum = ComputeChecksum(ack);
    tolayer3(1, ack);


    /*if (TRACE > 2)
      printf("[DEBUG] B sending ACK %d\n", ack.acknum);*/

      
  }
}








/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  expectedseqnum = 0;

  for (i = 0; i < SEQSPACE; i++) {
    B_received[i] = 0;
    B_buffer[i].seqnum = NOTINUSE;
  }
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

