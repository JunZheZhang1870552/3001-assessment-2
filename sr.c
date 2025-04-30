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

struct pkt recv_buffer[SEQSPACE];      /* Receive buffer */
int recv_status[SEQSPACE];             /* 0 = not received, 1 = received */
int expectedseqnum = 0;                /* Base of receiver window */


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

/*static struct pkt buffer[WINDOWSIZE];   array for storing packets waiting for ACK */
/*static int windowfirst, windowlast;     array indexes of the first/last packet awaiting ACK */
/*static int windowcount;                 the number of packets currently awaiting an ACK */
/*static int A_nextseqnum;                the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message) {
    struct pkt packet;
    int i;
  /* Check if the sending window is full */
  if (nextseqnum < base + WINDOW_SIZE) {

    /* Fill the packet fields */
    packet.seqnum = nextseqnum;
    packet.acknum = -1;  /* Not used for data packets */
    memcpy(packet.payload, message.data, 20);

    /* Compute checksum */
    packet.checksum = 0;
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
    printf("----A: New message arrives, send window is full, do nothing!\n");
  }
}



/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet) {
  /*Check for corruption*/
  int acknum;
  acknum = packet.acknum;

  if (IsCorrupted(packet)) {
      if (TRACE > 0)
          printf("----A: corrupted ACK %d is received, ignored.\n", packet.acknum);
      return;
  }

  if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);


  /*Check if ACK is within the sending window*/
  if (acknum >= base && acknum < base + WINDOW_SIZE) {
      if (ack_status[acknum % BUFFER_SIZE] == 0) {  /* Not a duplicate */
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
void A_timerinterrupt(void) {
  int i;

  /* Find the first unacknowledged packet in the sending window */
  for (i = base; i < nextseqnum; i++) {
      if (ack_status[i % BUFFER_SIZE] == 0) {  /* Unacknowledged */
          if (TRACE > 0)
              printf("----A: Timer interrupt, resending packet %d\n", i);

          /* Resend the packet */
          tolayer3(0, send_buffer[i % BUFFER_SIZE]);

          /* Restart the timer */
          starttimer(0, 16.0);
          break;  /* Only retransmit one packet (the earliest unacked) */
      }
  }
}
   



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void) {
  int i;

  /* Initialize base and next sequence number */
  base = 0;
  nextseqnum = 0;

  /* Initialize ACK status array to 0 (not acknowledged) */
  for (i = 0; i < BUFFER_SIZE; i++) {
      ack_status[i] = 0;
  }

  /* No need to initialize send_buffer[], as packets will be written when sent */
}




/********* Receiver (B)  variables and procedures ************/

/*static int expectedseqnum; the sequence number expected next by the receiver */
/*static int B_nextseqnum;  the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet) {
  int seq;
  seq = packet.seqnum;
  
  if (IsCorrupted(packet)) {
      if (TRACE > 0)
        printf("----B: packet %d is corrupted, ignored\n", packet.seqnum);
      return;
  }

  

  if (TRACE > 0)
      printf("----B: received packet %d\n", seq);

  /* Check if within receiver window */
  if (seq >= expectedseqnum && seq < expectedseqnum + WINDOW_SIZE) {
      /* Not received before */
      if (recv_status[seq % SEQSPACE] == 0) {
          recv_buffer[seq % SEQSPACE] = packet;
          recv_status[seq % SEQSPACE] = 1;

          if (TRACE > 0)
              printf("----B: packet %d stored in buffer and ACKed\n", seq);
      } else {
          if (TRACE > 0)
              printf("----B: duplicate packet %d, ACK again\n", seq);
      }

      /* Always ACK it */
      struct pkt ack;
      ack.seqnum = 0;  /* Not used */
      ack.acknum = seq;
      ack.checksum = ComputeChecksum(ack);
      tolayer3(1, ack);

      /* Deliver in-order packets to layer 5 */
      while (recv_status[expectedseqnum % SEQSPACE] == 1) {
          tolayer5(1, recv_buffer[expectedseqnum % SEQSPACE].payload);
          recv_status[expectedseqnum % SEQSPACE] = 0;  /* Clear */
          expectedseqnum++;
      }
  } else {
      if (TRACE > 0)
          printf("----B: packet %d out of window, ignored or ACKed\n", seq);

      /* ACK anyway to stop sender from retransmitting */
      struct pkt ack;
      ack.seqnum = 0;
      ack.acknum = seq;
      ack.checksum = ComputeChecksum(ack);
      tolayer3(1, ack);
  }
}


/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void) {
  int i;
  expectedseqnum = 0;
  for (i = 0; i < SEQSPACE; i++) {
      recv_status[i] = 0;
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

