/**
* @file listener.c
* @brief Listen for client TCP connections
* @author John Melton, G0ORX/N6LYT
* @version 0.1
* @date 2009-10-13
*/

/* Copyright (C)
* 2009 - John Melton, G0ORX/N6LYT
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*
*/

/* Modified listener.c from the above author to create a simple protocol converter between 
* a physical Ozy USB to simulate a Metis Ethernet link.
*
* Version 0.01 (development) Bob Wisdom VK4YA May 2015
*
* Notes:	DEVELOPMENT/EXPERIMENTAL VERSION
*
* If this ever gets re-integrated into ghpsd3-alex then this filename 'listner.c' will have to change as it clashes with the original.
* The converter could simultaneously stand alongside all the other functionality in ghpsdr3-alex, so USB can be supported 
* through this converter (not yet tested May '15).
* The directory does have other changes to various source files which will also need integrating properly -
*	mostly the usb files, and various brutal ugly hacks to stop John's original ghpsdr3-alex code running.
*
* As an alternative, the converter could easily be extracted from John's file arrangement and made into a stand-alone application.
* At present it's left John's stuff in a bit of a mess!
*
*		There are two Makefiles Makefile.x86 and Makefile.rpi2 for each platform.
*			gcc-4.8 not gcc-4.6 used for RPi2  (On Raspian 'apt-get install gcc-4.8')
*
*
* Bugs: Timing and mutual exclusion have not explicitly been addressed here so far, so there could be some glaring issues. 
*		The program takes about 5-10 seconds to be ready after loading the firmware into Ozy, so don't try to discover/connect too early.
*		Discovery is only partially supported - the converter will release any existing connection and attach to any new discovery attempt
*			Therefore multiple HPSDRs on the LAN are not supported
*		Multiple Mercury boards in one HPSDR are not supported.
*		I2C access seems to be a bit of an issue blocking the data streams too much
*		There is sync bug where the EP6 512 byte packet stream seems to collect a prefix of maybe 20-60 bytes of what looks like EP4 data.
*			The resultant data bytes appearing before the 0x7f7f7f sync starts can only be resolved by reloading the Ozy firmware. 
*			The problem may be in the USB processor on the Ozy, usblib, or its probably triggered by doing something wrong in this code.
*			When the sync error occurs the program will detect this error and output to stderr the bad packets.
*		Metis sequence numbers are ignored on recieve from PC path
*		Not sure if we collect the correct fwd power readings from Pennylane or Pennywhistle: is one the ALC value?
*		Do not try to regenerate the Makefiles using the config / buildtools as these have not yet been addressed.
*
*		The source code needs a good tidy up, headers and defines re-arranging etc.
* 
*/

#include <stdio.h>
#include <stdlib.h>
#ifdef __linux__
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <unistd.h>
#else
#include "pthread.h"
#endif

#define METIS_VER 26

#include <string.h>

#include "ozyio.h"
#include "ozy.h"
#include "listener.h"
#include "util.h"

#define PORT 1024
#define inaddrr(x) (*(struct in_addr *) &ifr->x[sizeof sa.sin_port])
#define ETH_BUFSIZE 1032
#define USB_BUFSIZE 1024
#define USB_BUFFERS 1
#define BANDSCOPE_BUFFER_SIZE (8192)
#define EP6_BUFFER_SIZE 2048

static unsigned char hw_address[6];
static long ip_address;
static char *interface = "eth0";
#if 0
static unsigned char cmd_buffer[ETH_BUFSIZE + 4];
static unsigned char cmd_to_usb_buffer[2048 + 4];
static unsigned char disco_rply_buffer[60 + 4];
static unsigned char ep4_inbuffer[BANDSCOPE_BUFFER_SIZE + 4];
static unsigned char ep6_inbuffer[BANDSCOPE_BUFFER_SIZE + 4];
static unsigned char ep4_outbuffer[ETH_BUFSIZE + 4];
static unsigned char ep6_outbuffer[ETH_BUFSIZE + 4];
#else
static unsigned char cmd_buffer[ETH_BUFSIZE];
static unsigned char cmd_to_usb_buffer[2048];
static unsigned char disco_rply_buffer[60];
static unsigned char ep4_inbuffer[BANDSCOPE_BUFFER_SIZE];       // bug zzz
static unsigned char ep6_inbuffer[BANDSCOPE_BUFFER_SIZE];
static unsigned char ep4_outbuffer[ETH_BUFSIZE];
static unsigned char ep6_outbuffer[ETH_BUFSIZE];
#endif
pthread_mutex_t lock;
void *listener_thread(void *arg);
void *client_thread(void *arg);
void writepenny(unsigned char mode);
void *ozy_ep4_read_thread(void *arg);
void *ozy_ep6_read_thread(void *arg);
void resync(unsigned char *inbuff, int size);
static int started_iq = 0;
static int started_wband = 0;
static int metis_sending = 0;
static int s;                   /* socket */
static struct sockaddr_in address;
static unsigned int length;
extern void ozy_i2c_readpwr();
unsigned char penny_fw = 0, mercury_fw = 0;
unsigned short penny_fp = 0, penny_rp = 0, penny_alc = 0;
int mox = 0;
int adc_overflow;

static int get_addr(int sock, char *ifname)
{

  struct ifreq *ifr;
  struct ifreq ifrr;
  struct sockaddr_in sa;
  unsigned char *u;
  int i;

  ifr = &ifrr;
  ifrr.ifr_addr.sa_family = AF_INET;
  strncpy(ifrr.ifr_name, ifname, sizeof(ifrr.ifr_name));

  if (ioctl(sock, SIOCGIFADDR, ifr) < 0) {
    printf("No %s interface.\n", ifname);
    return -1;
  }

  ip_address = inaddrr(ifr_addr.sa_data).s_addr;

  if (ioctl(sock, SIOCGIFHWADDR, ifr) < 0) {
    printf("No %s interface.\n", ifname);
    return -1;
  }

  u = (unsigned char *)&ifr->ifr_addr.sa_data;

  for (i = 0; i < 6; i++)
    hw_address[i] = u[i];

  return 0;
}

void create_listener_thread()
{

  pthread_t thread_id;
  pthread_t ozy_ep4_read_thread_id;
  pthread_t ozy_ep6_read_thread_id;
  int rc;

  // create the thread to listen for TCP connections
  rc = pthread_create(&thread_id, NULL, listener_thread, NULL);
  if (rc < 0) {
    perror("pthread_create listener_thread failed");
    exit(1);
  }
  rc = pthread_create(&ozy_ep4_read_thread_id, NULL, ozy_ep4_read_thread, NULL);
  if (rc != 0) {
    fprintf(stderr, "pthread_create failed on ozy_ep4_read_thread: rc=%d\n", rc);
    exit(1);
  }
  rc = pthread_create(&ozy_ep6_read_thread_id, NULL, ozy_ep6_read_thread, NULL);
  if (rc != 0) {
    fprintf(stderr, "pthread_create failed on ozy_ep6_read_thread: rc=%d\n", rc);
    exit(1);
  }

}

// listens on ethernet impersonating metis, and processes ethernet commands -> usb
void *listener_thread(void *arg)
{
  struct hostent *h;
  int i;
  int bytes_read;
  int on = 1;
  int buf_flag = 0, usbtxbytes = 0;
  unsigned char micboost = 0x55;
  unsigned char srcaddr[] = "255.255.255.255";
  unsigned short srcport = 1024;

  fprintf(stderr, "Starting main listener thread v1.1\n");

  if (pthread_mutex_init(&lock, NULL)) {
    printf("Unable to initialize lock mutex\n");
    exit(1);
  }
  // create TCP socket to listen on
  s = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) {
    perror("Metis Listen socket failed");
    exit(1);
  }
  // get my MAC address and IP address
  if (get_addr(s, interface) < 0) {
    exit(1);
  }
  printf("Metis %s IP Address: %ld.%ld.%ld.%ld\n",
         interface, ip_address & 0xFF, (ip_address >> 8) & 0xFF, (ip_address >> 16) & 0xFF, (ip_address >> 24) & 0xFF);

  printf("Metis %s MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
         interface, hw_address[0], hw_address[1], hw_address[2], hw_address[3], hw_address[4], hw_address[5]);

  setsockopt(s, SOL_SOCKET, /* SO_BROADCAST */ SO_REUSEADDR, &on, sizeof(on));

  // bind to listening port
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY; // ip_address
  address.sin_port = htons(PORT);
  if (bind(s, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("Metis bind failed");
    exit(1);
  }

  fprintf(stderr, "Metis Listening for UDP datagrams on port %d\n", PORT);
  while (1) {

    bytes_read = recvfrom(s, cmd_buffer, sizeof(cmd_buffer), 0, (struct sockaddr *)&address, &length);
    //            fprintf(stderr,"listen:s=%x, address=%lx, length=%d,\n",s,(long int)&address,length);        
    if (bytes_read < 0) {
      perror("recvfrom socket failed for metis listener thread");
      exit(1);
    }
    //        fprintf(stderr,"Metis read %d bytes, %02x:%02x:%02x:%02x\n",bytes_read,
    //                  cmd_buffer[0],cmd_buffer[1],cmd_buffer[2],cmd_buffer[3]);           

    strcpy(srcaddr,inet_ntoa(address.sin_addr));
 //   fprintf(stderr,"Src Address=%s\n",srcaddr); //  (void*)&addr,sizeof(addr)

    srcport = ntohs(address.sin_port);
//    fprintf(stderr,"Src Port=%d\n",srcport);

newtry:
    if ((bytes_read == 63) && (cmd_buffer[0] == 0xEF && // discovery received
                               cmd_buffer[1] == 0xFE && cmd_buffer[2] == 0x02 && cmd_buffer[3] == 0x00)) {
      fprintf(stderr, "Metis Received discovery UDP\n");
      fprintf(stderr, "Metis Stopping Tx Data (Kludge)\n");

      started_wband = 0;        // Kludge
      started_iq = 0;           // Kludge
      metis_sending = 0;        // Kludge
// send the discovery reply

      disco_rply_buffer[0] = 0xEF;
      disco_rply_buffer[1] = 0xFE;
      if (metis_sending == 0) {
        disco_rply_buffer[2] = 0x02;    // metis is not sending ( 3 == already sending)
      } else {
        disco_rply_buffer[2] = 0x03;
      }
      for (i = 0; i < 6; i++) {
        disco_rply_buffer[3 + i] = hw_address[i];
      }
      disco_rply_buffer[9] = METIS_VER; // Code serialno or Version guess
      disco_rply_buffer[10] = 0x00;     // board id == Metis
      for (i = 11; i < 60; i++) {
        disco_rply_buffer[i] = 0x00;
      }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = inet_addr(srcaddr);
//  address.sin_port = htons(PORT);
  address.sin_port = htons(srcport);

      
      if (sendto(s, disco_rply_buffer, 60, 0, (struct sockaddr *)&address, length) < 0) {
        perror("sendto socket failed for metis_send_data discovery reply\n");
        exit(1);
      } else {
        fprintf(stderr, "Metis replied to discovery UDP\n");
      }
    }
    if ((bytes_read >= 60) && (cmd_buffer[0] == 0xEF && cmd_buffer[1] == 0xFE && cmd_buffer[2] == 0x04)) {
      fprintf(stderr, "Metis Received start/stop 2 command %d, size %d\n", cmd_buffer[3], bytes_read);
      switch (cmd_buffer[3]) {
      case 00:                 // stop I&Q and wide Bandscope data
        started_wband = 0;
        started_iq = 0;
        metis_sending = 0;
        buf_flag = 0;
        usbtxbytes = 0;
        break;
      case 01:                 // start I&Q (stop wide bandscope)
        started_iq = 1;
        started_wband = 0;
        metis_sending = 1;
        break;
      case 02:                 // start wide bandscope data streem
        started_iq = 0;
        started_wband = 1;
        metis_sending = 1;
        break;
      case 03:                 // start IQ and wide bandscope
        started_iq = 1;
        started_wband = 1;
        metis_sending = 1;
        break;
      default:
        fprintf(stderr, "Metis Start/Stop command 2 with unknown data\n");
        exit(1);
        break;
      }                         // end switch
      //        dump_metis_buffer("X",1,cmd_buffer);
    }

    switch (buf_flag) {         // bundle two UDP 1kB packets into one 2kB USB packet
    case 0:                    // this is the first buffer
      // read data from the PC - this goes to the USB       
      if ((bytes_read == 1032) && (cmd_buffer[0] == 0xEF &&     // data for ozy
                                   cmd_buffer[1] == 0xFE && cmd_buffer[2] == 0x01 && cmd_buffer[3] == 0x02)) {  // Metis C&C 1K packet
        buf_flag = 1;
        memcpy(cmd_to_usb_buffer, &cmd_buffer[8], 1024);        // fill the first half of the tousb buff      
      }
      break;
    case 1:                    // this is the second buffer        
      if ((bytes_read == 1032) && (cmd_buffer[0] == 0xEF &&     // data for ozy
                                   cmd_buffer[1] == 0xFE && cmd_buffer[2] == 0x01 && cmd_buffer[3] == 0x02)) {  // Metis C&C 1K packet
        buf_flag = 0;
        memcpy(&cmd_to_usb_buffer[1024], &cmd_buffer[8], 1024); // fill the second half of the tousb buff 
#if 0
        if (usbtxbytes <= (8192 * 8)) {
          dump_ozy_header("1", usbtxbytes, cmd_to_usb_buffer);
          dump_ozy_header("2", usbtxbytes, &cmd_to_usb_buffer[512]);
          dump_ozy_header("3", usbtxbytes, &cmd_to_usb_buffer[1024]);
          dump_ozy_header("4", usbtxbytes, &cmd_to_usb_buffer[1536]);
        }
#endif
        i = ozy_write(0x02, cmd_to_usb_buffer, 2048);   // and write the 4 usb frames to the usb in one 2k packet
        usbtxbytes += i;
        if (i != 2048) {
          perror("listener: OzyBulkWrite ozy failed");
        }
      }
      break;
    default:
      buf_flag = 0;
      fprintf(stderr, "buf_flag case error\n");
      break;
    }                           // end switch

    mox = (cmd_to_usb_buffer[3] | cmd_to_usb_buffer[3 + 512] | cmd_to_usb_buffer[3 + 1024] | cmd_to_usb_buffer[3 + 1536]) & 0x01;       // read MOX state

    if (usbtxbytes >= 2048) {   // make sure data sent to metis before allowing read
      metis_sending = 1;
    }

    if ((bytes_read == 1032) && (cmd_buffer[0] == 0xEF && cmd_buffer[1] == 0xFE && cmd_buffer[2] == 0x01 && cmd_buffer[3] == 0x02)) {   // its a normal EP2 C&C / mic data pair of frames

      if ((cmd_buffer[8 + 3] & 0xFE) == 0x12) { // C&C type has mic boost and info in C2
        if (micboost != (cmd_buffer[8 + 5] & 0x03)) {   // changed
          micboost = (cmd_buffer[8 + 5] & 0x03);
          writepenny(micboost); // also does Mic/Line in selection
        }
      } else {
        if ((cmd_buffer[8 + 3] & 0xFE) == 0x00) {       //
          // not implemented yet
          // unrecognised packet     
        }
      }
    } else {
      fprintf(stderr, "Metis expecting packet but got %02x:%02x:%02x:%02x,%02x:%02x:%02x:%02x\n",
              cmd_buffer[0], cmd_buffer[1], cmd_buffer[2], cmd_buffer[3],
              cmd_buffer[4], cmd_buffer[5], cmd_buffer[6], cmd_buffer[7]);
	//goto newtry;
    }
  }                             // end while
  exit(0);
}

/*
*
*  READ THREADS  ----------------------------------
* These read USB EP4 and EP6 and send to the PC
*
*/

// ep4 from hpsdr is the block of 4096x16 raw adc for the bandscope
void *ozy_ep4_read_thread(void *arg)
{
  int bytes;
  register unsigned char v;
  long int seqno = 0, totbytes = 0, waiting = 0;
  int i, j;

  //      for(;;) sleep(99);
  ep4_outbuffer[0] = 0xEF;
  ep4_outbuffer[1] = 0xFE;
  ep4_outbuffer[2] = 0x01;      // metis is not sending ( 3 == already sending)
  ep4_outbuffer[3] = 4;         // end point EP4 
  ep4_outbuffer[4] = 0;         // seq no 4 bytes, big endian */
  ep4_outbuffer[5] = 0;
  ep4_outbuffer[6] = 0;
  ep4_outbuffer[7] = 0;

// EP4 8K USB transfer size version ------------------------------
  while (1) {
    usleep(40000);              // 40ms
    if (totbytes % 10240000 == 0) {
      fprintf(stderr, "ep4 bytes =%ld\n", totbytes);
    }

    while ((started_wband == 0) || !(metis_sending)) {
      // nothing to send to pc 
      waiting++;
      if (waiting % 100000 == 0) {
        fprintf(stderr, "ep4_thread waiting\n");
      }
      seqno = 0;
      totbytes = 0;
      usleep(100);
    }
    //pthread_mutex_lock(&lock);
    bytes = ozy_read(0x84, ep4_inbuffer, BANDSCOPE_BUFFER_SIZE);        // read an 8K buffer at a time
    if (bytes < 0 || bytes != BANDSCOPE_BUFFER_SIZE) {
      fprintf(stderr, "ozyMet_ep4_read: OzyBulkRead failed %d bytes\n", bytes);
      perror("ozy_read(0x84 failed");
      exit(1);
    }
    //pthread_mutex_unlock(&lock);
    totbytes += bytes;
    for (i = 0; i < BANDSCOPE_BUFFER_SIZE - 1; i++) {   // swap bytes
      v = ep4_inbuffer[i];
      ep4_inbuffer[i] = ep4_inbuffer[i + 1];
      ep4_inbuffer[i + 1] = v;
    }
    // process the buffer, make 8 packets to send down eth to the PC
    for (i = 0; i < 1; i++) {
      for (j = 0; j < (BANDSCOPE_BUFFER_SIZE / USB_BUFSIZE); j++) {     // we need 8 lots of 1K buffers
        memcpy(&ep4_outbuffer[8], &ep4_inbuffer[USB_BUFSIZE * j], USB_BUFSIZE); // fill the tail with 1k of USB Data
        ep4_outbuffer[7] = seqno & 0xff;
        ep4_outbuffer[6] = (seqno >> 8) & 0xff;
        ep4_outbuffer[5] = (seqno >> 16) & 0xff;
        ep4_outbuffer[4] = (seqno >> 24) & 0xff;
        seqno++;
        if (sendto(s, ep4_outbuffer, ETH_BUFSIZE, MSG_DONTWAIT, (struct sockaddr *)&address, length) < 0) {     // send 1K
          perror("ozyMet_ep4_read_thread: sendto socket failed for metis_send_data\n");
          exit(1);
        }
      }                         // for j
    }                           // for i
  }                             // end while
}

// ep6 from hpsdr USB is the 0x7f0x7f protocol stuff
// package it up and send it to the PC
void *ozy_ep6_read_thread(void *arg)
{
  int bytes;
  long int seqno = 0, totbytes = 0, badcount = 0, waiting = 0;
  int i, j, syncerr = 0;
  unsigned char cmd;

  ep6_outbuffer[0] = 0xEF;      // make metis packet headers
  ep6_outbuffer[1] = 0xFE;
  ep6_outbuffer[2] = 0x01;      // metis is not sending ( 3 == already sending)
  ep6_outbuffer[3] = 6;         // ep
  ep6_outbuffer[4] = 0;         // seq no 4 bytes, big endian */
  ep6_outbuffer[5] = 0;
  ep6_outbuffer[6] = 0;
  ep6_outbuffer[7] = 0;

// EP6 2K USB transfer size version ------------------------------
  while (1) {
    if (totbytes % 10240000 == 0) {
      fprintf(stderr, "ep6 bytes =%ld, badcount=%ld\n", totbytes, badcount);
//                      fprintf(stderr,"ep6 fwd=%d, rev=%d, alc=%d, adc_overflow=%d\n",penny_fp,penny_rp,penny_alc,adc_overflow);
    }

    while ((started_iq == 0) || !(metis_sending)) {
      // nothing to send to pc 
      waiting++;
      if (waiting % 100000 == 0) {
        fprintf(stderr, "ep6_thread waiting\n");
      }
      seqno = 0;
      totbytes = 0;
      badcount = 0;
      syncerr = 0;
      usleep(100);
      adc_overflow = 0;
    }
    //pthread_mutex_lock(&lock);
    while (1) {
      bytes = ozy_read(0x86, ep6_inbuffer, EP6_BUFFER_SIZE);    // read a 2K buffer at a time
      if (bytes < 0 || bytes != EP6_BUFFER_SIZE) {
        if (bytes == 0) {
          fprintf(stderr, "ozyMet_ep6_read: ozy_read returned 0 bytes... retrying\n");
          continue;
        }
        fprintf(stderr, "ozyMet_ep6_read: OzyBulkRead failed %d bytes\n", bytes);
        perror("ozy_read(0x86 failed");
        exit(1);
      }
      break;
    }
    //pthread_mutex_unlock(&lock);
    totbytes += bytes;

    for (i = 0; i < 4; i++) {   // 4x 512 byte usb bufers
      if ((ep6_inbuffer[i * 512] != 0x7f) || (ep6_inbuffer[(i * 512) + 1] != 0x7f)
          || (ep6_inbuffer[(i * 512) + 2] != 0x7f)) {
        //                              resync(ep6_inbuffer,bytes);
        syncerr++;
        if (totbytes % 100000 == 0) {
          dump_ozy_buffer("ep6 received bad sync on pkt ", i, &ep6_inbuffer[i * 512]);
        }
      }
      // process the buffer recv'd on the usb from ozy

      cmd = ep6_inbuffer[(i * 512) + 3];

      if ((cmd & 0xF8) == 0x00) {       // C0 = 0 serial no stuff
        ep6_inbuffer[(i * 512) + 7] = METIS_VER;
      }

      if ((cmd & 0xF8) == 0x20) {       // adc overflow cmd packet
        if (adc_overflow) {
          ep6_inbuffer[(i * 512) + 4] |= 1;
          adc_overflow--;
        } else {
          ep6_inbuffer[(i * 512) + 4] &= 0xFE;
        }
        if (totbytes % 819200L == 0) {
          ozy_i2c_readpwr(I2C_ADC_OFS); // time to get another ADC overflow status
        }
      }

      if ((cmd & 0x07) || (mox)) {      // PTT DOT DASH or MOX - Transmit is active
// zzz                          if (totbytes % 819200L == 0)    // i2c access seems to significantly degrade EP6 continuity, so do it infrequently
        // Maybe we should double buffer EP6 or similar?
        switch (cmd & 0xF8) {
        case 0x08:             // forward powers
          if (totbytes % 819200L == 0)
            ozy_i2c_readpwr(I2C_PENNY_FWD);     // get the PA fwd power
          ep6_inbuffer[(i * 512) + 4] = (penny_fp >> 8) & 0xFF;
          ep6_inbuffer[(i * 512) + 5] = penny_fp & 0xFF;
          break;
        case 0x10:             // its 0x10 reverse power
          if (totbytes % 819200L == 0)
            ozy_i2c_readpwr(I2C_PENNY_REV);     // get the reverse power
// zzz                                  fprintf(stderr,"Rev=%d\n",penny_rp);
          ep6_inbuffer[(i * 512) + 4] = (penny_rp >> 8) & 0xFF;
          ep6_inbuffer[(i * 512) + 5] = penny_rp & 0xFF;
          break;
        default:
          break;
        }

        // end case
      } else {                  // PTT DOT DASH Inactive
        penny_fp = 0;
        penny_rp = 0;
        penny_alc = 0;
      }
    }
    // make 2 packets to send down eth to the PC as 2 seperate sends

    if (syncerr == 0) {
      for (j = 0; j < 2; j++) { // we need 2 lots of 1K buffers
        memcpy(&ep6_outbuffer[8], &ep6_inbuffer[USB_BUFSIZE * (j)], USB_BUFSIZE);       // fill the tail with 1k of USB Data
        ep6_outbuffer[7] = seqno & 0xff;
        ep6_outbuffer[6] = (seqno >> 8) & 0xff;
        ep6_outbuffer[5] = (seqno >> 16) & 0xff;
        ep6_outbuffer[4] = (seqno >> 24) & 0xff;
        seqno++;
        if (sendto(s, ep6_outbuffer, ETH_BUFSIZE, MSG_DONTWAIT, (struct sockaddr *)&address, length) < 0) {     // send 1K
          perror("ozyMet_ep6_read_thread: sendto socket failed for metis_send_data\n");
          exit(1);
        }
      }                         // for j
    } else                      // was a syncerr
    {
      syncerr = 0;
      badcount += bytes;
    }
  }                             // end while
}
