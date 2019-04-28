/**
* @file ozy.c
* @brief Ozy protocol implementation
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef __linux__
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/timeb.h>
#include <sys/stat.h> // for stat
#include <pthread.h>
#include <unistd.h>   // for readlink
#include <limits.h>   // for PATH_MAX
#include <errno.h>
#else
#include "pthread.h"
#endif
#include <assert.h>

#include "ozyio.h"
#include "util.h"
#include "ozy.h"

#define THREAD_STACK 32768

#define DEFAULT_OZY_BUFFERS 16
#define OZY_BUFFER_SIZE 512
#define OZY_HEADER_SIZE 8

static int ozy_buffers=DEFAULT_OZY_BUFFERS;

#define SYNC 0x7F

// ozy command and control
#define MOX_DISABLED    0x00
#define MOX_ENABLED     0x01

#define MIC_SOURCE_JANUS 0x00
#define MIC_SOURCE_PENELOPE 0x80
#define CONFIG_NONE     0x00
#define CONFIG_PENELOPE 0x20
#define CONFIG_MERCURY  0x40
#define CONFIG_BOTH     0x60
#define PENELOPE_122_88MHZ_SOURCE 0x00
#define MERCURY_122_88MHZ_SOURCE  0x10
#define ATLAS_10MHZ_SOURCE        0x00
#define PENELOPE_10MHZ_SOURCE     0x04
#define MERCURY_10MHZ_SOURCE      0x08
#define SPEED_48KHZ               0x00
#define SPEED_96KHZ               0x01
#define SPEED_192KHZ              0x02
#define SPEED_384KHZ              0x03

#define MODE_CLASS_E              0x01
#define MODE_OTHERS               0x00

#define ALEX_ATTENUATION_0DB      0x00
#define ALEX_ATTENUATION_10DB     0x01
#define ALEX_ATTENUATION_20DB     0x02
#define ALEX_ATTENUATION_30DB     0x03
#define LT2208_GAIN_OFF           0x00
#define LT2208_GAIN_ON            0x04
#define LT2208_DITHER_OFF         0x00
#define LT2208_DITHER_ON          0x08
#define LT2208_RANDOM_OFF         0x00
#define LT2208_RANDOM_ON          0x10

#define SIMPLEX                   0x00
#define DUPLEX                    0x04


static pthread_t ep6_ep2_io_thread_id;
static pthread_t ep4_io_thread_id;
static pthread_t playback_thread_id;

static int configure=6;
static int rx_frame=0;
static int tx_frame=0;
static int receivers=1;
static int current_receiver=0;

static int speed=1;
static int sample_rate=96000;
static int output_sample_increment=2;

static int timing=0;
static struct timeb rx_start_time;
	static struct timeb rx_end_time;
	static struct timeb tx_start_time;
	static struct timeb tx_end_time;
	static int rx_sample_count=0;
static int tx_sample_count=0;
static int j16=0;

static unsigned char control_in[5]={0x00,0x00,0x00,0x00,0x00};
/* zzz
static unsigned char control_out_metis[5]={
MOX_DISABLED,
  CONFIG_BOTH | MERCURY_122_88MHZ_SOURCE | MERCURY_10MHZ_SOURCE | MIC_SOURCE_PENELOPE | SPEED_96KHZ,
  MODE_OTHERS,
  ALEX_ATTENUATION_0DB | LT2208_GAIN_OFF | LT2208_DITHER_ON | LT2208_RANDOM_ON,
  SIMPLEX
};

static unsigned char control_out_hermes[5]={
  MOX_DISABLED,
  CONFIG_BOTH | MERCURY_122_88MHZ_SOURCE | MERCURY_10MHZ_SOURCE | MIC_SOURCE_PENELOPE | SPEED_96KHZ,
  MODE_OTHERS,
  ALEX_ATTENUATION_0DB | LT2208_GAIN_OFF | LT2208_DITHER_ON | LT2208_RANDOM_ON,
  DUPLEX // changed from SIMPLEX in order to avoid Hermes frequency lagging
};
*/
static unsigned char control_out_metis[5]={0x00,0xe4,0x00,0x04,0x0c};   
/* zzz metis is used */

static unsigned char control_out_hermes[5]={0x00,0xe4,0x00,0x24,0x0c};  

static unsigned char *control_out = control_out_metis;

/*
C0
0 0 0 1 0 0 1 x    


C1
0 0 0 0 0 0 0 0
|             |
+-------------+------------ Hermes/PennyLane Drive Level (0-255)1

1 Ignored by Penelope


C2
0 0 0 0 0 0 0 0
| | | | | | | |
| | | | | | | +------------ Hermes/Metis Penelope Mic boost (0 = 0dB, 1 = 20dB)
| | | | | | +-------------- Metis/Penelope or PennyLane Mic/Line-in (0 = mic, 1 = Line-in)
| | | | | +---------------- Hermes – Enable/disable Apollo filter (0 = disable, 1 = enable)
| | | | +------------------ Hermes – Enable/disable Apollo tuner (0 = disable, 1 = enable)
| | | +-------------------- Hermes – Apollo auto tune (0 = end, 1 = start)
| | +---------------------- Hermes – select filter board (0 = Alex, 1 = Apollo)
| +------------------------ Alex   - manual HPF/LPF filter select (0 = disable, 1 = enable)2
+-------------------------- VNA Mode (0 = off, 1 = on)

C3
0 0 0 0 0 0 0 0
| | | | | | | |
| | | | | | | +------------ Alex   -	select 13MHz  HPF (0 = disable, 1 = enable)2
| | | | | | +-------------- Alex   -	select 20MHz  HPF (0 = disable, 1 = enable)2
| | | | | +---------------- Alex   -	select 9.5MHz HPF (0 = disable, 1 = enable)2
| | | | +------------------ Alex   -	select 6.5MHz HPF (0 = disable, 1 = enable)2
| | | +-------------------- Alex   -	select 1.5MHz HPF (0 = disable, 1 = enable)2
| | +---------------------- Alex   -	Bypass all HPFs   (0 = disable, 1 = enable)2
| +------------------------ Alex   -	6M low noise amplifier (0 = disable, 1 = enable)2
+-------------------------- Disable Alex T/R relay (0 = enable, 1 = disable) 


C4
0 0 0 0 0 0 0 0
  | | | | | | |
  | | | | | | +------------ Alex   - 	select 30/20m LPF (0 = disable, 1 = enable) (2)
  | | | | | +-------------- Alex   - 	select 60/40m LPF (0 = disable, 1 = enable) (2)
  | | | | +---------------- Alex   - 	select 80m    LPF (0 = disable, 1 = enable) (2)
  | | | +------------------ Alex   - 	select 160m   LPF (0 = disable, 1 = enable) (2)
  | | +-------------------- Alex   - 	select 6m     LPF (0 = disable, 1 = enable) (2)
  | +---------------------- Alex   - 	select 12/10m LPF (0 = disable, 1 = enable) (2)
  +------------------------ Alex   - 	select 17/15m LPF (0 = disable, 1 = enable) (2)

(2) Only valid when Alex - manual HPF/LPF filter select is enabled
*/
#if 0
static unsigned char control_out_hermes_pow[5]={
	/* C0 */   0x12 | MOX_DISABLED,                         
	/* C1 */   0,                    //   Hermes/PennyLane Drive Level
	/* C2 */   0,   
	/* C3 */   0x20, // 0, zzz
	/* C4 */   0x80,  // 0,
};
#else
static unsigned char control_out_hermes_pow[5]={
	/* C0 */   0x12 | MOX_DISABLED,                         
	/* C1 */   0,                    //   Hermes/PennyLane Drive Level
	/* C2 */   0x40,   
	/* C3 */   0x40, // disable all HPF
	/* C4 */   0x40,  // 10m LPF
};
#endif
/*
C0
0 0 0 1 0 1 0 x   

C1
0 0 0 0 0 0 0 0
        | | | |
        | | | +------------ Rx1 pre-amp (0=OFF, 1= ON)
        | | +-------------- Rx2 pre-amp (0=OFF, 1= ON)
        | +---------------- Rx3 pre-amp (0=OFF, 1= ON)
        +------------------ Rx4 pre-amp (0=OFF, 1= ON)

C2
0 0 0 0 0 0 0 0
      | | | | |
      | | | | +------------ TLV320 Line-in Gain bit 0 (3) 
      | | | +-------------- TLV320 Line-in Gain bit 1 (3)
      | | +---------------- TLV320 Line-in Gain bit 2 (3)
      | +------------------ TLV320 Line-in Gain bit 3 (3)
      +-------------------- TLV320 Line-in Gain bit 4 (3)

(3) Sets TLV320 line_boost value when Metis or Hermes is used.

C3
0 0 0 0 0 0 0 0
        | | | |
        | | | +------------ Metis DB9 pin 1 Open Drain Output (0=OFF, 1= ON)
        | | +-------------- Metis DB9 pin 2 Open Drain Output (0=OFF, 1= ON)
        | +---------------- Metis DB9 pin 3 3.3v TTL Output (0=OFF, 1= ON)
        +------------------ Metis DB9 pin 4 3.3v TTL Output (0=OFF, 1= ON)

C4
0 0 0 0 0 0 0 0
      |       |
      +-------+------------ Hermes Input Attenuator (0 - 31dB) [4:0]
*/

#if 0
static unsigned char control_out_hermes_att[5]={
	/* C0 */   0x14 | MOX_DISABLED,
	/* C1 */   0x00,                 //   Rx preamps
	/* C2 */   0,                    //   TLV320 Line-in Gain
	/* C3 */   0,                    //   Metis DB9
	/* C4 */   0x20,  zzz               //   Hermes Input Attenuator: ACTIVE
};
#else
static unsigned char control_out_hermes_att[5]={
	/* C0 */   0x14,
	/* C1 */   0x41,                 //   Rx preamps
	/* C2 */   0x20,                    //   TLV320 Line-in Gain
	/* C3 */   0x00,                    //   Metis DB9
	/* C4 */   0x0F,                //   Hermes Input Attenuator: ACTIVE
};
#endif

static int mox=0;

static int ptt=0;
static int dot=0;
static int dash=0;
static int lt2208ADCOverflow=0;

static unsigned char ozy_firmware_version[9];
static int mercury_software_version=0;
static int penelope_software_version=0;
static int ozy_software_version=0;
static int hermes_software_version=0;

static int forwardPower=0;
static int alexForwardPower=0;
static int alexReversePower=0;
static int AIN3=0;
static int AIN4=0;
static int AIN6=0;
static int IO1=1; // 1 is inactive
static int IO2=1;
static int IO3=1;
static int IO4=1;


static int samples=0;

static float mic_gain=0.26F;
//static float mic_left_buffer[BUFFER_SIZE];
//static float mic_right_buffer[BUFFER_SIZE];

static char ozy_firmware[64] = {0};
static char ozy_fpga[64] = {0};

static unsigned char ozy_output_buffer[OZY_BUFFER_SIZE];
static int ozy_output_buffer_index=OZY_HEADER_SIZE;


static char filename[256];
static int record=0;
static int playback=0;
static int playback_sleep=0;
static FILE* recording;

int metis=0;
int hermes=0;


void  (* write_ozy_output_buffer)(void) = 0;
void ozy_i2c_readvars();
void ozy_prime();
void* ozy_ep6_ep2_io_thread(void* arg);
void* ozy_ep4_io_thread(void* arg);
void* playback_thread(void* arg);

#ifndef __linux__
#define bool int
bool init_hpsdr();
#endif

void ozy_set_fpga_image(const char *s) {
	strcpy (ozy_fpga, s);
}

void ozy_set_hex_image(const char *s) {
	strcpy (ozy_firmware, s);
}

void ozy_set_buffers(int buffers, int hermes) {
	ozy_buffers=buffers;
	write_ozy_output_buffer = write_ozy_output_buffer_metis;
	fprintf(stderr,"metis output\n");
}

int create_ozy_thread() {
	int rc;

		ozy_init();
		ozy_i2c_readvars();
		ozy_close();
		sleep(1);
		ozy_open();		
	return 0;
}



/*
J16 pins
=====
17 out1   D    bit 0
18 out2   C    bit 1
19 out3   B    bit 2
20 out4   A    bit 3

e.g D switches 15m filter.

Band-----------------------------J16 --------------BPF/LPF DB25 Pins ---
1 ---->    39.85 - 64.4 MHz   BC      (6m)              1 2              

2 ---->    23.2 - 39.85 MHz  ABC     (12m/10m)        4 2 1

3 ---->    19.6 - 23.2 MHz      D    (15m)                3

4 ---->    16.2 - 19.6 MHz   A  D    (17m)              4 3

5 ---->    12.1 - 16.2 MHz    B D    (20m)              1 3

6 ---->    8.7 - 12.1 MHz    AB D    (30m)            4 1 3

7 ---->    6.2 - 8.7 MHz,      CD    (40m)              2 3

8 ---->    4.665 - 6.2 MHz   A CD    (60m)            4 2 3

9 ---->    2.75 - 4.665 MHz   BCD    (80m)            1 2 3

10 -- >    1.70 - 2.75 MHz   ABCD    (160m)         4 1 2 3


4. OC6 User open-collector output 7 (23)
5. OC5 User open-collector output 6 (22)
6. OC4 User open-collector output 5 (21)
7. OC3 User open-collector output 4 (20)
8. OC2 User open-collector output 3 (19)
9. OC1 User open-collector output 2 (18).
10. OC0 User open-collector output 1 (17)

*/


typedef struct _filter_j16 {
	long f1;
	long f2;
	unsigned char j16; 
} filter_j16;

filter_j16 fltj16_tbl [] =
{
	{ 1700000,  2750000,   0x0f },
	{ 2750000,  4665000,   0x07 },
	{ 4665000,  6200000,   0x0b },
	{ 6200000,  8700000,   0x03 },
	{ 8700000,  12100000,  0x0d },
	{ 12100000, 16200000,  0x05 },
	{ 16200000, 19600000,  0x09 },
	{ 19600000, 23200000,  0x01 },	
	{ 23200000, 39850000,  0x0e },
	{ 39850000, 64400000,  0x06 },
};
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

int get_j16_from_freq (long f)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(fltj16_tbl); ++i) {
		if ( (f >= fltj16_tbl[i].f1)  &&  (f <= fltj16_tbl[i].f2) )
			return fltj16_tbl[i].j16;
	}
	return -1;
}

typedef struct _filter_rx {
	long f1;
	long f2;
	unsigned char rxf; 
} filter_rx;

filter_rx fltrx_tbl [] =
{
	{ 1,  1500000,   0x20 },   /* bypass all */
	{ 1500001,  6500000,   0x10 }, /* 1.5MHz HPF */
	{ 6500001,  9500000,   0x08 }, /* 6.5MHz HPF */
	{ 9500001,  13000000,  0x04 }, /* 9.5MHz HPF */
	{ 13000001, 20000000,  0x01 }, /* 13MHz HPF */
	{ 20000001, 50000000,  0x02 }, /* 20MHZ HPF */
	{ 50000001, 64400000,  0x42}, /* 20MHz HPF with pre-amp on */
};
int get_rx_from_freq (long f)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(fltrx_tbl); ++i) {
		if ( (f >= fltrx_tbl[i].f1)  &&  (f <= fltrx_tbl[i].f2) )
			return fltrx_tbl[i].rxf;
	}
	return -1;
}

void ozy_set_sample_rate(int r) {
	switch(r) {
		case 48000:
		sample_rate=r;
		speed=0;
		output_sample_increment = 1;
		break;
		case 96000:
		sample_rate=r;
		speed=1;
		output_sample_increment = 2;
		break;
		case 192000:
		sample_rate=r;
		speed=2;
		output_sample_increment = 4;
		break;
		case 384000:
		sample_rate=r;
		speed=3;
		output_sample_increment = 8;
		break;
		default:
		fprintf(stderr,"Invalid sample rate (48000,96000,192000,384000)!\n");
		exit(1);
		break;
	}
	control_out_metis[1] &= 0xfc;
	control_out_metis[1] |= speed;
	control_out_hermes[1] &= 0xfc;
	control_out_hermes[1] |= speed;

	//playback_sleep=(int)((1000000.0/380.0));
	playback_sleep=3090;
	fprintf(stderr,"************** receivers=%d sample_rate=%d playback_sleep=%d\n",receivers,sample_rate,playback_sleep);

}

int ozy_set_playback_sleep(int sleep) {
	playback_sleep=sleep;
	return 0;
}

int ozy_get_sample_rate() {
	return sample_rate;
}


static int file_exists (const char * fileName)
{
	struct stat buf;
	int i = stat ( fileName, &buf );
	return ( i == 0 ) ? 1 : 0 ;
}

#ifdef __linux__
int filePath (char *sOut, const char *sIn) {
	int rc = 0;

	if ((rc = file_exists (sIn))) {
		strcpy (sOut, sIn); 
		rc = 1;
	} else {
		char cwd[PATH_MAX];
		char s[PATH_MAX];
		char xPath [PATH_MAX] = {0};
		char *p;

		int  rc = readlink ("/proc/self/exe", xPath, sizeof(xPath));

		// try to detect the directory from which the executable has been loaded
		if (rc >= 0) {

			if ( (p = strrchr (xPath, '/')) ) *(p+1) = '\0';
			fprintf (stderr, "%d, Path of executable: [%s]\n", rc, xPath);

			strcpy (s, xPath); strcat (s, sIn);

			if ((rc = file_exists (s))) {
				// found in the same dir of executable
				fprintf (stderr, "File: [%s]\n", s);
				strcpy(sOut, s);
			} else { 
				if (getcwd(cwd, sizeof(cwd)) != NULL) {
					fprintf(stdout, "Current working dir: %s\n", cwd);

					strcpy (s, cwd); strcat (s, "/"); strcat (s, sIn);
					if ((rc = file_exists (s))) {
						fprintf (stderr, "File: [%s]\n", s);
						strcpy(sOut, s);
					}
				}
			}
		} else {
			fprintf (stderr, "%d: %s\n", errno, strerror(errno));
		}
	}
	return rc;
}
#endif


int ozy_init(void) {
	int rc;

	// On Windows, the following is replaced by init_hpsdr() in OzyInit.c
	#ifdef __linux__

	if (strlen(ozy_firmware) == 0) filePath (ozy_firmware,"ozyfw-sdr1k.hex");
	if (strlen(ozy_fpga) == 0)     filePath (ozy_fpga,"Ozy_Janus.rbf");

	// open ozy
	rc = ozy_open();
	if (rc != 0) {
		fprintf(stderr,"Cannot locate Ozy\n");
		exit(1);
	}
	// load Ozy FW
	ozy_reset_cpu(1);
	ozy_load_firmware(ozy_firmware);
	ozy_reset_cpu(0);
	ozy_close();
	sleep(4);
	ozy_open();
	ozy_set_led(1,1);
	ozy_load_fpga(ozy_fpga);
	ozy_set_led(1,0);
	ozy_close();
	ozy_open();
	rc=ozy_get_firmware_string(ozy_firmware_version,8);
	fprintf(stderr,"Ozy FX2 version: %s\n",ozy_firmware_version);
	#else
	strcpy(ozy_firmware,"ozyfw-sdr1k.hex");
	strcpy(ozy_fpga,"Ozy_Janus.rbf");
	#endif

	return rc;
}


void write_ozy_output_buffer_metis() {
	int bytes;
	static int metis_send_status = 0;

	unsigned char rxfilter = 0, txfilter = 0;

		bytes=ozy_write(0x02,ozy_output_buffer,OZY_BUFFER_SIZE);
		if(bytes!=OZY_BUFFER_SIZE) {
			perror("OzyBulkWrite ozy failed");
		}	
return;


}

