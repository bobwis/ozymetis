/**
* @file server.c
* @brief HPSDR server application
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __linux__
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <getopt.h>
#include <unistd.h>
#else // Windows
#include "pthread.h"
#include <winsock.h>
#include "getopt.h"
#endif

#include "listener.h"
#include "ozy.h"

static struct option long_options[] = {

    {"receivers",required_argument, 0, 0},
    {"samplerate",required_argument, 0, 1},
    {"dither",required_argument, 0, 2},
    {"random",required_argument, 0, 3},
    {"preamp",required_argument, 0, 4},
    {"10mhzsource",required_argument, 0, 5},
    {"122.88mhzsource",required_argument, 0, 6},
    {"micsource",required_argument, 0, 7},
    {"class",required_argument, 0, 8},
    {"timing",no_argument, 0, 9},
    {"record",required_argument,0,10},
    {"playback",required_argument,0,11},
    {"sleep",required_argument,0,12},
    {"metis",no_argument,0,13},
    {"interface",required_argument,0,14},
    {"metisip",required_argument,0,15},
    {"fpga",required_argument,0,16},
    {"ozyhex",required_argument,0,17},
    {"hermes",required_argument,0,18},
    {"j16",no_argument,0,19},
    {0,0,0,0},

};
static char* short_options="";
static int option_index;

static int metis=0;
static int hermes=0;
static char* interface="eth0";
static char* metisip="0.0.0.0";

void process_args(int argc,char* argv[]);

int main(int argc,char* argv[]) {

    process_args(argc,argv);
	ozy_set_buffers(2,0);
	create_ozy_thread();	
sleep(5);	//wait for FPGA top load
    create_listener_thread();

	while(1) {
	    char ch;
	    while((ch = getc(stdin)) != EOF) 
	      if (ch == 'q') break;
    }
}

void process_args(int argc,char* argv[]) {
    int i;

    while((i=getopt_long(argc,argv,short_options,long_options,&option_index))!=EOF) {

        switch(i) {
            	default:
                break;
               
        }
    }
}

