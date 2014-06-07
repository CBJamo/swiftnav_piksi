/***************************************************************************//**
* \file piksi.c
*
* \brief Standalone C Driver for the Swift-Nav Piksi
* \author Scott K Logan
* \author Caleb Jamison
* \date Febuary 23, 2014
*
* This is a standolone C driver for the Swift Navigation Piksi GPS.
*
* \section license License (BSD-3)
* Copyright (c) 2013, Scott K Logan\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "swiftnav_piksi/piksi.h"
#include "swiftnav_piksi/piksi_priv.h"

#include <libswiftnav/sbp.h>
#include <libswiftnav/sbp_messages.h>

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

/*!
 * \brief Maximum number of simultaneously open device handles.
 */
#define MAX_HANDLES 256

#define NO_FLUSH_BUFFER 1

struct piksi_priv
{
	int fd;
	int baud;
	char *port;
	sbp_state_t state;
	sbp_msg_callbacks_node_t time_callback_node;
	sbp_msg_callbacks_node_t dop_callback_node;
	sbp_msg_callbacks_node_t pos_ecef_callback_node;
	sbp_msg_callbacks_node_t pos_llh_callback_node;
	sbp_msg_callbacks_node_t baseline_ecef_callback_node;
	sbp_msg_callbacks_node_t baseline_ned_callback_node;
	sbp_msg_callbacks_node_t vel_ecef_callback_node;
	sbp_msg_callbacks_node_t vel_ned_callback_node;
	sbp_gps_time_t time;
	sbp_dops_t dops;
	sbp_pos_ecef_t pos_ecef;
	sbp_pos_llh_t pos_llh;
	sbp_baseline_ecef_t baseline_ecef;
	sbp_baseline_ned_t baseline_ned;
	sbp_vel_ecef_t vel_ecef;
	sbp_vel_ned_t vel_ned;
};

/*!
 * \brief List of communication handles.
 */
static struct piksi_priv * piksi_list[MAX_HANDLES] = { NULL };


/*!
 * \brief Grabs the next available device handle slot.
 *
 * Iterates through the ::MAX_HANDLES slots for the lowest available index.
 *
 * \returns Open slot index between 0 and ::MAX_HANDLES
 * \retval -1 No available slots
 */
static int next_available_handle( )
{
	unsigned short int i;
	for( i = 0; i < MAX_HANDLES; i++ )
	{
		if( !piksi_list[i] )
			return i;
	}
	return -1;
}

static int baud2term( int baud )
{
	switch( baud )
	{
	case 1200:
		return B1200;
		break;
	case 2400:
		return B2400;
		break;
	case 4800:
		return B4800;
		break;
	case 9600:
		return B9600;
		break;
	case 19200:
		return B19200;
		break;
	case 38400:
		return B38400;
		break;
	case 57600:
		return B57600;
		break;
	case 115200:
		return B115200;
		break;
	case 230400:
		return B230400;
		break;
	case 460800:
		return B460800;
		break;
	case 921600:
		return B921600;
		break;
	case 1000000:
		return B1000000;
		break;
	default:
		return B0;
		break;
	}
}

u32 send_cmd( u8 *data, u32 num_bytes, void* context )
{
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	return write( piksid->fd, data, num_bytes );
}

u32 read_data( u8 *data, u32 num_bytes, void* context )
{
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	u32 bytes_recv = 0;
	u32 bytes_recvd = 0;
	while( num_bytes )
	{
		bytes_recv = read( piksid->fd, data, num_bytes );
		if( bytes_recv <= 0 )
			return 0;
		num_bytes -= bytes_recv;
		bytes_recvd += bytes_recv;
	}
	return bytes_recvd;
}


int piksi_open( const char *port )
{
	/* Step 1: Make sure the device opens OK */
	int fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 )
		return PIKSI_ERROR_NO_DEVICE;

	fcntl( fd, F_SETFL, 0 );

	struct termios options;
	cfmakeraw( &options );
	if( cfsetispeed( &options, B1000000 ) < 0 )
	{
		close( fd );
		return PIKSI_ERROR_IO;
	}
	options.c_cflag &= ~HUPCL;
	options.c_lflag &= ~ICANON;
	options.c_cc[VTIME] = 2;
	options.c_cc[VMIN] = 0;
	if( tcsetattr( fd, TCSANOW, &options ) < 0 )
	{
		close( fd );
		return PIKSI_ERROR_IO;
	}


	/* Step 2: Allocate a private struct */
	int mydev = next_available_handle( );
	if( mydev < 0 )
		goto close_mem_error;

	piksi_list[mydev] = malloc( sizeof( struct piksi_priv ) );
	if( !piksi_list[mydev] )
		goto null_struct;

	memset( piksi_list[mydev], 0, sizeof( struct piksi_priv ) );

	piksi_list[mydev]->port = malloc( strlen( port ) + 1 );
	if( !piksi_list[mydev]->port )
		goto free_struct;

	memcpy( piksi_list[mydev]->port, port, strlen( port ) + 1 );
	piksi_list[mydev]->fd = fd;
	piksi_list[mydev]->baud = baud2term( 1000000 );

	/* Step 3: Setup SBP state and regiser callbacks */

	sbp_state_init(&piksi_list[mydev]->state);
	sbp_state_set_io_context(&piksi_list[mydev]->state, piksi_list[mydev]);

	sbp_register_callback(&piksi_list[mydev]->state, SBP_GPS_TIME, &time_callback, piksi_list[mydev], &piksi_list[mydev]->time_callback_node);
	sbp_register_callback(&piksi_list[mydev]->state, SBP_DOPS, &dop_callback, piksi_list[mydev], &piksi_list[mydev]->dop_callback_node);
	sbp_register_callback(&piksi_list[mydev]->state, SBP_POS_ECEF, &pos_ecef_callback, piksi_list[mydev], &piksi_list[mydev]->pos_ecef_callback_node);
	sbp_register_callback(&piksi_list[mydev]->state, SBP_POS_LLH, &pos_llh_callback, piksi_list[mydev], &piksi_list[mydev]->pos_llh_callback_node);
	sbp_register_callback(&piksi_list[mydev]->state, SBP_BASELINE_ECEF, &baseline_ecef_callback, piksi_list[mydev], &piksi_list[mydev]->baseline_ecef_callback_node);
	sbp_register_callback(&piksi_list[mydev]->state, SBP_BASELINE_NED, &baseline_ned_callback, piksi_list[mydev], &piksi_list[mydev]->baseline_ned_callback_node);
	sbp_register_callback(&piksi_list[mydev]->state, SBP_VEL_ECEF, &vel_ecef_callback, piksi_list[mydev], &piksi_list[mydev]->vel_ecef_callback_node);
	sbp_register_callback(&piksi_list[mydev]->state, SBP_VEL_NED, &vel_ned_callback, piksi_list[mydev], &piksi_list[mydev]->vel_ned_callback_node);

	return mydev;

	/* Free gotos */

	free_struct:
		free( piksi_list[mydev] );
	null_struct:	
		piksi_list[mydev] = NULL;
	close_mem_error:
		close( fd );
		return PIKSI_ERROR_NO_MEM;
}

void piksi_close( const int piksid )
{
	if( piksid < 0 || piksid > MAX_HANDLES || !piksi_list[piksid] )
		return;

	close( piksi_list[piksid]->fd );

	free( piksi_list[piksid]->port );
	free( piksi_list[piksid] );
	piksi_list[piksid] = NULL;

	return;
}

void time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In time cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->time = *(sbp_gps_time_t*) msg;
	return;
}

void dop_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In dop cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->dops = *(sbp_dops_t*) msg;
	return;
}

void pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In pos_ecef cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->pos_ecef = *(sbp_pos_ecef_t*) msg;
	return;
}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In pos_llh cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->pos_llh = *(sbp_pos_llh_t*) msg;
	printf("lat_cb: %d\n", piksid->pos_llh.lat);
	printf("lon_cb: %d\n", piksid->pos_llh.lon);
	return;
}

void baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In baseline_ecef cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->baseline_ecef = *(sbp_baseline_ecef_t*) msg;
	return;
}

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In baseline_ned cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->baseline_ned = *(sbp_baseline_ned_t*) msg;
	return;
}

void vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In vel_ecef cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->vel_ecef = *(sbp_vel_ecef_t*) msg;
	return;
}

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	printf("In vel_ned cb\n");
	struct piksi_priv* piksid = ( struct piksi_priv* ) context;
	piksid->vel_ned = *(sbp_vel_ned_t*) msg;
}

u8 piksi_spin( const int piksid )
{
	return sbp_process( &piksi_list[piksid]->state, &read_data );
}

s8 piksi_reset( const int piksid )
{
	return sbp_send_message( &piksi_list[piksid]->state, 0xB2, 0, 0, 0, &send_cmd );
}

s8 piksi_cw_start( const int piksid )
{
	return sbp_send_message( &piksi_list[piksid]->state, 0xC1, 0, 0, 0, &send_cmd );
}

sbp_gps_time_t piksi_get_time( const int piksid )
{
	printf("Getting time: %d\n", piksi_list[piksid]->time.tow);
	return piksi_list[piksid]->time;
}

sbp_dops_t piksi_get_dops( const int piksid )
{
	return piksi_list[piksid]->dops;
}

sbp_pos_ecef_t piksi_get_pos_ecef( const int piksid )
{
	return piksi_list[piksid]->pos_ecef;
}

sbp_pos_llh_t piksi_get_pos_llh( const int piksid )
{
	printf("Getting lat: %d\n", piksi_list[piksid]->pos_llh.lat);
	printf("Getting lon: %d\n", piksi_list[piksid]->pos_llh.lon);
	return piksi_list[piksid]->pos_llh;
}

sbp_baseline_ecef_t piksi_get_baseline_ecef( const int piksid )
{
	return piksi_list[piksid]->baseline_ecef;
}

sbp_baseline_ned_t piksi_get_baseline_ned( const int piksid )
{
	return piksi_list[piksid]->baseline_ned;
}

sbp_vel_ecef_t piksi_get_vel_ecef( const int piksid )
{
	return piksi_list[piksid]->vel_ecef;
}

sbp_vel_ned_t piksi_get_vel_ned( const int piksid )
{
	return piksi_list[piksid]->vel_ned;
}

int main()
{
	printf("start\n");
	int hand;
	s8 sbpp;

	if( ( hand = piksi_open( "/dev/ttyUSB0" ) ) < 0 )
	{
		printf("piksi connection error: %d\n", hand);
		return -1;
	}

	printf("Piksi open handle#:%d\n", hand);

	while(true)
	{
		sbpp = piksi_spin(hand);

		piksi_get_pos_llh(hand);
	}

	return 0;
}
