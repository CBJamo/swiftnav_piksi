/***************************************************************************//**
* \file piksi.h
*
* \brief Standalone C Driver for the Swift Navigation Piksi GPS (header)
* \author Scott K Logan
* \author Caleb Jamison
* \date Febuary 23, 2014
*
* API for the standalone C driver
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

#ifndef _piksi_h
#define _piksi_h

#ifdef __cplusplus
extern "C" {
#endif

#include <libswiftnav/sbp.h>
#include <libswiftnav/sbp_messages.h>

enum piksi_error
{
	PIKSI_SUCCESS = 0,
	PIKSI_ERROR_IO = -1,
	PIKSI_ERROR_NO_DEVICE = -4,
	PIKSI_ERROR_NO_MEM = -11,
	PIKSI_ERROR_OTHER = -13,
	PIKSI_ERROR_TIMEOUT = -14
};

int piksi_open( const char *port );
void piksi_close( const int piksid );

s8 piksi_reset( const int piksid );
s8 piksi_cw_start( const int piksid );

u8 piksi_spin( const int piksid );

sbp_gps_time_t piksi_get_time( const int piksid );
sbp_dops_t piksi_get_dops( const int piksid );
sbp_pos_ecef_t piksi_get_pos_ecef( const int piksid );
sbp_pos_llh_t piksi_get_pos_llh( const int piksid );
sbp_baseline_ecef_t piksi_get_baseline_ecef( const int piksid );
sbp_baseline_ned_t piksi_get_baseline_ned( const int piksid );
sbp_vel_ecef_t piksi_get_vel_ecef( const int piksid );
sbp_vel_ned_t piksi_get_vel_ned( const int piksid );

#ifdef __cplusplus
}
#endif

#endif /* _piksi_h */
