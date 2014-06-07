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

#ifndef _piksi_priv_h
#define _piksi_priv_h

#ifdef __cplusplus
extern "C" {
#endif

#include <libswiftnav/sbp.h>
#include <libswiftnav/sbp_messages.h>

void time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void dop_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);

#ifdef __cplusplus
}
#endif

#endif /* _piksi_priv_h */
