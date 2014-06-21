#include "swiftnav_piksi/piksi_driver.h"

#include <libswiftnav/sbp.h>
#include <libswiftnav/sbp_messages.h>

#include <iomanip>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <ros/time.h>
#include <tf/tf.h>

namespace swiftnav_piksi
{	
	PIKSI::PIKSI( const ros::NodeHandle &_nh,
		const ros::NodeHandle &_nh_priv,
		const std::string _port ) :
		nh( _nh ),
		nh_priv( _nh_priv ),
		port( _port ),
		frame_id( "gps" ),
		piksid( -1 ),
		min_update_rate( 20.0 ),
		max_update_rate( 80.0 ),
		diag_pub_freq( diagnostic_updater::FrequencyStatusParam( &min_update_rate, &max_update_rate, 0.1, 10 ) ),
		io_failure_count( 0 ),
		open_failure_count( 0 ),
		spin_rate( 2000 ),
		spin_thread( &PIKSI::spin, this )
	{
		cmd_lock.unlock( );
		diag.setHardwareID( "Swift Navigation Piksi (not connected)" );
		diag.add( "Swift Navigation Piksi Status", this, &PIKSI::DiagCB );
		diag.add( diag_pub_freq );

		nh_priv.param( "frame_id", frame_id, (std::string)"gps" );
	}

	PIKSI::~PIKSI( )
	{
		spin_thread.interrupt( );
		PIKSIClose( );
	}

	bool PIKSI::PIKSIOpen( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		return PIKSIOpenNoLock( );
	}

	bool PIKSI::PIKSIOpenNoLock( )
	{
		if( piksid >= 0 )
			return true;

		piksid = piksi_open( port.c_str( ) );

		if( piksid < 0 )
		{
			open_failure_count++;
			return false;
		}

		diag.setHardwareIDf( "Swift Navigation PIKSI on %s", port.c_str( ) );

		sbp_state_init(&state);
		sbp_state_set_io_context(&state, &piksid);

		sbp_register_callback(&state, SBP_HEARTBEAT, &heartbeatCallback, (void*) this, &heartbeat_callback_node);
		sbp_register_callback(&state, SBP_GPS_TIME, &timeCallback, (void*) this, &time_callback_node);
//		sbp_register_callback(&state, SBP_POS_ECEF, &pos_ecefCallback, (void*) this, &pos_ecef_callback_node);
		sbp_register_callback(&state, SBP_POS_LLH, &pos_llhCallback, (void*) this, &pos_llh_callback_node);
//		sbp_register_callback(&state, SBP_BASELINE_ECEF, &baseline_ecefCallback, (void*) this, &baseline_ecef_callback_node);
//		sbp_register_callback(&state, SBP_BASELINE_NED, &baseline_nedCallback, (void*) this, &baseline_ned_callback_node);
//		sbp_register_callback(&state, SBP_VEL_ECEF, &vel_ecefCallback, (void*) this, &vel_ecef_callback_node);
//		sbp_register_callback(&state, SBP_VEL_NED, &vel_nedCallback, (void*) this, &vel_ned_callback_node);

		llh_pub = nh.advertise<sensor_msgs::NavSatFix>( "gps/fix", 1 );
		time_pub = nh.advertise<sensor_msgs::TimeReference>( "gps/time", 1 );

		return true;
	}

	void PIKSI::PIKSIClose( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		PIKSICloseNoLock( );
	}

	void PIKSI::PIKSICloseNoLock( )
	{
		int8_t old_piksid = piksid;
		if( piksid < 0 )
		{
			return;
		}
		piksid = -1;
		piksi_close( old_piksid );
		if( llh_pub )
			llh_pub.shutdown( );
		if( time_pub )
			time_pub.shutdown( );
	}

	void heartbeatCallback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}
		
		sbp_heartbeat_t hb = *(sbp_heartbeat_t*) msg;

		class PIKSI *driver = (class PIKSI*) context;

		if (hb.flags & 1)
			std::cout << "an error has occured in a heartbeat message" << std::endl;
			
		return;
	}

	void timeCallback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_gps_time_t time = *(sbp_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
		driver->diag_pub_freq.tick( );

		return;
	}

	void pos_llhCallback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_pos_llh_t llh = *(sbp_pos_llh_t*) msg;

		sensor_msgs::NavSatFixPtr llh_msg( new sensor_msgs::NavSatFix );

		llh_msg->header.frame_id = driver->frame_id;
		llh_msg->header.stamp = ros::Time::now( );

		llh_msg->status.status = 0;
		llh_msg->status.service = 1;

		llh_msg->latitude = llh.lat;
		llh_msg->longitude = llh.lon;
		llh_msg->altitude = llh.height;

		driver->llh_pub.publish( llh_msg );
		driver->diag_pub_freq.tick( );

		return;
	}

/*	void baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{

		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_gps_time_t time = *(sbp_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
		driver->diag_pub_freq.tick( );

		return;
	}

	void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{

		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_gps_time_t time = *(sbp_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
		driver->diag_pub_freq.tick( );

		return;
	}

	void vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_gps_time_t time = *(sbp_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
		driver->diag_pub_freq.tick( );

		return;
	}

	void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_gps_time_t time = *(sbp_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
		driver->diag_pub_freq.tick( );

		return;
	}
*/
	void PIKSI::spin( )
	{
		while( ros::ok( ) )
		{
			boost::this_thread::interruption_point( );
			PIKSI::spinOnce( );
			diag.update( );
			spin_rate.sleep( );
		}
	}

	void PIKSI::spinOnce( )
	{
		int ret;

		cmd_lock.lock( );
		if( piksid < 0 && !PIKSIOpenNoLock( ) )
		{
			cmd_lock.unlock( );
			return;
		}

		ret = sbp_process( &state, &read_data );

		cmd_lock.unlock( );

	}

	void PIKSI::DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		if( piksid < 0 && !PIKSIOpenNoLock( ) )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected" );
			return;
		}

		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "PIKSI status OK" );

		int ret;

		static unsigned int last_io_failure_count = io_failure_count;
		if( io_failure_count > last_io_failure_count )
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "I/O Failure Count Increase" );
		stat.add( "io_failure_count", io_failure_count );
		last_io_failure_count = io_failure_count;

		static unsigned int last_open_failure_count = open_failure_count;
		if( open_failure_count > last_open_failure_count )
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "Open Failure Count Increase" );
		stat.add( "open_failure_count", open_failure_count );
		last_open_failure_count = open_failure_count;
	}

}
