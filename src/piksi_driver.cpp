#include "swiftnav_piksi/piksi_driver.h"
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
		spin_rate( 50 ),
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

		fix_pub = nh.advertise<sensor_msgs::NavSatFix>( "gps/fix", 1 );
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
		int old_piksid = piksid;
		if( piksid < 0 )
		{
			cmd_lock.unlock( );
			return;
		}
		piksid = -1;
		piksi_close( old_piksid );
		if( fix_pub )
			fix_pub.shutdown( );
		if( time_pub )
			time_pub.shutdown( );
	}

	void PIKSI::spin( )
	{
		while( ros::ok( ) )
		{
			boost::this_thread::interruption_point( );
			spinOnce( );
			diag.update( );
			spin_rate.sleep( );
		}
	}

	void PIKSI::spinOnce( )
	{
		cmd_lock.lock( );
		if( piksid < 0 && !PIKSIOpenNoLock( ) )
			return;

		int ret;

		float cov[9];

		sbp_gps_time_t time = piksi_get_time( piksid );
		std::cout << "PIKSID " << piksid << "\n";

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		time_pub.publish( time_msg );

		/*
		 * Fix Data
		 */

		sbp_pos_llh_t llh = piksi_get_pos_llh( piksid );

		cmd_lock.unlock( );

		sensor_msgs::NavSatFixPtr fix_msg( new sensor_msgs::NavSatFix );

		fix_msg->header.frame_id = frame_id;
		fix_msg->header.stamp = ros::Time::now( );

		fix_msg->status.status = -1; // STATUS_NO_FIX;
		fix_msg->status.service = 1; // SERVICE_GPS;

		fix_msg->latitude = llh.lat;
		fix_msg->longitude = llh.lon;
		fix_msg->altitude = llh.height;

		for( int i = 0; i < 9; i++ )
			fix_msg->position_covariance[i] = cov[i];

		fix_msg->position_covariance_type = 0; //COVARIANCE_TYPE_UNKNOWN;

		fix_pub.publish( fix_msg );

		diag_pub_freq.tick( );
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
