/**
 * @file /rplidar_extras/src/rplidar_nodelet.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "pluginlib/class_list_macros.h"

#include "rplidar_extras/rplidar_nodelet.hpp"


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)


namespace rplidar_extras {

  void RPlidarNodelet::onInit()
  {
    drv = NULL;
    serial_baudrate = 115200;
    inverted = false;
    angle_compensate = true;

    initialised = false;
    res = 0;

    bad_health_counter = 0;
    result_fail_counter = 0;
    result_timeout_counter = 0;

    starting_time = ros::Time::now();

    // Always use rate_error_deques[<0, 1, 2>] to push_back elements
    // due to weird referencing issues
    rate_error_deques.push_back(bad_health_deque);        // 0
    rate_error_deques.push_back(result_fail_deque);       // 1
    rate_error_deques.push_back(result_timeout_deque);    // 2

    ros::NodeHandle &nh = this->getNodeHandle();
    ros::NodeHandle &nh_private = this->getPrivateNodeHandle();
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, "false");
    nh_private.param<bool>("angle_compensate", angle_compensate, "true");
    nh_private.param<int>("diag_time_window", diag_time_window, 30);
    diag_time_window_seconds = ros::Duration(diag_time_window * 60); // for seconds

    res = RPlidarNodelet::init_driver(serial_port, serial_baudrate);
    if (res < 0)
    {
      NODELET_ERROR_STREAM("Failed to initialise driver. Will retry in the spin...");
      // if not initialised here, will retry in devicePoll()
    }
    else
    {
      initialised = true;
    }

    stop_motor_service = nh_private.advertiseService("stop_motor", &RPlidarNodelet::stop_motor, this);
    start_motor_service = nh_private.advertiseService("start_motor", &RPlidarNodelet::start_motor, this);
    reset_device_service = nh_private.advertiseService("reset_device", &RPlidarNodelet::reset_device, this);
    reset_scan_service = nh_private.advertiseService("reset_scan", &RPlidarNodelet::reset_scan, this);

    scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

    updater.setHardwareID("none");
    updater.add("RPLidar Device Status", this, &RPlidarNodelet::device_diagnostics);

    device_thread_ = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&RPlidarNodelet::devicePoll, this)));

    // thread that clears older error timestamps in deques every 5.0 seconds
    diagnostics_rate_thread_ = boost::shared_ptr< boost::thread >
      (new boost::thread(boost::bind(&RPlidarNodelet::diagnosticsRatePoll, this)));
  }

  void RPlidarNodelet::device_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (!initialised)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "RPLidar not initialised.");
    }
    else
    {
      if (op_result == RESULT_OK)
      {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "RPLidar in good health and working.");
      }
      else if (op_result == RESULT_OPERATION_FAIL)
      {
        // this is the case of non-360 degree scan
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "RPLidar scan result fail!(non-360 scan result)");
      }
      else if (op_result == RESULT_OPERATION_TIMEOUT)
      {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "RPLidar scan result timeout!");
      }
      else
      {
        // can publish op_result(byte) here
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "RPLidar result status undefined!");
      }
    }

    elapsed_time = ros::Time::now() - starting_time;
    // output the counter and frequency of error with respect to elapsed time
    stat.addf("elapsed_time", "%.3fs", elapsed_time.toSec());
    stat.add("bad_health_counter", bad_health_counter);
    //stat.addf("bad_health_rate", "%d errors occured in last %d minutes", bad_health_deque.size(), diag_time_window);
    stat.addf("bad_health_rate", "%d errors occured in last %d minutes", rate_error_deques[0].size(), diag_time_window);
    stat.add("result_fail_counter", result_fail_counter);
    //stat.addf("result_fail_rate", "%d errors occured in last %d minutes", result_fail_deque.size(), diag_time_window);
    stat.addf("result_fail_rate", "%d errors occured in last %d minutes", rate_error_deques[1].size(), diag_time_window);
    stat.add("result_timeout_counter", result_timeout_counter);
    //stat.addf("result_timeout_rate", "%d errors occured in last %d minutes", result_timeout_deque.size(), diag_time_window);
    stat.addf("result_timeout_rate", "%d errors occured in last %d minutes", rate_error_deques[2].size(), diag_time_window);

  }

  void RPlidarNodelet::diagnosticsRatePoll()
  {
    while(ros::ok())
    {
      // empty deque of older than time window errors
      for (  std::vector< std::deque<ros::Time> >::size_type i = 0; i != rate_error_deques.size(); i++ )
      // loop through all counter rates
      {
        for ( std::deque<ros::Time>::iterator it = rate_error_deques[i].begin(); it != rate_error_deques[i].end(); it++ )
        // loop through each time stamp of error and erase out of window errors
        {
          if ( (ros::Time::now() - *it) > diag_time_window_seconds )
          {
            // remove element
            rate_error_deques[i].erase(it);
            if (it == rate_error_deques[i].end()) break;
          }
          else
          {
            break;
          }
        }
      }
      ros::Duration(5.0).sleep(); //every 5 seconds
    }
  }

  void RPlidarNodelet::devicePoll()
  {
    while(ros::ok())
    {

      if (initialised)
      {
        boost::mutex::scoped_lock lock(mutex_);
        this->read_scan(); // publish inside this
      } // release mutex lock
      else
      {
        if (!drv)
        {
          NODELET_INFO("RPLidar: no driver initialised, trying to re-initialise it again.");
          res = init_driver(serial_port, serial_baudrate);
          if (res < 0)
          {
            NODELET_ERROR_STREAM("RPLidar: failed to re-initialise driver, will keep trying.");
            ros::Duration(1.0).sleep();
          }
          else
          {
            initialised = true;
          }
        }
        else
        {
          bad_health_counter++;
          // bad_health_deque.push_back(ros::Time::now());
          rate_error_deques[0].push_back(ros::Time::now());
          drv->reset(); // reset before checking health
          // this has to do with express scan mode in new scoped_lock
          // which is reading from cached data and when timeout occurs
          // it is stuck in waiting for the thread to finish
          if (!checkRPLIDARHealth(drv))
          {
            NODELET_INFO("Bad health. Let's better re-initialise the driver.");
            res = init_driver(serial_port, serial_baudrate);
            if (res < 0)
            {
              NODELET_ERROR_STREAM("Failed to re-initialise driver. Will keep trying.");
              ros::Duration(1.0).sleep();
            }
            else
            {
              initialised = true;
            }
          }
          else
          {
            NODELET_WARN("RPLidar: driver is ok, but health check failed -> reset and restart scanning");
            u_result op_result;
            op_result = drv->stop();
            if (op_result == RESULT_OK)
            {
              drv->startMotor();
              op_result = drv->startScan();
              if (op_result == RESULT_OK)
              {
                initialised = true;
              }
              else
              {
                NODELET_ERROR("RPLidar: failed to restart scanning. (%i)", op_result);
              }
            }
            else
            {
              NODELET_ERROR("RPLidar: failed to stop scanning. (%i)", op_result);
            }
          }
        }
      }

      updater.update();
    }
    // done!
    RPlidarDriver::DisposeDriver(&drv);
  }

  void RPlidarNodelet::read_scan()
  {
    // Read
    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    start_scan_time = ros::Time::now();
    op_result = drv->grabScanData(nodes, count);
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

    if (op_result == RESULT_OK) {
        op_result = drv->ascendScanData(nodes, count);

        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);
        if (op_result == RESULT_OK) {
            if (angle_compensate) {
                const int angle_compensate_nodes_count = 360;
                const int angle_compensate_multiple = 1;
                int angle_compensate_offset = 0;
                rplidar_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_t));
                int i = 0, j = 0;
                for( ; i < count; i++ ) {
                    if (nodes[i].distance_q2 != 0) {
                        float angle = (float)((nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                        int angle_value = (int)(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                        for (j = 0; j < angle_compensate_multiple; j++) {
                            angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                        }
                    }
                }

                this->publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                         start_scan_time, scan_duration, inverted,
                         angle_min, angle_max,
                         frame_id);
            } else {
                int start_node = 0, end_node = 0;
                int i = 0;
                // find the first valid node and last valid node
                while (nodes[i++].distance_q2 == 0);
                start_node = i-1;
                i = count -1;
                while (nodes[i--].distance_q2 == 0);
                end_node = i+1;

                angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

                this->publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1,
                         start_scan_time, scan_duration, inverted,
                         angle_min, angle_max,
                         frame_id);
           }
      } else if (op_result == RESULT_OPERATION_FAIL) {
          result_fail_counter++;
          // result_fail_deque.push_back(ros::Time::now());
          rate_error_deques[1].push_back(ros::Time::now());
            // All the data is invalid
            // SHOULD NOT PUBLISH ANY DATA FROM here
            // BECAUSE IT CAN CRASH THE PROGRAMS USING THE DATA

            /*
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            NODELET_WARN_STREAM("RPLidar: publishing invalid data; might burst! watch out..");
            this->publish_scan(&scan_pub, nodes, count,
                         start_scan_time, scan_duration, inverted,
                         angle_min, angle_max,
                         frame_id);
            */
        }

    }
    else if (op_result == RESULT_OPERATION_TIMEOUT)
    {
      result_timeout_counter++;
      // result_timeout_deque.push_back(ros::Time::now());
      rate_error_deques[2].push_back(ros::Time::now());

      // to catch this error before we set initialized to false
      // will quickly fade away but good to catch on timeline
      updater.force_update();

      NODELET_WARN_STREAM("RPLidar: she's dead Jim! [timed out waiting for a full 360 scan]");
      initialised = false;
    }
  }

  void RPlidarNodelet::publish_scan(ros::Publisher *pub,
                    rplidar_response_measurement_node_t *nodes,
                    size_t node_count, ros::Time start,
                    double scan_time, bool inverted,
                    float angle_min, float angle_max,
                    std::string frame_id)
  {
      static int scan_count = 0;

      // Allocate a new shared pointer for zero-copy sharing to other nodelet
      sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan);

      scan_msg->header.stamp = start;
      scan_msg->header.frame_id = frame_id;
      scan_count++;

      bool reversed = (angle_max > angle_min);
      if ( reversed ) {
        scan_msg->angle_min =  M_PI - angle_max;
        scan_msg->angle_max =  M_PI - angle_min;
      } else {
        scan_msg->angle_min =  M_PI - angle_min;
        scan_msg->angle_max =  M_PI - angle_max;
      }
      scan_msg->angle_increment =
          (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

      scan_msg->scan_time = scan_time;
      scan_msg->time_increment = scan_time / (double)(node_count-1);

      scan_msg->range_min = 0.15;
      scan_msg->range_max = 6.;

      scan_msg->intensities.resize(node_count);
      scan_msg->ranges.resize(node_count);
      bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
      if (!reverse_data) {
          for (size_t i = 0; i < node_count; i++) {
              float read_value = (float) nodes[i].distance_q2/4.0f/1000;
              if (read_value == 0.0)
                  scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
              else
                  scan_msg->ranges[i] = read_value;
              scan_msg->intensities[i] = (float) (nodes[i].sync_quality >> 2);
          }
      } else {
          for (size_t i = 0; i < node_count; i++) {
              float read_value = (float)nodes[i].distance_q2/4.0f/1000;
              if (read_value == 0.0)
                  scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
              else
                  scan_msg->ranges[node_count-1-i] = read_value;
              scan_msg->intensities[node_count-1-i] = (float) (nodes[i].sync_quality >> 2);
          }
      }

      // Check if the data size is not equal to 360. if not don't publish it
      // This is strict check which could to lose to see only if node_count is 0
      // But 99% of the time RPlidar produces 360 sized data
      if (node_count == 360) {  // Only publish when data size is 360
        pub->publish(scan_msg);
      } else {
        NODELET_WARN_STREAM("RPLidar: not publishing since data count < 360 [" << node_count << "]");
      }
  }

  int RPlidarNodelet::init_driver(std::string& serial_port, int& serial_baudrate)
  {
    // check if there is an existing driver instance
    if (drv)
    {
      if (drv->isConnected())
      {
        NODELET_INFO_STREAM("RPlidar: disconnecting old driver instance.");
        drv->disconnect();
      }
      NODELET_INFO_STREAM("RPlidar: disposing old driver instance.");
      RPlidarDriver::DisposeDriver(&drv);
    }

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv)
    {
      NODELET_ERROR_STREAM("RPlidar: failed to create driver!");
      return -2;
    }

    // make connection...
    NODELET_INFO_STREAM("RPlidar: connecting on " << serial_port << " [" << serial_baudrate << "]");
    u_result result = drv->connect(serial_port.c_str(), (_u32)serial_baudrate);
    if (IS_FAIL(result))
    {
      NODELET_ERROR_STREAM("RPlidar: cannot bind to the specified serial port [" << serial_port << "]");
      RPlidarDriver::DisposeDriver(&drv);
      return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv))
    {
      RPlidarDriver::DisposeDriver(&drv);
      return -1;
    }

    drv->startMotor();
    // start scan...can pass true to this to 'force' it, whatever that is
    u_result start_scan_result = drv->startScan();
    if ( start_scan_result != RESULT_OK )
    {
      NODELET_ERROR_STREAM("RPLidar: failed to put the device into scanning mode [" << start_scan_result << "]");
      RPlidarDriver::DisposeDriver(&drv);
      return -1;
    }

    return 0;
  }

  bool RPlidarNodelet::checkRPLIDARHealth(RPlidarDriver * drv)
  {
      u_result     op_result;
      rplidar_response_device_health_t healthinfo;

      op_result = drv->getHealth(healthinfo);
      if (IS_OK(op_result)) {
        switch (healthinfo.status) {
          case(RPLIDAR_STATUS_OK) : {
            return true;
          }
          case(RPLIDAR_STATUS_ERROR) : {
            NODELET_ERROR_STREAM("RPLidar: health check failed, please reboot the device");
            return false;
          }
          default: {
            NODELET_WARN_STREAM("RPLidar: health not ok, but unhandled status returned [" << healthinfo.status << "]");
            return true;
          }
        }
      } else {
        NODELET_ERROR("RPLidar: cannot retrieve the rplidar health status %x", op_result);
        return false;
      }
  }

  bool RPlidarNodelet::stop_motor(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res)
  {
    if(!drv)
          return false;

    NODELET_DEBUG("RPLidar : stopping the motor");
    drv->stop();
    drv->stopMotor();
    return true;
  }

  bool RPlidarNodelet::start_motor(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res)
  {
    if(!drv)
          return false;
    NODELET_DEBUG("RPLidar : starting the motor");
    drv->startMotor();
    drv->startScan();
    return true;
  }

  bool RPlidarNodelet::reset_device(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res)
  {
    if(!drv)
    {
      return false;
    }
    NODELET_INFO("RPLidar: resetting the device.");
    u_result op_result;
    op_result = drv->reset();
    if (op_result == RESULT_OK)
    {
      res.success = true;
    }
    else
    {
      NODELET_ERROR("RPLidar: failed to reset the device! (%i)", op_result);
      res.success = false;
    }
    return true;
  }

  bool RPlidarNodelet::reset_scan(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res)
  {
    if(!drv)
    {
      return false;
    }
    NODELET_DEBUG("RPLidar: stopping and restarting scanning.");
    u_result op_result;
    op_result = drv->stop();
    if (op_result == RESULT_OK)
    {
      drv->startMotor();
      op_result = drv->startScan();
      if (op_result == RESULT_OK)
      {
        res.success = true;
      }
      else
      {
        NODELET_ERROR("RPLidar: failed to restart scanning. (%i)", op_result);
        res.success = false;
      }
    }
    else
    {
      NODELET_ERROR("RPLidar: failed to stop scanning. (%i)", op_result);
      res.success = false;
    }
    return true;
  }

} // namespace rplidar_extras

PLUGINLIB_DECLARE_CLASS(rplidar_nodelet, RPlidarNodelet, rplidar_extras::RPlidarNodelet, nodelet::Nodelet);
