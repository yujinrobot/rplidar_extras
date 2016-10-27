/**
 * @file /rplidar_extras/include//rplidar_extras/rplidar_nodelet.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rplidar_ros_INCLUDE_RPLIDAR_NODELET_HPP_
#define rplidar_ros_INCLUDE_RPLIDAR_NODELET_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nodelet/nodelet.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "boost/thread.hpp"

#include <rplidar.h> //RPLIDAR standard sdk, all-in-one header

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>


using namespace rp::standalone::rplidar;

namespace rplidar_ros {

  class RPlidarNodelet: public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    virtual void devicePoll();

    void read_scan();

    void publish_scan(ros::Publisher *pub,
                        rplidar_response_measurement_node_t *nodes,
                        size_t node_count, ros::Time start,
                        double scan_time, bool inverted,
                        float angle_min, float angle_max,
                        std::string frame_id);
    int init_driver(std::string& serial_port, int& serial_baudrate);
    bool checkRPLIDARHealth(RPlidarDriver * drv);
    bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool start_motor(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
    bool reset_device(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool reset_scan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void device_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void diagnosticsRatePoll();

    RPlidarNodelet(){};
    ~RPlidarNodelet(){
      drv->stop();
      drv->stopMotor();
      RPlidarDriver::DisposeDriver(drv);
    };

  private:
    RPlidarDriver * drv;//
    bool initialised;
    ros::Time start_scan_time;
    ros::Time end_scan_time;
    u_result op_result;
    double scan_duration;
    bool angle_compensate;
    bool inverted;
    std::string frame_id;
    std::string serial_port;
    int diag_time_window;
    int serial_baudrate;
    int res;

    int bad_health_counter;
    int result_fail_counter;
    int result_timeout_counter;

    std::deque<ros::Time> result_timeout_deque;
    std::deque<ros::Time> result_fail_deque;
    std::deque<ros::Time> bad_health_deque;
    std::vector< std::deque<ros::Time> > rate_error_deques;

    boost::mutex mutex_;
    ros::Publisher scan_pub;
    ros::ServiceServer stop_motor_service;
    ros::ServiceServer start_motor_service;
    ros::ServiceServer reset_device_service;
    ros::ServiceServer reset_scan_service;
    ros::Time starting_time;
    ros::Duration elapsed_time;
    ros::Duration diag_time_window_seconds;

    boost::shared_ptr<boost::thread> device_thread_;
    boost::shared_ptr<boost::thread> diagnostics_rate_thread_;
    diagnostic_updater::Updater updater;

  };


} // namespace rplidar_ros

#endif /* rplidar_ros_INCLUDE_RPLIDAR_NODELET_HPP_ */