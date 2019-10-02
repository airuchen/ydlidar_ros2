/*
 Copyright 2018 ADLINK Technology, Inc.
 Developer: 
  * Alan Chen (alan.chen@adlinktech.com
  * HaoChih, LIN (haochih.lin@adlinktech.com)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 This is the modified version of ydlidar pkg for ROS 2.0 environment.
 The original (ROS1) verion and sdk source code are belong to "EAI TEAM"
 (For further info: EAI official website: http://www.ydlidar.com)

*/

// Define common dialogue functions
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_FATAL RCUTILS_LOG_FATAL
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_DEBUG RCUTILS_LOG_DEBUG
#define _USE_MATH_DEFINES

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "ydlidar_driver.h"
#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <signal.h>
#include <math.h>

#define DELAY_SECONDS 2
#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace ydlidar;

#define ROS2Verision "0.0.1"

static bool flag = true;
static int nodes_count = 720;
static double each_angle = 0.5;
int type = 0;
int print = 0;

void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub, node_info* nodes, size_t node_count, rclcpp::Time start,
    double scan_time, double angle_min, double angle_max, std::string frame_id,
    std::vector<double> ignore_array, double min_range, double max_range)
{
    sensor_msgs::msg::LaserScan scan_msg;

    int counts = (int)(node_count * ((angle_max - angle_min) / 360.0));
    int angle_start = (int)(180.0 + angle_min);
    int node_start = (int)(node_count * (angle_start / 360.0));

    scan_msg.ranges.resize(counts);
    scan_msg.intensities.resize(counts);

    double range = 0.0;
    double intensity = 0.0;
    int index = 0;
    for (size_t i = 0; i < node_count; i++)
    {
        range = (double)(nodes[i].distance_q2 / 4.0 / 1000.0);
        intensity = (double)(nodes[i].sync_quality >> 2);

        if (i < node_count / 2)
        {
            index = node_count / 2 - 1 - i;
        }
        else
        {
            index = node_count - 1 - (i - node_count / 2);
        }

        if ((range > max_range) || (range < min_range))
        {
            range = 0.0;
        }

        int pos = index - node_start;
        if (0 <= pos && pos < counts)
        {
            if (ignore_array.size() != 0)
            {
                double angle = angle_min + pos * (angle_max - angle_min) / (double)counts;
                for (uint16_t j = 0; j < ignore_array.size(); j = j + 2) //magic
                {
                    if ((ignore_array[j] < angle) && (angle <= ignore_array[j + 1]))
                    {
                        range = 0.0;
                        break;
                    }
                }
            }

            scan_msg.ranges[pos] = range;
            scan_msg.intensities[pos] = intensity;
        }
    }

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    double radian_min = DEG2RAD(angle_min);
    double radian_max = DEG2RAD(angle_max);
    scan_msg.angle_min = radian_min;
    scan_msg.angle_max = radian_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(counts - 1);
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)counts;
    scan_msg.range_min = min_range;
    scan_msg.range_max = max_range;

    pub->publish(scan_msg);
}


std::vector<double> split(const std::string& s, char delim)
{
    std::vector<double> elems;
    std::stringstream ss(s);
    std::string number;
    while (std::getline(ss, number, delim))
    {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


bool getDeviceInfo(std::string port, int& samp_rate, double _frequency, int baudrate)
{
    if (!YDlidarDriver::singleton())
    {
        return false;
    }

    device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) != RESULT_OK)
    {
        if (print == 3)
            ROS_ERROR("YDLIDAR get DeviceInfo Error\n");
        return false;
    }
    sampling_rate _rate;
    scan_frequency _scan_frequency;
    int _samp_rate = 4;
    std::string model;
    double freq = 7.0;
    int hz = 0;
    type = devinfo.model;
    switch (devinfo.model)
    {
        case 1:
            model = "F4";
            _samp_rate = 4;
            freq = 7.0;
            break;
        case 4:
            model = "S4";
            if (baudrate == 153600)
            {
                model = "S4Pro";
            }
            _samp_rate = 4;
            freq = 7.0;
            break;
        case 5:
            model = "G4";
            YDlidarDriver::singleton()->getSamplingRate(_rate);
            switch (samp_rate)
            {
                case 4:
                    _samp_rate = 0;
                    break;
                case 8:
                    _samp_rate = 1;
                    break;
                case 9:
                    _samp_rate = 2;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
            }
            while (_samp_rate != _rate.rate)
            {
                YDlidarDriver::singleton()->setSamplingRate(_rate);
            }
            switch (_rate.rate)
            {
                case 0:
                    _samp_rate = 4;
                    break;
                case 1:
                    nodes_count = 1440;
                    each_angle = 0.25;
                    _samp_rate = 8;
                    break;
                case 2:
                    nodes_count = 1440;
                    each_angle = 0.25;
                    _samp_rate = 9;
                    break;
            }

            if (YDlidarDriver::singleton()->getScanFrequency(_scan_frequency) != RESULT_OK)
            {
                ROS_ERROR("YDLIDAR get frequency Error\n");
                return false;
            }
            freq = _scan_frequency.frequency / 100;
            hz = _frequency - freq;
            if (hz > 0)
            {
                while (hz != 0)
                {
                    YDlidarDriver::singleton()->setScanFrequencyAdd(_scan_frequency);
                    hz--;
                }
                freq = _scan_frequency.frequency / 100.0;
            }
            else
            {
                while (hz != 0)
                {
                    YDlidarDriver::singleton()->setScanFrequencyDis(_scan_frequency);
                    hz++;
                }
                freq = _scan_frequency.frequency / 100.0;
            }
            break;
        case 6:
            model = "X4";
            _samp_rate = 5;
            freq = 7.0;
            break;
        case 8:
            model = "F4Pro";
            YDlidarDriver::singleton()->getSamplingRate(_rate);
            switch (samp_rate)
            {
                case 4:
                    _samp_rate = 0;
                    break;
                case 6:
                    _samp_rate = 1;
                    break;
                default:
                    _samp_rate = _rate.rate;
                    break;
            }
            while (_samp_rate != _rate.rate)
            {
                YDlidarDriver::singleton()->setSamplingRate(_rate);
            }
            switch (_rate.rate)
            {
                case 0:
                    _samp_rate = 4;
                    break;
                case 1:
                    nodes_count = 1440;
                    each_angle = 0.25;
                    _samp_rate = 6;
                    break;
            }

            if (YDlidarDriver::singleton()->getScanFrequency(_scan_frequency) != RESULT_OK)
            {
                ROS_ERROR("YDLIDAR get frequency Error\n");
                return false;
            }
            freq = _scan_frequency.frequency / 100;
            hz = _frequency - freq;
            if (hz > 0)
            {
                while (hz != 0)
                {
                    YDlidarDriver::singleton()->setScanFrequencyAdd(_scan_frequency);
                    hz--;
                }
                freq = _scan_frequency.frequency / 100.0;
            }
            else
            {
                while (hz != 0)
                {
                    YDlidarDriver::singleton()->setScanFrequencyDis(_scan_frequency);
                    hz++;
                }
                freq = _scan_frequency.frequency / 100.0;
            }
            break;
        case 9:
            model = "G4C";
            _samp_rate = 4;
            freq = 7.0;
            break;
        default:
            model = "Unknown";
    }

    samp_rate = _samp_rate;
    uint16_t maxv = (uint16_t)(devinfo.firmware_version >> 8);
    uint16_t midv = (uint16_t)(devinfo.firmware_version & 0xff) / 10;
    uint16_t minv = (uint16_t)(devinfo.firmware_version & 0xff) % 10;

    printf("[YDLIDAR INFO] Connection established in %s:\n"
           "Firmware version: %u.%u.%u\n"
           "Hardware version: %u\n"
           "Model: %s\n"
           "Serial: ",
        port.c_str(), maxv, midv, minv, (uint16_t)devinfo.hardware_version, model.c_str());

    for (int i = 0; i < 16; i++)
    {
        printf("%01X", devinfo.serialnum[i] & 0xff);
    }
    printf("\n");

    printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", _samp_rate);
    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n", freq);

    return true;
}


bool getDeviceHealth()
{
    if (!YDlidarDriver::singleton())
    {
        return false;
    }

    result_t op_result;
    device_health healthinfo;

    op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK)
    {
        printf("[YDLIDAR INFO] YDLIDAR running correctly! The health status: %s\n",
            healthinfo.status == 0 ? "well" : "bad");

        if (healthinfo.status == 2)
        {
            if (print == 3)
                ROS_ERROR(
                    "Error, YDLIDAR internal error detected. Please reboot the device to retry.");
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        if (print == 3)
            ROS_ERROR("Error, cannot retrieve YDLIDAR health code: %x", op_result);
        return false;
    }
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ydlidar_node");    

    std::string port;
    int baudrate = 115200;
    std::string model;
    std::string frame_id;
    bool angle_fixed, intensities_, low_exposure, reversion, resolution_fixed, heartbeat;
    double angle_max, angle_min;
    result_t op_result;
    int samp_rate;
    std::string list;
    std::vector<double> ignore_array;
    double max_range, min_range;
    double _frequency;

    auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rmw_qos_profile_sensor_data);

    node->get_parameter_or("angle_fixed", angle_fixed, true);
    node->get_parameter_or("resolution_fixed", resolution_fixed, true);
    node->get_parameter_or("heartbeat", heartbeat, false);
    node->get_parameter_or("low_exposure", low_exposure, false);
    node->get_parameter_or("baudrate", baudrate, 115200);
    node->get_parameter_or("angle_max", angle_max, 180.0);
    node->get_parameter_or("angle_min", angle_min, -180.0); // degree
    node->get_parameter_or("samp_rate", samp_rate, 4); 
    node->get_parameter_or("range_max", max_range, 16.0); // m
    node->get_parameter_or("range_min", min_range, 0.08); // m
    node->get_parameter_or("frequency", _frequency, 7.0); // Hz
    node->get_parameter_or("port", port, std::string("/dev/ydlidar"));
    node->get_parameter_or("frame_id", frame_id, std::string("laser_frame"));
    node->get_parameter_or("ignore_array", list, std::string(" "));
    ignore_array = split(list, ',');
    reversion = false;

    if (ignore_array.size() % 2)
    {
        ROS_ERROR("ignore array is odd need be even");
    }

    for (uint16_t i = 0; i < ignore_array.size(); i++)
    {
        if (ignore_array[i] < -180 && ignore_array[i] > 180)
        {
            ROS_ERROR("ignore array should be between -180 and 180");
        }
    }

    YDlidarDriver::initDriver();
    if (!YDlidarDriver::singleton())
    {
        ROS_ERROR("YDLIDAR Create Driver fail, exit\n");
        return -2;
    }

    if (_frequency < 5)
    {
        _frequency = 7.0;
    }
    if (_frequency > 12)
    {
        _frequency = 12;
    }
    if (angle_max < angle_min)
    {
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    // check lidar type
    std::map<int, bool> checkmodel;
    checkmodel.insert(std::map<int, bool>::value_type(115200, false));
    checkmodel.insert(std::map<int, bool>::value_type(128000, false));
    checkmodel.insert(std::map<int, bool>::value_type(153600, false));
    checkmodel.insert(std::map<int, bool>::value_type(230400, false));
    printf("[YDLIDAR INFO] Current ROS 2.0 Driver Version: %s\n", ((std::string)ROS2Verision).c_str());
    printf("[YDLIDAR INFO] Current SDK Version: %s\n",
        YDlidarDriver::singleton()->getSDKVersion().c_str());


again:
    op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
    if (op_result != RESULT_OK)
    {
        int seconds = 0;
        while (seconds <= DELAY_SECONDS && flag)
        {
            //sleep(2);
            rclcpp::sleep_for(std::chrono::milliseconds(2000) );
            seconds = seconds + 2;
            YDlidarDriver::singleton()->disconnect();
            op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
            printf("[YDLIDAR INFO] Try to connect the port %s again  after %d s .\n", port.c_str(),
                seconds);
            if (op_result == RESULT_OK)
            {
                break;
            }
        }

        if (seconds > DELAY_SECONDS)
        {
            ROS_ERROR("YDLIDAR Cannot bind to the specified serial port %s", port.c_str());
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            return -1;
        }
    }

    bool ret = getDeviceHealth();
    if (!getDeviceInfo(port, samp_rate, _frequency, baudrate) && !ret)
    {
        checkmodel[baudrate] = true;
        map<int, bool>::iterator it;
        for (it = checkmodel.begin(); it != checkmodel.end(); ++it)
        {
            if (it->second)
                continue;
            print++;
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            YDlidarDriver::initDriver();
            if (!YDlidarDriver::singleton())
            {
                ROS_ERROR("YDLIDAR Create Driver fail, exit\n");
                return -1;
            }
            baudrate = it->first;
            goto again;
        }

        ROS_ERROR("[YDLIDAR ERROR] Unsupported lidar\n");
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
        return -1;
    }
    printf("[YDLIDAR INFO] Connected to YDLIDAR on port %s at %d \n", port.c_str(), baudrate);
    print = 0;

    if (type != 4)
    {
        intensities_ = false;
    }
    else
    {
        intensities_ = true;
        if (baudrate != 153600)
        {
            intensities_ = false;
        }
    }


    YDlidarDriver::singleton()->setIntensities(intensities_);
    if (intensities_)
    {
        scan_exposure exposure;
        int cnt = 0;
        while ((YDlidarDriver::singleton()->setLowExposure(exposure) == RESULT_OK) && (cnt < 3))
        {
            if (exposure.exposure != low_exposure)
            {
                ROS_INFO("set EXPOSURE MODEL SUCCESS!!!");
                break;
            }
            cnt++;
        }
        if (cnt >= 3)
        {
            ROS_ERROR("set LOW EXPOSURE MODEL FALIED!!!");
        }
    }

    if (type == 5 || type == 8 || type == 9)
    {
        scan_heart_beat beat;
        if (type != 8)
            reversion = true;
        result_t ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
        if (heartbeat)
        {
            if (beat.enable && ans == RESULT_OK)
            {
                ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
            }
            if (!beat.enable && ans == RESULT_OK)
            {
                YDlidarDriver::singleton()->setHeartBeat(true);
            }
        }
        else
        {
            if (!beat.enable && ans == RESULT_OK)
            {
                ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
            }
            if (beat.enable && ans == RESULT_OK)
            {
                YDlidarDriver::singleton()->setHeartBeat(false);
            }
        }

        if (_frequency < 7 && samp_rate > 6)
        {
            nodes_count = 1600;
        }
        else if (_frequency < 6 && samp_rate == 9)
        {
            nodes_count = 1800;
        }
    }

    result_t ans = YDlidarDriver::singleton()->startScan();
    if (ans != RESULT_OK)
    {
        ans = YDlidarDriver::singleton()->startScan();
        if (ans != RESULT_OK)
        {
            ROS_ERROR("start YDLIDAR is failed! Exit!! ......");
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            return 0;
        }
    }

    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    flag = false;
    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    double scan_duration;
    rclcpp::Rate rate(30);


    int max_nodes_count = nodes_count;
    each_angle = 360.0 / (double)(nodes_count);

    while (rclcpp::ok())
    {
        try
        {
            node_info nodes[nodes_count];
            size_t count = _countof(nodes);

            start_scan_time = rclcpp::Clock(RCL_ROS_TIME).now();
            op_result = YDlidarDriver::singleton()->grabScanData(nodes, count);
            end_scan_time = rclcpp::Clock(RCL_ROS_TIME).now();

            if (op_result == RESULT_OK)
            {
                op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);

                if (op_result == RESULT_OK)
                {
                    if (angle_fixed)
                    {
                        if (!resolution_fixed)
                        {
                            max_nodes_count = count;
                        }
                        else
                        {
                            max_nodes_count = nodes_count;
                        }

                        each_angle = 360.0 / (double)(max_nodes_count);
                        node_info all_nodes[max_nodes_count];
                        memset(all_nodes, 0, max_nodes_count * sizeof(node_info));

                        uint64_t end_time = nodes[0].stamp;
                        uint64_t start_time = nodes[0].stamp;

                        for (size_t i = 0; i < count; i++)
                        {
                            if (nodes[i].distance_q2 != 0)
                            {
                                double angle = (double)((nodes[i].angle_q6_checkbit
                                                          >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)
                                    / 64.0);
                                if (reversion)
                                {
                                    angle = angle + 180;
                                    if (angle >= 360)
                                    {
                                        angle = angle - 360;
                                    }
                                    nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0))
                                        << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                                }
                                int inter = (int)(angle / each_angle);
                                double angle_pre = angle - inter * each_angle;
                                double angle_next = (inter + 1) * each_angle - angle;
                                if (angle_pre < angle_next)
                                {
                                    if (inter < max_nodes_count)
                                    {
                                        all_nodes[inter] = nodes[i];
                                    }
                                }
                                else
                                {
                                    if (inter < max_nodes_count - 1)
                                    {
                                        all_nodes[inter + 1] = nodes[i];
                                    }
                                }
                            }

                            if (nodes[i].stamp < start_time)
                            {
                                start_time = nodes[i].stamp;
                            }
                            if (nodes[i].stamp > end_time)
                            {
                                end_time = nodes[i].stamp;
                            }
                        }

                        scan_duration = (end_scan_time - start_scan_time).nanoseconds()/10e9;

                        publish_scan(scan_pub, all_nodes, max_nodes_count, start_scan_time,
                            scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range,
                            max_range);
                    }
                    else
                    {
                        int start_node = 0, end_node = 0;
                        int i = 0;
                        while (nodes[i++].distance_q2 == 0 && i < count)
                            ;
                        start_node = i - 1;
                        i = count - 1;
                        while (nodes[i--].distance_q2 == 0 && i >= 0)
                            ;
                        end_node = i + 1;

                        angle_min = (double)(nodes[start_node].angle_q6_checkbit
                                        >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)
                            / 64.0;
                        angle_max = (double)(nodes[end_node].angle_q6_checkbit
                                        >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)
                            / 64.0;

                        publish_scan(scan_pub, &nodes[start_node], end_node - start_node + 1,
                            start_scan_time, scan_duration, angle_min, angle_max, frame_id,
                            ignore_array, min_range, max_range);
                    }
                }
            }
            rclcpp::spin_some(node);            
            rate.sleep();
            
        }
        catch (std::exception& e)
        { //
            std::cout << "Unhandled Exception: " << e.what() << std::endl;
            break;
        }
        catch (...){//anthor exception
            ROS_ERROR("Unhandled Exception:Unknown ");
            break;
        }
    }

    YDlidarDriver::singleton()->disconnect();
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
    YDlidarDriver::done();
    return 0;
}
