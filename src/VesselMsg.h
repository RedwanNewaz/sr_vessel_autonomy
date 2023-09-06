#pragma once 
#include <iostream>
#include <sstream>
#include <string>
#include <regex>
#include <fmt/core.h>
namespace sea_robotics
{
    struct GPSdata
    {
        double lat, lon, alt; 
    };
    class VesselMsg
    {
    public:
        VesselMsg(const std::string& cmd_templete, double v_max, double v_min, double w_max, double w_min);
        std::string toStrMsg(double v, double w);
        bool getGPSdata(const std::string& msg, GPSdata& gps);  
    
    protected:
        double normalize(double value, double val_min, double val_max, double max_range, double min_range);
        std::string computeChecksum(const std::string& message); 
              

    private:
        double v_max_, v_min_; 
        double w_max_, w_min_;
        std::string cmd_templete_;  

    }; 

    VesselMsg::VesselMsg(const std::string& cmd_templete, double v_max, double v_min, double w_max, double w_min)
    :cmd_templete_(cmd_templete), v_max_(v_max), v_min_(v_min), w_max_(w_max), w_min_(w_min)
    {

    }
    bool VesselMsg::getGPSdata(const std::string& msg, GPSdata& gps)
    {
        if (msg.substr(0, 4) != "$GGA")
            return false; 
        // Split the GPS data by commas
            std::vector<std::string> gps_parts;
            std::istringstream gps_stream(msg);
            std::string gps_part;
            while (std::getline(gps_stream, gps_part, ',')) {
                gps_parts.push_back(gps_part);
            }

            if (gps_parts.size() < 13) {
                // RCLCPP_ERROR(node->get_logger(), "Invalid GPS data format");
                return false;
            }

            // Extract relevant data from the string
            std::string time_stamp = gps_parts[1];
            gps.lat = std::stod(gps_parts[2]);
            gps.lon = std::stod(gps_parts[4]);
            gps.alt = std::stod(gps_parts[9]);
    }

    std::string VesselMsg::toStrMsg(double v, double w)
    {
        // convert joystick value to vessel command 
        // it is assumed that joystick value ranges from -1, 1 
        int thrust_percent = normalize(std::max(0.0, v), 0, 1, v_max_, v_min_);
        int thrust_angle = normalize(w, -1, 1, w_max_, w_min_);
        // compute checksum string 
        std::string input_token = fmt::format(cmd_templete_, thrust_percent, thrust_angle);
        std::string checksum = computeChecksum(input_token); 

        std::string final_str_msg = fmt::format("{}{}\r", input_token, checksum);

        return final_str_msg;
    }

    double VesselMsg::normalize(double value, double val_min, double val_max, double max_range, double min_range)
    {
        //shift origin to 0
        double x_std = (value - val_min) / (val_max - val_min);
        // compute scale of new domain 
        double scale = max_range - min_range; 
        return x_std * scale + min_range; 
    }

    std::string VesselMsg::computeChecksum(const std::string& message)
    {
        int checksum = 0;
    
        // Iterate through the message characters between '$' and '*'
        bool foundDollar = false;
        for (char c : message) {
            if (c == '$') {
                foundDollar = true;
                continue; // Skip the '$' character itself
            }
            if (foundDollar && c == '*') {
                break; // Stop when '*' is encountered
            }
            
            // XOR the ASCII value of the character with the current checksum
            checksum ^= static_cast<int>(c);
        }
        
        // Convert the checksum to a hexadecimal string
        std::stringstream stream;
        stream << std::hex << checksum;
        std::string result = stream.str();
        
        // Ensure the checksum is always two characters (e.g., '0A' instead of just 'A')
        if (result.size() == 1) {
            result = "0" + result;
        }
        
        return result;
    }

}