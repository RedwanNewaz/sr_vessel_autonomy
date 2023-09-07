#pragma once 
#include <string> 
#include <vector>
#include <sstream> 
#include <mutex> 
#include <memory>


namespace sea_robotics
{
    class GPSData;
    typedef std::shared_ptr<GPSData> GPSDataPtr; 
    class GPSData: public std::enable_shared_from_this<GPSData>
    {
        double lat, lon, alt; 
        bool update;
        std::mutex dataMutex; // Mutex to protect data access

        

    public:
        GPSData() : lat(0.0), lon(0.0), alt(0.0), update(false) 
        {

        }
        
        bool isUpdated() const 
        {
            return update; 
        }

        GPSDataPtr getPtr()
        {
            return shared_from_this();
        }

        std::tuple<double, double, double> getData()
        {
            update = false; 
            return std::make_tuple(lat, lon, alt);
        }   

        void update_data(const std::string& msg)
        {
            
            std::lock_guard<std::mutex> lock(dataMutex); // Lock to ensure thread safety
            
            if (msg.substr(0, 4) != "$GGA")
            {
                std::cout << "[GPSData]: msg rejected " << std::endl;
                return;
            }

            // Split the GPS data by commas
            std::vector<std::string> gps_parts;
            std::istringstream gps_stream(msg);
            std::string gps_part;
            while (std::getline(gps_stream, gps_part, ',')) {
                gps_parts.push_back(gps_part);
            }

            if (gps_parts.size() < 13) {
                // RCLCPP_ERROR(node->get_logger(), "Invalid GPS data format");
                std::cout << "[GPSData]: Invalid GPS data format " << std::endl;
                return;
            }

            // Extract relevant data from the string
            std::string time_stamp = gps_parts[1];
            lat = std::stod(gps_parts[2]);
            lon = std::stod(gps_parts[4]);
            alt = std::stod(gps_parts[9]);
            update = true; 
            std::cout << "Received: " << msg << std::endl;
        }
    };
}