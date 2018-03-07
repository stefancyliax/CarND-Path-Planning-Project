#include <iostream>
#include <math.h>
// #include <cmath>

using namespace std;


void testtesttest(void)
{
    cout << "testtesttest" << endl;;
}

vector<double> findNearestVehicle(const double &car_s, const vector<vector<double>> &lane_vehicles)
{
    vector<double> result;
    double distance = 99999;
    double speed = 0;

    for (int i = 0; i < lane_vehicles.size(); i++)
    {
        double s = lane_vehicles[i][5];
        if ((s > car_s) && (distance > (s - car_s)))
        {  
            distance = (s - car_s);
            double vx = lane_vehicles[i][3];
            double vy = lane_vehicles[i][4];
            speed = sqrt(vx*vx+vy*vy)*2.24;
        }
    }

    result.push_back(distance);
    result.push_back(speed);

    return result;
}

vector<double> findVehicleInFront(const int lane, const double &car_s, const vector<vector<double>> &sensor_fusion)
{
    vector<double> result;
    double distance = 99999;
    double speed = 0;

    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        float s = sensor_fusion[i][5];

        if ((d > (2+lane*4)-2) && (d < (2+lane*4)+2))
        {
            // vehicle is in my lane
            if ((s > car_s) && (distance > (s - car_s)))
            {   
                // find vehicle in same lane that is closest
                distance = (s - car_s);
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                speed = sqrt(vx*vx+vy*vy)*2.24;
            }
        }
    }

    result.push_back(distance);
    result.push_back(speed);

    return result;
}