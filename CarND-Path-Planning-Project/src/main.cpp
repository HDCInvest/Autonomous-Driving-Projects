#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Define following 2 variables
  int lane = 1;   // We start with middle lane , which is 1
  double ref_vel = 0.0 ; // We start with 0.0 as the car is stand still at start

  h.onMessage([&ref_vel, &lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
        
          /* Previous path size is the last path the car was following before the current calculation/iteration, 
             actually similator will give this data also.Having previous list of points will help in transition
             Set prev_size variable which comes from previous path size.
          */
          int prev_size = previous_path_x.size() ;
          
          /* To avoid colusion with other cars we need to go through the sensor fusion list .Check whether
              any other car is in the lane or not. If any other car is very close we need to take some action
          */
          if(prev_size > 0 )
          {
            car_s = end_path_s; //We will using frenet here, as its easy to implement the logic in frenet
          }

          bool too_close = false; //We initialize this to false

          //Structure of sensor fusion data is vector of vector of double
          for(int i=0;i<sensor_fusion.size();i++)  //ith car
          {
            //Car is my lane
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d > (2+4*lane-2)) //Check if the car is the lane
            {
              //If in our lane , need to check the speed of the car
              //Get the velocity elements
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy); //Find the magnitude of the vector
              double check_car_s = sensor_fusion[i][5]; //S value is critical to know whether the car is close to  us or not
              
              //Speed calculated above used to predict the car position in future
              //Here we are using previous points to  project s value outwards in time
              check_car_s += ((double)prev_size*.02*check_speed);
              //From the prescpetive of previous path we need to see where the other cars will fit in the future
              
              //Check whether our car s is close to other car s
              if((check_car_s > car_s) && (check_car_s -car_s < 30))
              {
                //Need to take action if our car in future within some distance of the other car in future
                //We can reduce the ref velocity and/or initiate lane change
                too_close = true ; //Set this flag instead of setting a velocity. This will help reducing jerks.
                /* If the car is not in the left most lane try to get there if we have a slow moving vehicle 
                  in current lane. This trggers lane change. We can have better implementation of this logic
                  by having cost functions */
                if (lane > 0 ) 
                {
                  lane = 0 ;
                }
              }
            }
          }
          //If the car is too close we will be subtracting a constant value from reference velocity
          //Here its around 5m/sec2, which is under the 10 m/sec2 jerk requirement
          if(too_close)
          {
            ref_vel -= .224 ;
          }
          else if (ref_vel < 49.5 )
          {
            ref_vel += .224 ;  //Acclerate slowly if the velocity is less than ref vel
          }


          /* Create a list of widely placed spaced(x,y) waypoints, evenly spaced at 30m.Later we will
             interpolate  these waypoints with a spline and fill it in with more points that control spline
          */
          vector<double> ptsx;
          vector<double> ptsy;

          /* Get the reference x,y,yaw states. Either we will reference the starting point as where the car is 
          or at the previous path end points.If previous size is almost empty,use the car as starting reference 
          or else use the previous path's end point as starting reference */
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_size < 2)  //if previous size is almost empty,use the car as starting reference
          {
            //Use the 2 points that make the path tangent to the car
            //Looking at creating a path which is tangent to the angle of the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else //use the previous path's end point as starting reference
          {
          //redifine reference points as previous path end point
          //Create a tangent using last point and previous last point
          ref_x = previous_path_x[prev_size-1];
          ref_y = previous_path_y[prev_size-1];

          double ref_x_prev = previous_path_x[prev_size-2];
          double ref_y_prev = previous_path_y[prev_size-2];
          ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

          //Use the 2 points that make the path tangent to the previous path's end point
          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);
          }
          
          /* In summary in the above code snippet we took last couple of points from the previous path 
           the car was following and then calculated what angle the car was heading using those last couple 
           of points.Then we pushed them on to list of previous points.
           So far we have pushed 2 x and 2 y. So we have 2 points basically and this is our starting reference

            In Frenet coordinates, add evenly 30 mts spaced points ahead of the starting reference .
            Earlier we used 50 points, now we are using 3 points. Instead of being just .5 meters space now 
            they are all the way to 30m. Actually 30mts  is good enough, as during lane change it would
            to be smooth and lesser value can make lane change little aggressive
          */

          vector <double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector <double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector <double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          /* Now we will go a head and push these 3 points. After pushing this vector points has 5 points - 
             2 previous points and location of car at 30,60 and 90 meters.
          */
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          

          /* To make sure the car or the last point of the previous  path is at (0,0), the origin, and its angle
           at zero degrees we do a transformation to this car's local coordinates system. We are basically shifting 
           to  look into the car reference frame.This transformation is basically shift and rotation.
           Then we are creating a spline and adding all the reference points created.
          */
          for(int i=0;i<ptsx.size();i++)
          {
            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = ( shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = ( shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          tk::spline s;  //create spline
          s.set_points(ptsx,ptsy); //set x,y points to the spline

          //Now we will define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /* Start with all the previous path points from last time. Just add them to the path planner 
            if any previous path points and this helps in transtion. Instead of recreating the path 
            every single time, why not add to it and work with what we have left from last time. Basically 
            the number of points we add this time depends on how many points the car used during last time
          */
          for(int i=0;i<previous_path_x.size();i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;  // This is our horizon point going out at 30 mts
          double target_y = s(target_x); //Asking spline for y values for corresponding x values
          //Distance calculation from last point in the previous path or car itself to the target
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y)) ;
          
          // We start at 0 , This we create due to local transformation we do for the car and we start at the origin
          double x_add_on = 0 ; 
          /* Now we need to add all those points along the spline.Fill up the rest of our path planner 
            after filling it with previous points.Here we always output 50 points */
          for(int i=1;i<=50-previous_path_x.size();i++)
          {
            double N = (target_dist/(.02*ref_vel/2.24));
            //Dividing by 2.24 as we need the result in meters per sec and need to be converted from miles per hour
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            
            /* We are currently in local coordinates and  need to go back to global coordinates. Rotate back 
            to normal after rotating it earlier We are just inversing what we did earlier */
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            //We will push this to our next values
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}