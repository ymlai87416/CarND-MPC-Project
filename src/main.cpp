#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "Robot.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void transformWayPointToCarPerspective(int px, int py, double psi,
                                       vector<double> ptsx, vector<double> ptsy,
                                       vector<double>& transform_x, vector<double>& transform_y){
  transform_x.clear();
  transform_y.clear();
  for (size_t i = 0; i < ptsx.size(); i++) {
    double dx = ptsx[i] - px;
    double dy = ptsy[i] - py;
    transform_x.push_back(dx * cos(-psi) - dy * sin(-psi));
    transform_y.push_back(dx * sin(-psi) + dy * cos(-psi));
  }
}

double polyeval(vector<double> coeffs, double x) {
  double result = 0.0;
  for (size_t i = 0; i < coeffs.size(); i++) {
    result = result + coeffs[i] * pow(x, i);
  }
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  //MPC mpc;
  Robot robot;
  robot.Init(15, 0.2, 70);

  h.onMessage([&robot](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          double steer_value;
          double throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //Transformed ptsx and ptsy
          vector<double> transform_ptsx;
          vector<double> transform_ptsy;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<double> path_coeffs;

          double px_d, py_d, psi_d, v_d;
          robot.predictStateAfter(0.1, px, py, psi, v, delta, a, px_d, py_d, psi_d, v_d);
          //std::cout << px << " " << px_d << " " << py << " " << py_d << " " << psi << " " << psi_d << std::endl;

          transformWayPointToCarPerspective(px_d, py_d, psi_d, ptsx, ptsy, transform_ptsx, transform_ptsy);

          robot.calculateSteeringAngleAndThrottle(0, 0, 0, v_d, transform_ptsx, transform_ptsy,
                                                  steer_value, throttle_value,
                                                  path_coeffs,
                                                  mpc_x_vals, mpc_y_vals);

          double poly_inc = 2.5;
          int num_points = 25;

          for(int i=1; i<num_points; ++i){
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(path_coeffs, poly_inc*i));
          }

          //debug
          /*
          std::cout << "***********" << std::endl;
          for(size_t i=0; i<num_points; ++i){
            std::cout << next_x_vals[i] << " " << next_y_vals[i] << ", ";
          }
          std::cout << std::endl;

          for(size_t i=0; i<transform_ptsx.size(); ++i){
            std::cout << transform_ptsx[i] << " " << transform_ptsy[i] << ", ";
          }
          std::cout << std::endl;

          std::cout << "***********" << std::endl;
          */

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
