#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double a = j[1]["throttle"];
          double delta = j[1]["steering_angle"]; 

          Eigen::VectorXd poly_ptsx(ptsx.size());
          Eigen::VectorXd poly_ptsy(ptsy.size());

          /*Rotation angle of Global coordinates w.r.t car coordinates*/
          double psi_global = (0-psi);
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          /*Transform the waypoint into car coordinates*/
          /*Also populate the next_x_vals & next_y_vals*/
          for (unsigned int i = 0; i<ptsx.size(); i++)
          {
            double trans_x = ptsx[i] - px;
            double trans_y = ptsy[i] - py;
            double car_map_x = trans_x*cos(psi_global) - trans_y*sin(psi_global);
            double car_map_y = trans_x*sin(psi_global) + trans_y*cos(psi_global);

            poly_ptsx[i] = car_map_x;
            poly_ptsy[i] = car_map_y;
            next_x_vals.push_back(car_map_x);
            next_y_vals.push_back(car_map_y);
          }

          /*Car coordinates in car coordinate space*/
          px = 0.0;
          py = 0.0;
          psi = 0.0; 
 
          /*Find the coefficients for the polynomial curve of the waypoints in car coordinate space*/
          auto coeffs = polyfit(poly_ptsx, poly_ptsy, 3);
          /*In car coordinate space, car's location is at (0,0) and psi=0*/
          /*Calculate the cross track error & orientation error in car coordinate space*/
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          /*Account for latency of 0.1 seconds*/
          /*Transform the state of 0.1 seconds into the future.*/
          /*Providing the future state to MPC solver will provide
          the correct actuation that is to be applied 0.1 seconds in the future*/
          double dt = 0.1;
          const double Lf = 2.67;
          px = px + v*cos(psi)*dt;
          py = py + v*sin(psi)*dt;
          psi = psi - v*delta/Lf*dt;
          cte = cte + v*sin(epsi)*dt;
          epsi = epsi - v*delta/Lf*dt;
          v = v + a*dt;

          Eigen::VectorXd state(6);
          
          state << px, py, psi, v, cte, epsi;

          double steer_value;
          double throttle_value;

          json msgJson;

          /*result[0] is steer value to be applied*/
          /*result[1] is throttle value to be applied*/
          /*result[2] onwards N x-coordinates followed by N y-coordinates for the next N steps*/
          /*Calculate steering angle and throttle using MPC.*/
          /* Both are in between [-1, 1].*/
          auto result = mpc.Solve(state, coeffs);
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          /*Set steer value*/
          steer_value = result[0]/(deg2rad(25));
          // In simulator a postive steer value is right turn;
          steer_value = -1*steer_value;

          /*Set throttle value*/
          throttle_value = result[1];

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          /*Rotation angle of Global coordinates w.r.t car coordinates*/
          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /*Obtain MPC points that are in Car cordinates*/
          unsigned int mpc_points = (result.size() - 2)/2;
          for (unsigned int i = 0; i < mpc_points; i++)
          {
            mpc_x_vals.push_back(result[i+2]);
            mpc_y_vals.push_back(result[i+2+mpc_points]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          

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
