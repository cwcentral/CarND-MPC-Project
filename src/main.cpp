#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>

std::ofstream dataFile;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}
double deg2rad(double x) {
	return x * pi() / 180;
}
double rad2deg(double x) {
	return x * 180 / pi();
}

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

	dataFile.open ("control.dat");


	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage(
			[&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				string sdata = string(data).substr(0, length);
				//cout << sdata << endl;
				if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
					string s = hasData(sdata);
					if (s != "") {
						auto j = json::parse(s);
						string event = j[0].get<string>();
						if (event == "telemetry") {
							// j[1] is the data JSON object
							//  ptsx (Array) - The global x positions of the waypoints.
							//	ptsy (Array) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
							//	psi (float) - The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
							//	psi_unity (float) - The orientation of the vehicle in radians. This is an orientation commonly used in navigation.
							//	x (float) - The global x position of the vehicle.
							//	y (float) - The global y position of the vehicle.
							//	steering_angle (float) - The current steering angle in radians.
							//	throttle (float) - The current throttle value [-1, 1].
							//	speed (float) - The current velocity in mph.

							vector<double> ptsx = j[1]["ptsx"];
							vector<double> ptsy = j[1]["ptsy"];
							double px = j[1]["x"];
							double py = j[1]["y"];
							double psi = j[1]["psi"];
							double v = j[1]["speed"];

							/*
							 * TODO: Calculate steering angle and throttle using MPC.
							 *
							 * Both are in between [-1, 1].
							 *
							 */

							// See MPC Car quizzes
							// Initialize Eigen VectorXds for polyfitting. So init ptsx and ptsy arrays as Eigen types
							// The x and y coordinates are contained in the ptsx and ptsy vectors. Since these are 2-element vectors
							// a 1-degree polynomial (straight line) is sufficient.
							//
							// reference path to display
							Eigen::Map<Eigen::VectorXd> x_vals(ptsx.data(), ptsx.size());
							Eigen::Map<Eigen::VectorXd> y_vals(ptsy.data(), ptsy.size());

							//
							// preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure
							//
							// Tips and tricks: These (x,y) points must be displayed in reference to the vehicle's coordinate system.
							// convert the XY positions of the trajectory into car's reference frame (aka car is origin,
							// convert the trajectory distance w.r.t car)
							//
							for (int i = 0; i < ptsx.size(); ++i) {
								double x_diff = x_vals(i) - px;
								double y_diff = y_vals(i) - py;
								x_vals(i) = x_diff * cos(psi) + y_diff * sin(psi);
								y_vals(i) = -x_diff * sin(psi) + y_diff * cos(psi);
							}

							// HINT: car ref current state px, py, psi ARE NOW 0

							// Create the new line coeffs w.r.t car.
							auto coeffs = polyfit(x_vals, y_vals, 3);

							// The cross track error is calculated by evaluating at polynomial at x, f(x)
							double cte = polyeval(coeffs, 0.);

							// Due to the sign starting at 0, the orientation error is -f'(x)
							double epsi = -atan(coeffs[1]);

							Eigen::VectorXd state(6);

							// Since my state is w.r.t to the car frame, x,y,psi are 0.
							state << 0, 0, 0, v, cte, epsi;

							// RUN MPC against the current waypoints
							auto actuators = mpc.Solve(state, coeffs);

							// assign output from MPC solution
							double steer_value = -actuators[0] / (deg2rad(25) * MPC::Lf);
							double throttle_value = actuators[1];

							dataFile << steer_value << "," << throttle_value << "\n";

							json msgJson;
							// NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
							// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
							msgJson["steering_angle"] = steer_value;
							msgJson["throttle"] = throttle_value;

							//Display the MPC predicted trajectory

							//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
							// the points in the simulator are connected by a Green line

							msgJson["mpc_x"] = mpc.mpc_x_vals;
							msgJson["mpc_y"] = mpc.mpc_y_vals;

							//Display the waypoints/reference line

							//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
							// the points in the simulator are connected by a Yellow line

							mpc.next_x_vals.clear();
							mpc.next_y_vals.clear();

							// Quick pick horizon of 15 and space intervals of Lf
							for (int i = 1; i < 15; i++) {
								mpc.next_x_vals.push_back(i*MPC::Lf);
								mpc.next_y_vals.push_back(polyeval(coeffs, i*MPC::Lf));
							}

							msgJson["next_x"] = mpc.next_x_vals;
							msgJson["next_y"] = mpc.next_y_vals;

							auto msg = "42[\"steer\"," + msgJson.dump() + "]";
							//std::cout << msg << std::endl;

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
	    dataFile.close();

	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
	    dataFile.close();

		return -1;
	}
	h.run();
}
