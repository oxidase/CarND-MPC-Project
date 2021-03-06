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

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != std::string::npos)
    {
        return std::string();
    } else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return std::string();
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
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
                    std::string sdata = std::string(data).substr(0, length);
                    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
                    {
                        std::string s = hasData(sdata);
                        if (s != "")
                        {
                            auto j = json::parse(s);
                            std::string event = j[0].get<std::string>();
                            if (event == "telemetry")
                            {
                                // j[1] is the data JSON object
                                std::vector<double> ptsx = j[1]["ptsx"];
                                std::vector<double> ptsy = j[1]["ptsy"];
                                double px = j[1]["x"];
                                double py = j[1]["y"];
                                double psi = j[1]["psi"]; // in rad
                                double v = static_cast<double>(j[1]["speed"]) * 1609.34 / 3600.; // in m/s

                                std::vector<double> x_vals(ptsx.size()), y_vals(ptsy.size());
                                Eigen::VectorXd vx_vals(ptsx.size()), vy_vals(ptsy.size());
                                for (std::size_t i = 0; i < ptsx.size(); ++i)
                                {
                                    double x = ptsx[i] - px, y = ptsy[i] - py;
                                    vx_vals[i] = x_vals[i] =  x * std::cos(-psi) - y * std::sin(-psi);
                                    vy_vals[i] = y_vals[i] =  x * std::sin(-psi) + y * std::cos(-psi);
                                }

                                auto coeffs = polyfit(vx_vals, vy_vals, 3);
                                auto cte = polyeval(coeffs, 0.);
                                auto epsi = -atanf(coeffs[1]);

                                // state in car coordniates
                                Eigen::VectorXd state(6);
                                state << 0., 0., 0., v, cte, epsi;

                                double steer_value;
                                double throttle_value;
                                double cost;
                                double ref_v;
                                std::vector<double> mpc_x_vals, mpc_y_vals;

                                std::tie(steer_value, throttle_value, mpc_x_vals, mpc_y_vals, cost, ref_v) =
                                    mpc.Solve(state, coeffs, x_vals.front(), x_vals.back());

                                json msgJson;
                                // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                                // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                                msgJson["steering_angle"] = -steer_value / deg2rad(25);
                                msgJson["throttle"] = throttle_value;

                                //Display the MPC predicted trajectory
                                //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                                // the points in the simulator are connected by a Green line

                                msgJson["mpc_x"] = mpc_x_vals;
                                msgJson["mpc_y"] = mpc_y_vals;

                                //Display the waypoints/reference line
                                //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                                // the points in the simulator are connected by a Yellow line
                                msgJson["next_x"] = x_vals;
                                msgJson["next_y"] = y_vals;

                                std::cout << px << " " << py << " " << psi << " " << v << " "  << cost
                                          << " " << steer_value << " " << throttle_value << " " << ref_v
                                          << std::endl;

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
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
                       std::cerr << "Connected!!!" << std::endl;
                   });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
                          ws.close();
                          std::cerr << "Disconnected" << std::endl;
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
