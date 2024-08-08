#include "ros/ros.h"
#include <array>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <netinet/in.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <thread>
namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using udp = net::ip::udp;

union u {
  uint32_t val;
  float f;
};

bool verbose = false;

const std::string HOST = "172.20.10.9";
const std::string HTTP_PORT = "8080";
const std::string RTSP_PORT = "8086";
const std::string HTTP_TARGET = "/api/status";
const std::string RTSP_TARGET = "/?camera=gaze";
const int HTTP_VERSION = 11;
const int MAX_PACKET_SIZE = 1500;

// float ntohf(float netFloat) {
//   uint32_t netInt;
//   memcpy(&netInt, &netFloat, sizeof(netInt));
//   uint32_t hostInt = ntohl(netInt);
//   float hostFloat;
//   memcpy(&hostFloat, &hostInt, sizeof(hostFloat));
//   return hostFloat;
// }

// struct GazeData {
//   float x;
//   float y;
//   bool worn;
//   float pupil_diameter_left;
//   float eyball_center_left_x;
//   float eyball_center_left_y;
//   float eyball_center_left_z;
//   float optical_axis_left_x;
//   float optical_axis_left_y;
//   float optical_axis_left_z;
//   float pupil_diameter_right;
//   float eyball_center_right_x;
//   float eyball_center_right_y;
//   float eyball_center_right_z;
//   float optical_axis_right_x;
//   float optical_axis_right_y;
//   float optical_axis_right_z;
//   float timestamp_unix_sec;
// };

struct ClockSync {
  uint64_t ntp_timestamp;
  uint32_t rtp_timestamp;
  double clock_offset;
};
ClockSync clock_sync;

std::string perform_http_request(net::io_context &ioc) {
  tcp::resolver resolver(ioc);
  beast::tcp_stream stream(ioc);

  auto const results = resolver.resolve(HOST, HTTP_PORT);
  stream.connect(results);

  http::request<http::string_body> req{http::verb::get, HTTP_TARGET,
                                       HTTP_VERSION};
  req.set(http::field::host, HOST);
  req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

  http::write(stream, req);

  beast::flat_buffer buffer;
  http::response<http::dynamic_body> res;
  http::read(stream, buffer, res);

  std::string response_body = beast::buffers_to_string(res.body().data());

  beast::error_code ec;
  stream.socket().shutdown(tcp::socket::shutdown_both, ec);
  if (ec && ec != beast::errc::not_connected)
    throw beast::system_error{ec};

  return response_body;
}

uint64_t rtp_to_system_time(uint32_t rtp_timestamp) {
  if (clock_sync.rtp_timestamp == 0)
    return 0; // Not synchronized yet

  int32_t delta = rtp_timestamp - clock_sync.rtp_timestamp;
  double time_diff =
      delta / 90000.0; // Assuming 90kHz clock rate, adjust if different

  return clock_sync.ntp_timestamp + static_cast<uint64_t>(time_diff * 1e9) +
         clock_sync.clock_offset;
}

std::string send_rtsp_request(tcp::socket &socket, const std::string &request) {
  net::write(socket, net::buffer(request));
  boost::asio::streambuf response;
  boost::asio::read_until(socket, response, "\r\n\r\n");
  return boost::asio::buffer_cast<const char *>(response.data());
}

void setup_and_play_rtsp_stream(net::io_context &ioc, tcp::socket &rtsp_socket,
                                udp::socket &rtp_socket,
                                udp::socket &rtcp_socket,
                                udp::endpoint &rtp_endpoint,
                                udp::endpoint &rtcp_endpoint) {
  tcp::resolver resolver(ioc);
  auto endpoints = resolver.resolve(HOST, RTSP_PORT);
  net::connect(rtsp_socket, endpoints);

  // Bind the RTP and RTCP sockets
  udp::endpoint listen_endpoint(udp::v4(), 0);
  rtp_socket.open(udp::v4());
  rtp_socket.bind(listen_endpoint);
  int client_rtp_port = rtp_socket.local_endpoint().port();

  rtcp_socket.open(udp::v4());
  rtcp_socket.bind(udp::endpoint(udp::v4(), client_rtp_port + 1));
  int client_rtcp_port = rtcp_socket.local_endpoint().port();

  // DESCRIBE
  std::string dersribe_response = send_rtsp_request(
      rtsp_socket, "DESCRIBE rtsp://" + HOST + ":" + RTSP_PORT + RTSP_TARGET +
                       " RTSP/1.0\r\nCSeq: 1\r\n\r\n");

#ifdef DEBUG
  std::cout << "Describe response:\n" << dersribe_response << std::endl;
#endif

  // SETUP
  std::string setup_request = "SETUP rtsp://" + HOST + ":" + RTSP_PORT +
                              RTSP_TARGET + " RTSP/1.0\r\nCSeq: 2\r\n" +
                              "Transport: RTP/AVP;unicast;client_port=" +
                              std::to_string(client_rtp_port) + "-" +
                              std::to_string(client_rtcp_port) + "\r\n\r\n";
  std::string setup_response = send_rtsp_request(rtsp_socket, setup_request);

#ifdef DEBUG
  std::cout << "Setup response:\n" << setup_response << std::endl;
#endif

  std::string server_rtp_port_str;
  std::string server_rtcp_port_str;
  std::string session_id;
  // parse
  size_t pos = setup_response.find("server_port=");
  if (pos != std::string::npos) {
    pos += 12;
    while (setup_response[pos] != '-')
      server_rtp_port_str += setup_response[pos++];
    pos++;
    while (setup_response[pos] != ';')
      server_rtcp_port_str += setup_response[pos++];
  } else {
    std::cerr << "Failed to parse server ports from SETUP response"
              << std::endl;
    return;
  }
#ifdef DEBUG
  std::cout << "Server RTP port: " << server_rtp_port_str << std::endl;
  std::cout << "Server RTCP port: " << server_rtcp_port_str << std::endl;
#endif
  rtcp_endpoint = udp::endpoint(net::ip::make_address(HOST),
                                std::stoi(server_rtcp_port_str));
  rtp_endpoint = udp::endpoint(net::ip::make_address(HOST),
                               std::stoi(server_rtp_port_str));

  // parse session id
  pos = setup_response.find("Session: ");
  if (pos != std::string::npos) {
    pos += 9;
    while (setup_response[pos] != '\r')
      session_id += setup_response[pos++];
  } else {
    std::cerr << "Failed to parse session id from SETUP response" << std::endl;
    return;
  }
#ifdef DEBUG
  std::cout << "Session ID: " << session_id << std::endl;
#endif

  send_rtsp_request(rtsp_socket,
                    "PLAY rtsp://" + HOST + ":" + RTSP_PORT + RTSP_TARGET +
                        " RTSP/1.0\r\nCSeq: 3\r\nSession:" + session_id +
                        "\r\n\r\n");
}

void process_rtcp_packet(const char *data, size_t length) {
  if (data[1] != 200)
    return; // 200 is OK

  uint64_t ntp_timestamp =
      be64toh(*reinterpret_cast<const uint64_t *>(data + 8));
  uint32_t rtp_timestamp =
      be32toh(*reinterpret_cast<const uint32_t *>(data + 16));

  constexpr uint64_t NTP_EPOCH_OFFSET = 2208988800ULL;
  uint64_t ntp_to_unix = (ntp_timestamp >> 32) - NTP_EPOCH_OFFSET;
  uint64_t ntp_frac = (ntp_timestamp & 0xFFFFFFFF);
  uint64_t unix_ns =
      ntp_to_unix * 1000000000ULL + (ntp_frac * 1000000000ULL >> 32);

  double current_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
  clock_sync.clock_offset = current_time - unix_ns;
  clock_sync.ntp_timestamp = unix_ns;
  clock_sync.rtp_timestamp = rtp_timestamp;
  std::cout << "Clock sync updated. Offset: " << clock_sync.clock_offset
            << " ns" << std::endl;
}

void process_rtp_packet(const char *data, size_t length, ros::Publisher &pub) {
  float pupil_diameter_left = u{ntohl(*((uint32_t *)(data + 21)))}.f;
  float pupil_diameter_right = u{ntohl(*((uint32_t *)(data + 49)))}.f;

  std_msgs::Float32MultiArray msg;
  msg.data.resize(2);

  msg.data[0] = u{ntohl(*((uint32_t *)(data + 21)))}.f; // pupil_diameter_left
  msg.data[1] = u{ntohl(*((uint32_t *)(data + 49)))}.f; // pupil_diameter_right
  pub.publish(msg);

#ifdef DEBUG
  std::cout << "Pupil diameter left: " << pupil_diameter_left << std::endl;
  std::cout << "Pupil diameter right: " << pupil_diameter_right << std::endl;
#endif
}

void read_rtp_stream(udp::socket &socket, udp::endpoint &sender_endpoint,
                     ros::Publisher &pub) {
  std::array<char, MAX_PACKET_SIZE> data;
  int packet_count = 0;
  auto start_time = std::chrono::steady_clock::now();

  while (ros::ok()) {
    boost::system::error_code error;
    size_t length = socket.receive_from(boost::asio::buffer(data),
                                        sender_endpoint, 0, error);

    if (error) {
      if (error == boost::asio::error::connection_refused) {
        ROS_WARN("Connection refused, the stream may have ended.");
        break;
      }
      throw boost::system::system_error(error);
    }

    process_rtp_packet(data.data(), length, pub);
    packet_count++;

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       current_time - start_time)
                       .count();
    if (verbose) {
      std::cout << "Packets received: " << packet_count << std::endl;
      std::cout << "Time elapsed: " << elapsed << " seconds" << std::endl;
      std::cout << "Average packet rate: "
                << (elapsed > 0 ? static_cast<double>(packet_count) / elapsed
                                : 0)
                << " packets/second" << std::endl;
    }
  }
}

void read_rtcp_stream(udp::socket &rtcp_socket,
                      udp::endpoint &sender_endpoint) {
  std::array<char, MAX_PACKET_SIZE> data;

  while (ros::ok()) {
    size_t length =
        rtcp_socket.receive_from(boost::asio::buffer(data), sender_endpoint);
    process_rtcp_packet(data.data(), length);
  }
}

void print_json(const std::string &json) {
  for (size_t i = 0; i < json.size(); ++i) {
    std::cout << json[i];
    if (json[i] == '{' || json[i] == '[') {
      std::cout << std::endl;
    } else if (json[i] == '}' || json[i] == ']') {
      std::cout << std::endl;
    } else if (json[i] == ',') {
      std::cout << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  assert(sizeof(float) == sizeof(uint32_t));
  if (argc > 1 && std::string(argv[1]) == "-v") {
    verbose = true;
  }
  ros::init(argc, argv, "gaze_data_publisher");
  ros::NodeHandle nh;
  ROS_INFO("pupil_size_publisher node started");

  ros::Publisher pupil_pub =
      nh.advertise<std_msgs::Float32MultiArray>("pupil_sizes", 2);
  try {
    net::io_context ioc;

#ifdef DEBUG
    std::string http_response = perform_http_request(ioc);
    std::cout << "HTTP Response:\n" << std::endl;
    print_json(http_response);
#endif

    // while (ros::ok()) {
    //   std_msgs::Float32MultiArray msg;
    //   msg.data.resize(2);
    //   msg.data[0] = 1.0;
    //   msg.data[1] = 1.0;
    //   pupil_pub.publish(msg);
    //   if (verbose) {
    //     std::cout << "Pupil sizes published" << std::endl;
    //   }
    //   ros::spinOnce();
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    tcp::socket rtsp_socket(ioc);
    udp::socket rtp_socket(ioc);
    udp::socket rtcp_socket(ioc);
    // rtp_socket.set_option(net::detail::socket_option::integer<SOL_SOCKET,
                                                           // SO_RCVTIMEO>{1000});
    // rtcp_socket.set_option(net::detail::socket_option::integer<SOL_SOCKET,
                                                            // SO_RCVTIMEO>{1000});
    // rtsp_socket.set_option(net::detail::socket_option::integer<SOL_SOCKET,
                                                            // SO_RCVTIMEO>{1000});
    udp::endpoint rtp_endpoint;
    udp::endpoint rtcp_endpoint;
    setup_and_play_rtsp_stream(ioc, rtsp_socket, rtp_socket, rtcp_socket,
                               rtp_endpoint, rtcp_endpoint);
    std::thread rtp_thread([&rtp_socket, &rtp_endpoint, &pupil_pub] {
      read_rtp_stream(rtp_socket, rtp_endpoint, pupil_pub);
    });
    std::thread rtcp_thread([&rtcp_socket, &rtcp_endpoint] {
      read_rtcp_stream(rtcp_socket, rtcp_endpoint);
    });
    ros::spin();
    rtp_thread.join();
    rtcp_thread.join();

  } catch (std::exception const &e) {
    ROS_ERROR("Error: %s", e.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
