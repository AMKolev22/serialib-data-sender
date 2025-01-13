#pragma once
#define SERVER_PORT 8000
#define ASIO_STANDALONE
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#define _WEBSOCKETPP_CPP11_CHRONO_
#define _WEBSOCKETPP_NOEXCEPT_

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <iostream>


using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;


//class Network {
//
//public:
//
//    websocketpp::server<websocketpp::config::asio> server;
//    uint16_t serverPort = SERVER_PORT;
//
//
//    void runServer() {
//        this->server.run();
//    }
//
//    void setServerPort(const uint16_t& port) {
//        this->serverPort = port;
//    }
//
//    void initServer() {
//        this->server.listen(this->serverPort);
//        this->server.start_accept();
//    }
//
//    void stopServer() {
//        this->server.stop_listening();
//    }
//
//    auto channelConfig() {
//        this->server.set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_header);
//        this->server.clear_access_channels(websocketpp::log::alevel::frame_header);
//    }
//
//
//
//};