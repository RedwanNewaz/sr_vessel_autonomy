//
// Created by redwan on 6/30/23.
//

#ifndef SOCKETCLIENT_THREADEDSOCKETCLIENT_H
#define SOCKETCLIENT_THREADEDSOCKETCLIENT_H

#include <iostream>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <queue>
extern std::queue<std::string> payload;

using namespace std::chrono_literals;

class ThreadedSocketClient {
public:
    ThreadedSocketClient(const std::string& serverIP, int serverPort);

    ~ThreadedSocketClient();

    bool Connect();

    void Disconnect();

    void Send(const std::string& message);

    bool IsConnected() const;

private:
    void ReceiveThread();

    void run();

private:
    std::string serverIP_;
    int serverPort_;
    int socket_;
    struct sockaddr_in serverAddress_;
    std::thread receiveThread_;
    bool isRunning_;
    std::thread thread_;
};


#endif //SOCKETCLIENT_THREADEDSOCKETCLIENT_H
