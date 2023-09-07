#include "ThreadedSocketClient.h"

ThreadedSocketClient::ThreadedSocketClient(const std::string& serverIP, int serverPort, GPSDataPtr gps)
        : serverIP_(serverIP), serverPort_(serverPort), isRunning_(false), thread_(), gps_(gps) {
    std::memset(&serverAddress_, 0, sizeof(serverAddress_));
    serverAddress_.sin_family = AF_INET;
    serverAddress_.sin_port = htons(serverPort_);
    inet_pton(AF_INET, serverIP_.c_str(), &(serverAddress_.sin_addr));
    thread_ = std::thread(&ThreadedSocketClient::run, this);
}

ThreadedSocketClient::~ThreadedSocketClient() {
    Disconnect();
    if(thread_.joinable())
        thread_.join();
}

bool ThreadedSocketClient::Connect() {
    if (isRunning_) {
        std::cout << "Client already connected." << std::endl;
        return false;
    }

    // Create socket
    socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_ == -1) {
        std::cout << "Failed to create socket." << std::endl;
        return false;
    }

    // Connect to server
    if (connect(socket_, (struct sockaddr*)&serverAddress_, sizeof(serverAddress_)) < 0) {
        std::cout << "Failed to connect to server." << std::endl;
        return false;
    }

    // Start the receive thread
    isRunning_ = true;
    receiveThread_ = std::thread(&ThreadedSocketClient::ReceiveThread, this);

    return true;
}

void ThreadedSocketClient::Disconnect() {
    if (!isRunning_) {
        return;
    }

    // Stop the receive thread
    isRunning_ = false;
    if (receiveThread_.joinable()) {
        receiveThread_.join();
    }

    // Close socket
    close(socket_);
    socket_ = -1;
}

void ThreadedSocketClient::Send(const std::string& message) {
    if (!isRunning_) {
        std::cout << "Client not connected." << std::endl;
        return;
    }

    send(socket_, message.c_str(), message.length(), 0);
}

bool ThreadedSocketClient::IsConnected() const {
    return isRunning_;
}


void ThreadedSocketClient::ReceiveThread() {
    while (isRunning_) {
        char buffer[1024];
        std::memset(buffer, 0, sizeof(buffer));

        int bytesRead = recv(socket_, buffer, sizeof(buffer) - 1, 0);
        if (bytesRead <= 0) {
            // Error or connection closed
            isRunning_ = false;
            break;
        }

        // std::cout << "Received: " << buffer << std::endl;
        gps_->update_data(buffer);
        
 

    }
}

void ThreadedSocketClient::run() {

    if (Connect()) {
        while (IsConnected()) {
            // Do other tasks while the client is connected
            std::string input;

            if(!payload_.empty())
            {
                Send(payload_.front());
                payload_.pop();
            }
//            std::cout << "[input:] ";
//            std::getline(std::cin, input);

            std::this_thread::sleep_for(1ms);
        }
    }

    std::cout << "client disconnected \n";
    delete this;



}

void ThreadedSocketClient::addPayload(const std::string &message) {
    payload_.push(message);
}
