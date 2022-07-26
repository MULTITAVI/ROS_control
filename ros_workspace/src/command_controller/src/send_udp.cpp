#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

typedef struct sockaddr *saddrp;

using std::placeholders::_1;
using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
public:
    Publisher() : Node("Feedback_sent") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("feedback", 10);
    }

    void send_done() {
        auto message = std_msgs::msg::String();
        message.data = "Done";
        publisher_->publish(message);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("UDP_sent") {
        subscription_ = this->create_subscription<std_msgs::msg::String>("send_data", 10, std::bind(&Subscriber::send_UDP, this, _1));
        publisher_ = std::make_shared<Publisher>();
    }
private:
    void send_UDP(std_msgs::msg::String::SharedPtr msg) {
        
        std::string con = "Connect";

        if (msg->data == con) {
            this->create_UDP();
            publisher_->send_done();
        }
        else {
            this->send_command(msg);
            publisher_->send_done();
        }
        std::cout << "send UDP done";
        RCLCPP_INFO(this->get_logger(), "send UDP done");
    }

    void send_command(std_msgs::msg::String::SharedPtr msg) {
        std::string ip = "192.168.1.21";
        int port = 8888;
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        if (sockfd < 0) {
            perror("Socket err");
        }

        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        sendto(sockfd, msg->data.c_str(), msg->data.size() + 1, 0, (saddrp)&addr, sizeof(addr));
        std::cout << "send command done";
        RCLCPP_INFO(this->get_logger(), "send command done");
    }

    void create_UDP() {
        std::string ip = "192.168.1.21";
        int port = 8888;
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        std::string start = "r";

        if (sockfd < 0) {
            perror("Socket err");
        }

        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        sendto(sockfd, start.c_str(), start.size() + 1, 0, (saddrp)&addr, sizeof(addr));
        
        if (errno) {
            RCLCPP_INFO(this->get_logger(), strerror(errno));
            return;    
        }

        std::cout << "create UDP done";
        RCLCPP_INFO(this->get_logger(), "create UDP done");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<Publisher> publisher_;
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    
    return 0;
}