#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("callback"), package_(false) {
        subscription_ = this->create_subscription<std_msgs::msg::String>("feedback", 10, std::bind(&Subscriber::feedback, this, _1));
    }
    bool package_;
private:
    void feedback(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), msg->data.c_str());
        std::string done = "Done";
        if (msg->data == done) {
            this->package_ = true;
        }
        else {
            this->package_ = false;
        }
        std::cout << "feedback done";
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class Publisher : public rclcpp::Node {
public:
    Publisher() : Node("user_data") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("send_data", 10);
    }

    // x - координата рабочего инструмента по оси x
    // y - координата рабочего инструмента по оси y
    // z - координата рабочего инструмента по оси z
    // eef - желаемое состояние рабочего инструмента (1 - активное, 0 - неактивное)
    void send_data(int x, int y, int z, int eef) {
        auto message = std_msgs::msg::String();
        std::string p = "p:";
        std::string symb = ":";
        std::string cage = "#";
        message.data = p + std::to_string(x) + symb + std::to_string(y) + symb + std::to_string(z) + symb + std::to_string(eef) + cage;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        std::cout << "Data send";
        RCLCPP_INFO(this->get_logger(), "Data send");
    }
    void connect() {
        auto message = std_msgs::msg::String();
        message.data = "Connect";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        std::cout << "Connect done";
        RCLCPP_INFO(this->get_logger(), "Connect done");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

bool manipulators_work() {
    auto publisher = std::make_shared<Publisher>();
    auto subscriber = std::make_shared<Subscriber>();

    int choice;

    std::cout << "1. Создать соединение\n2.Ввести координаты\n";
    std::cin >> choice;

    switch (choice) {
        case 1: {
            publisher->connect();
            break;
        }
        case 2: {
            std::cout << "Введите координаты x, y, z и eef(0 или 1) - режим рабочего инструмента.\nВведите -1 -1 -1 -1, чтобы выйти из программы" << std::endl;
            int x, y, z, eef;
            std::cin >> x >> y >> z >> eef;

            if (eef == -1) {
                return false;
            }

            if (x > 0 and x < 300 and y > -300 and y < 300 and z > 160 and z < 290) {
                publisher->send_data(x, y, z, eef);
            }
            break;
        } 
    }

    std::cout << "Manipulators work done\n";
    return true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    while (manipulators_work());
    
    rclcpp::shutdown();
    return 0;
}