#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class SpaceKeyPublisher : public rclcpp::Node {
public:
    SpaceKeyPublisher() : Node("space_key_publisher"), is_space_pressed(false) {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("space_key", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SpaceKeyPublisher::publish_space_key, this));
    }

private:
    void publish_space_key() {
        auto message = std_msgs::msg::Bool();
        // スペースキーが押されたかどうかを確認
        if (check_space_key_pressed()) {
            if (!is_space_pressed) {
                message.data = true;
                is_space_pressed = true;
            } else {
                message.data = false;
            }
        } else {
            message.data = false;
            is_space_pressed = false;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data ? "true" : "false");
        publisher_->publish(message);
    }

    bool check_space_key_pressed() {
        // termiosで入力を非同期モードに設定
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);  // 現在のターミナル設定を保存
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);  // 非カノニカルモードとエコーを無効化
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        bool is_pressed = false;
        if (kbhit()) {
            char ch = getchar();
            if (ch == ' ') {
                is_pressed = true;
            }
        }

        // 元の設定に戻す
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return is_pressed;
    }

    int kbhit() {
        // 標準入力のファイル記述子の確認用
        struct timeval tv = {0L, 0L};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    }
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_space_pressed;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpaceKeyPublisher>());
    rclcpp::shutdown();
    return 0;
}
