#pragma once

#include <cstring>
#include <string>
#include <iostream>

#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <unistd.h>


class ServoDriver {
public:
  ServoDriver(std::string name, int can_id) : can_id_(can_id) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (ifr.ifr_ifindex == 0) {
      std::cout << "can device not found" << std::endl;
      return;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int res = bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (res != 0) {
      std::cout << "bind error: " << strerror(errno) << std::endl; 
      return;
    }

    int val = 1;
    ioctl(sock_, FIONBIO, &val);
  }

  void write(double angle, int16_t motor_vel) {
    if (sock_ == -1) return;

    struct can_frame frame;
    frame.can_id = can_id_;
    frame.len = sizeof(ApsControl);

    ApsControl msg;
    msg.motor_command = angle / 180 * 255;
    msg.servo_command = motor_vel;

    memcpy(frame.data, &msg, sizeof(ApsControl));

    int ret = ::write(sock_, &frame, sizeof(frame));
    (void)ret;
  }

  struct ApsControl {
    int16_t motor_command; // モーターの指令値 255を掛けてモータードライバに送る
    uint8_t servo_command; // サーボモータの指令値　サーボモータードライバで用いられるものと同じフォーマット
  } __attribute__((packed));

private:
  int sock_ = -1;
  int can_id_;
};