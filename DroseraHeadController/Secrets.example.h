#pragma once

#include <Arduino.h>
#include <IPAddress.h>

// Copy this file to Secrets.h and fill in your own local values.
#define WIFI_SSID     "your-network"
#define WIFI_PASSWORD "your-password"

inline constexpr char ROBOT_NAME[] = "robot-name";
inline constexpr uint16_t LOCAL_PORT = 8888;
inline constexpr uint16_t MAX_PORT = 9999;

extern IPAddress maxIP;
