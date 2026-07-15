#pragma once
#include <Arduino.h>

// ====== MQTT (bez TLS) ======
constexpr const char* MQTT_HOST = "homeassistant.local";
constexpr uint16_t MQTT_PORT = 1883;

constexpr const char* MQTT_USER = HA_MQTT_USER;
constexpr const char* MQTT_PASS = HA_MQTT_PASS;

constexpr const char* MQTT_CLIENT_ID = "ws_esp32_p4_nano_test"; // ustaw unikalne

// ====== TOPICS ======
constexpr const char* TOPIC_RESTART_COUNTER = "test/restarts";
constexpr const char* TOPIC_TIMESTAMP = "test/timestamp";

constexpr uint32_t TIMESTAMP_PUBLISH_INTERVAL_MS = 1000;
