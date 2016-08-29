#pragma once

// function prototypes over in packetparser.cpp
void ble_setup();
bool ble_update();
bool ble_get_color(uint32_t& color);
bool ble_get_button(uint8_t& button, bool& pressed);


