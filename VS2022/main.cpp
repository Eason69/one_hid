#include <iostream>
#include "3rdparty/HidDevice.h"
#include "3rdparty/HidTable.h"

int main() {
    HidDevice device;

    // 打开设备，指定 VID, PID
    if (device.open(0x52e9, 0x8f0d));
    else 
        std::cerr << "设备打开失败！" << std::endl;

    for (int i = 0; i < 100; ++i) {
        device.mouseMove(1,1);
        Sleep(0.1);
    }

    //device.mouseMoveAuto(100, 100, 50);

    //device.mouseButton(2, 1);
    //Sleep(0.05);
    //device.mouseButton(2, 0);

    //device.tapMouseButton(2, 50);

    //device.keyboardButton(KEY_A, 1);
    //Sleep(0.05);
    //device.keyboardButton(KEY_A, 0);

    //device.tapKeyboardButton(KEY_A, 50);

    return 0;
}