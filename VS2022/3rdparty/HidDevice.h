#ifndef HIDDEVICE_H
#define HIDDEVICE_H

#include <windows.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdint.h>
#include <cmath>
#include <cstdint>
#include <thread>

typedef struct
{
    uint8_t cmd;
    uint8_t buttons;
    int16_t x;
    int16_t y;
    int8_t roller;
} MouseReport;

typedef struct
{
    uint8_t cmd;
    uint8_t Control;
    uint8_t Undefined;
    uint8_t button[6];
} KeyReport;

typedef struct
{
    uint8_t cmd;
    uint16_t PID;
    uint16_t VID;
} PIDReport;

class HidDevice {
public:
    HidDevice();
    ~HidDevice();

    // 打开设备，指定 VID, PID 和接口号
    bool open(unsigned short vendorID, unsigned short productID, unsigned short targetInterface = 2);
    // 关闭设备
    void close();

    /**
    * 鼠标移动
    * @param x 正值向右
    * @param y 正值向下
    * @return ErrorCode
    */
    bool mouseMove(int16_t x, int16_t y);

    /**
     * 鼠标算法优化移动
     * @param x 正值向下
     * @param y 正值向右
     * @param ms 移动时间
     * @return ErrorCode
     */
    bool mouseMoveAuto(int16_t x, int16_t y, int16_t ms);

    /**
     * 鼠标按键触发
     * @param code 按键值，1左键 2右键 3中键 4侧键1 5侧键2
     * @param value 按下1 释放0
     * @return ErrorCode
     */
    bool  mouseButton(uint16_t code, uint16_t value);

    /**
     * 鼠标按下多少ms后释放
     * @param code 按键值，1左键 2右键 3中键 4侧键1 5侧键2
     * @param ms 毫秒
     * @return ErrorCode
     */
    bool tapMouseButton(uint16_t code, int16_t ms);

    /**
     * 键盘按键触发
     * @param code 按键值，参考event-codes
     * @param value 按下1 释放0
     * @return ErrorCode
     */
    bool keyboardButton(uint16_t code, uint16_t value);

    /**
     * 键盘按键按下多少ms后释放
     * @param code 按键值，参考event-codes
     * @param ms 毫秒
     * @return ErrorCode
     */
    bool tapKeyboardButton(uint16_t code, int16_t ms);

    /**
     * 设置PID VID 设置完成后重启生效
     * @return ErrorCode
     */
    bool Set_PidVid(uint16_t PID, uint16_t VID);

    /**
     * 进入固件更新模式
     * @return ErrorCode
     */
    bool Firmware_Update(void);

private:
    HANDLE deviceHandle;
    bool err;
    MouseReport mouse = {};
    KeyReport keyboard = {};

    // 写入数据到设备
    bool write(const std::vector<uint8_t>& data);
    // 从设备读取数据
    bool read(std::vector<uint8_t>& buffer, size_t length);
    // 查找指定接口的设备路径
    std::wstring findDevicePath(unsigned short vendorID, unsigned short productID, unsigned short targetInterface);
    // 缓动函数
    double easeInOut(double t, double b, double c, double d);
};

#endif // HIDDEVICE_H
