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

    // ���豸��ָ�� VID, PID �ͽӿں�
    bool open(unsigned short vendorID, unsigned short productID, unsigned short targetInterface = 2);
    // �ر��豸
    void close();

    /**
    * ����ƶ�
    * @param x ��ֵ����
    * @param y ��ֵ����
    * @return ErrorCode
    */
    bool mouseMove(int16_t x, int16_t y);

    /**
     * ����㷨�Ż��ƶ�
     * @param x ��ֵ����
     * @param y ��ֵ����
     * @param ms �ƶ�ʱ��
     * @return ErrorCode
     */
    bool mouseMoveAuto(int16_t x, int16_t y, int16_t ms);

    /**
     * ��갴������
     * @param code ����ֵ��1��� 2�Ҽ� 3�м� 4���1 5���2
     * @param value ����1 �ͷ�0
     * @return ErrorCode
     */
    bool  mouseButton(uint16_t code, uint16_t value);

    /**
     * ��갴�¶���ms���ͷ�
     * @param code ����ֵ��1��� 2�Ҽ� 3�м� 4���1 5���2
     * @param ms ����
     * @return ErrorCode
     */
    bool tapMouseButton(uint16_t code, int16_t ms);

    /**
     * ���̰�������
     * @param code ����ֵ���ο�event-codes
     * @param value ����1 �ͷ�0
     * @return ErrorCode
     */
    bool keyboardButton(uint16_t code, uint16_t value);

    /**
     * ���̰������¶���ms���ͷ�
     * @param code ����ֵ���ο�event-codes
     * @param ms ����
     * @return ErrorCode
     */
    bool tapKeyboardButton(uint16_t code, int16_t ms);

    /**
     * ����PID VID ������ɺ�������Ч
     * @return ErrorCode
     */
    bool Set_PidVid(uint16_t PID, uint16_t VID);

    /**
     * ����̼�����ģʽ
     * @return ErrorCode
     */
    bool Firmware_Update(void);

private:
    HANDLE deviceHandle;
    bool err;
    MouseReport mouse = {};
    KeyReport keyboard = {};

    // д�����ݵ��豸
    bool write(const std::vector<uint8_t>& data);
    // ���豸��ȡ����
    bool read(std::vector<uint8_t>& buffer, size_t length);
    // ����ָ���ӿڵ��豸·��
    std::wstring findDevicePath(unsigned short vendorID, unsigned short productID, unsigned short targetInterface);
    // ��������
    double easeInOut(double t, double b, double c, double d);
};

#endif // HIDDEVICE_H
