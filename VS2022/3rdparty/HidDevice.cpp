#include "HidDevice.h"
#include <setupapi.h>
#include <hidsdi.h>
#include <vector>
#include "HidTable.h"

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "hid.lib")

HidDevice::HidDevice() : deviceHandle(INVALID_HANDLE_VALUE) {}

HidDevice::~HidDevice() {
    close();
}

bool HidDevice::open(unsigned short vendorID, unsigned short productID, unsigned short targetInterface) {
    // ����Ŀ��ӿڵ��豸·��
    std::wstring devicePath = findDevicePath(vendorID, productID, targetInterface);
    if (devicePath.empty()) {
        std::cerr << "δ�ҵ�Ŀ���豸��" << std::endl;
        return false;
    }

    // ���豸���
    deviceHandle = CreateFile(devicePath.c_str(), GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);

    if (deviceHandle == INVALID_HANDLE_VALUE) {
        std::cerr << "���豸ʧ�ܣ�������: " << GetLastError() << std::endl;
        return false;
    }

    mouse.cmd = 0;
    keyboard.cmd = 1;
    //std::cout << "�ɹ����豸�����: " << deviceHandle << std::endl;
    return true;
}

void HidDevice::close() {
    if (deviceHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(deviceHandle);
        deviceHandle = INVALID_HANDLE_VALUE;
        std::cout << "�豸�ѹرա�" << std::endl;
    }
}


bool HidDevice::mouseMove(int16_t x, int16_t y) {
    mouse.x = x;
    mouse.y = y;

    std::vector<uint8_t> Vector(
        reinterpret_cast<uint8_t*>(&mouse),
        reinterpret_cast<uint8_t*>(&mouse) + sizeof(MouseReport)
    );
    err = write(Vector);
    mouse.x = 0;
    mouse.y = 0;

    return err;
}

bool HidDevice::mouseMoveAuto(int16_t x, int16_t y, int16_t ms) {
    if (ms <= 0) return false;

    int steps = ms; // ÿ1ms����һ�Σ���������ʱ��
    double startX = 0, startY = 0; // ��ʼλ��
    double deltaX = static_cast<double>(x); // X��������ƶ�����
    double deltaY = static_cast<double>(y); // Y��������ƶ�����

    for (int i = 0; i <= steps; ++i) {
        double t = i; // ��ǰʱ�䣨�� ms Ϊ��λ��

        // ���㻺�����λ��
        double currentX = easeInOut(t, startX, deltaX, ms);
        double currentY = easeInOut(t, startY, deltaY, ms);

        // ������Ҫ�ƶ�������
        int16_t moveX = static_cast<int16_t>(std::round(currentX - startX));
        int16_t moveY = static_cast<int16_t>(std::round(currentY - startY));

        // �������Ϊ 0��������
        if (moveX == 0 && moveY == 0) continue;

        // ���� mouseMove ʵ��ʵ�ʵ��ƶ�
        if (!mouseMove(moveX, moveY)) return false;

        // ������ʼλ��
        startX += moveX;
        startY += moveY;

        // ��ʱ 1ms
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return true;
}

bool HidDevice::mouseButton(uint16_t code, uint16_t value) {
    if (value == 1) { // button press
        mouse.buttons |= (1 << code);
    }
    else if (value == 0) { // button release
        mouse.buttons &= ~(1 << code);
    }

    std::vector<uint8_t> Vector(
        reinterpret_cast<uint8_t*>(&mouse),
        reinterpret_cast<uint8_t*>(&mouse) + sizeof(MouseReport)
    );
    err = write(Vector);
    return err;
}

bool HidDevice::tapMouseButton(uint16_t code, int16_t ms) {
    err = mouseButton(code, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    err = mouseButton(code, 0);
    return err;
}

bool HidDevice::keyboardButton(uint16_t code, uint16_t value) {

    // ������Ƽ�
    if (code >= KEY_LEFTCONTROL && code <= KEY_RIGHT_GUI) {
        if (value == 1) { // button press
            keyboard.Control |= (1 << (code - KEY_LEFTCONTROL));
        }
        else if (value == 0) { // button release
            keyboard.Control &= ~(1 << (code - KEY_LEFTCONTROL));
        }
        std::vector<uint8_t> Vector(
            reinterpret_cast<uint8_t*>(&keyboard),
            reinterpret_cast<uint8_t*>(&keyboard) + sizeof(KeyReport)
        );
        err = write(Vector);
        return err;
    }
    // ������ͨ����
    if (value == 1) { // ��������
        bool key_found = false;
        // ���������Ƿ���ڰ���
        for (unsigned char i : keyboard.button) {
            if (i == code) {
                key_found = true;
                break;
            }
        }
        // ��������в����ڰ�������ӵ�����
        if (!key_found) {
            for (unsigned char& i : keyboard.button) {
                if (i == 0) {
                    i = code;
                    key_found = true;
                    break;
                }
            }
        }
        // ��������������Ƴ�����İ���
        if (!key_found) {
            memmove(&keyboard.button[0], &keyboard.button[1], sizeof(keyboard.button) - 1);
            keyboard.button[5] = code;
        }
    }
    else if (value == 0) { // �����ͷ�
     // �Ƴ�����
        for (int i = 0; i < 6; ++i) {
            if (keyboard.button[i] == code) {
                // ��ʣ��İ���ǰ��
                memmove(&keyboard.button[i], &keyboard.button[i + 1], sizeof(keyboard.button) - (i + 1));
                // ������һ��λ��
                keyboard.button[5] = 0;
                break;
            }
        }
    }
    std::vector<uint8_t> Vector(
        reinterpret_cast<uint8_t*>(&keyboard),
        reinterpret_cast<uint8_t*>(&keyboard) + sizeof(KeyReport)
    );
    err = write(Vector);
    return err;
}

bool HidDevice::tapKeyboardButton(uint16_t code, int16_t ms) {
    err = keyboardButton(code, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    err = keyboardButton(code, 0);
    return err;
}

bool HidDevice::Set_PidVid(uint16_t PID, uint16_t VID) {
    PIDReport pid_data;
    pid_data.cmd = 5;
    pid_data.PID = PID;
    pid_data.VID = VID;
    std::vector<uint8_t> Vector(
        reinterpret_cast<uint8_t*>(&pid_data),
        reinterpret_cast<uint8_t*>(&pid_data) + sizeof(PIDReport)
    );
    err = write(Vector);
    return err;
}

bool HidDevice::Firmware_Update(void) {
    std::vector<uint8_t> data = { 0xAB, 0xBA };
    err = write(data);
    return err;
}

bool HidDevice::write(const std::vector<uint8_t>& data) {
    if (deviceHandle == INVALID_HANDLE_VALUE) {
        std::cerr << "�豸δ�򿪣��޷�д�����ݣ�" << std::endl;
        return false;
    }

    // ���ݱ��벹�뵽 64 �ֽ�
    std::vector<uint8_t> buffer(65, 0x00); // ��ʼ��Ϊ 64 �ֽڣ���� 0x00
    std::copy(data.begin(), data.end(), buffer.begin()+1); // �����û�����

    DWORD bytesWritten = 0;
    if (!WriteFile(deviceHandle, buffer.data(), static_cast<DWORD>(buffer.size()), &bytesWritten, nullptr)) {
        std::cerr << "д������ʧ�ܣ�������: " << GetLastError() << std::endl;
        return false;
    }

    //std::cout << "�ɹ�д�� " << bytesWritten << " �ֽ����ݡ�" << std::endl;
    return true;
}

bool HidDevice::read(std::vector<uint8_t>& buffer, size_t length) {
    if (deviceHandle == INVALID_HANDLE_VALUE) {
        std::cerr << "�豸δ�򿪣��޷���ȡ���ݣ�" << std::endl;
        return false;
    }

    buffer.resize(length);
    DWORD bytesRead = 0;
    if (!ReadFile(deviceHandle, buffer.data(), static_cast<DWORD>(length), &bytesRead, nullptr)) {
        std::cerr << "��ȡ����ʧ�ܣ�������: " << GetLastError() << std::endl;
        return false;
    }

    buffer.resize(bytesRead);
    std::cout << "�ɹ���ȡ " << bytesRead << " �ֽ����ݡ�" << std::endl;
    return true;
}

double HidDevice::easeInOut(double t, double b, double c, double d) {
    t /= d / 2.0;
    if (t < 1) return c / 2.0 * t * t + b;
    t--;
    return -c / 2.0 * (t * (t - 2) - 1) + b;
}

std::wstring HidDevice::findDevicePath(unsigned short vendorID, unsigned short productID, unsigned short targetInterface) {
    GUID hidGuid;
    HidD_GetHidGuid(&hidGuid); // ��ȡ HID �豸�� GUID

    HDEVINFO deviceInfoSet = SetupDiGetClassDevs(&hidGuid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (deviceInfoSet == INVALID_HANDLE_VALUE) {
        std::cerr << "��ȡ�豸��Ϣʧ�ܣ�" << std::endl;
        return L"";
    }

    SP_DEVICE_INTERFACE_DATA deviceInterfaceData;
    deviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    // �����豸
    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(deviceInfoSet, nullptr, &hidGuid, i, &deviceInterfaceData); ++i) {
        DWORD requiredSize = 0;
        SetupDiGetDeviceInterfaceDetail(deviceInfoSet, &deviceInterfaceData, nullptr, 0, &requiredSize, nullptr);
        if (requiredSize == 0) continue;

        std::vector<BYTE> detailDataBuffer(requiredSize);
        auto* detailData = reinterpret_cast<PSP_DEVICE_INTERFACE_DETAIL_DATA>(detailDataBuffer.data());
        detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

        if (!SetupDiGetDeviceInterfaceDetail(deviceInfoSet, &deviceInterfaceData, detailData, requiredSize, nullptr, nullptr)) {
            continue;
        }

        HANDLE tempHandle = CreateFile(detailData->DevicePath, GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
        if (tempHandle == INVALID_HANDLE_VALUE) {
            continue;
        }

        // ����豸���ԣ�VID��PID��Interface��
        HIDD_ATTRIBUTES attributes;
        attributes.Size = sizeof(HIDD_ATTRIBUTES);
        if (HidD_GetAttributes(tempHandle, &attributes)) {
            if (attributes.VendorID == vendorID && attributes.ProductID == productID) {
                // ��ȡ�ӿ���Ϣ
                PHIDP_PREPARSED_DATA preparsedData;
                if (HidD_GetPreparsedData(tempHandle, &preparsedData)) {
                    HIDP_CAPS caps;
                    if (HidP_GetCaps(preparsedData, &caps) == HIDP_STATUS_SUCCESS) {
                        if (caps.UsagePage == 0xFF00 && caps.Usage == 0x01 && targetInterface == 2) {
                            CloseHandle(tempHandle);
                            SetupDiDestroyDeviceInfoList(deviceInfoSet);
                            return std::wstring(detailData->DevicePath);
                        }
                    }
                    HidD_FreePreparsedData(preparsedData);
                }
            }
        }
        CloseHandle(tempHandle);
    }

    SetupDiDestroyDeviceInfoList(deviceInfoSet);
    return L"";
}
