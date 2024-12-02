import time
import signal
import sys
import oneHid
import event_code #按键头文件

hid = oneHid.HidDevice()

if hid.open(0x52e9, 0x8f0d, 2): #PID VID 默认端口为2
    print('设备打开成功')
else:
    print('设备打开失败')

# for i in range(100):
#     hid.mouseMove(1,1) #立即移动 x的距离 y的距离
#     time.sleep(0.001)

# hid.mouseMoveAuto(100, 100, 10) #自动移动 x的距离 y的距离 多少时间完成

# hid.mouseButton(2, 1) #鼠标按键控制 按键值，1左键 2右键 3中键 4侧键1 5侧键2
# time.sleep(0.01)
# hid.mouseButton(2, 0)

# hid.tapMouseButton(2, 50) #鼠标按键按下50ms后释放

# hid.keyboardButton(event_code.KEY_A, 1)
# time.sleep(0.01)
# hid.keyboardButton(event_code.KEY_A, 0)

# hid.tapKeyboardButton(event_code.KEY_A, 50) #键盘按键按下50ms后释放

def signal_handler(sig, frame):
    print('捕获到 Ctrl+C,正在关闭...')
    hid.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)







