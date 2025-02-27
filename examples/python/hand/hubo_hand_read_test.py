from libpcanbasic.pcanbasic.include.PCANBasic import *
import ctypes
import time

pcan = PCANBasic()
CHANNEL = PCAN_USBBUS1
BITRATE = PCAN_BAUD_1M

status = pcan.Initialize(CHANNEL, BITRATE)
if status != PCAN_ERROR_OK:
    print("PCAN 초기화 실패:", pcan.GetErrorText(status))
    exit()



# 에러 프레임 허용
status = pcan.SetValue(CHANNEL, PCAN_ALLOW_ERROR_FRAMES, 1)
print(f"error frame: {pcan.GetErrorText(status)}")
if status != PCAN_ERROR_OK:
    print("에러 프레임 설정 실패:", pcan.GetErrorText(status))
else:
    print("에러 프레임 허용 설정 완료!")

# clock_value = 24  # 예시로 24MHz 설정
# status = pcan.SetValue(CHANNEL, PCAN_BR_CLOCK_MHZ, clock_value)
# print("Clock:", pcan.GetErrorText(status))


ERR_FRAME_MASK = 0x40  # 보통 PCAN_MESSAGE_ERRFRAME = 0x40
print("📡 CAN 메시지(에러 프레임 포함) 수신 대기 중... (Ctrl+C로 종료)")

try:
    while True:
        result, msg, timestamp = pcan.Read(CHANNEL)
        if result == PCAN_ERROR_OK:
            # MSGTYPE(c_ubyte) & ERR_FRAME_MASK(int)
            if (int(msg.MSGTYPE) & ERR_FRAME_MASK) == ERR_FRAME_MASK:
                print(f"⚠️ Error Frame Detected! Data: {msg.DATA[:msg.LEN]}")
            else:
                print(f"ID: {hex(msg.ID)}, DATA: {msg.DATA[:msg.LEN]}, DLC: {msg.LEN}")
        time.sleep(0.01)
except KeyboardInterrupt:
    pass



pcan.Uninitialize(CHANNEL)
print("🛑 PCAN 채널 해제 완료")

