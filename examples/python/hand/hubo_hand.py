import time
import threading
from enum import Enum
from libpcanbasic.pcanbasic.include.PCANBasic import *  # PCAN-Basic Python 래퍼

class FingerAction(Enum):
    OPEN = 0x3C
    CLOSE = 0xBC
    HOLD = 0x00

class HandController:
    def __init__(self, channel=PCAN_USBBUS1, bitrate=PCAN_BAUD_1M):
        self.pcan = PCANBasic()
        self.channel = channel
        
        status = self.pcan.Initialize(channel, bitrate)
        if status != PCAN_ERROR_OK:
            raise RuntimeError(f"PCAN 초기화 실패")

        print("✅ PCAN 초기화 성공")
    
    def close(self):
        self.pcan.Uninitialize(self.channel)
        print("🛑 PCAN 해제 완료")

    def send_actions(self, can_id, actions, duration=1.0, freq=500):
        """단일 단계 송신 (동기)"""
        data = [fa.value for fa in actions]  # Enum→int 변환
        period = 1.0 / freq
        end_time = time.time() + duration
        
        msg = TPCANMsg()
        msg.ID = can_id
        msg.MSGTYPE = PCAN_MESSAGE_STANDARD
        msg.LEN = 5
        for i, val in enumerate(data):
            msg.DATA[i] = val
        
        while time.time() < end_time:
            st = self.pcan.Write(self.channel, msg)
            time.sleep(period)

    def chain_actions(self, can_id, steps):
        """ 
        steps: [(actions, duration, freq), (actions, duration, freq), ...] 
        동기적으로 순차 실행
        """
        for (act, dur, fq) in steps:
            self.send_actions(can_id, act, dur, fq)
    
    def chain_actions_async(self, can_id, steps):
        """
        비동기로 chain_actions 실행 (새로운 스레드 생성).
        반환값: Thread 객체
        """
        def worker():
            self.chain_actions(can_id, steps)

        th = threading.Thread(target=worker, daemon=True)
        th.start()
        return th  # 필요하면 main에서 th.join() 가능
