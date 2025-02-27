from hubo_hand import HandController, FingerAction

if __name__ == "__main__":
    hand = HandController()
    can_id = 0x34

    steps = [
        ([FingerAction.OPEN]*5, 1.0, 500),  
        ([FingerAction.HOLD, FingerAction.CLOSE, FingerAction.HOLD, FingerAction.HOLD, FingerAction.HOLD], 0.3, 1000),
        ([FingerAction.HOLD, FingerAction.CLOSE, FingerAction.CLOSE, FingerAction.HOLD, FingerAction.HOLD], 0.3, 1000),
        ([FingerAction.HOLD, FingerAction.CLOSE, FingerAction.CLOSE, FingerAction.CLOSE, FingerAction.HOLD], 0.3, 1000),
        ([FingerAction.HOLD, FingerAction.HOLD, FingerAction.CLOSE, FingerAction.CLOSE, FingerAction.CLOSE], 0.3, 1000),
        ([FingerAction.CLOSE, FingerAction.HOLD, FingerAction.HOLD, FingerAction.CLOSE, FingerAction.CLOSE], 0.3, 1000),
        ([FingerAction.CLOSE, FingerAction.HOLD, FingerAction.HOLD, FingerAction.HOLD, FingerAction.CLOSE], 0.3, 1000),


        ([FingerAction.OPEN]*5, 1.0, 500),
    ]

    # 비동기로 실행
    thread = hand.chain_actions_async(can_id, steps)

    print("메인 스레드는 여기서 자유롭게 동작 가능!")
    # ... 다른 작업 수행 ...

    # 필요하면 나중에 완료 대기
    thread.join()

    hand.close()
