# dh_hand_test.py
from hubo_hand import HandController, FingerAction

if __name__ == "__main__":
    hand = HandController()

    actions = [
        FingerAction.OPEN,   
        FingerAction.OPEN,  
        FingerAction.OPEN,   
        FingerAction.OPEN,   
        FingerAction.OPEN    
    ]
    # 1초 동안 500Hz 전송
    hand.send_actions(can_id=0x34, actions=actions, duration=1.0, freq=500)


    actions = [
        FingerAction.CLOSE,   
        FingerAction.OPEN,  
        FingerAction.OPEN,   
        FingerAction.CLOSE,   
        FingerAction.CLOSE    
    ]
    hand.send_actions(can_id=0x34, actions=actions, duration=1.0, freq=500)

    actions = [
        FingerAction.HOLD,   
        FingerAction.CLOSE,  
        FingerAction.CLOSE,   
        FingerAction.HOLD,   
        FingerAction.HOLD    
    ]
    hand.send_actions(can_id=0x34, actions=actions, duration=1.0, freq=500)

    hand.close()