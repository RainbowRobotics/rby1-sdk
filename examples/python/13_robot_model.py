"""
Robot Model Setting Example

This example includes the following features:
1. Loading the current robot model (in URDF format) from the robot.
2. Saving a custom robot model to the robot with a specified name.
3. Assigning the robot model name for the robot to use (applied after reboot).

Robot 모델 설정 예제

이 예제에는 아래와 같은 기능을 담고 있습니다.
1. 현재 로봇에서 사용하고 있는 로봇 모델 (URDF 포맷)을 불러옵니다.
2. 사용자 로봇 모델을 로봇에 이름과 함께 저장합니다.
3. 로봇이 사용할 로봇 모델 이름을 지정합니다. (로봇이 재부팅 될 때 반영됩니다.)
"""

import rby1_sdk
import argparse
import xml.etree.ElementTree as ET


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Failed to connect robot")
        exit(1)
    model = robot.get_robot_model()

    print("Current robot model: ")
    print(model)

    # Modify model
    # 현재 사용하고 있는 모델 중 head_1의 effort 리밋 값을 변경 (예시)
    model_tree = ET.ElementTree(ET.fromstring(model))
    model_root = model_tree.getroot()
    for joint in model_root.findall("joint"):
        if joint.get("name") == "head_1":
            joint.find("limit").set("effort", "500")

    # Upload model and save model with name 'temp'
    print(robot.import_robot_model("temp", ET.tostring(model_root).decode()))

    # Set robot model
    robot.set_parameter("model_name", '"temp"')

    # After reboot, the robot will use uploaded robot model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="13_robot_model")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
