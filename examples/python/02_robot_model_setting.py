# Robot Model Setting Example
#
# This example includes the following features:
# 1. Loading the current robot model (in URDF format) from the robot.
# 2. Saving a custom robot model to the robot with a specified name.
# 3. Assigning the robot model name for the robot to use (applied after reboot).
#
# Note:
# If the modified URDF causes any issue, users can recover by downloading the
# proper official URDF for their robot model from the rby1-sdk repository below,
# parsing it in the same way as this example, and importing it again with a new model name:
# https://github.com/RainbowRobotics/rby1-sdk/tree/main/models
#
# If an issue arises while modifying the urdf again after the above step,
# You can skip the previous step by running only the robot.set_parameter() command.
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk
import argparse
import xml.etree.ElementTree as ET


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Failed to connect robot")
        exit(1)

    # 1. Load current robot model from robot
    urdf_str = robot.get_robot_model()

    print("Current robot model loaded.")

    # 2. Parse URDF
    model_tree = ET.ElementTree(ET.fromstring(urdf_str))
    model_root = model_tree.getroot()

    # 3. Modify ee_right mass
    target_link_name = "ee_right"
    mass_delta = 0.5  # increase by 0.1 kg

    modified = False

    for link in model_root.findall("link"):
        if link.get("name") == target_link_name:
            inertial = link.find("inertial")
            if inertial is None:
                print(f"[ERROR] link '{target_link_name}' has no <inertial> tag.")
                exit(1)

            mass_tag = inertial.find("mass")
            if mass_tag is None:
                print(f"[ERROR] link '{target_link_name}' has no <mass> tag.")
                exit(1)

            old_mass = float(mass_tag.get("value"))
            new_mass = old_mass + mass_delta
            mass_tag.set("value", f"{new_mass:.8f}")

            print(f"Modified '{target_link_name}' mass: {old_mass:.8f} -> {new_mass:.8f}")
            modified = True
            break

    if not modified:
        print(f"[ERROR] link '{target_link_name}' not found in URDF.")
        exit(1)

    # 4. Upload modified model with a new name
    new_model_name = "temp_ee_right_mass_up"
    modified_urdf = ET.tostring(model_root, encoding="unicode")

    ok = robot.import_robot_model(new_model_name, modified_urdf)
    print(f"Import robot model: {ok}")

    if not ok:
        print("[ERROR] Failed to import modified robot model.")
        exit(1)

    # 5. Set robot model name
    robot.set_parameter("model_name", f'"{new_model_name}"')
    print(f"Set model_name to '{new_model_name}'")
    print("The modified robot model will be applied after reboot.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Modify ee_right mass example")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--model", type=str, default="a", help="Robot Model Name (default: 'a')")
    args = parser.parse_args()

    main(address=args.address, model=args.model)
