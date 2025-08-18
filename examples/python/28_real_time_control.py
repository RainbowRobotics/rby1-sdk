import rby1_sdk as rby
from helper import *
import numpy as np
import argparse
import logging
import threading

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


class RealTimeControl:
    def __init__(self, address, power, servo):
        self.robot: rby.Robot_A = initialize_robot(address, "a", power, servo)
        self.model = self.robot.model()

        self.target_position = None
        self.minimum_time = 1

        self.local_t = 0.0
        self.generator = None
        state = self.robot.get_state()
        self.last_target_position = state.position
        self.last_target_velocity = state.velocity

        self.rt_thread = threading.Thread(
            target=self.robot.control,
            args=(self.control,),
        )

    def set_target(self, position, minimum_time=1):
        self.target_position = position
        self.minimum_time = minimum_time

    def control(self, state: rby.Robot_A_ControlState):
        i = rby.Robot_A_ControlInput()

        if self.target_position is not None:
            if self.generator is None:
                self.generator = rby.math.TrapezoidalMotionGenerator()

            gen_inp = rby.math.TrapezoidalMotionGenerator.Input()
            gen_inp.current_position = self.last_target_position
            gen_inp.current_velocity = self.last_target_velocity
            gen_inp.target_position = self.target_position
            gen_inp.velocity_limit = np.array([5.0] * self.model.robot_dof)
            gen_inp.acceleration_limit = np.array([5.0] * self.model.robot_dof)
            gen_inp.minimum_time = self.minimum_time
            self.generator.update(gen_inp)
            self.local_t = 0.002
            
            self.target_position = None
            
        if self.generator is not None:
            out = self.generator(self.local_t)
            self.last_target_position = out.position
            self.last_target_velocity = out.velocity

        i.target = self.last_target_position
        i.feedback_gain.fill(10)
        i.feedforward_torque.fill(0)
        i.finish = False
        
        self.local_t += 0.002

        return i

    def start(self):
        self.rt_thread.start()

    def wait_for_done(self):
        self.rt_thread.join()


def main(address, power, servo):
    rt_control = RealTimeControl(address, power, servo)
    rt_control.start()

    rt_control.set_target(
        np.deg2rad(
            [
                # wheel
                0.0,
                0.0,
                # torso
                0.0,
                45.0,
                -90.0,
                45.0,
                0.0,
                0.0,
                # right arm
                0.0,
                -5.0,
                0.0,
                -120.0,
                0.0,
                70.0,
                0.0,
                # left arm
                0.0,
                5.0,
                0.0,
                -120.0,
                0.0,
                70.0,
                0.0,
                # head
                0.0,
                0.0,
            ]
        ),
        minimum_time=2,
    )

    rt_control.wait_for_done()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="28_real_time_control")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    parser.add_argument(
        "--servo",
        type=str,
        default=".*",
        help="Servo name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(
        address=args.address,
        power=args.power,
        servo=args.servo,
    )
