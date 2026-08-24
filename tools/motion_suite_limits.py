# QP baseline motion suite - limit-grazing and target-switch scenarios.
#
# Complements examples/python/24_demo_motion.py (which covers joint position,
# cartesian, impedance, optimal control and mixed commands) with the two
# scenarios the QP upgrade plan needs for baseline capture:
#   1. limit-graze: optimal-control posture targets at ~95% of torso position
#      limits, so the position-braking constraint becomes active.
#   2. target-switch: rapidly alternating cartesian targets, stressing solver
#      re-activation transients (worst case for warm start).
#
# Usage:
#     python3 motion_suite_limits.py --address localhost:50051 --model m

import argparse
import time

import numpy as np
import rby1_sdk as rby

D2R = np.pi / 180
WEIGHT = 1.0
STOP_COST = WEIGHT * WEIGHT * 2e-3
MIN_DELTA_COST = WEIGHT * WEIGHT * 1e-4
PATIENCE = 10
COMMAND_TIMEOUT = 30


def make_transform(r, t):
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def rot_y(rad):
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def send_body_optimal_control(robot, builder):
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(builder)
    )
    rv = robot.send_command(rc, COMMAND_TIMEOUT).get()
    return rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok


def recover_if_fault(robot):
    """Reset control-manager fault (if any) and re-enable. Returns True if a
    fault had to be cleared - the suite records this as baseline data."""
    state = robot.get_control_manager_state().state
    if state not in [
        rby.ControlManagerState.State.MajorFault,
        rby.ControlManagerState.State.MinorFault,
    ]:
        return False
    print(f"[recover] control manager fault: {state}; resetting")
    if not robot.reset_fault_control_manager():
        raise SystemExit("failed to reset control manager fault")
    if not robot.enable_control_manager():
        raise SystemExit("failed to re-enable control manager")
    return True


def go_ready_pose(robot, model):
    q = np.zeros(len(model.body_idx))
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.JointPositionCommandBuilder()
            .set_position(q)
            .set_minimum_time(4)
        )
    )
    return (
        robot.send_command(rc, COMMAND_TIMEOUT).get().finish_code
        == rby.RobotCommandFeedback.FinishCode.Ok
    )


def scenario_limit_graze(robot):
    """Drive torso joints to ~95% of their URDF position limits via the
    on-robot optimal-control QP, so the braking constraint becomes active."""
    print("[limit-graze] torso_1 -> +85.5deg (95% of +90deg)")
    ok1 = send_body_optimal_control(
        robot,
        rby.OptimalControlCommandBuilder()
        .add_joint_position_target("torso_1", 90 * 0.95 * D2R, WEIGHT)
        .add_joint_position_target("torso_2", -150 * 0.6 * D2R, WEIGHT)
        .add_joint_position_target("torso_3", 90 * 0.5 * D2R, WEIGHT)
        .set_velocity_limit_scaling(0.8)
        .set_error_scaling(1.0)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE),
    )
    print("[limit-graze] torso_5 yaw -> -128deg (95% of -135deg)")
    ok2 = send_body_optimal_control(
        robot,
        rby.OptimalControlCommandBuilder()
        .add_joint_position_target("torso_5", -135 * 0.95 * D2R, WEIGHT)
        .set_velocity_limit_scaling(0.8)
        .set_error_scaling(1.0)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE),
    )
    return ok1 and ok2


def scenario_target_switch(robot, cycles=6):
    """Alternate two whole-body cartesian targets back-to-back. Each command
    re-activates the controller, which is the worst case for QP warm start."""
    T_a = make_transform(rot_y(-np.pi / 2), [0.55, -0.25, 1.1])
    T_b = make_transform(rot_y(-np.pi / 2), [0.35, 0.25, 0.85])
    ok = True
    for i in range(cycles):
        T_right, T_left = (T_a, T_b) if i % 2 == 0 else (T_b, T_a)
        print(f"[target-switch] cycle {i + 1}/{cycles}")
        ok &= send_body_optimal_control(
            robot,
            rby.OptimalControlCommandBuilder()
            .add_cartesian_target("base", "link_torso_5", make_transform(np.eye(3), [0, 0, 1.0]), WEIGHT, WEIGHT)
            .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
            .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
            .set_velocity_limit_scaling(0.8)
            .set_error_scaling(1.0)
            .set_stop_cost(STOP_COST * 10)  # loose convergence: switch early
            .set_min_delta_cost(MIN_DELTA_COST)
            .set_patience(3),
        )
    return ok


def scenario_deep_fold(robot, model):
    """Command a deep-fold torso posture that lies INSIDE the self-collision
    region. With dynamic limits ON the controller must stop at the admissible
    boundary (clamped posture, no fault); with limits OFF it will drive to the
    URDF box. Reports the achieved torso angles either way."""
    target = {"torso_1": 60, "torso_2": -140, "torso_3": 80}  # deg, forbidden
    print(f"[deep-fold] target {target} (self-collision region)")
    builder = (
        rby.OptimalControlCommandBuilder()
        .set_velocity_limit_scaling(0.6)
        .set_error_scaling(1.0)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(30)
    )
    for joint, deg in target.items():
        builder.add_joint_position_target(joint, deg * D2R, WEIGHT)
    ok = send_body_optimal_control(robot, builder)
    st = robot.get_state()
    achieved = np.degrees(np.array(st.position)[model.torso_idx])
    print(f"[deep-fold] finish_ok={ok} achieved torso angles (deg): {np.round(achieved, 1)}")
    return True  # informational: evaluation is done by the caller/log


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="localhost:50051")
    parser.add_argument("--model", default="m")
    parser.add_argument("--scenario", default="all", choices=["all", "deepfold"])
    args = parser.parse_args()

    robot = rby.create_robot(args.address, args.model)
    if not robot.connect():
        raise SystemExit(f"failed to connect: {args.address}")
    if not robot.is_power_on(".*"):
        if not robot.power_on(".*"):
            raise SystemExit("failed to power on")
    if not robot.is_servo_on(".*"):
        if not robot.servo_on(".*"):
            raise SystemExit("failed to servo on")
    cm = robot.get_control_manager_state().state
    if cm in [rby.ControlManagerState.State.MajorFault, rby.ControlManagerState.State.MinorFault]:
        if not robot.reset_fault_control_manager():
            raise SystemExit("failed to reset control manager fault")
    if not robot.enable_control_manager():
        raise SystemExit("failed to enable control manager")

    model = robot.model()
    results = {}
    faults = {}

    if args.scenario == "deepfold":
        recover_if_fault(robot)
        assert go_ready_pose(robot, model), "ready pose failed"
        scenario_deep_fold(robot, model)
        recover_if_fault(robot)
        go_ready_pose(robot, model)
        return

    def ready(label):
        faults[label] = recover_if_fault(robot)
        if not go_ready_pose(robot, model):
            recover_if_fault(robot)
            faults[label] = True
            if not go_ready_pose(robot, model):
                raise SystemExit(f"ready pose failed twice at {label}")

    ready("start")
    results["limit_graze"] = scenario_limit_graze(robot)
    time.sleep(0.5)
    ready("after_limit_graze")
    results["target_switch"] = scenario_target_switch(robot)
    ready("after_target_switch")

    print("\n=== motion_suite_limits results ===")
    failed = [k for k, v in results.items() if not v]
    for k, v in results.items():
        print(f"  {k}: {'OK' if v else 'FAIL'}")
    for k, v in faults.items():
        print(f"  fault@{k}: {'YES' if v else 'no'}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
