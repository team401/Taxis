package org.team401.taxis.diffdrive.control;

import org.team401.taxis.geometry.Pose2d;
import org.team401.taxis.physics.DifferentialDrivetrainDynamics;
import org.team401.taxis.util.Units;
import org.team401.taxis.util.Util;

/**
 * @author Cameron Earle
 * @version 12/3/2018
 */
public class NonlinearFeedbackPathController implements PathController {
    public NonlinearFeedbackPathController(double kBeta, double kZeta) {
        this.kBeta = kBeta;
        this.kZeta = kZeta;
    }

    public NonlinearFeedbackPathController() {
        this(2.0, .7);
    }

    private double kBeta;
    private double kZeta;

    DifferentialDrivetrainDynamics.ChassisState prev_velocity_ = new DifferentialDrivetrainDynamics.ChassisState();

    @Override
    public DrivetrainPathManager.Output update(DifferentialDrivetrainDynamics.DriveDynamics dynamics, Pose2d current_state, Pose2d mError, DifferentialDrivetrainDynamics mModel, double mDt) {
        // Compute gain parameter.
        final double k = 2.0 * kZeta * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity
                .linear + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);

        // Compute error components.
        final double angle_error_rads = mError.getRotation().getRadians();
        final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ?
                1.0 : mError.getRotation().sin() / angle_error_rads;
        final DifferentialDrivetrainDynamics.ChassisState adjusted_velocity = new DifferentialDrivetrainDynamics.ChassisState(
                dynamics.chassis_velocity.linear * mError.getRotation().cos() +
                        k * Units.inches_to_meters(mError.getTranslation().x()),
                dynamics.chassis_velocity.angular + k * angle_error_rads +
                        dynamics.chassis_velocity.linear * kBeta * sin_x_over_x * Units.inches_to_meters(mError
                                .getTranslation().y()));

        // Compute adjusted left and right wheel velocities.
        dynamics.chassis_velocity = adjusted_velocity;
        dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);

        dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.linear - prev_velocity_
                .linear) / mDt;
        dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.angular - prev_velocity_
                .angular) / mDt;

        prev_velocity_ = dynamics.chassis_velocity;

        DifferentialDrivetrainDynamics.WheelState feedforward_voltages = mModel.solveInverseDynamics(dynamics.chassis_velocity,
                dynamics.chassis_acceleration).voltage;

        return new DrivetrainPathManager.Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration
                .left, dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);
    }
}
