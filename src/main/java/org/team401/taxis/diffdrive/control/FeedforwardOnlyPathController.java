package org.team401.taxis.diffdrive.control;

import org.team401.taxis.geometry.Pose2d;
import org.team401.taxis.physics.DifferentialDrivetrainDynamics;

/**
 * @author Cameron Earle
 * @version 12/3/2018
 */
public class FeedforwardOnlyPathController implements PathController {
    @Override
    public DrivetrainPathManager.Output update(DifferentialDrivetrainDynamics.DriveDynamics dynamics, Pose2d current_state, Pose2d mError, DifferentialDrivetrainDynamics mModel, double mDt) {
        return new DrivetrainPathManager.Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics
                .wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage
                .left, dynamics.voltage.right);
    }
}
