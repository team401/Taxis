package org.team401.taxis.diffdrive.control;

import org.team401.taxis.geometry.Pose2d;
import org.team401.taxis.physics.DifferentialDrivetrainDynamics;

/**
 * @author Cameron Earle
 * @version 12/3/2018
 */
public interface PathController {
    DrivetrainPathManager.Output update(DifferentialDrivetrainDynamics.DriveDynamics dynamics,
                                        Pose2d current_state,
                                        Pose2d mError,
                                        DifferentialDrivetrainDynamics mModel,
                                        double mDt);
}
