package org.team401.taxis.diffdrive.control;

import org.team401.taxis.geometry.Pose2d;
import org.team401.taxis.physics.DifferentialDrive;

/**
 * @author Cameron Earle
 * @version 12/3/2018
 */
public interface PathController {
    DrivetrainPathManager.Output update(DifferentialDrive.DriveDynamics dynamics,
                                        Pose2d current_state,
                                        Pose2d mError,
                                        DifferentialDrive mModel,
                                        double mDt);
}
