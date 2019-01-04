package org.team401.taxis.diffdrive.component

import org.snakeskin.component.TankDrivetrain
import org.snakeskin.hardware.Hardware
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.FullStateDiffDriveModel
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.diffdrive.odometry.DriveDataProvider
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

/**
 * @author Cameron Earle
 * @version 9/15/18
 *
 * Pulls in the values from
 */
interface PathFollowingDiffDrive: TankDrivetrain {
    val fullStateModel: FullStateDiffDriveModel
    val driveState: DifferentialDriveState
    val pathManager: DrivetrainPathManager
    val driveDataProvider: DriveDataProvider

    fun setPose(pose: Pose2d, time: Double = Hardware.getRelativeTime())
    fun getHeading(): Rotation2d
    fun setHeading(heading: Rotation2d)
}