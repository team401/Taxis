package org.team401.taxis.diffdrive.component

import org.snakeskin.component.TankDrivetrain
import org.snakeskin.hardware.Hardware
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.diffdrive.odometry.Kinematics
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrive

/**
 * @author Cameron Earle
 * @version 9/15/18
 *
 * Pulls in the values from
 */
interface PathFollowingDiffDrive: TankDrivetrain {
    val motorModel: DCMotorTransmission
    val dynamicsModel: DifferentialDrive
    val kinematicsModel: Kinematics
    val driveState: DifferentialDriveState
    val pathManager: DrivetrainPathManager

    fun setPose(pose: Pose2d, time: Double = Hardware.getRelativeTime())
    fun getHeading(): Rotation2d
    fun setHeading(heading: Rotation2d)
}