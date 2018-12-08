package org.team401.taxis.diffdrive.component

import org.snakeskin.component.TankDrivetrain
import org.snakeskin.hardware.Hardware
import org.snakeskin.template.TankDrivetrainGeometryTemplate
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.PathFollowingConfig
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.diffdrive.odometry.Kinematics
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.template.DriveDynamicsTemplate
import org.team401.taxis.template.PathFollowingTemplate
import java.util.concurrent.atomic.AtomicReference

/**
 * @author Cameron Earle
 * @version 9/15/18
 *
 * Pulls in the values from
 */
interface PathFollowingDiffDrive: TankDrivetrain {
    val leftMotorModelRef: AtomicReference<DCMotorTransmission>
    val rightMotorModelRef: AtomicReference<DCMotorTransmission>
    val dynamicsModelRef: AtomicReference<DifferentialDrive>
    val kinematicsModelRef: AtomicReference<Kinematics>
    val pathFollowingConfigRef: AtomicReference<PathFollowingConfig>

    val leftMotorModel: DCMotorTransmission
    val rightMotorModel: DCMotorTransmission
    val dynamicsModel: DifferentialDrive
    val kinematicsModel: Kinematics
    val driveState: DifferentialDriveState
    val pathManager: DrivetrainPathManager

    fun updateModel(geometryTemplate: TankDrivetrainGeometryTemplate, dynamicsTemplate: DriveDynamicsTemplate, pathFollowingTemplate: PathFollowingTemplate)
    fun setPose(pose: Pose2d, time: Double = Hardware.getRelativeTime())
    fun getHeading(): Rotation2d
    fun setHeading(heading: Rotation2d)
}