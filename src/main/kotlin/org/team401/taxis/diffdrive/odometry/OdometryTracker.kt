package org.team401.taxis.diffdrive.odometry

import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.FullStateDiffDriveModel

/**
 * @author Cameron Earle
 * @version 9/10/18
 *
 * Provides a loop to track odometry
 */
class OdometryTracker(private val fullStateModel: FullStateDiffDriveModel, private val provider: DriveDataProvider, private val driveState: DifferentialDriveState): RealTimeTask {
    constructor(drivetrain: IPathFollowingDiffDrive<ISensoredGearbox>) : this(drivetrain.fullStateModel, drivetrain.driveDataProvider, drivetrain.driveState)

    override val name = "Odometry Tracker"

    private var leftEncoderPrevDistance = provider.getLeftPositionInches()
    private var rightEncoderPrevDistance = provider.getRightPositionInches()

    override fun action(ctx: RealTimeExecutor.RealTimeContext) {
        val leftDistance = provider.getLeftPositionInches()
        val rightDistance = provider.getRightPositionInches()
        val deltaLeft = leftDistance - leftEncoderPrevDistance
        val deltaRight = rightDistance - rightEncoderPrevDistance
        val gyroAngle = provider.getGyroHeading()
        val odometryVelocity = driveState.generateOdometryFromSensors(deltaLeft, deltaRight, gyroAngle)
        val predictedVelocity = fullStateModel.drivetrainKinematicsModel.forwardKinematics(provider.getLeftVelocityIps(), provider.getRightVelocityIps())
        driveState.addObservations(ctx.time, odometryVelocity, predictedVelocity)
        leftEncoderPrevDistance = leftDistance
        rightEncoderPrevDistance = rightDistance
    }
}
