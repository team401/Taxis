package org.team401.taxis.diffdrive.odometry

import org.snakeskin.component.TankDrivetrain
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.snakeskin.units.AngularDistanceUnit
import org.snakeskin.units.LinearDistanceUnit
import org.snakeskin.units.LinearVelocityUnit
import org.team401.taxis.diffdrive.component.PathFollowingDiffDrive
import org.team401.taxis.geometry.Rotation2d

/**
 * @author Cameron Earle
 * @version 9/10/18
 *
 * Provides a loop to track odometry
 */
class OdometryTracker(private val drivetrain: PathFollowingDiffDrive): RealTimeTask {
    override val name = "Odometry Tracker"

    private fun getLeftPositionInches(): Double {
        return drivetrain.left.getPosition().toLinearDistance(drivetrain.wheelRadius).toUnit(LinearDistanceUnit.Standard.INCHES).value
    }

    private fun getRightPositionInches(): Double {
        return drivetrain.right.getPosition().toLinearDistance(drivetrain.wheelRadius).toUnit(LinearDistanceUnit.Standard.INCHES).value
    }

    private fun getLeftVelocityIps(): Double {
        return drivetrain.left.getVelocity().toLinearVelocity(drivetrain.wheelRadius).toUnit(LinearVelocityUnit.Standard.INCHES_PER_SECOND).value
    }

    private fun getRightVelocityIps(): Double {
        return drivetrain.right.getVelocity().toLinearVelocity(drivetrain.wheelRadius).toUnit(LinearVelocityUnit.Standard.INCHES_PER_SECOND).value
    }

    private fun getGyroHeading(): Rotation2d {
        return Rotation2d.fromDegrees(drivetrain.getYaw().toUnit(AngularDistanceUnit.Standard.DEGREES).value)
    }


    private var leftEncoderPrevDistance = getLeftPositionInches()
    private var rightEncoderPrevDistance = getRightPositionInches()

    override fun action(ctx: RealTimeExecutor.RealTimeContext) {
        val leftDistance = getLeftPositionInches()
        val rightDistance = getRightPositionInches()
        val deltaLeft = leftDistance - leftEncoderPrevDistance
        val deltaRight = rightDistance - rightEncoderPrevDistance
        val gyroAngle = getGyroHeading()
        val odometryVelocity = drivetrain.driveState.generateOdometryFromSensors(deltaLeft, deltaRight, gyroAngle)
        val predictedVelocity = drivetrain.kinematicsModel.forwardKinematics(getLeftVelocityIps(), getRightVelocityIps())
        drivetrain.driveState.addObservations(ctx.time, odometryVelocity, predictedVelocity)
        leftEncoderPrevDistance = leftDistance
        rightEncoderPrevDistance = rightDistance
    }
}
