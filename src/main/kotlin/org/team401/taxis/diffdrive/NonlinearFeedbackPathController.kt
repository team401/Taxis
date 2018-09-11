package org.team401.taxis.diffdrive

import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.TimedState
import org.team401.taxis.util.Units
import org.team401.taxis.util.Util

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 */
class NonlinearFeedbackPathController(val kBeta: Double, val kZeta: Double): PathController {
    var prevVelocity = DifferentialDrive.ChassisState()

    override fun update(dt: Double, dynamics: DifferentialDrive.DriveDynamics, currentState: Pose2d, model: DifferentialDrive, trajectory: TrajectoryIterator<TimedState<Pose2dWithCurvature>>, setpoint: TimedState<Pose2dWithCurvature>, error: Pose2d, reversed: Boolean): Output {
        val k = 2.0 * kZeta * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity
                .linear + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular)

        val angleErrorRads = error.rotation.radians
        val sinXOverX = if (Util.epsilonEquals(angleErrorRads, 0.0, 1E-2)) 1.0
                        else error.rotation.sin() / angleErrorRads

        val adjustedVelocity =  DifferentialDrive.ChassisState(
                dynamics.chassis_velocity.linear * error.rotation.cos() +
                        k * Units.inches_to_meters(error.translation.x()),
                dynamics.chassis_velocity.angular + k * angleErrorRads +
                        dynamics.chassis_velocity.linear * kBeta * sinXOverX * Units.inches_to_meters(error
                        .translation.y()))

        dynamics.chassis_velocity = adjustedVelocity
        dynamics.wheel_velocity = model.solveInverseKinematics(adjustedVelocity)

        dynamics.chassis_acceleration.linear = if (dt == 0.0) 0.0
                                               else (dynamics.chassis_velocity.linear - prevVelocity.linear) / dt
        dynamics.chassis_acceleration.angular = if (dt == 0.0) 0.0
                                                else (dynamics.chassis_velocity.angular - prevVelocity.angular) / dt

        prevVelocity = dynamics.chassis_velocity

        val feedforwardVoltages = model.solveInverseDynamics(
                dynamics.chassis_velocity,
                dynamics.chassis_acceleration
        ).voltage

        return Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration
                .left, dynamics.wheel_acceleration.right, feedforwardVoltages.left, feedforwardVoltages.right)
    }

    override fun reset() {
        prevVelocity = DifferentialDrive.ChassisState()
    }
}