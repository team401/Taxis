package org.team401.taxis.diffdrive.odometry

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Twist2d
import org.team401.taxis.template.DriveDynamicsTemplate
import java.lang.ref.Reference
import java.util.concurrent.atomic.AtomicReference

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 * A kinematics model for a robot
 */
class Kinematics(private val wheelbase: LinearDistanceMeasureInches, private val trackScrubFactor: Double) {
    companion object {
        private const val kEpsilon = 1e-9
    }

    fun forwardKinematics(leftWheelDelta: Double, rightWheelDelta: Double): Twist2d {
        val deltaRotation = (rightWheelDelta - leftWheelDelta) /
                (wheelbase.value * trackScrubFactor)
        return forwardKinematics(leftWheelDelta, rightWheelDelta, deltaRotation)
    }

    fun forwardKinematics(leftWheelDelta: Double, rightWheelDelta: Double, deltaRotationRads: Double): Twist2d {
        val dx = (leftWheelDelta + rightWheelDelta) / 2.0
        return Twist2d(dx, 0.0, deltaRotationRads)
    }

    fun forwardKinematics(prevHeading: Rotation2d, leftWheelDelta: Double, rightWheelDelta: Double, currentHeading: Rotation2d): Twist2d {
        val dx = (leftWheelDelta + rightWheelDelta) / 2.0
        val dy = 0.0
        return Twist2d(dx, dy, prevHeading.inverse().rotateBy(currentHeading).radians)
    }

    fun integrateForwardKinematics(currentPose: Pose2d, forwardKinematics: Twist2d): Pose2d {
        return currentPose.transformBy(Pose2d.exp(forwardKinematics))
    }
}