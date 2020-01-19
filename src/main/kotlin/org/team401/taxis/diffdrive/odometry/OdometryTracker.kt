package org.team401.taxis.diffdrive.odometry

import org.snakeskin.component.provider.IAngularPositionProvider
import org.snakeskin.component.provider.IAngularVelocityProvider
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.rt.RealTimeTask
import org.snakeskin.template.DifferentialDrivetrainGeometry
import org.team401.taxis.diffdrive.component.IModeledDifferentialDrivetrain
import org.team401.taxis.diffdrive.component.provider.IHeadingProvider

/**
 * @author Cameron Earle
 * @version 9/10/18
 *
 * Provides a loop to track odometry
 */
class OdometryTracker(
        private val geometry: DifferentialDrivetrainGeometry,
        private val kinematics: DifferentialDrivetrainKinematics,
        private val driveState: DifferentialDriveState,
        private val leftPositionProvider: IAngularPositionProvider,
        private val rightPositionProvider: IAngularPositionProvider,
        private val leftVelocityProvider: IAngularVelocityProvider,
        private val rightVelocityProvider: IAngularVelocityProvider,
        private val headingProvider: IHeadingProvider
): RealTimeTask() {

    /**
     * Creates a drive odometry tracker from a modeled drivetrain object
     */
    constructor(drive: IModeledDifferentialDrivetrain) : this(
            drive.geometry,
            drive.model.driveKinematicsModel,
            drive.driveState,
            drive.left,
            drive.right,
            drive.left,
            drive.right,
            drive.headingSource
    )

    private var leftPrevDistance = leftPositionProvider.getAngularPosition().toLinearDistance(geometry.wheelRadius)
    private var rightPrevDistance = rightPositionProvider.getAngularPosition().toLinearDistance(geometry.wheelRadius)

    override fun action(timestamp: TimeMeasureSeconds, dt: TimeMeasureSeconds) {
        val leftDistance = leftPositionProvider.getAngularPosition().toLinearDistance(geometry.wheelRadius)
        val rightDistance = rightPositionProvider.getAngularPosition().toLinearDistance(geometry.wheelRadius)
        val deltaLeft = leftDistance - leftPrevDistance
        val deltaRight = rightDistance - rightPrevDistance
        val heading = headingProvider.getHeading()
        val odometryVelocity = driveState.generateOdometryFromSensors(deltaLeft.value, deltaRight.value, heading)
        val predictedVelocity = kinematics.forwardKinematics(
                leftVelocityProvider.getAngularVelocity().toLinearVelocity(geometry.wheelRadius).value,
                rightVelocityProvider.getAngularVelocity().toLinearVelocity(geometry.wheelRadius).value
        )
        driveState.addObservations(timestamp, odometryVelocity, predictedVelocity)
        leftPrevDistance = leftDistance
        rightPrevDistance = rightDistance
    }
}
