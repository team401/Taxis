package org.team401.taxis.diffdrive.component.impl

import com.ctre.phoenix.sensors.PigeonIMU
import org.snakeskin.component.Gearbox
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.component.impl.SmartTankDrivetrain
import org.snakeskin.hardware.Hardware
import org.snakeskin.template.TankDrivetrainGeometryTemplate
import org.snakeskin.units.LinearDistanceUnit
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.taxis.diffdrive.component.PathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.PathController
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.diffdrive.odometry.Kinematics
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.template.DriveDynamicsTemplate
import org.team401.taxis.template.PathFollowingTemplate

/**
 * @author Cameron Earle
 * @version 9/15/18
 *
 * Represents a "smart" (CTRE) differential drivetrain that is capable of path following.
 * Essentially gathers all the components required for path following into a single class
 */
class SmartPathFollowingDiffDrive(geometryTemplate: TankDrivetrainGeometryTemplate,
                                  dynamicsTemplate: DriveDynamicsTemplate,
                                  pathFollowingTemplate: PathFollowingTemplate,
                                  left: Gearbox,
                                  right: Gearbox,
                                  imu: PigeonIMU,
                                  pathController: PathController,
                                  driveStateObservationBufferSize: Int = 100): TankDrivetrain by SmartTankDrivetrain(geometryTemplate, left, right, imu), PathFollowingDiffDrive {
    
    override val motorModel = DCMotorTransmission(
            1.0 / dynamicsTemplate.kV,
            (wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                    (wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                    dynamicsTemplate.inertialMass / (2.0 * dynamicsTemplate.kA),
            dynamicsTemplate.kV
    )
    
    override val dynamicsModel = DifferentialDrive(
            dynamicsTemplate.inertialMass,
            dynamicsTemplate.momentOfInertia,
            dynamicsTemplate.angularDrag,
            wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value,
            LinearDistanceMeasureInches(
                    wheelRadius.toUnit(LinearDistanceUnit.Standard.INCHES).value
                            * dynamicsTemplate.trackScrubFactor).toUnit(LinearDistanceUnit.Standard.METERS).value, //TODO check math after removing radius conversion
            motorModel,
            motorModel
    )

    override val kinematicsModel = Kinematics(wheelbase, dynamicsTemplate.trackScrubFactor)

    override val driveState = DifferentialDriveState(driveStateObservationBufferSize, kinematicsModel)

    override val pathManager = DrivetrainPathManager(dynamicsModel, pathFollowingTemplate, pathController)

    /**
     * Resets the pose.  Use this instead of directly resetting driveState to ensure that heading tracks properly.
     */
    fun resetPose(pose: Pose2d, time: Double = Hardware.getRelativeTime()) {
        //TODO change the drivetrain class (or maybe directly in the robot state?) to track heading itself to use the compass heading
        setYaw(AngularDistanceMeasureDegrees(pose.rotation.degrees)) //TODO This apparently takes a lot of time to finish? So we need to stop using this
        left.setPosition(AngularDistanceMeasureCTREMagEncoder(0.0)) //We don't technically have to do this, since the pose estimator just tracks delta, but it can't hurt
        right.setPosition(AngularDistanceMeasureCTREMagEncoder(0.0))
        driveState.reset(time, pose)
    }
}