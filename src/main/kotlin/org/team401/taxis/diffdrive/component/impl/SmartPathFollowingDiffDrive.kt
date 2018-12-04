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
import org.team401.taxis.diffdrive.control.PathFollowingConfig
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.diffdrive.odometry.Kinematics
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.template.DriveDynamicsTemplate
import org.team401.taxis.template.PathFollowingTemplate
import org.team401.taxis.util.Units
import java.util.concurrent.atomic.AtomicReference

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

    init {
        updateModel(geometryTemplate, dynamicsTemplate, pathFollowingTemplate)
    }

    private val leftMotorModelRef = AtomicReference<DCMotorTransmission>()
    private val rightMotorModelRef = AtomicReference<DCMotorTransmission>()
    private val dynamicsModelRef = AtomicReference<DifferentialDrive>()
    private val kinematicsModelRef = AtomicReference<Kinematics>()
    private val pathFollowingConfigRef = AtomicReference<PathFollowingConfig>()

    override val leftMotorModel: DCMotorTransmission
    get() = leftMotorModelRef.get()

    override val rightMotorModel: DCMotorTransmission
    get() = rightMotorModelRef.get()
    
    override val dynamicsModel: DifferentialDrive
    get() = dynamicsModelRef.get()

    override val kinematicsModel: Kinematics
    get() = kinematicsModelRef.get()

    override val driveState = DifferentialDriveState(driveStateObservationBufferSize, kinematicsModelRef)

    override val pathManager = DrivetrainPathManager(dynamicsModelRef, pathFollowingConfigRef, pathController)

    override fun updateModel(geometryTemplate: TankDrivetrainGeometryTemplate, dynamicsTemplate: DriveDynamicsTemplate, pathFollowingTemplate: PathFollowingTemplate) {
        leftMotorModelRef.set(DCMotorTransmission(
                1.0 / dynamicsTemplate.leftKv,
                (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        dynamicsTemplate.inertialMass / (2.0 * dynamicsTemplate.leftKa),
                dynamicsTemplate.leftKs
        ))

        rightMotorModelRef.set(DCMotorTransmission(
                1.0 / dynamicsTemplate.leftKv,
                (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        dynamicsTemplate.inertialMass / (2.0 * dynamicsTemplate.leftKa),
                dynamicsTemplate.leftKs
        ))

        dynamicsModelRef.set(DifferentialDrive(
                dynamicsTemplate.inertialMass,
                dynamicsTemplate.momentOfInertia,
                dynamicsTemplate.angularDrag,
                geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value,
                Units.inches_to_meters(geometryTemplate.wheelbase.toUnit(LinearDistanceUnit.Standard.INCHES).value / 2.0 * dynamicsTemplate.trackScrubFactor),
                leftMotorModelRef.get(),
                rightMotorModelRef.get()
        ))

        kinematicsModelRef.set(Kinematics(geometryTemplate.wheelbase, dynamicsTemplate.trackScrubFactor))

        pathFollowingConfigRef.set(PathFollowingConfig(pathFollowingTemplate.maxErrorX, pathFollowingTemplate.maxErrorY, pathFollowingTemplate.maxErrorTheta))
    }

    /**
     * Resets the pose.  Use this instead of directly resetting driveState to ensure that heading tracks properly.
     */
    override fun setPose(pose: Pose2d, time: Double) {
        setHeading(pose.rotation)
        driveState.reset(time, pose)
    }

    private var gyroOffset = Rotation2d.identity() //The offset to use when getting the heading

    /**
     * Gets the heading of the drive from the compass.  This is more accurate than the getYaw method.
     * This also uses a robot controller side heading offset, which avoids the issue of the IMU taking a long time to set the yaw
     * The heading is returned as a Rotation2d to make it easy to use with other parts of this library
     */
    override fun getHeading(): Rotation2d {
        return Rotation2d.fromDegrees(imu.fusedHeading).rotateBy(gyroOffset)
    }

    /**
     * Marks the input heading as the current heading of the robot
     */
    override fun setHeading(heading: Rotation2d) {
        gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(imu.fusedHeading).inverse())
    }
}