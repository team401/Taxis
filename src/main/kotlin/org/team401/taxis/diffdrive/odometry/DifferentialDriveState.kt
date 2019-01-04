package org.team401.taxis.diffdrive.odometry

import org.snakeskin.hardware.Hardware
import org.team401.taxis.diffdrive.control.FullStateDiffDriveModel
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Twist2d
import org.team401.taxis.util.InterpolatingDouble
import org.team401.taxis.util.InterpolatingTreeMap
import java.util.concurrent.atomic.AtomicReference

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 */
class DifferentialDriveState(val observationBufferSize: Int = 100, val fullStateModel: FullStateDiffDriveModel): OdometryProvider {
    private lateinit var fieldToVehicle: InterpolatingTreeMap<InterpolatingDouble, Pose2d>
    private lateinit var vehicleVelocityPredicted: Twist2d
    private lateinit var vehicleVelocityMeasured: Twist2d
    private var distanceDriven = 0.0

    init {
        reset(Hardware.getRelativeTime(), Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))
    }

    @Synchronized fun reset(startTime: Double, initialFieldToVehicle: Pose2d) {
        fieldToVehicle = InterpolatingTreeMap(observationBufferSize)
        fieldToVehicle[InterpolatingDouble(startTime)] = initialFieldToVehicle
        vehicleVelocityPredicted = Twist2d.identity()
        vehicleVelocityMeasured = Twist2d.identity()
        distanceDriven = 0.0
    }

    @Synchronized fun resetDistanceDriven() {
        distanceDriven = 0.0
    }

    @Synchronized fun getFieldToVehicle(timestamp: Double): Pose2d {
        return fieldToVehicle.getInterpolated(InterpolatingDouble(timestamp))
    }

    @Synchronized fun getLatestFieldToVehicle(): Map.Entry<InterpolatingDouble, Pose2d> {
        return fieldToVehicle.lastEntry()
    }

    @Synchronized fun getPredictedFieldToVehicle(lookaheadTime: Double): Pose2d {
        return getLatestFieldToVehicle().value
                .transformBy(Pose2d.exp(vehicleVelocityPredicted.scaled(lookaheadTime)))
    }

    @Synchronized fun addFieldToVehicleObservation(timestamp: Double, observation: Pose2d) {
        fieldToVehicle[InterpolatingDouble(timestamp)] = observation
    }

    @Synchronized fun addObservations(timestamp: Double, measuredVelocity: Twist2d, predictedVelocity: Twist2d) {
        addFieldToVehicleObservation(timestamp, fullStateModel.drivetrainKinematicsModel.integrateForwardKinematics(getLatestFieldToVehicle().value, measuredVelocity))
        vehicleVelocityMeasured = measuredVelocity
        vehicleVelocityPredicted = predictedVelocity
    }

    @Synchronized fun generateOdometryFromSensors(leftEncoderDeltaDistance: Double, rightEncoderDeltaDistance: Double, currentGyroAngle: Rotation2d): Twist2d {
        val lastMeasurement = getLatestFieldToVehicle().value
        val delta = fullStateModel.drivetrainKinematicsModel.forwardKinematics(
                lastMeasurement.rotation,
                leftEncoderDeltaDistance, rightEncoderDeltaDistance,
                currentGyroAngle
        )
        distanceDriven += delta.dx
        return delta
    }

    @Synchronized fun getDistanceDriven(): Double {
        return distanceDriven
    }

    @Synchronized fun getPredictedVelocity(): Twist2d {
        return vehicleVelocityPredicted
    }

    @Synchronized fun getMeasuredVelocity(): Twist2d {
        return vehicleVelocityMeasured
    }

    override fun getLatestXInches(): Double {
        return getLatestFieldToVehicle().value.translation.x()
    }

    override fun getLatestYInches(): Double {
        return getLatestFieldToVehicle().value.translation.y()
    }

    override fun getLatestThetaRadians(): Double {
        return getLatestFieldToVehicle().value.rotation.radians
    }
}