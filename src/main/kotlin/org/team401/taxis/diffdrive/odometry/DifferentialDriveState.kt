package org.team401.taxis.diffdrive.odometry

import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.runtime.SnakeskinRuntime
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Twist2d
import org.team401.taxis.util.InterpolatingDouble
import org.team401.taxis.util.InterpolatingTreeMap

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 */
open class DifferentialDriveState(val observationBufferSize: Int = 100, private val kinematics: DifferentialDrivetrainKinematics) {
    protected lateinit var fieldToVehicle: InterpolatingTreeMap<InterpolatingDouble, Pose2d>
    var vehicleVelocityPredicted = Twist2d.identity()
        @Synchronized get
        protected set

    var vehicleVelocityMeasured = Twist2d.identity()
        @Synchronized get
        protected set

    var distanceDriven = 0.0
        @Synchronized get
        protected set

    init {
        reset(SnakeskinRuntime.timestamp, Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))
    }

    @Synchronized fun reset(startTime: TimeMeasureSeconds, initialFieldToVehicle: Pose2d) {
        fieldToVehicle = InterpolatingTreeMap(observationBufferSize)
        fieldToVehicle[InterpolatingDouble(startTime.value)] = initialFieldToVehicle
        vehicleVelocityPredicted = Twist2d.identity()
        vehicleVelocityMeasured = Twist2d.identity()
        distanceDriven = 0.0
    }

    @Synchronized fun resetDistanceDriven() {
        distanceDriven = 0.0
    }

    @Synchronized fun getFieldToVehicle(timestamp: TimeMeasureSeconds): Pose2d {
        return fieldToVehicle.getInterpolated(InterpolatingDouble(timestamp.value))
    }

    @Synchronized fun getLatestFieldToVehicle(): Map.Entry<InterpolatingDouble, Pose2d> {
        return fieldToVehicle.lastEntry()
    }

    @Synchronized fun getPredictedFieldToVehicle(lookaheadTime: TimeMeasureSeconds): Pose2d {
        return getLatestFieldToVehicle().value
                .transformBy(Pose2d.exp(vehicleVelocityPredicted.scaled(lookaheadTime.value)))
    }

    @Synchronized fun addFieldToVehicleObservation(timestamp: TimeMeasureSeconds, observation: Pose2d) {
        fieldToVehicle[InterpolatingDouble(timestamp.value)] = observation
    }

    @Synchronized fun addObservations(timestamp: TimeMeasureSeconds, measuredVelocity: Twist2d, predictedVelocity: Twist2d) {
        addFieldToVehicleObservation(timestamp, kinematics.integrateForwardKinematics(getLatestFieldToVehicle().value, measuredVelocity))
        vehicleVelocityMeasured = measuredVelocity
        vehicleVelocityPredicted = predictedVelocity
    }

    @Synchronized fun generateOdometryFromSensors(leftEncoderDeltaDistance: Double, rightEncoderDeltaDistance: Double, currentGyroAngle: Rotation2d): Twist2d {
        val lastMeasurement = getLatestFieldToVehicle().value
        val delta = kinematics.forwardKinematics(
                lastMeasurement.rotation,
                leftEncoderDeltaDistance, rightEncoderDeltaDistance,
                currentGyroAngle
        )
        distanceDriven += delta.dx
        return delta
    }
}