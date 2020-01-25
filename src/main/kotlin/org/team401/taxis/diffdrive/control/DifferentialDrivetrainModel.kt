package org.team401.taxis.diffdrive.control

import org.snakeskin.template.DifferentialDrivetrainGeometry
import org.team401.taxis.diffdrive.odometry.DifferentialDrivetrainKinematics
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrivetrainDynamics
import org.team401.taxis.template.DifferentialDrivetrainDynamicsParameters

/**
 * Full model of a differential drivetrain.  Models both the dynamics and the kinematics of a differential drivetrain.
 */
class DifferentialDrivetrainModel(val geometryConstants: DifferentialDrivetrainGeometry,
                                  val dynamicsConstants: DifferentialDrivetrainDynamicsParameters) {

    /**
     * Model of the left side transmission of the robot drivetrain
     */
    val leftModel = DCMotorTransmission(
            1.0 / dynamicsConstants.leftKv,
            (geometryConstants.wheelRadius.toMeters().value) *
                    (geometryConstants.wheelRadius.toMeters().value) *
                    dynamicsConstants.inertialMass / (2.0 * dynamicsConstants.leftKa),
            dynamicsConstants.leftKs
    )

    /**
     * Model of the right side transmission of the robot drivetrain
     */
    val rightModel = DCMotorTransmission(
            1.0 / dynamicsConstants.rightKv,
            (geometryConstants.wheelRadius.toMeters().value) *
                    (geometryConstants.wheelRadius.toMeters().value) *
                    dynamicsConstants.inertialMass / (2.0 * dynamicsConstants.rightKa),
            dynamicsConstants.rightKs
    )

    /**
     * Model of the dynamics of the whole drivetrain
     */
    val driveDynamicsModel = DifferentialDrivetrainDynamics(
            dynamicsConstants.inertialMass,
            dynamicsConstants.momentOfInertia,
            dynamicsConstants.angularDrag,
            geometryConstants.wheelRadius.toMeters().value,
            geometryConstants.wheelbase.toMeters().value / 2.0 * dynamicsConstants.trackScrubFactor,
            leftModel,
            rightModel
    )

    /**
     * Model of the kinematics of the whole drivetrain
     */
    val driveKinematicsModel = DifferentialDrivetrainKinematics(geometryConstants.wheelbase, dynamicsConstants.trackScrubFactor)
}