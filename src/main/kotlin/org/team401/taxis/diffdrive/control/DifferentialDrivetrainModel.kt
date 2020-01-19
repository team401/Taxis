package org.team401.taxis.diffdrive.control

import org.snakeskin.template.DifferentialDrivetrainGeometry
import org.team401.taxis.diffdrive.odometry.DifferentialDrivetrainKinematics
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrivetrainDynamics
import org.team401.taxis.template.DifferentialDrivetrainDynamicsParameters

/**
 * Full model of a differential drivetrain.  Models both the dynamics and the kinematics of a differential drivetrain.
 */
class DifferentialDrivetrainModel(geometry: DifferentialDrivetrainGeometry,
                                  dynamics: DifferentialDrivetrainDynamicsParameters) {

    /**
     * Model of the left side transmission of the robot drivetrain
     */
    val leftModel = DCMotorTransmission(
            1.0 / dynamics.leftKv,
            (geometry.wheelRadius.toMeters().value) *
                    (geometry.wheelRadius.toMeters().value) *
                    dynamics.inertialMass / (2.0 * dynamics.leftKa),
            dynamics.leftKs
    )

    /**
     * Model of the right side transmission of the robot drivetrain
     */
    val rightModel = DCMotorTransmission(
            1.0 / dynamics.rightKv,
            (geometry.wheelRadius.toMeters().value) *
                    (geometry.wheelRadius.toMeters().value) *
                    dynamics.inertialMass / (2.0 * dynamics.rightKa),
            dynamics.rightKs
    )

    /**
     * Model of the dynamics of the whole drivetrain
     */
    val driveDynamicsModel = DifferentialDrivetrainDynamics(
            dynamics.inertialMass,
            dynamics.momentOfInertia,
            dynamics.angularDrag,
            geometry.wheelRadius.toMeters().value,
            geometry.wheelbase.toMeters().value / 2.0 * dynamics.trackScrubFactor,
            leftModel,
            rightModel
    )

    /**
     * Model of the kinematics of the whole drivetrain
     */
    val driveKinematicsModel = DifferentialDrivetrainKinematics(geometry.wheelbase, dynamics.trackScrubFactor)
}