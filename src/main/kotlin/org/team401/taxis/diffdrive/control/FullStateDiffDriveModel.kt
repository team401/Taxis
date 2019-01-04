package org.team401.taxis.diffdrive.control

import org.snakeskin.logic.LockingDelegate
import org.snakeskin.template.TankDrivetrainGeometryTemplate
import org.snakeskin.units.LinearDistanceUnit
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.taxis.diffdrive.odometry.Kinematics
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.template.DriveDynamicsTemplate
import org.team401.taxis.util.Units

/**
 * @author Cameron Earle
 * @version 1/4/19
 */
class FullStateDiffDriveModel(geometryTemplate: TankDrivetrainGeometryTemplate,
                              dynamicsTemplate: DriveDynamicsTemplate) {
    var leftTransmissionModel by LockingDelegate(DCMotorTransmission(0.0, 0.0, 0.0))
    var rightTransmissionModel by LockingDelegate(DCMotorTransmission(0.0, 0.0, 0.0))
    var drivetrainDynamicsModel by LockingDelegate(DifferentialDrive(0.0, 0.0, 0.0, 0.0, 0.0, leftTransmissionModel, rightTransmissionModel))
    var drivetrainKinematicsModel by LockingDelegate(Kinematics(LinearDistanceMeasureInches(0.0), 0.0))

    init {
        updateModel(geometryTemplate, dynamicsTemplate)
    }

    fun updateModel(geometryTemplate: TankDrivetrainGeometryTemplate, dynamicsTemplate: DriveDynamicsTemplate) {
        leftTransmissionModel = DCMotorTransmission(
                1.0 / dynamicsTemplate.leftKv,
                (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        dynamicsTemplate.inertialMass / (2.0 * dynamicsTemplate.leftKa),
                dynamicsTemplate.leftKs
        )

        rightTransmissionModel = DCMotorTransmission(
                1.0 / dynamicsTemplate.leftKv,
                (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        (geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value) *
                        dynamicsTemplate.inertialMass / (2.0 * dynamicsTemplate.leftKa),
                dynamicsTemplate.leftKs
        )

        drivetrainDynamicsModel = DifferentialDrive(
                dynamicsTemplate.inertialMass,
                dynamicsTemplate.momentOfInertia,
                dynamicsTemplate.angularDrag,
                geometryTemplate.wheelRadius.toUnit(LinearDistanceUnit.Standard.METERS).value,
                Units.inches_to_meters(geometryTemplate.wheelbase.toUnit(LinearDistanceUnit.Standard.INCHES).value / 2.0 * dynamicsTemplate.trackScrubFactor),
                leftTransmissionModel,
                rightTransmissionModel
        )

        drivetrainKinematicsModel = Kinematics(geometryTemplate.wheelbase, dynamicsTemplate.trackScrubFactor)
    }
}