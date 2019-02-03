package org.team401.taxis.diffdrive.autotune

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.units.AngularDistanceUnit
import org.snakeskin.units.LinearDistanceUnit
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasure

/**
 * @author Cameron Earle
 * @version 9/16/18
 */
class TuneWheelRadius(val drive: IDifferentialDrivetrain<ISensoredGearbox>, val distance: LinearDistanceMeasure): AutoStep() {
    override fun entry(currentTime: Double) {
        drive.left.setPosition(AngularDistanceMeasureCTREMagEncoder(0.0))
        drive.right.setPosition(AngularDistanceMeasureCTREMagEncoder(0.0))
        println("Entering Wheel Radius tuning mode.  Please push the robot ${distance.value} ${distance.unit.toString().toLowerCase()}")
        println("Once at the distance, disable the robot to calculate the radius.")
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        //no-op
        return false //Never signal that we are complete, so that we hold the execution until the user disables the robot
    }

    override fun exit(currentTime: Double) {
        val wheelRadians = (drive.left.getPosition().toUnit(AngularDistanceUnit.Standard.RADIANS).value
                + drive.right.getPosition().toUnit(AngularDistanceUnit.Standard.RADIANS).value) / 2.0

        val distanceInches = distance.toUnit(LinearDistanceUnit.Standard.INCHES).value

        val radiusInches = distanceInches / wheelRadians

        println("Wheel Radius (inches): $radiusInches")
    }
}