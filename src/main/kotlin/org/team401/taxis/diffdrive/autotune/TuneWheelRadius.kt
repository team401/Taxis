package org.team401.taxis.diffdrive.autotune

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.measure.MeasureUnitless
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches

/**
 * @author Cameron Earle
 * @version 9/16/18
 */
class TuneWheelRadius(val drive: IDifferentialDrivetrain<ISensoredGearbox>, val distance: LinearDistanceMeasureInches): AutoStep() {
    override fun entry(currentTime: Double) {
        drive.left.setPosition(AngularDistanceMeasureRadians(0.0))
        drive.right.setPosition(AngularDistanceMeasureRadians(0.0))
        println("Entering Wheel Radius tuning mode.  Please push the robot $distance")
        println("Once at the distance, disable the robot to calculate the radius.")
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        //no-op
        return false //Never signal that we are complete, so that we hold the execution until the user disables the robot
    }

    override fun exit(currentTime: Double) {
        val wheelRadians = (drive.left.getPosition().value
                + drive.right.getPosition().value) / 2.0

        val distanceInches = distance.value

        val radiusInches = distanceInches / wheelRadians

        println("Wheel Radius (inches): $radiusInches")
    }
}