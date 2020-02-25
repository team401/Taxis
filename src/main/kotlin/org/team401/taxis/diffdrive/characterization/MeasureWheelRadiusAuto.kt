package org.team401.taxis.diffdrive.characterization

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.measure.Radians
import org.snakeskin.measure.Revolutions
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.Unitless
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.time.TimeMeasureSeconds

class MeasureWheelRadiusAuto(val drivetrain: IDifferentialDrivetrain, val pushDistance: LinearDistanceMeasureInches): RobotAuto(preRate = 0.0.Seconds) {
    private inner class Step: AutoStep() {
        override fun entry(currentTime: TimeMeasureSeconds) {
            drivetrain.left.setAngularPosition(0.0.Radians)
            drivetrain.right.setAngularPosition(0.0.Radians)
            println("Entering Wheel Radius measurement mode.  Please push the robot $pushDistance")
            println("Once at the distance, disable the robot to calculate the radius.")
        }

        override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds) = false

        override fun exit(currentTime: TimeMeasureSeconds) {
            val avgDistance = (drivetrain.left.getAngularPosition() + drivetrain.right.getAngularPosition()) / 2.0.Unitless

            val radiusInches = pushDistance.value / avgDistance.toRadians().value
            println("Wheel Radius (inches): $radiusInches")
        }
    }

    override fun assembleAuto() = SequentialSteps(Step())
}