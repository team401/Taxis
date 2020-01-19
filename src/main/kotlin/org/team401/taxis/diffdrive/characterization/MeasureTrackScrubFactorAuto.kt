package org.team401.taxis.diffdrive.characterization

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRevolutions
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.template.DifferentialDrivetrainGeometry

class MeasureTrackScrubFactorAuto(
        val drivetrain: IDifferentialDrivetrain,
        val geometry: DifferentialDrivetrainGeometry,
        val numTurns: AngularDistanceMeasureRevolutions = 10.0.Revolutions,
        val turnPower: Double = 0.5): RobotAuto(0.01.Seconds, 0.0.Seconds) {
    private inner class Step: AutoStep() {
        override fun entry(currentTime: TimeMeasureSeconds) {
            drivetrain.setYaw(0.0.Degrees)
            drivetrain.left.setAngularPosition(0.0.Revolutions)
            drivetrain.right.setAngularPosition(0.0.Revolutions)
            Thread.sleep(1000) //Wait some time for settings (specifically heading) to apply
        }

        override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds): Boolean {
            drivetrain.arcade(0.0, turnPower)
            val yaw = drivetrain.getYaw().toRevolutions().abs()
            if (yaw >= numTurns) {
                val leftDistance = drivetrain.left.getAngularPosition().toLinearDistance(geometry.wheelRadius).abs()
                val rightDistance = drivetrain.right.getAngularPosition().toLinearDistance(geometry.wheelRadius).abs()

                val trackWidthInches = (leftDistance.value + rightDistance.value) / yaw.toRadians().value

                println("Empirical Track Width (inches): $trackWidthInches")
                println("Track Scrub Factor: ${trackWidthInches / geometry.wheelbase.value}")

                drivetrain.stop()
                return true
            }
            return false
        }

        override fun exit(currentTime: TimeMeasureSeconds) {}
    }

    override fun assembleAuto() = SequentialSteps(Step())
}