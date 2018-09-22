package org.team401.taxis.diffdrive.autotune

import com.ctre.phoenix.motorcontrol.ControlMode
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import org.team401.taxis.physics.DriveCharacterization

/**
 * @author Cameron Earle
 * @version 9/22/2018
 *
 */
class CollectLinearStictionData(val drivetrain: TankDrivetrain,
                                val power: Double,
                                val rampRate: Double): AutoStep() {

    val data = arrayListOf<DriveCharacterization.VelocityDataPoint>()

    private var startTime = 0.0

    override fun entry(currentTime: Double) {
        data.clear()
        println("Starting linear stiction test")
        startTime = currentTime
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        val percentPower = rampRate * (currentTime - startTime)
        if (percentPower > power) {
            return true //Done
        }
        drivetrain.arcade(ControlMode.PercentOutput, percentPower, 0.0)
        data.add(DriveCharacterization.VelocityDataPoint(
                (Math.abs(drivetrain.left.getVelocity().value) + Math.abs(drivetrain.right.getVelocity().value)) / 4096.0 * Math.PI * 10,
                percentPower * 12.0
        ))
        return false
    }

    override fun exit(currentTime: Double) {
        drivetrain.stop()
    }
}