package org.team401.taxis.diffdrive.autotune

import com.ctre.phoenix.motorcontrol.ControlMode
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.hardware.Hardware
import org.snakeskin.units.AngularDistanceUnit
import org.snakeskin.units.AngularVelocityUnit
import org.team401.taxis.physics.DriveCharacterization

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 * Collects data on the "stiction" of the drive.  This is used to determine the voltage required to break static friction,
 * and in turn tune the kS gain of the characterized drivetrain
 */
class CollectLinearStictionData(val drivetrain: TankDrivetrain,
                                val maxPower: Double,
                                val rampRate: Double): AutoStep() {
    private var startTime = 0.0

    val data = arrayListOf<DriveCharacterization.VelocityDataPoint>()

    override fun entry(currentTime: Double) {
        println("Collecting linear stiction data")
        data.clear()
        startTime = currentTime
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        val percentOut = rampRate * (currentTime - startTime)
        if (percentOut > maxPower) {
            return true //We are done collecting data
        }
        drivetrain.tank(ControlMode.PercentOutput, percentOut, percentOut)
        data.add(DriveCharacterization.VelocityDataPoint(
                (Math.abs(drivetrain.left.getVelocity().toUnit(AngularVelocityUnit.Standard.RADIANS_PER_SECOND).value) +
                        Math.abs(drivetrain.right.getVelocity().toUnit(AngularVelocityUnit.Standard.RADIANS_PER_SECOND).value)) / 2.0,
                12.0 * percentOut //Assume nominal voltage?
        ))
        return false
    }

    override fun exit(currentTime: Double) {
        drivetrain.stop()
        println("Done collecting linear stiction data")
    }

}