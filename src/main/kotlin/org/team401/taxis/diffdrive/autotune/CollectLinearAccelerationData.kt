package org.team401.taxis.diffdrive.autotune

import com.ctre.phoenix.motorcontrol.ControlMode
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.hardware.Hardware
import org.snakeskin.units.AngularVelocityUnit
import org.snakeskin.units.TimeUnit
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.physics.DriveCharacterization
import org.team401.taxis.util.Util

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 */
class CollectLinearAccelerationData(val drivetrain: TankDrivetrain,
                                    val power: Double,
                                    val runtime: TimeMeasure): AutoStep() {

    private var startTime = 0.0
    private var prevVelocity = 0.0
    private var prevTime = 0.0

    data class AccelerationPacket(val dp: DriveCharacterization.AccelerationDataPoint,
                                  val timestamp: Double,
                                  val current: Double,
                                  val included: Boolean) {
        fun toCSV(): String {
            return "$timestamp,${dp.acceleration},${dp.power},${dp.velocity},$current,$included"
        }
    }

    val data = arrayListOf<AccelerationPacket>()

    override fun entry(currentTime: Double) {
        println("Collecting linear acceleration data")
        data.clear()
        startTime = currentTime
        prevTime = startTime
        prevVelocity = 0.0
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        if (currentTime - startTime > runtime.toUnit(TimeUnit.Standard.SECONDS).value) {
            return true
        }
        drivetrain.tank(ControlMode.PercentOutput, power, power)
        val currentVelocity = (Math.abs(drivetrain.left.getVelocity().toUnit(AngularVelocityUnit.Standard.RADIANS_PER_SECOND).value) +
                              Math.abs(drivetrain.right.getVelocity().toUnit(AngularVelocityUnit.Standard.RADIANS_PER_SECOND).value)) / 2.0
        val avgCurrent = (drivetrain.left.master.outputCurrent + drivetrain.right.master.outputCurrent) / 2.0

        if (lastTime == startTime) {
            prevTime = currentTime
            prevVelocity = currentVelocity
            return false //Initialize all variables
        }

        val acceleration = (currentVelocity - prevVelocity) / (currentTime - prevTime)

        data.add(AccelerationPacket(DriveCharacterization.AccelerationDataPoint(
                currentVelocity,
                power * 12,
                acceleration
        ), currentTime - startTime, avgCurrent, acceleration >= Util.kEpsilon))

        prevTime = currentTime
        prevVelocity = currentVelocity
        return false
    }

    override fun exit(currentTime: Double) {
        drivetrain.stop()
        println("Done collecting linear acceleration data")
    }
}