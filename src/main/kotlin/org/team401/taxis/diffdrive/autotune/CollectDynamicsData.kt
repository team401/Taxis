package org.team401.taxis.diffdrive.autotune

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.networktables.NetworkTableInstance
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.TankDrivetrain
import edu.wpi.first.wpilibj.RobotController
import org.snakeskin.units.Radians
import org.snakeskin.units.RadiansPerSecond
import edu.wpi.first.networktables.NetworkTableEntry




/**
 * @author Cameron Earle
 * @version 12/4/2018
 *
 */
class CollectDynamicsData(val drivetrain: TankDrivetrain): AutoStep() {
    private val numberArray = Array<Number>(9) { 0 }
    private var priorAutospeed = 0.0

    private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
    private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

    override fun entry(currentTime: Double) {
        drivetrain.left.master.configNeutralDeadband(0.0, 0)
        drivetrain.right.master.configNeutralDeadband(0.0, 0)
        drivetrain.setNeutralMode(NeutralMode.Brake)

        NetworkTableInstance.getDefault().setUpdateRate(0.010)
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        val leftPosition = drivetrain.left.getPosition().toUnit(Radians).value
        val leftRate = drivetrain.left.getVelocity().toUnit(RadiansPerSecond).value

        val rightPosition = drivetrain.right.getPosition().toUnit(Radians).value
        val rightRate = drivetrain.right.getVelocity().toUnit(RadiansPerSecond).value

        val battery = RobotController.getBatteryVoltage()

        val leftMotorVolts = drivetrain.left.master.motorOutputVoltage
        val rightMotorVolts = drivetrain.right.master.motorOutputVoltage

        // Retrieve the commanded speed from NetworkTables
        val autospeed = autoSpeedEntry.getDouble(0.0)
        priorAutospeed = autospeed

        // command motors to do things
        drivetrain.arcade(ControlMode.PercentOutput, autospeed, 0.0)

        // send telemetry data array back to NT
        numberArray[0] = currentTime
        numberArray[1] = battery
        numberArray[2] = autospeed
        numberArray[3] = leftMotorVolts
        numberArray[4] = rightMotorVolts
        numberArray[5] = leftPosition
        numberArray[6] = leftRate
        numberArray[7] = rightPosition
        numberArray[8] = rightRate

        telemetryEntry.setNumberArray(numberArray)

        return false
    }

    override fun exit(currentTime: Double) {
        drivetrain.stop()
    }
}