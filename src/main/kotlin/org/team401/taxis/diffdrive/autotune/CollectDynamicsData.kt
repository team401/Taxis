package org.team401.taxis.diffdrive.autotune

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotController
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.component.ISmartGearbox

/**
 * @author Cameron Earle
 * @version 12/4/2018
 *
 */
class CollectDynamicsData(val drivetrain: IDifferentialDrivetrain<ISmartGearbox<*>>): AutoStep() {
    private val numberArray = Array<Number>(9) { 0 }
    private var priorAutospeed = 0.0

    private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
    private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

    override fun entry(currentTime: Double) {
        drivetrain.left.setDeadband(0.0)
        drivetrain.right.setDeadband(0.0)
        drivetrain.both {
            setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        }

        NetworkTableInstance.getDefault().setUpdateRate(0.010)
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        val leftPosition = drivetrain.left.getPosition().value
        val leftRate = drivetrain.left.getVelocity().value

        val rightPosition = drivetrain.right.getPosition().value
        val rightRate = drivetrain.right.getVelocity().value

        val battery = RobotController.getBatteryVoltage()

        val leftMotorVolts = drivetrain.left.getOutputVoltage()
        val rightMotorVolts = drivetrain.right.getOutputVoltage()

        // Retrieve the commanded speed from NetworkTables
        val autospeed = autoSpeedEntry.getDouble(0.0)
        priorAutospeed = autospeed

        // command motors to do things
        drivetrain.arcade(autospeed, 0.0)

        // send telemetry data array back to NT
        numberArray[0] = currentTime
        numberArray[1] = battery
        numberArray[2] = autospeed
        numberArray[3] = leftMotorVolts
        numberArray[4] = rightMotorVolts
        numberArray[5] = leftPosition
        numberArray[6] = rightPosition
        numberArray[7] = leftRate
        numberArray[8] = rightRate

        telemetryEntry.setNumberArray(numberArray)

        return false
    }

    override fun exit(currentTime: Double) {
        drivetrain.stop()
    }
}