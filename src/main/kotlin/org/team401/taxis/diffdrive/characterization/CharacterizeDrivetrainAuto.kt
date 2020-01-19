package org.team401.taxis.diffdrive.characterization

import edu.wpi.first.networktables.NetworkTableInstance
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.IDifferentialDrivetrain
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.runtime.SnakeskinRuntime
import kotlin.math.absoluteValue

/**
 * Characterization auto routine, designed to be used with WPILib's "frc-characterization" tool.
 * Sends data about a differential drivetrain in radians and radians per second.  Ensure you select "Radians" in the tool.
 *
 * @param drivetrain The drivetrain to characterize
 * @param rate The rate to collect data at.  Defaults to collecting every 10 milliseconds
 */
class CharacterizeDrivetrainAuto(val drivetrain: IDifferentialDrivetrain, rate: TimeMeasureSeconds = 0.01.Seconds): RobotAuto(rate, 0.0.Seconds) {
    private inner class Step: AutoStep() {
        private var prevCommand = 0.0
        private val arr = DoubleArray(10)

        private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
        private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")
        private val rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate")

        override fun entry(currentTime: TimeMeasureSeconds) {
            prevCommand = 0.0
            NetworkTableInstance.getDefault().setUpdateRate(0.01)
        }

        override fun action(currentTime: TimeMeasureSeconds, lastTime: TimeMeasureSeconds): Boolean {
            //Read left pos and vel (rad, rad/s)
            val leftPosition = drivetrain.left.getAngularPosition().toRadians()
            val leftVelocity = drivetrain.left.getAngularVelocity().toRadiansPerSecond()

            //Read right pos and vel (rad, rad/s)
            val rightPosition = drivetrain.right.getAngularPosition().toRadians()
            val rightVelocity = drivetrain.right.getAngularVelocity().toRadiansPerSecond()

            //Read robot heading in radians
            val heading = drivetrain.getYaw().toRadians()

            //Read voltage and command
            val battery = SnakeskinRuntime.voltage
            val motorVolts = battery * prevCommand.absoluteValue

            //Get new command from NT
            val command = autoSpeedEntry.getDouble(0.0)
            prevCommand = command

            //Apply drive commands
            val rotate = rotateEntry.getBoolean(false)
            drivetrain.tank(
                    if (rotate) -command else command,
                    command
            )

            //Send telemetry
            arr[0] = currentTime.value    //Time in seconds
            arr[1] = battery              //Battery voltage
            arr[2] = command              //Applied command
            arr[3] = motorVolts           //Applied voltage (left)
            arr[4] = motorVolts           //Applied voltage (right)
            arr[5] = leftPosition.value   //Left position (radians)
            arr[6] = rightPosition.value  //Right position (radians)
            arr[7] = leftVelocity.value   //Left velocity (radians/s)
            arr[8] = rightVelocity.value  //Right velocity (radians/s)
            arr[9] = heading.value        //Heading (radians)

            telemetryEntry.setDoubleArray(arr)

            return false //This step never finishes by itself
        }

        override fun exit(currentTime: TimeMeasureSeconds) {
            drivetrain.stop()
        }
    }

    override fun assembleAuto() = SequentialSteps(Step())
}
