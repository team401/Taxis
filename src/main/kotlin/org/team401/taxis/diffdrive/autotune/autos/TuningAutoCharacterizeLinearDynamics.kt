package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.DelayStep
import org.snakeskin.auto.steps.LambdaStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.measure.time.TimeMeasure
import org.snakeskin.units.measure.time.TimeMeasureSeconds
import org.team401.taxis.diffdrive.autotune.AutotunePhysics
import org.team401.taxis.diffdrive.autotune.CollectLinearAccelerationData
import org.team401.taxis.diffdrive.autotune.CollectLinearStictionData
import org.team401.taxis.physics.DriveCharacterization

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 * Characterizes the kS, kV, and kA gains by driving in a straight line.
 * Make sure you have sufficient room for the robot to accelerate!
 */
class TuningAutoCharacterizeLinearDynamics(val drivetrain: TankDrivetrain,
                                           val driveModel: AutotunePhysics.IdealDiffDriveModel,
                                           val stictionMaxPower: Double,
                                           val stictionRampRate: Double,
                                           val accelerationPower: Double,
                                           val accelerationTime: TimeMeasure): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        val collectStiction = CollectLinearStictionData(drivetrain, stictionMaxPower, stictionRampRate)
        val collectAcceleration = CollectLinearAccelerationData(drivetrain, accelerationPower, accelerationTime)
        return SequentialSteps(
                collectStiction,
                DelayStep(TimeMeasureSeconds(5.0)),
                collectAcceleration,
                LambdaStep {
                    val characterization = DriveCharacterization.characterizeDrive(
                            collectStiction.data,
                            collectAcceleration.data.map { it.dp }
                    )
                    val inertialMass = AutotunePhysics.drivetrainInertialMass(
                            driveModel,
                            collectAcceleration.data.map { it.dp.acceleration }.average(),
                            collectAcceleration.data.map { it.current }.average(),
                            collectAcceleration.data.map { it.current }.average()
                    )
                    println("kS: ${characterization.ks}")
                    println("kV: ${characterization.kv}")
                    println("kA: ${characterization.ka}")
                    println("Inertial Mass: $inertialMass")
                }
        )
    }

}