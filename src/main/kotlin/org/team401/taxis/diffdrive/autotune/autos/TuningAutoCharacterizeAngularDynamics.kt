package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.LambdaStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.diffdrive.autotune.AutotunePhysics
import org.team401.taxis.diffdrive.autotune.CollectAngularData

/**
 * @author Cameron Earle
 * @version 9/17/2018
 *
 */
class TuningAutoCharacterizeAngularDynamics(val drivetrain: TankDrivetrain,
                                            val driveModel: AutotunePhysics.IdealDiffDriveModel,
                                            val power: Double,
                                            val polarity: Double,
                                            val accelerationTime: TimeMeasure): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        val angularData = CollectAngularData(drivetrain, power, polarity, accelerationTime)
        return SequentialSteps(
                angularData,
                LambdaStep {
                    val moi = AutotunePhysics.drivetrainMomentOfInertia(
                            driveModel,
                            angularData.data.map { it.angularAcceleration }.average(),
                            angularData.data.map { it.current }.average(),
                            angularData.data.map { it.current }.average()
                    )

                    val angularDrag = AutotunePhysics.drivetrainAngularDrag(
                            driveModel,
                            angularData.data.map { it.velocity }.last(),
                            angularData.data.map { it.velocity }.last(),
                            angularData.data.map { it.voltage }.last(),
                            angularData.data.map { it.voltage }.last(),
                            angularData.data.map { it.current }.last(),
                            angularData.data.map { it.current }.last(),
                            angularData.data.map { it.angularVelocity }.last()
                    )

                    println("MOI: $moi")
                    println("Angular Drag: $angularDrag")
                }
        )
    }

}