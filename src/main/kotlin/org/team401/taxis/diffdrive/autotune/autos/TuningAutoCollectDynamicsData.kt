package org.team401.taxis.diffdrive.autotune.autos

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.ICTREGearbox
import org.snakeskin.component.IDifferentialDrivetrain
import org.team401.taxis.diffdrive.autotune.CollectDynamicsDataCTRE

/**
 * @author Cameron Earle
 * @version 12/4/2018
 *
 */
class TuningAutoCollectDynamicsData(val drivetrain: IDifferentialDrivetrain<ICTREGearbox>): RobotAuto() {
    override fun assembleAuto(): SequentialSteps {
        return SequentialSteps(CollectDynamicsDataCTRE(drivetrain))
    }
}