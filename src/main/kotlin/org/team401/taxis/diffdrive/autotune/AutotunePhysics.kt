package org.team401.taxis.diffdrive.autotune

/**
 * @author Cameron Earle
 * @version 9/16/18
 */
object AutotunePhysics {
    interface MotorModel {
        val freeSpeed: Double //rad/s
        val freeCurrent: Double //amps
        val maxPower: Double //watts
        val stallTorque: Double //N*m
        val stallCurrent: Double //amps
        val specVoltage: Double //volts

        fun kT() = stallTorque / (stallCurrent - freeCurrent)
        fun r() = specVoltage / stallCurrent
        fun kV() = (specVoltage - freeCurrent * r()) / freeSpeed

        fun torqueAtMotor(amps: Double): Double {
            return kT() * (amps - freeCurrent)
        }

        fun currentAtVelocityAndVoltage(velocity: Double, voltage: Double): Double {
            return (voltage - velocity * kV()) / r()
        }

        fun idealTorqueAtMotor(velocity: Double, voltage: Double): Double {
            return torqueAtMotor(currentAtVelocityAndVoltage(velocity, voltage))
        }
    }

    enum class FRCMotors(override val freeSpeed: Double,
                         override val freeCurrent: Double,
                         override val maxPower: Double,
                         override val stallTorque: Double,
                         override val stallCurrent: Double,
                         override val specVoltage: Double): MotorModel {
        MOTOR_775_PRO(
                1961.40101115,
                0.7,
                347.0,
                .71,
                134.0,
                12.0
        )
    }

    /**
     * All units SI
     */
    data class IdealTransmissionModel(val gearRatio: Double, val numMotors: Int, val motorModel: MotorModel, val wheelRadius: Double) {
        fun idealTorqueAtAxle(velocityAtAxle: Double, voltage: Double): Double {
            val velocityAtMotor = velocityAtAxle * gearRatio
            return motorModel.idealTorqueAtMotor(velocityAtMotor, voltage) * gearRatio * numMotors
        }

        fun torqueAtAxle(amps: Double): Double {
            return motorModel.torqueAtMotor(amps) * gearRatio * numMotors
        }

        fun idealForceAtGround(velocityAtAxle: Double, voltage: Double): Double {
            return idealTorqueAtAxle(velocityAtAxle, voltage) / wheelRadius
        }

        fun forceAtGround(amps: Double): Double {
            return torqueAtAxle(amps) / wheelRadius
        }
    }

    data class IdealDiffDriveModel(val leftTransmissionModel: IdealTransmissionModel,
                                   val rightTransmissionModel: IdealTransmissionModel,
                                   val wheelbaseRadius: Double) {
        fun idealTorqueAboutRobot(leftVoltage: Double, leftVelocity: Double, rightVoltage: Double, rightVelocity: Double): Double {
            return (leftTransmissionModel.idealForceAtGround(leftVelocity, leftVoltage) +
                    rightTransmissionModel.idealForceAtGround(rightVelocity, rightVoltage)) * wheelbaseRadius
        }

        fun torqueAboutRobot(leftAmps: Double, rightAmps: Double): Double {
            return (leftTransmissionModel.forceAtGround(leftAmps) +
                    rightTransmissionModel.forceAtGround(rightAmps)) * wheelbaseRadius
        }

        fun linearForce(leftAmps: Double, rightAmps: Double): Double {
            return (leftTransmissionModel.forceAtGround(leftAmps) + rightTransmissionModel.forceAtGround(rightAmps)) / 2.0
        }
    }

    fun drivetrainInertialMass(driveModel: IdealDiffDriveModel,
                               averageAcceleration: Double,
                               averageCurrentLeft: Double,
                               averageCurrentRight: Double): Double {
        return driveModel.linearForce(averageCurrentLeft, averageCurrentRight) / averageAcceleration
    }

    fun drivetrainMomentOfInertia(driveModel: IdealDiffDriveModel,
                                  averageAngularAcceleration: Double,
                                  averageCurrentLeft: Double,
                                  averageCurrentRight: Double): Double {
        return driveModel.torqueAboutRobot(averageCurrentLeft, averageCurrentRight) / averageAngularAcceleration
    }

    fun drivetrainAngularDrag(driveModel: IdealDiffDriveModel,
                              leftVelocity: Double,
                              rightVelocity: Double,
                              leftVoltage: Double,
                              rightVoltage: Double,
                              leftAmps: Double,
                              rightAmps: Double,
                              angularVelocity: Double): Double {
        return -(driveModel.idealTorqueAboutRobot(leftVoltage, leftVelocity, rightVoltage, rightVelocity) -
                 driveModel.torqueAboutRobot(leftAmps, rightAmps)) / angularVelocity
    }
}