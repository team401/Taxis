package org.team401.taxis.template

/**
 * @author Cameron Earle
 * @version 8/30/2018
 *
 */
interface DriveDynamicsTemplate {
    /**
     * Voltage "intercept" gain
     */
    val leftKs: Double

    /**
     * Velocity gain
     */
    val leftKv: Double

    /**
     * Acceleration gain
     */
    val leftKa: Double

    /**
     * Voltage "intercept" gain
     */
    val rightKs: Double

    /**
     * Velocity gain
     */
    val rightKv: Double

    /**
     * Acceleration gain
     */
    val rightKa: Double

    /**
     * Robot inertial mass, in kg
     */
    val inertialMass: Double

    /**
     * Robot moment of inertia, in kg*m^2
     */
    val momentOfInertia: Double

    /**
     * Robot angular drag, in N*m/rad/s
     */
    val angularDrag: Double

    /**
     * (effective wheelbase) / (geometric wheelbase).  Units do not matter as long as they are the same
     */
    val trackScrubFactor: Double
}