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
    val kS: Double

    /**
     * Velocity gain
     */
    val kV: Double

    /**
     * Acceleration gain
     */
    val kA: Double

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
     * (geometric wheelbase)/(effective wheelbase)
     */
    val trackScrubFactor: Double
}