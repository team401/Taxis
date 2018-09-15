package org.team401.taxis.diffdrive.control

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 * Aggregate class representing the output signal for a differential drive.
 * Contains velocity error commands for the left and right sides, as well as
 * acceleration and feed-forward parameters, which together represent a single
 * drive command.
 *
 * Velocity units are in inches per second.  Unit classes are not used for simplicity
 */
data class Output(val leftVelocity: Double = 0.0,
                  val rightVelocity: Double = 0.0,
                  val leftAccel: Double = 0.0,
                  val rightAccel: Double = 0.0,
                  val leftFeedforwardVoltage: Double = 0.0,
                  val rightFeedforwardVoltage: Double = 0.0)