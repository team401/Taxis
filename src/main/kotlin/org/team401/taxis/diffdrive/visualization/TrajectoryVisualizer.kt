package org.team401.taxis.diffdrive.visualization

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.template.DifferentialDrivetrainGeometry
import org.team401.taxis.diffdrive.control.DifferentialDrivetrainModel
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.FeedforwardOnlyPathController
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.template.DifferentialDrivetrainDynamicsParameters
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimingConstraint
import java.awt.event.ComponentEvent
import java.awt.event.ComponentListener
import javax.swing.JFrame
import javax.swing.SwingUtilities

class TrajectoryVisualizer {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        val model = DifferentialDrivetrainModel(
                object : DifferentialDrivetrainGeometry {
                    override val wheelRadius = 3.0.Inches
                    override val wheelbase = 20.0.Inches

                },
                object : DifferentialDrivetrainDynamicsParameters {
                    override val leftKs = .1
                    override val leftKv = .1
                    override val leftKa = .1
                    override val rightKs = .1
                    override val rightKv = .1
                    override val rightKa = .1
                    override val inertialMass = 40.0
                    override val momentOfInertia = 4.0
                    override val angularDrag = 0.0
                    override val trackScrubFactor = 1.0
                }
        )
        val pm = DrivetrainPathManager(
                model,
                FeedforwardOnlyPathController()
        )

        val traj = pm.generateTrajectory(
                false,
                listOf(
                        Pose2d(12.0, 2.0 * 12.0, Rotation2d.identity()),
                        Pose2d(10.0 * 12.0, 8.0 * 12.0, Rotation2d.fromDegrees(90.0)),
                        Pose2d(15.0 * 12.0, 15.0 * 12.0, Rotation2d.identity())
                ),
                listOf<TimingConstraint<Pose2dWithCurvature>>(CentripetalAccelerationConstraint(100.0)),
                10.0 * 12.0,
                20.0 * 12.0,
                9.0
        )

        frame.add(TrajectoryViewCanvas(2, 250.0, 250.0, traj, model))
        SwingUtilities.invokeLater {
            frame.pack()
            frame.isVisible = true
        }
    }
}

fun main() {
    TrajectoryVisualizer().start()
}