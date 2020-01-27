package org.team401.taxis.diffdrive.visualization

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.template.DifferentialDrivetrainGeometry
import org.team401.taxis.diffdrive.component.IModeledDifferentialDrivetrain
import org.team401.taxis.diffdrive.control.DifferentialDrivetrainModel
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.FeedforwardOnlyPathController
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.template.DifferentialDrivetrainDynamicsParameters
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimedState
import org.team401.taxis.trajectory.timing.TimingConstraint
import java.awt.BorderLayout
import java.awt.event.ComponentEvent
import java.awt.event.ComponentListener
import javax.swing.*
import javax.swing.plaf.basic.BasicSplitPaneUI

/**
 * Runs a trajectory visualizer on the given trajectory
 */
fun Trajectory<TimedState<Pose2dWithCurvature>>.visualize(drivetrain: IModeledDifferentialDrivetrain) {
    TrajectoryVisualizer(this, drivetrain.model).start()
}

class TrajectoryVisualizer(val trajectory: Trajectory<TimedState<Pose2dWithCurvature>>, val model: DifferentialDrivetrainModel) {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        val canvas = TrajectoryViewCanvas(2, 250.0, 250.0, trajectory, model)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.X_AXIS)

        val playButton = JButton("Simulate")
        playButton.addActionListener { canvas.startSimulation() }
        val resetButton = JButton("Reset")
        resetButton.addActionListener { canvas.resetSimulation() }

        buttonPanel.add(playButton)
        buttonPanel.add(resetButton)

        frame.layout = BoxLayout(frame.contentPane, BoxLayout.Y_AXIS)

        frame.contentPane.add(canvas)
        frame.contentPane.add(buttonPanel)

        SwingUtilities.invokeLater {
            frame.pack()
            frame.isVisible = true
        }
    }
}