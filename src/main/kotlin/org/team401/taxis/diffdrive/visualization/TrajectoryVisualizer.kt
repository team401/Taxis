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
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.template.DifferentialDrivetrainDynamicsParameters
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimedState
import org.team401.taxis.trajectory.timing.TimingConstraint
import java.awt.BorderLayout
import java.awt.event.ComponentEvent
import java.awt.event.ComponentListener
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import javax.swing.*
import javax.swing.plaf.basic.BasicSplitPaneUI
import kotlin.system.exitProcess

/**
 * Runs a trajectory visualizer on the given trajectory
 */
fun Trajectory<TimedState<Pose2dWithCurvature>>.visualize(drivetrain: IModeledDifferentialDrivetrain, fieldGeometry: List<Pair<Translation2d, Translation2d>>) {
    TrajectoryVisualizer(this, drivetrain.model, fieldGeometry).start()
}

class TrajectoryVisualizer(val trajectory: Trajectory<TimedState<Pose2dWithCurvature>>, val model: DifferentialDrivetrainModel, val fieldGeometry: List<Pair<Translation2d, Translation2d>> = listOf()) {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        frame.addWindowListener(object : WindowAdapter() {
            override fun windowClosing(e: WindowEvent?) {
                exitProcess(0) //Force the application to close (this is needed to stop the executor)
            }
        })

        val canvas = TrajectoryViewCanvas(2, 250.0, 250.0, trajectory, model, fieldGeometry)

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