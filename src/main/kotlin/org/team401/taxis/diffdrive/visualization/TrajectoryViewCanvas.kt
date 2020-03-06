package org.team401.taxis.diffdrive.visualization

import org.team401.taxis.diffdrive.control.DifferentialDrivetrainModel
import org.team401.taxis.geometry.*
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.TimedState
import java.awt.*
import java.lang.RuntimeException
import java.text.DecimalFormat
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledThreadPoolExecutor
import java.util.concurrent.ThreadFactory
import java.util.concurrent.TimeUnit
import javax.swing.BorderFactory
import javax.swing.JPanel
import javax.swing.SwingUtilities
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.roundToInt
import kotlin.math.roundToLong

class TrajectoryViewCanvas(val ppi: Int, val fieldWidth: Double, val fieldHeight: Double, val trajectory: Trajectory<TimedState<Pose2dWithCurvature>>, val model: DifferentialDrivetrainModel, val fieldGeometry: List<Pair<Translation2d, Translation2d>>): JPanel(true) {
    private fun horizontalInchesToPixels(inches: Double): Int {
        return width - (ppi * inches).roundToInt()
    }

    private fun verticalInchesToPixels(inches: Double): Int {
        return height - (ppi * inches).roundToInt()
    }

    data class TrajectoryStats(val maxVel: Double, val time: Double)

    private fun computeStats(): TrajectoryStats {
        var maxVel = 0.0

        for (i in 0 until trajectory.length()) {
            val state = trajectory.getState(i)
            val velocity = Twist2d(state.velocity(), 0.0, state.velocity() * state.state().curvature)
            val wheelVelocities = model.driveKinematicsModel.inverseKinematics(velocity)
            val leftVelocity = abs(wheelVelocities.left)
            val rightVelocity = abs(wheelVelocities.right)
            if (leftVelocity > maxVel) maxVel = leftVelocity
            if (rightVelocity > maxVel) maxVel = rightVelocity
        }

        val time = trajectory.lastState.t()

        return TrajectoryStats(maxVel, time)
    }

    private val executor = ScheduledThreadPoolExecutor(1)

    inner class Simulation(val rate: Double) {
        private val iterator = TrajectoryIterator(TimedView(trajectory))

        private fun timeSeconds(): Double {
            return System.nanoTime() * 1e-9 * rate
        }

        var startTime = 0.0
        var done = false

        var latestState = Pose2d.identity()
        var latestTime = 0.0

        fun start() {
            reset()
            startTime = timeSeconds()
            //Run at 60 fps
            executor.scheduleAtFixedRate(::update, 0L, ((1000.0 / 60.0) * 1e6).roundToLong(), TimeUnit.NANOSECONDS)
        }

        fun reset() {
            done = false
            executor.queue.clear()
            SwingUtilities.invokeLater {
                latestTime = 0.0
                latestState = trajectory.firstState.state().pose
                repaint()
            }
        }

        //Updates the simulation
        private fun update() {
            var time = timeSeconds() - startTime
            if (time > iterator.remainingProgress) {
                time = iterator.remainingProgress
                done = true
            }
            val state = iterator.preview(time).state()

            SwingUtilities.invokeLater {
                latestTime = time
                latestState = state.state().pose
                repaint()
            }

            if (done) throw RuntimeException() //This is a hack to easily stop the task on the executor
        }
    }

    private var activeSimulation = Simulation(1.0)

    private val stats = computeStats()
    private val pathStroke = BasicStroke(2.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL)
    private val leftTrackTransform = Pose2d(0.0, model.geometryConstants.wheelbase.value / 2.0, Rotation2d.identity())
    private val rightTrackTransform = Pose2d(0.0, model.geometryConstants.wheelbase.value / -2.0, Rotation2d.identity())
    private val frontTransform = Pose2d(model.geometryConstants.wheelbase.value / 2.0, 0.0, Rotation2d.identity())
    private val backTransform = Pose2d(model.geometryConstants.wheelbase.value / -2.0, 0.0, Rotation2d.identity())

    private fun drawRobot(g: Graphics2D) {
        val currentPose = activeSimulation.latestState
        val horiz = horizontalInchesToPixels(currentPose.translation.y())
        val vert = verticalInchesToPixels(currentPose.translation.x())

        //Draw the dot on the robot origin

        g.color = Color.BLACK
        g.fillOval(horiz - 3, vert - 3, 6, 6)

        //Draw the robot bounding box
        val robotFrontLeft = currentPose.transformBy(frontTransform).transformBy(leftTrackTransform)
        val robotFrontRight = currentPose.transformBy(frontTransform).transformBy(rightTrackTransform)
        val robotBackLeft = currentPose.transformBy(backTransform).transformBy(leftTrackTransform)
        val robotBackRight = currentPose.transformBy(backTransform).transformBy(rightTrackTransform)

        val frontLeftHoriz = horizontalInchesToPixels(robotFrontLeft.translation.y())
        val frontLeftVert = verticalInchesToPixels(robotFrontLeft.translation.x())
        val frontRightHoriz = horizontalInchesToPixels(robotFrontRight.translation.y())
        val frontRightVert = verticalInchesToPixels(robotFrontRight.translation.x())
        val backLeftHoriz = horizontalInchesToPixels(robotBackLeft.translation.y())
        val backLeftVert = verticalInchesToPixels(robotBackLeft.translation.x())
        val backRightHoriz = horizontalInchesToPixels(robotBackRight.translation.y())
        val backRightVert = verticalInchesToPixels(robotBackRight.translation.x())

        g.drawLine(frontLeftHoriz, frontLeftVert, frontRightHoriz, frontRightVert) //front left -> front right
        g.drawLine(frontRightHoriz, frontRightVert, backRightHoriz, backRightVert) //front right -> back right
        g.drawLine(backRightHoriz, backRightVert, backLeftHoriz, backLeftVert) //back right -> back left
        g.drawLine(backLeftHoriz, backLeftVert, frontLeftHoriz, frontLeftVert) //back left -> front left
    }

    private val fmt = DecimalFormat("0.####")
    private val velGradient = GradientPaint(10f, 60f, Color.GREEN, 130f, 60f, Color.RED)

    private fun drawField(g: Graphics2D) {
        g.color = Color.RED

        fieldGeometry.forEach {
            line ->
            val lineStartHoriz = horizontalInchesToPixels(line.first.y())
            val lineStartVert = verticalInchesToPixels(line.first.x())
            val lineEndHoriz = horizontalInchesToPixels(line.second.y())
            val lineEndVert = verticalInchesToPixels(line.second.x())

            g.drawLine(lineStartHoriz, lineStartVert, lineEndHoriz, lineEndVert)
        }
    }

    private fun drawText(g: Graphics2D) {
        val time = fmt.format(activeSimulation.latestTime)
        val state = activeSimulation.latestState

        g.color = Color.BLACK
        g.drawString("time: $time s", 10, 20)
        g.drawString("pose: $state", 10, 40)

        g.paint = velGradient
        g.fillRect(10, 60, 120, 10)

        g.color = Color.BLACK
        g.drawString("${fmt.format(stats.maxVel)} in/s", 135, 70)
    }
    
    private fun drawTrajectory(g: Graphics2D) {
        g.stroke = pathStroke
        for (i in 1 until trajectory.length()) {
            val curState = trajectory.getState(i)
            val lastState = trajectory.getState(i - 1)

            val curStateLeft = curState.state().transformBy(leftTrackTransform)
            val lastStateLeft = lastState.state().transformBy(leftTrackTransform)
            val curStateRight = curState.state().transformBy(rightTrackTransform)
            val lastStateRight = lastState.state().transformBy(rightTrackTransform)

            val curHoriz = horizontalInchesToPixels(curState.state().translation.y())
            val curVert = verticalInchesToPixels(curState.state().translation.x())
            val lastHoriz = horizontalInchesToPixels(lastState.state().translation.y())
            val lastVert = verticalInchesToPixels(lastState.state().translation.x())

            val curLeftHoriz = horizontalInchesToPixels(curStateLeft.translation.y())
            val curLeftVert = verticalInchesToPixels(curStateLeft.translation.x())
            val lastLeftHoriz = horizontalInchesToPixels(lastStateLeft.translation.y())
            val lastLeftVert = verticalInchesToPixels(lastStateLeft.translation.x())

            val curRightHoriz = horizontalInchesToPixels(curStateRight.translation.y())
            val curRightVert = verticalInchesToPixels(curStateRight.translation.x())
            val lastRightHoriz = horizontalInchesToPixels(lastStateRight.translation.y())
            val lastRightVert = verticalInchesToPixels(lastStateRight.translation.x())

            val curVel = model.driveKinematicsModel.inverseKinematics(Twist2d(curState.velocity(), 0.0, curState.velocity() * curState.state().curvature))
            val leftWeight = abs(curVel.left) / stats.maxVel
            val rightWeight = abs(curVel.right) / stats.maxVel
            val leftColor = Color((255 * leftWeight).roundToInt(), (255 * (1 - leftWeight)).roundToInt(), 0)
            val rightColor = Color((255 * rightWeight).roundToInt(), (255 * (1 - rightWeight)).roundToInt(), 0)
            g.color = Color.BLUE
            g.drawLine(curHoriz, curVert, lastHoriz, lastVert)
            g.color = leftColor
            g.drawLine(curLeftHoriz, curLeftVert, lastLeftHoriz, lastLeftVert)
            g.color = rightColor
            g.drawLine(curRightHoriz, curRightVert, lastRightHoriz, lastRightVert)
        }
    }

    //Main rendering function
    override fun paintComponent(g: Graphics) {
        super.paintComponent(g)
        val g2d = g as Graphics2D
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        drawField(g2d)
        drawTrajectory(g2d)
        drawRobot(g2d)
        drawText(g2d)
    }

    fun startSimulation() {
        activeSimulation.start()
    }

    fun resetSimulation() {
        activeSimulation.reset()
    }

    init {
        preferredSize = Dimension(ceil(fieldWidth * ppi).toInt(), ceil(fieldHeight * ppi).toInt())
        border = BorderFactory.createLineBorder(Color.BLACK, 1)
        resetSimulation()
    }
}