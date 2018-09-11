package org.team401.taxis.diffdrive

import org.snakeskin.units.LinearDistanceUnit
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.physics.DCMotorTransmission
import org.team401.taxis.physics.DifferentialDrive
import org.team401.taxis.template.DriveDynamicsTemplate
import org.team401.taxis.template.DriveGeometryTemplate
import org.team401.taxis.template.PathFollowingTemplate
import org.team401.taxis.trajectory.*
import org.team401.taxis.trajectory.timing.DifferentialDriveDynamicsConstraint
import org.team401.taxis.trajectory.timing.TimedState
import org.team401.taxis.trajectory.timing.TimingConstraint
import org.team401.taxis.trajectory.timing.TimingUtil
import org.team401.taxis.util.Units
import org.team401.taxis.util.Util

/**
 * @author Cameron Earle
 * @version 9/10/2018
 *
 * Manages path following for a drivetrain.  This class does not interact with any hardware,
 * and instead must be provided with the inputs by calling appropriate functions.
 *
 * @param geometryConfig The geometry data
 * @param dynamicsConfig The dynamics data
 * @param pathFollowingConfig The path following data
 * @param controller The drive controller to use
 */
class DrivetrainPathManager(val geometryConfig: DriveGeometryTemplate,
                            val dynamicsConfig: DriveDynamicsTemplate,
                            val pathFollowingConfig: PathFollowingTemplate,
                            val controller: PathController) {

    private val motorModel = DCMotorTransmission(
            1.0 / dynamicsConfig.kV,
            (geometryConfig.wheelDiameter.toUnit(LinearDistanceUnit.Standard.METERS).value / 2.0) *
                    (geometryConfig.wheelDiameter.toUnit(LinearDistanceUnit.Standard.METERS).value / 2.0) *
                    dynamicsConfig.inertialMass / (2.0 * dynamicsConfig.kA),
            dynamicsConfig.kV
    )

    /**
     * The constructed drivetrain model, made public for possible future use
     */
    val model = DifferentialDrive(
            dynamicsConfig.inertialMass,
            dynamicsConfig.momentOfInertia,
            dynamicsConfig.angularDrag,
            geometryConfig.wheelDiameter.toUnit(LinearDistanceUnit.Standard.METERS).value / 2.0,
            LinearDistanceMeasureInches(
                    geometryConfig.wheelDiameter.toUnit(LinearDistanceUnit.Standard.INCHES).value
                            / 2.0 * dynamicsConfig.trackScrubFactor).toUnit(LinearDistanceUnit.Standard.METERS).value,
            motorModel,
            motorModel
    )

    private val trajectories = hashMapOf<String, Trajectory<TimedState<Pose2dWithCurvature>>>()

    //Some process variables
    private var currentTrajectory: TrajectoryIterator<TimedState<Pose2dWithCurvature>>? = null
    private var reversed = false
    private var lastTime = Double.POSITIVE_INFINITY
    var setpoint = TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity())
        @Synchronized get
        private set
    var error = Pose2d.identity()
        @Synchronized get
        private set
    private var output = Output()
    private var dt = 0.0

    @Synchronized fun reset() {
        error = Pose2d.identity()
        output = Output()
        lastTime = Double.POSITIVE_INFINITY
        controller.reset()
    }

    /**
     * Generates a trajectory from a path
     */
    private fun generateTrajectory(path: Path): Trajectory<TimedState<Pose2dWithCurvature>> {
        val flip = Pose2d.fromRotation(Rotation2d(-1.0, 0.0, false))
        val waypoints = if (reversed) {
            path.waypoints.map { it.transformBy(flip) }
        } else {
            path.waypoints
        }

        var trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypoints,
                pathFollowingConfig.maxErrorX,
                pathFollowingConfig.maxErrorY,
                pathFollowingConfig.maxErrorTheta
        )

        if (reversed) {
            val flipped = ArrayList<Pose2dWithCurvature>(trajectory.length())
            for (i in 0 until trajectory.length()) {
                flipped.add(
                        Pose2dWithCurvature(
                                trajectory.getState(i).pose.transformBy(flip),
                                -trajectory.getState(i).curvature,
                                trajectory.getState(i).dCurvatureDs
                        )
                )
            }
            trajectory = Trajectory(flipped)
        }

        val driveConstraints = DifferentialDriveDynamicsConstraint<Pose2dWithCurvature>(model, path.maxVoltage)
        val allConstraints = arrayListOf<TimingConstraint<Pose2dWithCurvature>>()
        allConstraints.add(driveConstraints)
        allConstraints.addAll(path.constraints)

        return TimingUtil.timeParameterizeTrajectory(
                reversed,
                DistanceView(trajectory),
                pathFollowingConfig.maxErrorX,
                allConstraints,
                path.startVelocity,
                path.endVelocity,
                path.maxVelocity,
                path.maxAcceleration
        )
    }

    /**
     * Sets the current trajectory to follow, given an iterator
     */
    private fun setTrajectory(trajectory: TrajectoryIterator<TimedState<Pose2dWithCurvature>>) {
        currentTrajectory = trajectory
        setpoint = trajectory.state
        for (i in 0 until trajectory.trajectory().length()) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                reversed = false
                break
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                reversed = true
                break
            }
        }
    }

    /**
     * Adds a path to the path manager
     *
     * @param name The name of the path.  Will be used to recall the path to follow it
     * @param path The path
     */
    @Synchronized fun addPath(name: String, path: Path) {
        trajectories[name] = generateTrajectory(path)
    }

    /**
     * Sets the path by the given name as the current path.
     *
     * @param name The name of the path to follow
     */
    @Synchronized fun setPath(name: String) {
        if (trajectories.contains(name)) {
            val iterator = TrajectoryIterator(TimedView(trajectories[name]))
            setTrajectory(iterator)
        } else {
            throw NoSuchElementException("Could not find path '$name'")
        }
    }

    /**
     * Updates the path follower with new state information.
     * This information is passed to the controller
     */
    @Synchronized fun update(timestamp: Double, currentState: Pose2d): Output {
        if (currentTrajectory == null) return Output()
        val currentTrajectory = currentTrajectory!! //Bypass null checks

        if (currentTrajectory.progress == 0.0 && !java.lang.Double.isFinite(lastTime)) {
            lastTime = timestamp
        }

        dt = timestamp - lastTime
        lastTime = timestamp
        val samplePoint = currentTrajectory.advance(dt)
        setpoint = samplePoint.state()

        if (!currentTrajectory.isDone) {
            //Generate feedforward voltages
            val velocityM = Units.inches_to_meters(setpoint.velocity())
            val curvatureM = Units.meters_to_inches(setpoint.state().curvature)
            val dCurvatureDsM = Units.meters_to_inches(Units.meters_to_inches(setpoint.state().dCurvatureDs))
            val accelerationM = Units.inches_to_meters(setpoint.acceleration())
            val dynamics = model.solveInverseDynamics(
                    DifferentialDrive.ChassisState(velocityM, velocityM * curvatureM),
                    DifferentialDrive.ChassisState(accelerationM,
                            accelerationM * curvatureM + velocityM * velocityM * dCurvatureDsM)
            )
            error = currentState.inverse().transformBy(setpoint.state().pose)
            output = controller.update(
                    dt,
                    dynamics,
                    currentState,
                    model,
                    currentTrajectory,
                    setpoint,
                    error,
                    reversed
            )
        } else {
            output = Output()
        }

        return output
    }

    @Synchronized fun isDone() = currentTrajectory != null && currentTrajectory!!.isDone
}