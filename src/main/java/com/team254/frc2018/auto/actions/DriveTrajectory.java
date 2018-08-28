package com.team254.frc2018.auto.actions;

import com.team254.frc2018.RobotState;
import com.team254.frc2018.subsystems.Drive;
import org.team401.taxis.geometry.Pose2dWithCurvature;
import org.team401.taxis.trajectory.TimedView;
import org.team401.taxis.trajectory.Trajectory;
import org.team401.taxis.trajectory.TrajectoryIterator;
import org.team401.taxis.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrajectory implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }


    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
    }

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            System.out.println("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }
        mDrive.setTrajectory(mTrajectory);
    }
}

