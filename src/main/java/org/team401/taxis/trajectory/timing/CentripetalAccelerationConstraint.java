package org.team401.taxis.trajectory.timing;

import org.team401.taxis.geometry.Pose2dWithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint<Pose2dWithCurvature> {
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithCurvature state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithCurvature state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
