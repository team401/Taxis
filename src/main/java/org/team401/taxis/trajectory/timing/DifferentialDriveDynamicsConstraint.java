//File originally from FRC Team 254's 2018 Robot code

package org.team401.taxis.trajectory.timing;

import org.team401.taxis.geometry.ICurvature;
import org.team401.taxis.geometry.IPose2d;
import org.team401.taxis.physics.DifferentialDrivetrainDynamics;
import org.team401.taxis.util.Units;

public class DifferentialDriveDynamicsConstraint<S extends IPose2d<S> & ICurvature<S>> implements TimingConstraint<S> {

    protected final DifferentialDrivetrainDynamics drive_;
    protected final double abs_voltage_limit_;

    public DifferentialDriveDynamicsConstraint(final DifferentialDrivetrainDynamics drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(S state) {
        return Units.meters_to_inches(drive_.getMaxAbsVelocity(
                Units.meters_to_inches(state.getCurvature()),  // Curvature is in inverse inches, so meters_to_inches is correct.
                /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                abs_voltage_limit_));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(S state,
                                                    double velocity) {
        // TODO figure out a units convention for generic states.  Traditionally we use inches...
        // NOTE: units cancel on angular velocity.
        DifferentialDrivetrainDynamics.MinMax min_max = drive_.getMinMaxAcceleration(new DifferentialDrivetrainDynamics.ChassisState(
                        Units.inches_to_meters(velocity), state.getCurvature() * velocity),
                Units.meters_to_inches(state.getCurvature()),  // Curvature is in inverse inches, so meters_to_inches is correct.
                /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                abs_voltage_limit_);
        return new MinMaxAcceleration(Units.meters_to_inches(min_max.min), Units.meters_to_inches(min_max.max));
    }
}
