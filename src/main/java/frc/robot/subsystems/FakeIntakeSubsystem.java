package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class FakeIntakeSubsystem extends SubsystemBase {
    public enum IntakeArmState {
        OUT,
        MOVINGOUT,
        IN,
        MOVINGIN
    }
    
    public enum IntakeWheelState {
        FORWARD,
        OFF,
        REVERSE
    }


    private IntakeArmState intakePositionState;
    private IntakeWheelState intakeWheelState;
    private double intakeAngle;
    private double lastIntakeAngle;
    private double intakeDesiredAngle;

    public FakeIntakeSubsystem() {
        // things like the motors would get setup in the constructor
        intakePositionState = IntakeArmState.OUT;
        intakeWheelState = IntakeWheelState.OFF;
        intakeAngle = Constants.FakeIntakeSubsystem.IntakeInAngle; // assume it is up and travels from 0 to 120 degrees
        lastIntakeAngle = intakeAngle;
        intakeDesiredAngle = intakeAngle; // assume we want it up to start thing off
    }

    public boolean intakeIsOut() {
        return intakePositionState == IntakeArmState.OUT;
    }

    public boolean intakeIsIn() {
        return intakePositionState == IntakeArmState.IN;
    }

    private void setArmState(IntakeArmState newState) {
        intakePositionState = newState;
    }

    // the desired angle
    public void setIntakeAngle(double angle) {
        if (intakeAngle > angle) {
            setArmState(IntakeArmState.MOVINGOUT);
            System.out.printf("setIntakeAngle MOVINGOUT intakeAngle: %f, angle: %f\n",intakeAngle, angle);
        } else if (intakeAngle < angle) {
            setArmState(IntakeArmState.MOVINGIN);
            System.out.printf("setIntakeAngle MOVINGIN intakeAngle: %f, angle: %f\n",intakeAngle, angle);
        }
        intakeDesiredAngle = angle;

    }

    public double getIntakeAngle() {
        return intakeAngle;
    }

    public IntakeArmState getIntakeArmState() {
        return intakePositionState;
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            if (Math.abs(intakeAngle-intakeDesiredAngle) < Constants.FakeIntakeSubsystem.IntakeDeadband) {
                if (getIntakeArmState() == IntakeArmState.MOVINGOUT) {
                    setArmState(IntakeArmState.OUT);
                    // System.out.printf("periodic: Intake OUT: lastIntakeAngle: %f, intakeAngle: %f\n", lastIntakeAngle, intakeAngle);
                } else {
                    setArmState(IntakeArmState.IN);
                    // System.out.printf("periodic: Intake IN: lastIntakeAngle: %f, intakeAngle: %f\n", lastIntakeAngle, intakeAngle);
                }
            }
            lastIntakeAngle = intakeAngle;
        }
    }

    public void setWheelState(IntakeWheelState state) {
        intakeWheelState = state;
    }
    /*
     * simulationPeriodic will make the subsystem "pretend" to move
     */
    public void simulationPeriodic() {
        if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            // System.out.printf("entered: simulationPeriodic: intakeDesiredAngle: %f\n", intakeDesiredAngle);
            switch (intakePositionState) {
                case MOVINGIN:
                intakeAngle = intakeAngle + Constants.FakeIntakeSubsystem.IntakeMovementRate;
                // System.out.printf("simultionatPeriodic: intakeAngle: %f, MOVING IN\n", intakeAngle);
                break;
                case MOVINGOUT:
                intakeAngle = intakeAngle - Constants.FakeIntakeSubsystem.IntakeMovementRate;
                // System.out.printf("simultionatPeriodic: intakeAngle: %f, MOVING OUT\n", intakeAngle);
                break;
                default:
                break;
            }
            // System.out.printf("leaving: simulationPeriodic");
        }
    }
}
