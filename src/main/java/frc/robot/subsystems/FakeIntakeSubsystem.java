package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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
        intakePositionState = IntakeArmState.IN;
        intakeWheelState = IntakeWheelState.FORWARD;
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
        intakeDesiredAngle = angle;

    }

    public IntakeArmState getIntakeArmState() {
        return intakePositionState;
    }

    @Override
    public void periodic() {
        if (Math.abs(intakeAngle-intakeDesiredAngle) < Constants.FakeIntakeSubsystem.IntakeDeadband) {
            if (lastIntakeAngle > intakeAngle) {
                setArmState(IntakeArmState.OUT);
                System.out.printf("periodic: Intake OUT: lastIntakeAngle: %f, intakeAngle: %f\n", lastIntakeAngle, intakeAngle);
            } else {
                setArmState(IntakeArmState.IN);
                System.out.printf("periodic: Intake IN: lastIntakeAngle: %f, intakeAngle: %f\n", lastIntakeAngle, intakeAngle);
            }
        } else if (lastIntakeAngle > intakeAngle) {
            setArmState(IntakeArmState.MOVINGOUT);
            System.out.printf("periodic: IntakeArm MOVINGOUT: intakeAngle: %f\n", intakeAngle);
        } else {
            setArmState(IntakeArmState.MOVINGIN);
            System.out.printf("periodic: IntakeArm MOVINGIN: intakeAngle: %f\n", intakeAngle);
        }
        lastIntakeAngle = intakeAngle;
    }

    /*
     * simulationPeriodic will make the subsystem "pretend" to move
     */
    public void simulationPeriodic() {
        switch (intakePositionState) {
            case MOVINGIN:
                intakeAngle = intakeAngle + Constants.FakeIntakeSubsystem.IntakeMovementRate;
                System.out.printf("simultionatPeriodic: intakeAngle: %f, MOVING IN\n");
                break;
                case MOVINGOUT:
                intakeAngle = intakeAngle - Constants.FakeIntakeSubsystem.IntakeMovementRate;
                System.out.printf("simultionatPeriodic: intakeAngle: %f, MOVING OUT\n");
                break;
            default:
                break;
        }
    }
}
