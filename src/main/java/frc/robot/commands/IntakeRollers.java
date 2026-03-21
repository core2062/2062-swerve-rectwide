package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FakeIntakeSubsystem;

public class IntakeRollers extends Command {
    private final FakeIntakeSubsystem m_intake;
    private final FakeIntakeSubsystem.IntakeWheelState m_wheelState;

    public IntakeRollers(FakeIntakeSubsystem subsystem, FakeIntakeSubsystem.IntakeWheelState wheelState) {
        m_intake = subsystem;
        m_wheelState = wheelState;
        addRequirements(m_intake);
        m_intake.setWheelState(m_wheelState);
        switch( wheelState){
            case FORWARD:
            System.out.printf("IntakeRollers: Constructor: state: FORWARD\n");
            break;
            case OFF:
            System.out.printf("IntakeRollers: Constructor: state: OFF\n");
            break;
            case REVERSE:
            System.out.printf("IntakeRollers: Constructor: state: REVERSE\n");
            break;
        }
    }

    @Override
    public void execute() {
        // System.out.printf("intake state: %f\n",m_intake.getIntakeAngle());
    }

    @Override
    public boolean isFinished() {
        boolean state;

        state = true; // instant on
        if (state) {
            System.out.printf("IntakeRollers: is finished: %b\n",state);
        }
        return state;
    }
}
