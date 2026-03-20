package frc.robot.commands;

import frc.robot.subsystems.FakeIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeMovement extends Command {

    private final FakeIntakeSubsystem m_intake;
    private final double m_angle;

    public IntakeMovement(FakeIntakeSubsystem subsystem, double angle) {
        m_intake = subsystem;
        m_angle = angle;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setIntakeAngle(m_angle);
    }

    @Override
    public boolean isFinished() {
        return m_intake.getIntakeArmState() == FakeIntakeSubsystem.IntakeArmState.OUT || m_intake.getIntakeArmState() == FakeIntakeSubsystem.IntakeArmState.IN;
    }
}
