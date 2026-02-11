package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MotorTurn extends Command {
    private final LauncherSubsystem m_turn;
    private final double m_speed;


    public MotorTurn (LauncherSubsystem subsystem, double speed) {
        m_turn = subsystem;
        m_speed = speed;
        addRequirements(m_turn);
    }

    @Override
    public void execute() {
        m_turn.setConveyerSpeed(m_speed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}