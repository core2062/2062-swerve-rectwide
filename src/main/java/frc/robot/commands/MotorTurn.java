package frc.robot.commands;

import frc.robot.subsystems.Turnmotor;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class MotorTurn extends Command {
    private final Turnmotor m_turn;
    private final double m_speed;


    public MotorTurn (Turnmotor subsystem, double speed) {
        m_turn = subsystem;
        m_speed = speed;
        addRequirements(m_turn);
    }

    @Override
    public void execute() {
        m_turn.setSpeed(m_speed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
