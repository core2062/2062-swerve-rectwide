package frc.robot.commands;

import frc.robot.subsystems.FakeIntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeMovement extends Command {

    private final FakeIntakeSubsystem m_intake;
    private double m_angle;
    private final DoubleSupplier angleSupplier;

    // public IntakeMovement(FakeIntakeSubsystem subsystem, double angle) {
    //     m_intake = subsystem;
    //     m_angle = angle;
    //     addRequirements(m_intake);
    //     m_intake.setIntakeAngle(m_angle);
    //     System.out.printf("IntakeMovement: Constructor: angle: %f\n", angle);
    // }

    public IntakeMovement(FakeIntakeSubsystem subsystem, DoubleSupplier angle) {
        m_intake = subsystem;
        angleSupplier = angle;
        m_angle = angle.getAsDouble();
        addRequirements(m_intake);
        m_intake.setIntakeAngle(m_angle);
        System.out.printf("IntakeMovement: Constructor(Supplier): angle: %f\n", m_angle);
    }

    @Override
    public void initialize() {
        m_angle = angleSupplier.getAsDouble();   // <-- reads fresh value
        System.out.printf("IntakeMovement: Intiailize setting angle to: %f\n", m_angle);
        m_intake.setIntakeAngle(m_angle);
    }

    @Override
    public void execute() {
        // System.out.printf("intake state: %f\n",m_intake.getIntakeAngle());
    }

    @Override
    public boolean isFinished() {
        boolean state;

        state = m_intake.intakeIsIn() || m_intake.intakeIsOut();
        if (state) {
            System.out.printf("IntakeMovement: is finished: %b\n",state);
        }
        return state;
    }
}
