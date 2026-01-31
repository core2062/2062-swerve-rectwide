package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;



public class Turnmotor extends SubsystemBase {
public Turnmotor() {}
private static TalonFX m_TestMotor = new TalonFX(Constants.MotorConstants.FXMotorPort);
//update port


public static void setSpeed(Double speed){
  m_TestMotor.setControl(new DutyCycleOut(speed));
}
  
  


}