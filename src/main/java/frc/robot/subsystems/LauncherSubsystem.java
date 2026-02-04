package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class LauncherSubsystem extends SubsystemBase {
  private TalonFX m_UpperShootMotor = new TalonFX(Constants.ShooterConstants.UpperMotorPort);
  private TalonFX m_LowerShootMotor = new TalonFX(Constants.ShooterConstants.LowerMotorPort);

  private TalonFX m_ConveyerMotor = new TalonFX(Constants.ShooterConstants.ConveyerMotorPort);
 
public LauncherSubsystem(){
  final TalonFXConfiguration commonConfigs = new TalonFXConfiguration()
   .withMotorOutput(
    new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive)
   )
   .withCurrentLimits(
    new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Amps.of(120))
    .withStatorCurrentLimitEnable(true)
   );

   final TalonFXConfiguration UpperShootMotor_configs = commonConfigs.clone()
   .withMotorOutput(
    commonConfigs.MotorOutput.clone()
    .withInverted(InvertedValue.Clockwise_Positive)
   );

   final TalonFXConfiguration LowerShootMotor_configs = commonConfigs.clone()
   .withMotorOutput(
    commonConfigs.MotorOutput.clone()
    .withInverted(InvertedValue.CounterClockwise_Positive)
   );

   final TalonFXConfiguration ConveyerMotor_configs = commonConfigs.clone();

  m_UpperShootMotor.getConfigurator().apply(UpperShootMotor_configs);
  m_LowerShootMotor.getConfigurator().apply(LowerShootMotor_configs);
  m_ConveyerMotor.getConfigurator().apply(ConveyerMotor_configs);

SmartDashboard.putNumber("desired UpperShootMotor RPS", Constants.ShooterConstants.upperMotorSpeed);
SmartDashboard.putNumber("desired LowerShootMotor RPS", Constants.ShooterConstants.lowerMotorSpeed);
SmartDashboard.putNumber("desired ConveyerMotor RPS", Constants.ShooterConstants.conveyerMotorSpeed);



  
  
}

public void setShooterSpeed(Double upperMotorSpeed, Double lowerMotorSpeed){
 m_UpperShootMotor.setControl(new DutyCycleOut(upperMotorSpeed));
 m_LowerShootMotor.setControl(new DutyCycleOut(lowerMotorSpeed));
 //Lower shooter motor double speed (.30,.18 but in RPM)
}

public void setConveyerSpeed(Double speed){
  m_ConveyerMotor.setControl(new DutyCycleOut(speed));

  

}
}

