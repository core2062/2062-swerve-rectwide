package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


//conveyer; up and down POV to unjam (consider, but only focus on One Direction(up))
// press a to start motors and have them run for whole match, 

public class LauncherSubsystem extends SubsystemBase {
  private TalonFX m_UpperShootMotor = new TalonFX(Constants.ShooterConstants.UpperMotorPort);
  private TalonFX m_LowerShootMotor = new TalonFX(Constants.ShooterConstants.LowerMotorPort);

  private TalonFX m_ConveyerMotor = new TalonFX(Constants.ShooterConstants.ConveyerMotorPort);
 
public LauncherSubsystem(){
  final TalonFXConfiguration commonConfigs = new TalonFXConfiguration()
   .withMotorOutput(
    new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Brake)
   )
   .withCurrentLimits(
    new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Amps.of(120))
    .withStatorCurrentLimitEnable(true)
   );

   final TalonFXConfiguration leaderConfigs = commonConfigs.clone()
   .withMotorOutput(
    commonConfigs.MotorOutput.clone()
    .withInverted(InvertedValue.Clockwise_Positive)
   );

  var talonFXConfigurator = m_UpperShootMotor.getConfigurator();
  var motorConfigs = new MotorOutputConfigs();

  motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
  talonFXConfigurator.apply(motorConfigs);

  m_UpperShootMotor.getConfigurator().apply(new TalonFXConfiguration());
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

