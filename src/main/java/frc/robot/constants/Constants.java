package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Swerve{
    public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot
  }

  public static class JoystickConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorConstants {
    public static final int FXMotorPort = 8;
  }

  public static class VisionConstants{
    public static int SpeakerID = 4;
    public static int farSpeakerID = 0;
    public static int AmpID = 0;

    public static double DesiredAngle = 0.0;
  }

  public static class FakeIntakeSubsystem{
    public static final double IntakeInAngle = 120.0;
    public static final double IntakeOutAngle = 0.0;
    public static final double IntakeDeadband = 0.02;
    public static final double IntakeMovementRate = 0.001;
  }
  
  public static class LauncherConstants {
    public static final int UpperMotorPort = 14; 
    public static final int LowerMotorPort = 9; 
    public static final int ConveyerMotorPort = 10; 

     public static final double UpperMotorSpeedRpm = 2062;
        public static final double LowerMotorSpeedRpm  = 1030;
        public static final double ConveyerMotorSpeedRpm = 0.3;
    //lower motor double the speed top
  }
  public static class IndexerConstants {
    public static final int kIndexMotorPort = 11;

    public static final double kIndexMotorSpeed = 0.30;
  }
  public static class IntakeConstants {
    public static final int kUpperIntakeMotorPort = 12;
    public static final int kLowerIntakeMotorPort = 13;
    public static final int kRotatingInakeMotorPort = 8;

    public static final double kUpperIntakeMotorSpeed = 0.50;
    public static final double kLowerIntakeMotorSpeed = 0.30;
    public static final int kRotatingMotorDegree = 90;
  }
} 