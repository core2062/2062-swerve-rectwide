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
  public static class JoystickConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ShooterConstants {
    public static final int UpperMotorPort = 8;
    public static final int LowerMotorPort = 9;
    public static final int ConveyerMotorPort = 10;

    public static final double upperMotorSpeed = 0.18;
    public static final double lowerMotorSpeed = 0.30;
    public static final double conveyerMotorSpeed = 0.50;
    //lower motor double the speed top
}
}