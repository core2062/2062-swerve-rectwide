package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveTrackingSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
public class AimToHub extends Command {
    private CommandSwerveDrivetrain s_Swerve;
    private PhotonCamera camera;
    private GenericHID controller;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final double anglekP=0.3;
    private final double driveKP=0.4;
    private final SlewRateLimiter fowardlimit=new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationlimit=new SlewRateLimiter(6.0);
    private double distanceToHub=0;
    private final double distanceAprilTagToHub=36.37; //inches
    private double aprilTagDistance=0;
    private double aprilTagRotation=0;
    private double xDistance=0;
    private double yDistance=0;
    private double zDistance=0;
      public AimToHub(CommandSwerveDrivetrain s_Swerve, PhotonCamera camera, GenericHID controller) {
        this.s_Swerve = s_Swerve;
        this.camera = camera;
        this.controller = controller;
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        double turnAngle = -controller.getRawAxis(4) * Constants.Swerve.maxAngularVelocity;
        boolean targetVisible = false;
        var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 1) {
                        targetVisible = true;
                        var transform = target.getBestCameraToTarget();
                        aprilTagRotation=Units.radiansToDegrees(transform.getRotation().getZ());
                        xDistance=transform.getX();
                        yDistance=transform.getY();
                        zDistance=transform.getZ();
                           if(aprilTagRotation>=0){
                            aprilTagRotation-=90;
                            }else{
                                aprilTagRotation+=270;
                            }
                        aprilTagDistance=Units.metersToInches(
                            Math.sqrt(Math.pow(xDistance,2)+Math.pow(yDistance, 2)+Math.pow(zDistance, 2)));
                    }
                }
            }
        }
     
        s_Swerve.setControl(driveRequest.withRotationalRate(-turnAngle));
        SmartDashboard.putNumber("Rotation of the april tag",aprilTagRotation);
        SmartDashboard.putNumber("Finds distance to april tag", aprilTagDistance);
    }
}
