package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
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
public class PhotonAligner extends Command {
    private CommandSwerveDrivetrain s_Swerve;
    private PhotonCamera camera;
    private GenericHID controller;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final double anglekP=0.4;
    private final double driveKP=0.4;
    private final SlewRateLimiter fowardlimit=new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationlimit=new SlewRateLimiter(7.0);
    private final double targetDistance=1; // in meters
    private double projectedDistance=0;
    private double projectedStraf=0;
    public PhotonAligner(CommandSwerveDrivetrain s_Swerve, PhotonCamera camera, GenericHID controller) {
        this.s_Swerve = s_Swerve;
        this.camera = camera;
        this.controller = controller;
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        double forward = -controller.getRawAxis(1) * Constants.Swerve.maxSpeed;
        double strafe = -controller.getRawAxis(0) * Constants.Swerve.maxSpeed;
        double turn = -controller.getRawAxis(4) * Constants.Swerve.maxAngularVelocity;


        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 1) {
                        // Found Tag 1, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                        var translation = target.getBestCameraToTarget();
                        projectedDistance=translation.getX();
                        projectedStraf=translation.getY();
                    }
                }
            }
        }
    

        // Auto-align when requested
        if (targetVisible==true) {
            if(Math.abs(targetYaw)>4){
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = Units.degreesToRadians(MathUtil.clamp(targetYaw * anglekP * Constants.Swerve.maxAngularVelocity,-Constants.Swerve.maxAngularVelocity,Constants.Swerve.maxAngularVelocity));
        }
        double distanceError=targetDistance-projectedDistance;
        if(Math.abs(distanceError)>0.05){
        forward=MathUtil.clamp(driveKP*Constants.Swerve.maxSpeed*distanceError,-Constants.Swerve.maxSpeed,Constants.Swerve.maxSpeed);
        }
        if(Math.abs(projectedStraf)>0.05){
        strafe=MathUtil.clamp(driveKP*Constants.Swerve.maxSpeed*projectedStraf,-Constants.Swerve.maxSpeed,Constants.Swerve.maxSpeed);
        }
    }
        // Command drivetrain motors based on target speeds
   double limitedForward=fowardlimit.calculate(forward);
    double limitedStrafe=fowardlimit.calculate(strafe);
    double limitedTurn=rotationlimit.calculate(turn);

        s_Swerve.setControl(driveRequest.withVelocityX(-limitedForward)
        .withVelocityY(limitedStrafe)
        .withRotationalRate(-limitedTurn)
        );

        // Put debug information to the dashboards
        SmartDashboard.putNumber("Raw Target Yaw", targetYaw);
        //SmartDashboard.put("Rotational Yaw",Rotation2d.fromDegrees(targetYaw));
        SmartDashboard.putNumber("Photon Turn Value", turn);
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        SmartDashboard.putNumber("Photon Forward", forward);
        SmartDashboard.putNumber("Photon strafe", strafe);
        SmartDashboard.putNumber("Photon estimated distance", projectedDistance);
}
}