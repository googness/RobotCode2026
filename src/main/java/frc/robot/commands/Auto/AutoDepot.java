package frc.robot.commands.Auto;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.*;
import frc.robot.subsystems.flywheels.Flywheel;
import frc.robot.subsystems.vision.VisionSystem;

public class AutoDepot extends Command {

  // Import drivetrain
  private final SwerveDrivetrain drivetrain;

  // Import VisionSystem
  private final VisionSystem vision;

  private final Flywheel flywheels;

  private double mRps = 0;

  // The PID controller built directly inside the command
  private final ProfiledPIDController thetaController;

  // Locations of the hubs
  private final Translation2d redHubLocation = new Translation2d(11.919, 4.029); // 11.919, 4.016
  private final Translation2d blueHubLocation = new Translation2d(4.634, 4.029); // 4.634, 4.016

  // Constructor
  public AutoDepot(
      double rps, SwerveDrivetrain drivetrain, VisionSystem vision, Flywheel flywheel) {
    this.mRps = rps;
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.flywheels = flywheel;

    // Build the PID controller using your constants
    this.thetaController =
        new ProfiledPIDController(
            RobotConfig.getInstance().getDriveFacingAngleThetaKP(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKI(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKD(),
            new TrapezoidProfile.Constraints(
                Math.PI * 1.5, // Speed limit: 0.75 rotations per second
                Math.PI * 3.0 // Acceleration limit
                ));

    // Tell the PID that a circle connects so it spins the shortest way
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Setting up a tolerance so the robot rotates until its within this range
    this.thetaController.setTolerance(0.05, 0.05);

    SmartDashboard.setDefaultNumber("Aim Offset Degrees", 0.0);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {

    // This runs once when you pull the trigger.
    // If you want the Limelight Failsafe, call your reset method here!

    // this.vision.resetOdometryToVisionPose();
  }

  @Override
  public void execute() {

    // In Autonomous, we want the robot to stand still while aiming
    double xVelocity = 0.0;
    double yVelocity = 0.0;

    // Aim at the red hub or the blue hub
    Translation2d targetHub;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      targetHub = redHubLocation;
    } else {
      targetHub = blueHubLocation;
    }

    // Get your perfectly accurate Pose
    Pose2d currentPose = drivetrain.getPose();

    // Calculate the exact compass angle to the Hub
    Rotation2d targetAngle = targetHub.minus(currentPose.getTranslation()).getAngle();

    // ADD THIS: Read the live number from the dashboard (defaults to 0.0 if something goes wrong)

    // If it aims too far left, add a negative degree offset to pull it right.
    // (You will need to tune this number on the carpet!)

    // double tuningOffsetDegrees = -3.0;

    // Run the PID math to get the spin speed
    double omegaRadiansPerSecond =
        thetaController.calculate(currentPose.getRotation().getRadians(), targetAngle.getRadians());

    // If the controller determines we are within the 0.05 radian tolerance, stop spinning!
    if (thetaController.atGoal()) {
      omegaRadiansPerSecond = 0.0;
    }

    // double distanceMeters = currentPose.getTranslation().getDistance(targetHub);
    // double distanceInches = Units.metersToInches(distanceMeters);

    // double useThis = flywheels.getTargetRps(distanceInches);
    // double useThis = flywheels.getTargetRps(distanceInches);
    // Notice how we wrap everything in MetersPerSecond and RadiansPerSecond for your template
    drivetrain.drive(
        MetersPerSecond.of(xVelocity),
        MetersPerSecond.of(yVelocity),
        RadiansPerSecond.of(omegaRadiansPerSecond),
        true, // field relative
        true // open loop
        );

    flywheels.setVelocity(mRps);
  }

  @Override
  public void end(boolean interrupted) {
    // Safely stop the drivetrain when the button is released
    // (This matches how your DriveToPose command safely ends)
    drivetrain.stop();
  }

  /** IsFinished: Finishes the exact moment the robot is within the 0.05 radian tolerance */
  @Override
  public boolean isFinished() {
    return thetaController.atGoal();
  }
}
