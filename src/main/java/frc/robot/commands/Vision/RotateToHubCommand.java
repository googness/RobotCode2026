package frc.robot.commands.Vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.*;
import frc.robot.subsystems.flywheels.Flywheel;
import frc.robot.subsystems.vision.VisionSystem;
import java.util.function.DoubleSupplier;

public class RotateToHubCommand extends Command {

  // Import drivetrain
  private final SwerveDrivetrain drivetrain;

  // Import VisionSystem
  private final VisionSystem vision;

  private final Flywheel flywheel;

  // Import driver controls
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  // The PID controller built directly inside the command
  private final ProfiledPIDController thetaController;

  // Locations of the hubs
  private final Translation2d redHubLocation = new Translation2d(11.919, 4.016);
  private final Translation2d blueHubLocation = new Translation2d(4.634, 4.016);

  // Constructor
  public RotateToHubCommand(
      SwerveDrivetrain drivetrain,
      VisionSystem vision,
      Flywheel flywheel,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    this.drivetrain = drivetrain;
    this.vision = vision;
    this.flywheel = flywheel;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    // Build the PID controller using your constants
    this.thetaController =
        new ProfiledPIDController(
            RobotConfig.getInstance().getDriveFacingAngleThetaKP(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKI(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKD(),
            new TrapezoidProfile.Constraints(
                Math.PI * 1.0, // Speed limit: 0.75 rotations per second
                Math.PI * 2.0 // Acceleration limit
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

    flywheel.extendHood();
  }

  @Override
  public void execute() {

    // Get driver joysticks
    double rawX = xSupplier.getAsDouble();
    double rawY = ySupplier.getAsDouble();

    double deadbandedX = MathUtil.applyDeadband(rawX, 0.1);
    double deadbandedY = MathUtil.applyDeadband(rawY, 0.1);

    // Allow driver inputs
    double xVelocity =
        deadbandedX * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);
    double yVelocity =
        deadbandedY * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);

    // Aim at the red hub or the blue hub
    Translation2d targetHub;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      targetHub = redHubLocation;
      // System.out.println("Red");
    } else {
      targetHub = blueHubLocation;
      // System.out.println("Blue");
    }

    // Get your perfectly accurate Pose
    Pose2d currentPose = drivetrain.getPose();

    // Calculate the exact compass angle to the Hub
    Rotation2d targetAngle = targetHub.minus(currentPose.getTranslation()).getAngle();

    double tuningOffsetDegrees = -4;
    targetAngle = targetAngle.plus(Rotation2d.fromDegrees(tuningOffsetDegrees));

    // Run the PID math to get the spin speed
    double omegaRadiansPerSecond =
        thetaController.calculate(currentPose.getRotation().getRadians(), targetAngle.getRadians());

    // If the controller determines we are within the 0.05 radian tolerance, stop spinning!
    if (thetaController.atGoal()) {
      omegaRadiansPerSecond = 0.0;
    }

    double distanceMeters = currentPose.getTranslation().getDistance(targetHub);
    double distanceInches = Units.metersToInches(distanceMeters);

    double rps = 36;

    distanceInches = distanceInches;

    double useThis = flywheel.getTargetRps(distanceInches);

    SmartDashboard.putNumber("distToHub", useThis);

    // if (distanceInches >= 90) {
    //   rps = 49;
    // } else if (distanceInches >= 80) {
    //   rps = 47;
    // } else if (distanceInches >= 75) {
    //   rps = 46;
    // } else if (distanceInches >= 70) {
    //   rps = 45;
    // } else if (distanceInches >= 65) {
    //   rps = 44;
    // } else if (distanceInches >= 60) {
    //   rps = 43;
    // } else if (distanceInches >= 55) {
    //   rps = 42;
    // } else if (distanceInches >= 50) {
    //   rps = 41;
    // } else if (distanceInches >= 45) {
    //   rps = 40;
    // } else if (distanceInches >= 40) {
    //   rps = 39;
    // }

    flywheel.setVelocity(useThis);

    // Notice how we wrap everything in MetersPerSecond and RadiansPerSecond for your template
    drivetrain.drive(
        MetersPerSecond.of(xVelocity),
        MetersPerSecond.of(yVelocity),
        RadiansPerSecond.of(omegaRadiansPerSecond),
        true, // field relative
        true // open loop
        );
  }

  @Override
  public void end(boolean interrupted) {
    // Safely stop the drivetrain when the button is released
    // (This matches how your DriveToPose command safely ends)
    flywheel.stopFlywheel();
    flywheel.stopAccelerator();
    drivetrain.stop();
    flywheel.retractHood();
    System.out.println("End");
  }

  /** IsFinished: Never finishes automatically. Waits for the driver to let go. */
  @Override
  public boolean isFinished() {
    return false;
  }
}
