package frc.robot.commands.Vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.*;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RotateToTag extends Command {

  // Bring in your vision and drivetrain subsystems
  private final SwerveDrivetrain drivetrain;
  private final VisionSystem vision;

  // Allow driver controls for strafing

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  // The PID controller to calculate the spin speed

  private ProfiledPIDController thetaController;

  // Define which tags belong to which alliance
  // (Change these numbers to match the actual game manual for the hubs!)
  private final List<Double> redHubTags = List.of(4.0, 5.0, 6.0);
  private final List<Double> blueHubTags = List.of(7.0, 8.0, 9.0);

  // Setup the constructor
  public RotateToTag(
      SwerveDrivetrain drivetrain,
      VisionSystem vision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    // Bring in the required subsystems and driver joystick inputs
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    // Set up the PID for the rotation

    this.thetaController =
        new ProfiledPIDController(
            RobotConfig.getInstance().getDriveFacingAngleThetaKP(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKI(),
            RobotConfig.getInstance().getDriveFacingAngleThetaKD(),
            new TrapezoidProfile.Constraints(
                Math.PI * 1.0, // Speed limit
                Math.PI * 2.0 // Acceleration limit
                ));

    // Add a tolerance so that the PID stops at the goal

    // Tell the PID that a circle connects so it spins the shortest way
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Setting up a tolerance so the robot rotates until its within this range
    this.thetaController.setTolerance(0.05, 0.05);

    // Add the drivetrain as a requirement for this command

    addRequirements(drivetrain);
    // End the constructor

  }
  // Create an execute function
  @Override
  public void execute() {

    // Store the tx of the april tag
    double tx = LimelightHelpers.getTX("limelight-thunder");

    // Does the camera have a target?

    boolean hasTarget = false;

    // Get the ID of the tag you are locked onto

    double getID = LimelightHelpers.getFiducialID("limelight-thunder");
    // Get the rotation of the robot

    double omegaRadiansPerSecond = 0;
    // Get the driver joysticks

    double rawX = xSupplier.getAsDouble();
    double rawY = ySupplier.getAsDouble();

    // Apply deadband so the robot is not affected by stick drift
    double deadbandedX = MathUtil.applyDeadband(rawX, 0.1);
    double deadbandedY = MathUtil.applyDeadband(rawY, 0.1);

    // Allow driver inputs
    double xVelocity =
        deadbandedX * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);
    double yVelocity =
        deadbandedY * RobotConfig.getInstance().getRobotMaxVelocity().in(MetersPerSecond);

    // Get the alliance color
    var alliance = DriverStation.getAlliance();

    // Store the tags you want to use
    List<Double> hubTags;

    // Check the alliance color
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      // Switch targeted tags to the red tags
      hubTags = redHubTags;
    } else {
      // Switch the targeted tags to the blue tags
      hubTags = blueHubTags;
    }

    // Make a variable to see if we have a valid tag or not

    boolean ValidTag = false;
    // Check to see if we have a target and that it contains one of our hub

    if (hasTarget && hubTags.contains(getID)) {
      ValidTag = true;
    }

    // --- ONLY do the spin math if we are looking at a correct tag ---
    if (ValidTag) {

      // Calculate how fast to spin using the tx variable!
      // Because the Limelight pipeline handles the offset, our target is ALWAYS 0.0
      omegaRadiansPerSecond = thetaController.calculate(Math.toRadians(tx), 0.0);
    }

    // If we are close enough to the target, stop spinning to prevent jittering
    if (thetaController.atGoal()) {
      omegaRadiansPerSecond = 0.0;
    } else {
      // If we don't have a valid tag, don't spin at all
      omegaRadiansPerSecond = 0.0;
    }

    // Drive the robot using the translation velocities and the new rotation speed
    drivetrain.drive(
        MetersPerSecond.of(xVelocity),
        MetersPerSecond.of(yVelocity),
        RadiansPerSecond.of(omegaRadiansPerSecond),
        true, // Field relative translation so the driver's joysticks still make sense
        true); // Open loop

    // Log important information
    // Log the raw tag ID the camera is currently looking at
    Logger.recordOutput("RotateToTag/RawTagID", getID);

    // Log whether the camera sees *any* tag at all
    Logger.recordOutput("RotateToTag/HasTarget", hasTarget);

    // Log whether the tag it sees is a valid one for your alliance
    Logger.recordOutput("RotateToTag/IsValidTarget", ValidTag);

    // Log the tx of the targeted april tag
    Logger.recordOutput("RotateToTag/AprilTagTX", tx);
  }

  // Safely end the command when the driver lets go of the button
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drivetrain.stop();
  }

  // Determine when the command should naturally finish
  @Override
  public boolean isFinished() {
    // Return false so it only finishes when the button is released
    return false;
  }
}
