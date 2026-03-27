package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.commands.DriveToPose;
import java.util.List;
import java.util.Set;

public class DriveToShoot {

  private static final List<Pose2d> BLUE_SPOTS =
      List.of(
          new Pose2d(1.878, 6.000, Rotation2d.fromDegrees(-35.368)),
          new Pose2d(1.878, 4.000, Rotation2d.fromDegrees(0)),
          new Pose2d(1.878, 1.713, Rotation2d.fromDegrees(38.089)));

  private static final List<Pose2d> RED_SPOTS =
      List.of(
          new Pose2d(14.506, 2.114, Rotation2d.fromDegrees(142.859)),
          new Pose2d(14.701, 4.016, Rotation2d.fromDegrees(180)),
          new Pose2d(14.636, 6.357, Rotation2d.fromDegrees(-141.975)));

  private static Pose2d getNearestTarget(Pose2d currentPose) {

    // Default to the blue list
    List<Pose2d> activeSpots = BLUE_SPOTS;

    // Switch to the red list if we are on the red alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      activeSpots = RED_SPOTS;
    }

    // Start by assuming the first spot in the list is the closest
    Pose2d nearestPose = activeSpots.get(0);
    double shortestDistance =
        currentPose.getTranslation().getDistance(nearestPose.getTranslation());

    // Check the remaining spots to see if any are closer
    for (Pose2d spot : activeSpots) {
      double distance = currentPose.getTranslation().getDistance(spot.getTranslation());

      if (distance < shortestDistance) {
        shortestDistance = distance; // We found a closer one! Save the new distance.
        nearestPose = spot; // Update our target pose.
      }
    }

    return nearestPose;
  }

  public static Command driveToShootCommand(SwerveDrivetrain drivetrain) {
    // DeferredCommand waits to calculate the nearest spot until the button is pressed
    return new DeferredCommand(
        () -> {
          // 1. Get our exact pose the moment the button is pressed
          Pose2d currentPose = drivetrain.getPose();

          // 2. Run your math to find the target spot
          Pose2d target = getNearestTarget(currentPose);

          // 3. Return the DriveToPose command using that target!
          // (Note: You will need to replace the example PID controllers and tolerances below
          // with the actual RobotConfig constants you use for your drivetrain)
          return new DriveToPose(
              drivetrain,
              () -> target, // Pass our found target into the supplier
              new ProfiledPIDController(
                  5.0, 0, 0, new TrapezoidProfile.Constraints(1.5, 1.5)), // xController
              new ProfiledPIDController(
                  5.0, 0, 0, new TrapezoidProfile.Constraints(1.5, 1.5)), // yController
              new ProfiledPIDController(
                  5.0, 0, 0, new TrapezoidProfile.Constraints(1.5, 1.5)), // thetaController
              new Transform2d(
                  new Translation2d(0.05, 0.05), Rotation2d.fromDegrees(2.0)), // targetTolerance
              true, // finishesWhenAtTarget
              (atTarget) -> {}, // atTargetConsumer (Leave empty if not using LEDs)
              (poseDiff) -> {}, // poseDifferenceConsumer (Leave empty)
              5.0 // timeout in seconds
              );
        },
        Set.of(drivetrain) // Tell WPILib this takes over the drivetrain
        );
  }
}
