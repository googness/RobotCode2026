package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.swerve_drivetrain.*;
import frc.robot.Field2d;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionSystem extends SubsystemBase {

  private final VisionSystemIO io;
  private final VisionSystemIOInputsAutoLogged VisionInputs = new VisionSystemIOInputsAutoLogged();

  private final Translation2d redHubTarget = new Translation2d(11.919, 4.041); // True center
  private final Translation2d blueHubTarget = new Translation2d(4.621, 4.029); // True center

  // private final Translation2d BlueLeftofHubZone = new Translation2d(2.085, 6.0); // True center
  // private final Translation2d BlueRightofHubZone = new Translation2d(2.085, 2.068); // True
  // center

  // private final Translation2d RedLeftofHubZone = new Translation2d(14.424, 6.0); // True center
  // private final Translation2d RedRightofHubZone = new Translation2d(14.424, 2.068); // True
  // center

  // private final Translation2d UpperMiddleZone = new Translation2d(8.249, 6.0); // True center
  // private final Translation2d LowerMiddleZone = new Translation2d(8.249, 2.068); // True center

  // Added translations of the shooting zones
  private static final List<Translation2d> SHUTTLE_ZONES =
      List.of(
          new Translation2d(2.085, 6.0), // Left of blue hub
          new Translation2d(2.085, 2.068), // Right of blue hub
          new Translation2d(14.424, 6.0), // Left of red hub
          new Translation2d(14.424, 2.068), // Right of red hub
          new Translation2d(8.249, 6.0), // Upper side of middle
          new Translation2d(8.249, 2.068) // Lower side of middle
          );

  public static Translation2d getNearestZone(Translation2d currentTranslation) {

    // Start by assuming the first spot in the list is the closest
    Translation2d nearestZone = SHUTTLE_ZONES.get(0);
    double shortestDistance = currentTranslation.getDistance(nearestZone);

    // Check all 6 spots to see if any are closer
    for (Translation2d zone : SHUTTLE_ZONES) {
      double distance = currentTranslation.getDistance(zone);

      if (distance < shortestDistance) {
        shortestDistance = distance; // We found a closer one! Save the new distance.
        nearestZone = zone; // Update our target translation.
      }
    }

    return nearestZone;
  }

  private final SwerveDrivetrain drivetrain;

  public VisionSystem(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    io = new VisionSystemIOLimelight(drivetrain);
  }

  public void resetOdometryToVisionPose() {
    io.resetOdometryToVisionPose();
  }

  private Translation2d getHubTarget() {
    if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
      return blueHubTarget;
    }

    return redHubTarget;
  }

  public double distanceToHub() {
    return this.drivetrain.getPose().getTranslation().getDistance(getHubTarget());
  }

  public double getOdometryDist(double targetX, double targetY) {
    Translation2d target = new Translation2d(targetX, targetY);
    return this.drivetrain.getPose().getTranslation().getDistance(target);
  }

  @Override
  public void periodic() {
    io.updateInputs(VisionInputs);

    Logger.processInputs(VisionSystemConstants.SUBSYSTEM_NAME, VisionInputs);
  }
}
