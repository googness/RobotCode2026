package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.swerve_drivetrain.*;
import org.littletonrobotics.junction.Logger;

public class VisionSystem extends SubsystemBase {

  private final SwerveDrivetrain drivetrain;

  // Trust Megatag 2
  double trustVision;

  public VisionSystem(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void periodic() {

    double gyroDeg = this.drivetrain.getPose().getRotation().getDegrees();

    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode("limelight-thunder", 1);
    } else {
      LimelightHelpers.SetIMUMode("limelight-thunder", 4);
    }

    double omegaDegPerSec =
        Units.radiansToDegrees(this.drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation("limelight-thunder", gyroDeg, omegaDegPerSec, 0, 0, 0, 0);

    // Get the MegaTag 2 Pose Estimate
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-thunder");

    // We check tagCount > 0 to ensure we actually see something
    if (mt2.tagCount > 0) {

      if (mt2.tagCount >= 2) {
        // if we have 2 tags, megatag 2 will be spot on
        trustVision = 0.1;
      } else {
        // create a scale factor based on our distance if we see 1 tag
        trustVision = 0.2 + (0.2 * mt2.avgTagDist);
      }

      // Add vision measurements to the odometry
      this.drivetrain.addVisionMeasurement(
          mt2.pose, mt2.timestampSeconds, VecBuilder.fill(trustVision, trustVision, 9999999));

      // Record the distance to apriltag
      double distMeters = mt2.rawFiducials[0].distToCamera;

      double distInch = Units.metersToInches(distMeters);

      Logger.recordOutput("Limelight" + "/Limelight/DistanceInch", distInch);
    }
  }

  public void resetOdometryToVisionPose(LimelightHelpers.PoseEstimate mt2) {
    if (mt2.tagCount >= 2) {
      this.drivetrain.resetPose(
          new Pose2d(mt2.pose.getX(), mt2.pose.getY(), this.drivetrain.getRotation()));
    } else {
      this.drivetrain.addVisionMeasurement(
          mt2.pose, mt2.timestampSeconds, VecBuilder.fill(trustVision, trustVision, 9999999));
    }
  }

  // Get the distance of the robot to the target based on odemetry
  public double getOdometryDist(double targetX, double targetY) {
    // Get the target
    Translation2d target = new Translation2d(targetX, targetY);

    return this.drivetrain.getPose().getTranslation().getDistance(target);
  }
}
