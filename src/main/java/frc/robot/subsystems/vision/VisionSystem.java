package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.swerve_drivetrain.*;

public class VisionSystem extends SubsystemBase {

  private final SwerveDrivetrain drivetrain;

  public VisionSystem(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    SmartDashboard.putNumber("distToHub", 0);
  }

  public void periodic() {

    // 1. Feed the Gyro to Limelight (CRITICAL)
    // We use your raw inputs to get the most up-to-date gyro data
    // double gyroDeg = inputs.drivetrain.rawHeading.in(Degrees);
    double gyroDeg = this.drivetrain.getPose().getRotation().getDegrees();

    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    //   gyroDeg = gyroDeg + 180;
    // }

    // System.out.println(gyroDeg);

    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode("limelight-thunder", 1);
    } else {
      LimelightHelpers.SetIMUMode("limelight-thunder", 4);
    }

    double omegaDegPerSec =
        Units.radiansToDegrees(this.drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation("limelight-thunder", gyroDeg, omegaDegPerSec, 0, 0, 0, 0);

    // 2. Get the MegaTag 2 Pose Estimate
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-thunder");

    // 3. Update the Odometry (Pose Estimator)
    // We check tagCount > 0 to ensure we actually see something
    if (mt2.tagCount > 0) {

      // We use 'this.odometry' because that is where your Pose Estimator lives.
      // VecBuilder.fill(0.7, 0.7, 9999999) means:
      // - Trust Vision X/Y moderately (0.7)
      // - COMPLETELY IGNORE Vision Rotation (9999999) -> Because we trust your Pigeon/Gyro more!

      // Trust Megatag 2
      double trustVision;
      if (mt2.tagCount >= 2) {
        // if we have 2 tags, megatag 2 will be spot on
        trustVision = 0.1;
      } else {
        // create a scale factor based on our distance if we see 1 tag
        trustVision = 0.2 + (0.2 * mt2.avgTagDist);
      }

      this.drivetrain.addVisionMeasurement(
          mt2.pose, mt2.timestampSeconds, VecBuilder.fill(trustVision, trustVision, 9999999));

      // this.drivetrain.resetPose(mt2.pose);

      // 1. Get Distance in Meters (Default)
      // double distMeters = mt2.rawFiducials[0].distToCamera;

      // 2. Convert to Feet
      // double distFeet = Units.metersToFeet(distMeters);

      // double distInch = Units.metersToInches(distMeters);

      // 3. Print/Log the Feet value
      // System.out.println("Distance (in): " + distInch);
      // Logger.recordOutput("Limelight" + "/Limelight/DistanceInch", distInch);
    }
  }

  public void resetOdometryToVisionPose() {
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-thunder");

    if (mt2.tagCount >= 2) {
      this.drivetrain.resetPose(
          new Pose2d(mt2.pose.getX(), mt2.pose.getY(), this.drivetrain.getRotation()));
    } else {
      this.drivetrain.addVisionMeasurement(
          mt2.pose, mt2.timestampSeconds, VecBuilder.fill(.5, .5, 9999999));
    }
  }

  public void getDistance() {}
}
