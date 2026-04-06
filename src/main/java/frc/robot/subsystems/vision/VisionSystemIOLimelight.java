package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;

public class VisionSystemIOLimelight implements VisionSystemIO {
  private final SwerveDrivetrain drivetrain;

  private Alert resetNoTagsAlert =
      new Alert("Failed to reset odemetry to vision.  Not enough tags.", AlertType.kError);

  LimelightHelpers.PoseEstimate mLastTag;

  public VisionSystemIOLimelight(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    LimelightHelpers.SetIMUMode(VisionSystemConstants.LIMELIGHT_NAME, 4);
  }

  @Override
  public void updateInputs(VisionSystemIOInputs inputs) {

    double gyroDeg = this.drivetrain.getPose().getRotation().getDegrees();
    double av = this.drivetrain.getAngularVelocityZWorld().in(Units.DegreesPerSecond);

    LimelightHelpers.SetRobotOrientation(
        VisionSystemConstants.LIMELIGHT_NAME, gyroDeg, av, 0, 0, 0, 0);

    mLastTag =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionSystemConstants.LIMELIGHT_NAME);

    inputs.gyroHeading = gyroDeg;
    inputs.gyroVelocity = av;
    inputs.tagCount = mLastTag.tagCount;

    if (mLastTag.tagCount > 0) {
      inputs.tagAverageDistance = mLastTag.avgTagDist;

      if (mLastTag.tagCount > 1) {
        inputs.tagTrust = 0.1;
      } else {
        inputs.tagTrust = 0.1 + (0.2 * inputs.tagAverageDistance);
      }

      // RobotOdometry.getInstance()
      //     .addVisionMeasurement(
      //         mLastTag.pose,
      //         mLastTag.timestampSeconds,
      //         mLastTag.latency,
      //         VecBuilder.fill(inputs.tagTrust, inputs.tagTrust, 9999999));

      // drivetrain.addVisionMeasurement(
      //     mLastTag.pose,
      //     mLastTag.timestampSeconds,
      //     VecBuilder.fill(inputs.tagTrust, inputs.tagTrust, 9999999));
    }
  }

  public void resetOdometryToVisionPose() {
    if (mLastTag.tagCount > 1) {
      this.drivetrain.resetPose(
          new Pose2d(mLastTag.pose.getX(), mLastTag.pose.getY(), this.drivetrain.getRotation()));
    } else {
      resetNoTagsAlert.set(true);
    }
  }
}
