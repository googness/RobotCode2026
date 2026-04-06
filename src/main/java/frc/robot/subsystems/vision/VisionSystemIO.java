package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionSystemIO {

  @AutoLog
  public static class VisionSystemIOInputs {
    double gyroHeading = 0.0;
    double gyroVelocity = 0.0;
    int tagCount = 0;
    double tagAverageDistance = 0.0;
    double tagTrust = 0.0;
  }

  public default void updateInputs(VisionSystemIOInputs inputs) {}

  public default void resetOdometryToVisionPose() {}
}
