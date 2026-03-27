package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSystem;

public class ResetPosetoVision extends Command {

  private VisionSystem vision;

  private LimelightHelpers.PoseEstimate mt2 =
      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-thunder");

  public ResetPosetoVision(VisionSystem vision) {

    this.vision = vision;
  }

  @Override
  public void initialize() {

    // Reset vision pose
    this.vision.resetOdometryToVisionPose(mt2);
  }
}
