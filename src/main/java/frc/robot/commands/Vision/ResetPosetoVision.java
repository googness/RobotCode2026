package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSystem;

public class ResetPosetoVision extends Command {

  private VisionSystem vision;

  public ResetPosetoVision(VisionSystem vision) {

    this.vision = vision;
  }

  @Override
  public void initialize() {
    // Fetch the latest pose estimate right when the command is scheduled
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-thunder");

    // Reset vision pose
    this.vision.resetOdometryToVisionPose(mt2);
  }
}
