package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionSystem;

public class ResetPosetoVision extends Command {

  private VisionSystem vision;

  public ResetPosetoVision(VisionSystem vision) {

    this.vision = vision;
  }

  @Override
  public void initialize() {
    this.vision.resetOdometryToVisionPose();
  }
}
