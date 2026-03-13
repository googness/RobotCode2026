package frc.robot.commands.Vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionSystem;

public class ResetPosetoVision extends Command {

  private VisionSystem vision;

  public ResetPosetoVision(VisionSystem vision) {

    this.vision = vision;
  }

  @Override
  public void initialize() {

    // This runs once when you pull the trigger.
    // If you want the Limelight Failsafe, call your reset method here!

    this.vision.resetOdometryToVisionPose();
  }
}
