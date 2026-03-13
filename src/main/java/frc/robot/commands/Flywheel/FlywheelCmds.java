package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flywheels.*;

public class FlywheelCmds {

  private FlywheelCmds() {}

  // public static Command getHubShot(Flywheel flywheel) {
  //   return new InstantCommand(() -> flywheel.setVelocity(0.5, 0.5), flywheel);
  // }

  public static Command getHubShot(Flywheel flywheel) {
    return Commands.startEnd(() -> flywheel.setVelocity(), () -> flywheel.stopFlywheel());
  }

  public static Command getFixedShot(Flywheel flywheel) {
    return new InstantCommand(() -> flywheel.setVelocity(39));
  }

  // public static Command getFixedShot(Flywheel flywheel) {
  //   return Commands.startEnd(
  //     () -> flywheel.setVelocity(39),
  //     () -> flywheel.stopFlywheel());
  // }

  public static Command runAccelerator(Flywheel flywheel) {
    return new InstantCommand(() -> flywheel.runAccelerator());
  }

  public static Command stopAccelerator(Flywheel flywheel) {
    return new InstantCommand(() -> flywheel.stopAccelerator());
  }

  public static Command extendHood(Flywheel flywheel) {
    return new InstantCommand(() -> flywheel.extendHood());
  }

  public static Command retractHood(Flywheel flywheel) {
    return new InstantCommand(() -> flywheel.retractHood());
  }
}
