package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.hopper.*;

public class HopperCmds {

  private HopperCmds() {}

  public static Command runIntake(Hopper hopper) {
    return new InstantCommand(() -> hopper.runIntake());
  }

  // public static Command runIntake(Hopper hopper) {
  //   return Commands.startEnd(() -> hopper.runIntake(), () -> hopper.stopIntake());
  // }

  // public static Command RunBeltCmd(Hopper hopper) {
  //   return Commands.startEnd(() -> hopper.runBelt(), () -> hopper.stopBelt());
  // }

  public static Command stopIntake(Hopper hopper) {
    return new InstantCommand(() -> hopper.stopIntake());
  }

  public static Command runBelt(Hopper hopper) {
    return new InstantCommand(() -> hopper.runBelt());
  }

  public static Command stopBelt(Hopper hopper) {
    return new InstantCommand(() -> hopper.stopBelt());
  }

  public static Command extendHopper(Hopper hopper) {
    return new InstantCommand(() -> hopper.extendHopper());
  }

  public static Command retractHopper(Hopper hopper) {
    return new InstantCommand(() -> hopper.retractHopper());
  }

  public static Command reverseIntake(Hopper hopper) {
    return new InstantCommand(() -> hopper.reverseIntake());
  }
}
