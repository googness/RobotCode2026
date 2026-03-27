// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import java.util.function.DoubleSupplier;

public class ManualAimCmd extends Command {
  private final DoubleSupplier ySupplier;
  private final Hopper hopper;

  public ManualAimCmd(Hopper hopper, DoubleSupplier ySupplier) {
    this.ySupplier = ySupplier;
    this.hopper = hopper;

    addRequirements(hopper);
  }

  @Override
  public void execute() {

    double y = ySupplier.getAsDouble();
    Boolean isNegative = y < 0;

    y = Math.abs(y) < 0.15 ? 0 : (Math.abs(y) - 0.15) / 0.9;

    double val = 0;
    if (y != 0) {
      if (isNegative) {
        y = y * -1;
      }

      val = y * 0.2;
      hopper.setHopperSpeed(val);
    } else {
      hopper.setHopperSpeed(val);
    }

    System.out.println(val);
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setHopperSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
