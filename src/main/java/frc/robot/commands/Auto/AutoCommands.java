package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.commands.Flywheel.FlywheelCmds;
import frc.robot.commands.Hopper.HopperCmds;
import frc.robot.subsystems.flywheels.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.vision.*;

public class AutoCommands {

  // Pass in your subsystems so the commands have access to them
  public static void registerNamedCommands(
      SwerveDrivetrain drivetrain, Flywheel flywheel, Hopper hopper, VisionSystem vision) {

    // Intake balls
    NamedCommands.registerCommand(
        "Intake",
        HopperCmds.runIntake(hopper)
            .andThen(new WaitCommand(2.0))
            .andThen(HopperCmds.stopIntake(hopper)));

    // Extend Hopper
    NamedCommands.registerCommand("ExtendHopper", HopperCmds.extendHopper(hopper));

    // Retract Hopper
    NamedCommands.registerCommand("RetractHopper", HopperCmds.retractHopper(hopper));

    // Auto Intake
    NamedCommands.registerCommand(
        "AutoIntake",
        HopperCmds.extendHopper(hopper)
            .alongWith(HopperCmds.runIntake(hopper))
            .alongWith(new WaitCommand(5).andThen(HopperCmds.stopIntake(hopper))));

    // Shoot
    NamedCommands.registerCommand(
        "AutoShoot",
        new AutoAim(drivetrain, vision, flywheel)
            .alongWith(hopper.RunClearBeltCmd())
            // .alongWith(FlywheelCmds.extendHood(flywheel))
            .alongWith(
                new WaitCommand(1)
                    .andThen(
                        flywheel.FeederRpsCmd()
                            .alongWith(hopper.RunBeltCmd())
                            .alongWith(
                                new WaitCommand(3.5)
                                    .andThen(() -> hopper.retractHopper())
                                    .andThen(hopper.RunClearIntakeCmd())))));

    // AutoAim
    NamedCommands.registerCommand("AutoAim", new AutoAim(drivetrain, vision, flywheel));

    // Xstance
    NamedCommands.registerCommand("XStance", Commands.run(drivetrain::holdXstance, drivetrain));

    // FixedShot
    NamedCommands.registerCommand("FixedShot", FlywheelCmds.getFixedShot(flywheel));

    NamedCommands.registerCommand("RunFeeder", FlywheelCmds.runAccelerator(flywheel));

    NamedCommands.registerCommand("RunBelt", HopperCmds.runBelt(hopper));

    NamedCommands.registerCommand("StopBelt", HopperCmds.stopBelt(hopper));

    NamedCommands.registerCommand("StopFeeder", FlywheelCmds.stopAccelerator(flywheel));

    NamedCommands.registerCommand("RetractHood", FlywheelCmds.retractHood(flywheel));
  }
}
