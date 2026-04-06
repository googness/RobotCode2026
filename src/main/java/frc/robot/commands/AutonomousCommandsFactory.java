package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.commands.Auto.AutoAim;
import frc.robot.commands.Hopper.HopperCmds;
import frc.robot.subsystems.flywheels.Flywheel;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.vision.VisionSystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousCommandsFactory {

  private static AutonomousCommandsFactory autonomousCommandFactory = null;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);

  /**
   * Returns the singleton instance of this class.
   *
   * @return the singleton instance of this class
   */
  public static AutonomousCommandsFactory getInstance() {
    if (autonomousCommandFactory == null) {
      autonomousCommandFactory = new AutonomousCommandsFactory();
    }
    return autonomousCommandFactory;
  }

  private AutonomousCommandsFactory() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutoCommands(
      SwerveDrivetrain drivetrain, Hopper hopper, Flywheel flywheel, VisionSystem jaysVision) {

    NamedCommands.registerCommand("RunFeeder", HopperCmds.runAccelerator(hopper));

    NamedCommands.registerCommand("RunBelt", HopperCmds.runBelt(hopper));

    NamedCommands.registerCommand("StopBelt", HopperCmds.stopBelt(hopper));

    NamedCommands.registerCommand("StopFeeder", HopperCmds.stopAccelerator(hopper));

    NamedCommands.registerCommand(
        "StopFlywheel", new InstantCommand(() -> flywheel.stopFlywheel()));

    NamedCommands.registerCommand("AutoIntake", HopperCmds.autonomousIntake(hopper));

    NamedCommands.registerCommand("ReturnHopper", HopperCmds.GoToHomePos(hopper));

    NamedCommands.registerCommand(
        "AutoAim",
        new AutoAim(
            drivetrain,
            // jaysVision,
            flywheel));

    NamedCommands.registerCommand(
        "Intake",
        HopperCmds.runIntake(hopper)
            .andThen(new WaitCommand(2.0))
            .andThen(HopperCmds.stopIntake(hopper)));

    NamedCommands.registerCommand(
        "AutoShoot",
        new AutoAim(
                drivetrain,
                // jaysVision,
                flywheel)
            .alongWith(
                HopperCmds.runAccelerator(hopper)
                    .alongWith(HopperCmds.runBelt(hopper))
                    .alongWith(
                        new WaitCommand(1.5)
                            .andThen(
                                HopperCmds.GoToIntakePos(hopper)
                                    .alongWith(hopper.RunClearIntakeCmd())
                                    .withTimeout(3)))));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    Command startPoint =
        Commands.runOnce(
            () -> {
              try {
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
              } catch (Exception e) {
                pathFileMissingAlert.setText("Could not find the specified path file: Start Point");
                pathFileMissingAlert.set(true);
              }
            },
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    Command Jay = new PathPlannerAuto("Jay");
    autoChooser.addOption("Jay", Jay);

    // BALL SIDE
    Command BS_SimpleShoot = new PathPlannerAuto("BS_SimpleShoot");
    autoChooser.addOption("BS_SimpleShoot", BS_SimpleShoot);

    // NON BALL SIDE
    Command NBS_SimpleShoot = new PathPlannerAuto("NBS_SimpleShoot");
    autoChooser.addOption("NBS_SimpleShoot", NBS_SimpleShoot);

    Command BS_DepotShoot = new PathPlannerAuto("BS_DepotShoot");
    autoChooser.addOption("BS_DepotShoot", BS_DepotShoot);

    Command NBS_ParkInTrench = new PathPlannerAuto("NBS_ParkInTrench");
    autoChooser.addOption("NBS_ParkInTrench", NBS_ParkInTrench);

    // Center
    Command CenterShoot = new PathPlannerAuto("CenterShoot");
    autoChooser.addDefaultOption("CenterShoot", CenterShoot);

    // St Cloud Autos
    Command NBS_Cheese = new PathPlannerAuto("NBS_Cheese");
    autoChooser.addOption("NBS_Cheese", NBS_Cheese);

    Command ResetVision = new PathPlannerAuto("ResetVision");
    autoChooser.addOption("ResetVision", ResetVision);

    // Test Auto for my dad
    Command JayTestAuto = new PathPlannerAuto("JayTestAuto");
    autoChooser.addOption("JayTestAuto", JayTestAuto);

    // Extend Hopper
    // NamedCommands.registerCommand("ExtendHopper", HopperCmds.extendHopper(hopper));

    // // Retract Hopper
    // NamedCommands.registerCommand("RetractHopper", HopperCmds.retractHopper(hopper));

    // FixedShot
    // NamedCommands.registerCommand(
    //     "FixedShot",
    //     FlywheelCmds.extendHood(flywheel)
    //         .andThen(
    //             FlywheelCmds.getFixedShot(flywheel)
    //                 .andThen(
    //                     new WaitCommand(1)
    //                         .andThen(
    //                             flywheel.FeederRpsCmd()
    //                                 .alongWith(
    //                                     hopper.RunBeltCmd()
    //                                         .alongWith(
    //                                             new WaitCommand(2.5)
    //                                                 .andThen(() -> hopper.retractHopper())
    //                                                 .andThen(hopper.RunClearIntakeCmd())))))));

    // Shoot Commands for autonomous

    // Auto Intake
    // NamedCommands.registerCommand(
    //     "AutoIntake",
    //     HopperCmds.extendHopper(hopper)
    //         .alongWith(
    //             HopperCmds.runIntake(hopper)
    //                 .alongWith(HopperCmds.runBelt(hopper))
    //                 .andThen(new WaitCommand(8))
    //                 .andThen(HopperCmds.stopIntake(hopper))));

    // Shoot
    // NamedCommands.registerCommand(
    //     "AutoShoot",
    //     FlywheelCmds.extendHood(flywheel)
    //         .andThen(
    //             new AutoAim(drivetrain, jaysVision, flywheel)
    //                 // .alongWith(hopper.RunClearBeltCmd())
    //                 .alongWith(
    //                     new WaitCommand(1)
    //                         .andThen(
    //                             new WaitCommand(1)
    //                                 .andThen(
    //                                     flywheel.FeederRpsCmd()
    //                                         .alongWith(
    //                                             hopper.RunBeltCmd()
    //                                                 .alongWith(
    //                                                     new WaitCommand(2.5)
    //                                                         .andThen(() ->
    // hopper.retractHopper())
    //                                                         .andThen(
    //
    // hopper.RunClearIntakeCmd()))))))));
    // NamedCommands.registerCommand(
    //     "depotShot",
    //     FlywheelCmds.extendHood(flywheel)
    //         .andThen(
    //             new AutoDepot(40, drivetrain, jaysVision, flywheel)
    //                 .alongWith(hopper.RunClearBeltCmd())
    //                 .alongWith(
    //                     new WaitCommand(1)
    //                         .andThen(
    //                             new WaitCommand(1)
    //                                 .andThen(
    //                                     flywheel.FeederRpsCmd()
    //                                         .alongWith(
    //                                             hopper.RunBeltCmd()
    //                                                 .alongWith(
    //                                                     new WaitCommand(2.5)
    //                                                         .andThen(() ->
    // hopper.retractHopper())
    //                                                         .andThen(
    //
    // hopper.RunClearIntakeCmd()))))))));

    // NamedCommands.registerCommand(
    //     "JersieDepotShot",
    //     FlywheelCmds.extendHood(flywheel)
    //         .andThen(
    //             new AutoDepot(43, drivetrain, jaysVision, flywheel)
    //                 // .alongWith(hopper.RunClearBeltCmd())
    //                 .alongWith(
    //                     new WaitCommand(1)
    //                         .andThen(
    //                             new WaitCommand(1)
    //                                 .andThen(
    //                                     flywheel.FeederRpsCmd()
    //                                         .alongWith(
    //                                             hopper.RunBeltCmd()
    //                                                 .alongWith(
    //                                                     new WaitCommand(2.5)
    //                                                         .andThen(() ->
    // hopper.retractHopper())
    //                                                         .andThen(
    //
    // hopper.RunClearIntakeCmd()))))))));

    // AutoAim

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    /************ Distance Test ************
     *
     * used for empirically determining the wheel radius
     *
     */
    // autoChooser.addOption(
    //     "Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true, drivetrain));
    // autoChooser.addOption(
    //     "Distance Test Med", createTuningAutoPath("DistanceTestMed", true, drivetrain));
    // autoChooser.addOption(
    //     "Distance Test Fast", createTuningAutoPath("DistanceTestFast", true, drivetrain));

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    // autoChooser.addOption(
    //     "Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false, drivetrain));
    // autoChooser.addOption(
    //     "Rotation Test Fast", createTuningAutoPath("RotationTestFast", false, drivetrain));

    // autoChooser.addOption(
    //     "Oval Test Slow", createTuningAutoPath("OvalTestSlow", false, drivetrain));
    // autoChooser.addOption(
    //     "Oval Test Fast", createTuningAutoPath("OvalTestFast", false, drivetrain));

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    // autoChooser.addOption("Drive Velocity Tuning",
    // this.getDriveVelocityTuningCommand(drivetrain));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    // autoChooser.addOption(
    //     "Swerve Rotation Tuning", this.getSwerveRotationTuningCommand(drivetrain));

    /************ Drive Wheel Radius Characterization ************
     *
     * useful for characterizing the drive wheel Radius
     *
     */
    // autoChooser.addOption( // start by driving slowing in a circle to align wheels
    //     "Drive Wheel Radius Characterization",
    //     this.getDriveWheelRadiusCharacterizationCommand(drivetrain));
  }

  //   public void configureAutoCommands(DifferentialDrivetrain drivetrain, Vision vision) {
  //     // add commands to the auto chooser
  //     autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

  //     /************ Start Point ************
  //      *
  //      * useful for initializing the pose of the robot to a known location
  //      *
  //      */

  //     Command startPoint =
  //         Commands.runOnce(
  //             () -> {
  //               try {
  //                 drivetrain.resetPose(
  //                     PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
  //               } catch (Exception e) {
  //                 pathFileMissingAlert.setText("Could not find the specified path file: Start
  // Point");
  //                 pathFileMissingAlert.set(true);
  //               }
  //             },
  //             drivetrain);
  //     autoChooser.addOption("Start Point", startPoint);

  //     /************ Differential Auto ************
  //      *
  //      * example PathPlanner auto for XRP
  //      *
  //      */

  //     autoChooser.addOption("Differential Example", new PathPlannerAuto("Differential Auto"));
  //   }

  //   private Command getDriveVelocityTuningCommand(SwerveDrivetrain drivetrain) {
  //     return Commands.sequence(
  //         Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
  //         Commands.repeatingSequence(
  //             Commands.deadline(
  //                 Commands.waitSeconds(1.0),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(2.0),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(1.0),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(-0.5),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(1.0),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(1.0),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(0.5),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(3.0),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(2.0),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(1.0),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(2.0),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(-1.0),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(0.5),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(-3.0),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(2.0),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(-1.0),
  //                             MetersPerSecond.of(0.0),
  //                             RadiansPerSecond.of(0.0),
  //                             false,
  //                             false),
  //                     drivetrain))));
  //   }

  //   private Command getSwerveRotationTuningCommand(SwerveDrivetrain drivetrain) {
  //     return Commands.sequence(
  //         Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
  //         Commands.repeatingSequence(
  //             Commands.deadline(
  //                 Commands.waitSeconds(0.5),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(0.1),
  //                             MetersPerSecond.of(0.1),
  //                             RadiansPerSecond.of(0.0),
  //                             true,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(0.5),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(-0.1),
  //                             MetersPerSecond.of(0.1),
  //                             RadiansPerSecond.of(0.0),
  //                             true,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(0.5),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(-0.1),
  //                             MetersPerSecond.of(-0.1),
  //                             RadiansPerSecond.of(0.0),
  //                             true,
  //                             false),
  //                     drivetrain)),
  //             Commands.deadline(
  //                 Commands.waitSeconds(0.5),
  //                 Commands.run(
  //                     () ->
  //                         drivetrain.drive(
  //                             MetersPerSecond.of(0.1),
  //                             MetersPerSecond.of(-0.1),
  //                             RadiansPerSecond.of(0.0),
  //                             true,
  //                             false),
  //                     drivetrain))));
  //   }

  //   private Command getDriveWheelRadiusCharacterizationCommand(SwerveDrivetrain drivetrain) {
  //     return CharacterizationCommands.wheelRadiusCharacterization(drivetrain);
  //   }

  //   private Command createTuningAutoPath(
  //       String autoName, boolean measureDistance, SwerveDrivetrain drivetrain) {
  //     return Commands.sequence(
  //         Commands.runOnce(drivetrain::captureInitialConditions),
  //         new PathPlannerAuto(autoName),
  //         Commands.runOnce(() -> drivetrain.captureFinalConditions(autoName, measureDistance)));
  //   }

}
