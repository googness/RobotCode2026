// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.RobotConfig.CameraConfig;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainIO;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainIOCTRE;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutonomousCommandsFactory;
import frc.robot.commands.SwerveDrivetrainCommandFactory;
import frc.robot.commands.Vision.DriveToShoot;
import frc.robot.commands.Vision.ResetPosetoVision;
import frc.robot.commands.Vision.RotateToHubCommand;
import frc.robot.commands.Vision.ShootingZoneCommand;
import frc.robot.configs.ThunderRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.flywheels.Flywheel;
import frc.robot.subsystems.flywheels.FlywheelIO;
import frc.robot.subsystems.flywheels.FlywheelIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.vision.*;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private SwerveDrivetrain swerveDrivetrain;
  private Alliance lastAlliance = Field2d.getInstance().getAlliance();
  private Vision vision;
  private VisionSystem jayvision;

  // private XboxController TunerStick = new XboxController(5);

  private Flywheel flywheel;
  private Hopper hopper;

  private static final String LAYOUT_FILE_MISSING =
      "Could not find the specified AprilTags layout file";
  private Alert layoutFileMissingAlert = new Alert(LAYOUT_FILE_MISSING, AlertType.kError);

  private Alert tuningAlert = new Alert("Tuning mode enabled", AlertType.kInfo);

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */
    createRobotConfig();

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {

      switch (Constants.getRobot()) {
        case ROBOT_THUNDER:
          {
            createCTRESubsystems();
            break;
          }
        case ROBOT_SIMBOT:
          {
            createCTRESimSubsystems();
            break;
          }
        default:
          break;
      }

    } else {
      swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});

      CameraConfig[] cameraConfigs = config.getCameraConfigs();
      VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
      for (int i = 0; i < visionIOs.length; i++) {
        visionIOs[i] = new VisionIO() {};
      }
      vision = new Vision(visionIOs);

      flywheel = new Flywheel(new FlywheelIO() {});
      hopper = new Hopper(new HopperIO() {});
      jayvision = new VisionSystem(swerveDrivetrain);
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    // !!! Removing Diff Drive Checks
    // register autonomous commands
    // if (RobotConfig.getInstance().getDrivetrainType() ==
    // RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
    //   AutonomousCommandsFactory.getInstance().configureAutoCommands(differentialDrivetrain,
    // vision);
    // } else if (RobotConfig.getInstance().getDrivetrainType()
    //     == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
    //   AutonomousCommandsFactory.getInstance().configureAutoCommands(swerveDrivetrain, vision);
    // }
    AutonomousCommandsFactory.getInstance()
        .configureAutoCommands(swerveDrivetrain, hopper, flywheel, jayvision);

    // Alert when tuning
    if (Constants.TUNING_MODE) {
      this.tuningAlert.set(true);
    }
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    switch (Constants.getRobot()) {
      case ROBOT_THUNDER, ROBOT_SIMBOT:
        config = new ThunderRobotConfig();
        break;
      default:
        break;
    }
  }

  private void createCTRESubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);

      layoutFileMissingAlert.setText(
          LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      layoutFileMissingAlert.set(true);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIOPhotonVision(cameraConfigs[i].id(), layout);
    }
    vision = new Vision(visionIOs);

    flywheel = new Flywheel(new FlywheelIOTalonFX() {});
    hopper = new Hopper(new HopperIOTalonFX() {});
    jayvision = new VisionSystem(swerveDrivetrain);
  }

  private void createCTRESimSubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      layoutFileMissingAlert.setText(
          LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      layoutFileMissingAlert.set(true);
    }

    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] =
          new VisionIOSim(
              cameraConfigs[i].id(),
              layout,
              swerveDrivetrain::getPose,
              cameraConfigs[i].robotToCameraTransform());
    }
    vision = new Vision(visionIOs);

    flywheel = new Flywheel(new FlywheelIOTalonFX());
    hopper = new Hopper(new HopperIOTalonFX());
    jayvision = new VisionSystem(swerveDrivetrain);
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   */
  private void constructField() {
    Field2d.getInstance().setRegions(new Region2d[] {});
  }

  /**
   * This method scans for any changes to the connected operator interface (e.g., joysticks). If
   * anything changed, it creates a new OI object and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }

    configureButtonBindings();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();

    oi.testSpeed()
        .toggleOnTrue(new InstantCommand(() -> flywheel.setVelocity(47)))
        .onFalse(new InstantCommand(() -> flywheel.setVelocity(0)));

    oi.setAgitator()
        .whileTrue(
            new ShootingZoneCommand(
                swerveDrivetrain, jayvision, () -> oi.getTranslateX(), () -> oi.getTranslateY()));

    oi.jayShot()
        .whileTrue(
            flywheel
                .runScoreWithVisionCmd(jayvision)
                .alongWith(
                    new WaitCommand(.75)
                        .andThen(
                            hopper
                                .runFeedBallsToShooterCmd()
                                .alongWith(hopper.runHopperPulseCmd()))));

    oi.runIntake()
        .onTrue(
            new InstantCommand(hopper::GoToIntakePOS)
                .alongWith(new WaitCommand(0.15))
                .andThen(hopper::runIntake))
        .onFalse(new InstantCommand(hopper::stopIntake));

    // Hopper Commands
    oi.HopperToHomePos().onTrue(new InstantCommand(hopper::GoToHomePOS));

    oi.HopperToIntakePos().onTrue(new InstantCommand(hopper::GoToIntakePOS));

    oi.resetVisionPos().onTrue(new ResetPosetoVision(jayvision));

    oi.RotateToTag()
        .whileTrue(
            new RotateToHubCommand(
                swerveDrivetrain,
                jayvision,
                flywheel,
                () -> oi.getTranslateX(),
                () -> oi.getTranslateY()));

    oi.DriveToShoot().whileTrue(DriveToShoot.driveToShootCommand(swerveDrivetrain));

    // configureVisionCommands();

    // new JoystickButton(TunerStick, XboxController.Button.kY.value)
    //     .whileTrue(new ManualAimCmd(hopper, TunerStick::getLeftY));

    // new JoystickButton(TunerStick, XboxController.Button.kA.value)
    //     .onTrue(new InstantCommand(hopper::GoToHomePOS));

    // new JoystickButton(TunerStick, XboxController.Button.kB.value)
    //     .onTrue(new InstantCommand(hopper::GoToIntakePOS));

    // new JoystickButton(TunerStick, XboxController.Button.kX.value)
    //     .onTrue(new InstantCommand(hopper::runIntake))
    //     .onFalse(new InstantCommand(hopper::stopIntake));

    // Shooter Commands
    // oi.lockOn()
    //     .toggleOnTrue(
    //         new RotateToHubCommand(
    //             swerveDrivetrain,
    //             jayvision,
    //             flywheel,
    //             () -> oi.getTranslateX(),
    //             () -> oi.getTranslateY()));

    /*     oi.testSpeed()
           .toggleOnTrue(new InstantCommand(() -> flywheel.testSpeed()))
           .onFalse(new InstantCommand(() -> flywheel.stopFlywheel()));
    */

    // oi.jayShot()
    //     .whileTrue(
    //         FlywheelCmds.runAccelerator(flywheel)
    //             .alongWith(new InstantCommand(() -> hopper.setAgitator(-0.7)))
    //             .alongWith(new InstantCommand(() -> hopper.runIntake()))
    //             .alongWith(HopperCmds.runBelt(hopper))
    //             .alongWith(
    //                 new WaitCommand(1.5)
    //                     .andThen(
    //                         HopperCmds.GoToHomePos(hopper)
    //                             // .alongWith(hopper.RunClearIntakeCmd())
    //                             .withTimeout(3))))
    //     .onFalse(
    //         FlywheelCmds.stopAccelerator(flywheel)
    //             .alongWith(new InstantCommand(() -> hopper.setAgitator(0)))
    //             .alongWith(HopperCmds.stopBelt(hopper))
    //             .alongWith(HopperCmds.stopIntake(hopper)));

    // oi.jayShot()
    //     .whileTrue(
    //         FlywheelCmds.getFixedShot(flywheel)
    //             .andThen(new WaitCommand(.75))
    //
    // .andThen(hopper.runFeedBallsToShooterCmd().alongWith(hopper.runHopperPulseCmd())));

    // oi.jayShot()
    //     .whileTrue(
    //         FlywheelCmds.getFixedShot(flywheel)
    //             .andThen(new WaitCommand(.75))
    //             .andThen(
    //                 new InstantCommand(() -> flywheel.runAccelerator())
    //                     .alongWith(new InstantCommand(() -> hopper.runBelt()))
    //                     .alongWith(new InstantCommand(() -> hopper.runAgitator())))
    //             .alongWith(new InstantCommand(() -> hopper.runIntake()))
    //             .andThen(new WaitCommand(1.75))
    //             .andThen(HopperCmds.SetHopperPOS(17, hopper))
    //             .andThen(new WaitCommand(.5))
    //             .andThen(HopperCmds.SetHopperPOS(21, hopper))
    //             .andThen(new WaitCommand(.5))
    //             .andThen(HopperCmds.SetHopperPOS(17, hopper))
    //             .andThen(new WaitCommand(.5))
    //             .andThen(HopperCmds.SetHopperPOS(21, hopper))
    //             .andThen(new WaitCommand(.5))
    //             .andThen(HopperCmds.SetHopperPOS(17, hopper))
    //             .andThen(new WaitCommand(.5))
    //             .andThen(HopperCmds.SetHopperPOS(21, hopper))
    //             .andThen(new WaitCommand(1))
    //             .andThen(HopperCmds.SetHopperPOS(15, hopper))
    //             .andThen(new WaitCommand(.5))
    //             .andThen(HopperCmds.SetHopperPOS(20, hopper))
    //             .andThen(new WaitCommand(.5))
    //             .andThen(HopperCmds.SetHopperPOS(2, hopper)))
    //     .onFalse(
    //         new InstantCommand(() -> flywheel.stopFlywheel())
    //             .alongWith(new InstantCommand(() -> flywheel.stopAccelerator()))
    //             .alongWith(new InstantCommand(() -> hopper.stopBelt()))
    //             .alongWith(new InstantCommand(() -> hopper.stopAgitator()))
    //             .alongWith(new InstantCommand(() -> hopper.stopIntake())));

    // oi.jayShot().whileTrue(
    //               FlywheelCmds.getFixedShot(flywheel)
    //               .alongWith(new WaitCommand(1))
    //               .andThen(new InstantCommand(() -> flywheel.runAccelerator())
    //               .alongWith(new InstantCommand(() -> hopper.runBelt()))
    //               .alongWith(new InstantCommand(() -> hopper.setAgitator(0.3)))
    //               .alongWith(new WaitCommand(0.5))
    //               .andThen(HopperCmds.GoToHalfPos(hopper)))
    //               .alongWith(new WaitCommand(1))
    //               .andThen(HopperCmds.GoToAgitatePos(hopper)))
    //               .alongWith(new WaitCommand(1))
    //               .andThen(HopperCmds.GoToHomePos(hopper));

    // Intake Commands

    // oi.runIntake()
    //     .onTrue(new InstantCommand(hopper::runIntake))
    //     .onFalse(new InstantCommand(hopper::stopIntake));

    // oi.theShooterFix()
    //     .toggleOnTrue(
    //         // flywheel.ShootRpsCmd()
    //         new RotateToHubCommand(
    //                 swerveDrivetrain,
    //                 jayvision,
    //                 flywheel,
    //                 () -> oi.getTranslateX(),
    //                 () -> oi.getTranslateY())
    //             .alongWith(hopper.RunClearBeltCmd())
    //             .alongWith(
    //                 new WaitCommand(1)
    //                     .andThen(
    //                         flywheel.FeederRpsCmd()
    //                             .alongWith(hopper.RunBeltCmd())
    //                             .alongWith(
    //                                 new WaitCommand(3.5)
    //                                     .andThen(() -> hopper.retractHopper())
    //                                     .andThen(hopper.RunClearIntakeCmd())))));

    // oi.getHubShot()
    //     .toggleOnTrue(
    //         // FlywheelCmds.getHubShot(flywheel)
    //         new RotateToHubCommand(
    //                 swerveDrivetrain,
    //                 jayvision,
    //                 flywheel,
    //                 () -> oi.getTranslateX(),
    //                 () -> oi.getTranslateY())
    //             .alongWith(hopper.RunClearBeltCmd()));

    // oi.resetVisionPos().onTrue(new ResetPosetoVision(jayvision));

    // oi.DriveToShoot()
    //     .onTrue(
    //         DriveToShoot.driveToShootCommand(swerveDrivetrain)
    //             .until(
    //                 () -> {
    //                   // Read the joysticks directly from your OI
    //                   double xStick = Math.abs(oi.getTranslateX());
    //                   double yStick = Math.abs(oi.getTranslateY());
    //                   double rotStick = Math.abs(oi.getRotate());

    //                   // If any stick is pushed further than 0.1 (your deadband),
    //                   // the tripwire is triggered, and the command is instantly killed!
    //                   return xStick > 0.1 || yStick > 0.1 || rotStick > 0.1;
    //                 }));

    // oi.RotateToTag()
    //     .onTrue(
    //         new RotateToTag(
    //             swerveDrivetrain, jayvision, () -> oi.getTranslateX(), () ->
    // oi.getTranslateY()));

    // oi.getRotation()
    //     .toggleOnTrue(
    //         new RotateToHubCommand(
    //             swerveDrivetrain,
    //             jayvision,
    //             flywheel,
    //             () -> oi.getTranslateX(),
    //             () -> oi.getTranslateY()));

    // oi.extendHood().onTrue(FlywheelCmds.extendHood(flywheel));

    // oi.retractHood().onTrue(FlywheelCmds.retractHood(flywheel));

    // oi.extendHopper().onTrue(HopperCmds.extendHopper(hopper));

    // oi.retractHopper()
    //     .onTrue(
    //
    // HopperCmds.retractHopper(hopper).alongWith(hopper.RunClearIntakeCmd()).withTimeout(3));

    // oi.runIntake()
    //     .onTrue(
    //         HopperCmds.extendHopper(hopper)
    //             .alongWith(HopperCmds.runIntake(hopper).alongWith(HopperCmds.runIntake(hopper))))
    //     .onFalse(HopperCmds.stopIntake(hopper));

    // oi.runAccelerator()
    //     .whileTrue(
    //         FlywheelCmds.runAccelerator(flywheel)
    //             .alongWith(HopperCmds.runBelt(hopper))
    //             .alongWith(
    //                 new WaitCommand(1.5)
    //                     .andThen(
    //                         HopperCmds.retractHopper(hopper)
    //                             .alongWith(hopper.RunClearIntakeCmd())
    //                             .withTimeout(3))))
    //     .onFalse(
    //         FlywheelCmds.stopAccelerator(flywheel)
    //             .alongWith(HopperCmds.stopBelt(hopper))
    //             .alongWith(HopperCmds.stopIntake(hopper)));

    // oi.runBelt().onTrue(HopperCmds.runBelt(hopper)).onFalse(HopperCmds.stopBelt(hopper));

    // oi.theShooterFix().onTrue(new InstantCommand(() -> hopper.runIntake()));
    // .toggleOnTrue(
    //     new StartEndCommand(() -> flywheel.setVelocity(), () -> flywheel.stopFlywheel()));

    // oi.reverseIntake()
    //     .onTrue(HopperCmds.reverseIntake(hopper))
    //     .onFalse(HopperCmds.stopIntake(hopper));

    // oi.dShot()
    //     .toggleOnTrue(
    //         new StartEndCommand(
    //             () -> {
    //               flywheel.extendHood();
    //               flywheel.setVelocity(45.5);
    //             },
    //             () -> {
    //               flywheel.stopFlywheel();
    //               flywheel.retractHood();
    //             }));

    // register commands for other subsystems
    // ArmCommandFactory.registerCommands(oi, arm);
    // ElevatorCommandsFactory.registerCommands(oi, elevator);

    // if (RobotConfig.getInstance().getDrivetrainType() ==
    // RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
    //   CrossSubsystemsCommandsFactory.registerCommands(oi, differentialDrivetrain, vision, arm);
    // } else if (RobotConfig.getInstance().getDrivetrainType()
    //     == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
    //   CrossSubsystemsCommandsFactory.registerCommands(
    //       oi, swerveDrivetrain, vision, arm, elevator, manipulator, shooter);
    // }

    // Endgame alerts

  }

  private void configureDrivetrainCommands() {
    SwerveDrivetrainCommandFactory.registerCommands(oi, swerveDrivetrain, vision);

    // if (RobotConfig.getInstance().getDrivetrainType() ==
    // RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
    //   DifferentialDrivetrainCommandFactory.registerCommands(oi, differentialDrivetrain);
    // } else if (RobotConfig.getInstance().getDrivetrainType()
    //     == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
    //   SwerveDrivetrainCommandFactory.registerCommands(oi, swerveDrivetrain, vision);
    // }
  }

  // private void configureVisionCommands() {
  //   // enable/disable vision
  //   oi.getVisionIsEnabledTrigger()
  //       .onTrue(
  //           Commands.runOnce(() -> vision.enable(true))
  //               .ignoringDisable(true)
  //               .withName("enable vision"));
  //   oi.getVisionIsEnabledTrigger()
  //       .onFalse(
  //           Commands.runOnce(() -> vision.enable(false))
  //               .ignoringDisable(true)
  //               .withName("disable vision"));
  // }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      Field2d.getInstance().updateAlliance(this.lastAlliance);
    }
  }

  public void periodic() {
    // add robot-wide periodic code here
    // visualization.update();
  }

  public void autonomousInit() {
    // add robot-wide code here that will be executed when autonomous starts
  }

  public void teleopInit() {
    // check if the alliance color has changed based on the FMS data; if the robot power cycled
    // during a match, this would be the first opportunity to check the alliance color based on FMS
    // data.
    this.checkAllianceColor();
  }
}
