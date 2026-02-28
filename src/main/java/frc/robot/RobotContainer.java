// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Constants.SwerveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SpinIndexer;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.SpinSpindexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.endeffector.IndexerIOKraken;
import frc.robot.subsystems.endeffector.Shooter;
import frc.robot.subsystems.endeffector.ShooterIOKraken;
import frc.robot.subsystems.endeffector.SpindexerIO;
import frc.robot.subsystems.endeffector.SpindexerIOKraken;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //   private final Vision vision;
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  // private final Vision vision;
  private SwerveDriveKinematics swerveKinematics;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  private final Trigger controllerrt = controller.rightTrigger();
  private final Trigger controllerrb = controller.rightBumper();
  private final Trigger controllerlt = controller.leftTrigger();
  private final Trigger controllerb = controller.b();
  private final Trigger controller2rt = controller2.rightTrigger();

  private final SpinSpindexer spinSpindexer;
  private final SpinIndexer spinIndexer;
  private final SpinShooter spinShooter;
  private final SpinIntake spinIntake;
  private final SpinShooter passShooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(SwerveConstants.FrontLeft),
                new ModuleIOTalonFX(SwerveConstants.FrontRight),
                new ModuleIOTalonFX(SwerveConstants.BackLeft),
                new ModuleIOTalonFX(SwerveConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOLimelight("limelight-intake", drive::getRotation));

        shooter =
            new Shooter(
                new ShooterIOKraken(),
                // new TurretIOKraken(),
                drive,
                new IndexerIOKraken(),
                new SpindexerIOKraken());

        intake = new Intake(new IntakeIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(SwerveConstants.FrontLeft),
                new ModuleIOSim(SwerveConstants.FrontRight),
                new ModuleIOSim(SwerveConstants.BackLeft),
                new ModuleIOSim(SwerveConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOLimelight("limelight-intake", drive::getRotation));
        shooter =
            new Shooter(
                new ShooterIOKraken(),
                // new TurretIOSim(),
                drive,
                new IndexerIOKraken(),
                new SpindexerIO() {});

        intake = new Intake(new IntakeIOReal());
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOLimelight("limelight-intake", drive::getRotation));
        shooter =
            new Shooter(
                new ShooterIOKraken(),
                // new TurretIOKraken(),
                drive,
                new IndexerIOKraken(),
                new SpindexerIO() {});

        intake = new Intake(new IntakeIOReal());
        break;
    }

    spinIndexer = new SpinIndexer(shooter);
    spinIntake = new SpinIntake(intake);
    spinSpindexer = new SpinSpindexer(shooter);
    spinShooter = new SpinShooter(shooter, -40);
    passShooter = new SpinShooter(shooter, -150);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    // intake.setDefaultCommand(
    //     new InstantCommand(() -> intake.changeAngleTest(controller.getLeftY()), intake));

    // Lock to 0° when A button is held

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.y().onTrue(new SimulateShotTrajectory(drive, shooter));
    // controller.leftBumper().onTrue(new InstantCommand(() -> shooter.spinShooter(1)));
    // controller
    //     .a()
    //     .whileTrue(new InstantCommand(() -> shooter.spinShooter(-.9)))
    //     .whileTrue(new InstantCommand(() -> shooter.setIndexerSpeed(-1)))
    //     .whileTrue(new InstantCommand(() -> shooter.setSpindexerSpeed(1)))
    //     .whileTrue(new InstantCommand(() -> intake.setIntakeSpeed(.8)))
    //     .whileFalse(new InstantCommand(() -> shooter.setIndexerSpeed(0)))
    //     .whileFalse(new InstantCommand(() -> shooter.spinShooter(0)))
    //     .whileFalse(new InstantCommand(() -> shooter.setSpindexerSpeed(0)))
    //     .whileFalse(new InstantCommand(() -> intake.setIntakeSpeed(0)));

    // Reset gyro to 0° when B button is pressed
    controllerb.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true));

    controllerlt.whileTrue(spinIntake);

    // controller.a().onTrue(new InstantCommand(() -> intake.resetEncoder()));

    // controller.x().whileTrue(DriveCommands.joystickDrive(drive, () -> -1, () -> 0, () -> 0));
    controllerrt.whileTrue(spinShooter);
    controller2rt.whileTrue(passShooter);

    controllerrb.whileTrue(spinIndexer).whileTrue(spinSpindexer);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
