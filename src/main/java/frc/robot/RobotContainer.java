// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BothClimbers;
import frc.robot.commands.BothClimbersDown;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.PSIntake;
import frc.robot.commands.PrepareSpeakerShoot;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here.
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final SuperStructure m_superstructure = new SuperStructure();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Limelight m_Limelight = Limelight.getLimelightInstance();

  // Important Booleans. Controls Limelight Usage Along With Enabling The Backup For An Autonomous.
  // boolean usingVision = false; // Tells If The Limelight Should Be Modifying The Shooter Angle. Will be manually set to true during testing.

  boolean alwaysUseBackupAuto = true; // Should always be false, except for testing or if main auto isn't working. The backup auto will always run if no alliance is selected.

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("prepareShoot", new PrepareSpeakerShoot(m_shooter));
    NamedCommands.registerCommand("shootSpeaker", new SpeakerShoot(m_shooter, m_superstructure));
    NamedCommands.registerCommand("intakeGround", new GroundIntake(m_intake, m_superstructure));
    NamedCommands.registerCommand("intakeGroundAuto", useIntake());
    NamedCommands.registerCommand("intakeLower", lowerIntake());
    NamedCommands.registerCommand("intakeRaise", raiseIntake());
    NamedCommands.registerCommand("stopShooter", m_shooter.stopShooter());
    NamedCommands.registerCommand("stopFeeder", m_superstructure.stopFeeder());
    NamedCommands.registerCommand("stopIntakeFeeder", stopIntake());
    NamedCommands.registerCommand("zeroGyro", Commands.runOnce(drivebase::zeroGyro));
    NamedCommands.registerCommand("AngleShooterAuto", m_shooter.rotateShooter(0).withTimeout(1));
    configureBindings();
  }
  private void configureBindings() {
    // Main Driver Controller
    Constants.DriverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro))); // Zero Gyro
    Constants.DriverController.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly()); // Stop Movement On Swerve
    Constants.DriverController.y().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              )); // ? We may never know.. Something with swerve, I believe
    Constants.DriverController.leftTrigger().whileTrue(m_climber.runLeft(0.6)); // Raise Left Climber Side
    Constants.DriverController.rightTrigger().whileTrue(m_climber.runRight(0.6)); // Raise Right Climber Side
    Constants.DriverController.leftBumper().whileTrue(m_climber.runLeft(-0.6)); // Lower Left Climber Side
    Constants.DriverController.rightBumper().whileTrue(m_climber.runRight(-0.6)); // Lower Right Climber Side
    Constants.DriverController.povUp().whileTrue(new BothClimbers(m_climber)); // Raise Both Climbers
    Constants.DriverController.povDown().whileTrue(new BothClimbersDown(m_climber)); // Lower Both Climbers
    /*Constants.DriverController.povLeft().onTrue(Commands.runOnce(() -> {
      m_shooter.stopPivot();
      usingVision = !usingVision;
      printUsingVision();
    }));
    Constants.DriverController.povRight().onTrue(Commands.runOnce(() -> 
    System.out.println(m_Limelight.yOffset))); // Print Limelight's Desired Angle, If Valid. Will Print 0.0 Otherwise.
    */
    // Secondary Operator Controller
    Constants.OperatorController.y().whileTrue(new PSIntake(m_shooter, m_superstructure)); // Hold To Intake Through Shooter
    Constants.OperatorController.x().whileTrue(new GroundIntake(m_intake, m_superstructure)); // Hold To Intake Through Ground
    Constants.OperatorController.b().whileTrue(
                                        new PrepareSpeakerShoot(m_shooter)
                                        .andThen(new WaitCommand(1))
                                        .andThen (new SpeakerShoot(m_shooter, m_superstructure))
                                        .andThen(new WaitCommand(1))
                                        .andThen(Commands.parallel(m_shooter.stopShooter(),m_superstructure.stopFeeder()))
                                        .handleInterrupt(() -> Commands.parallel(m_shooter.stopShooter(),m_superstructure.stopFeeder()))); // Hold B To Shoot
    Constants.OperatorController.a().whileTrue(
                                     prepareAmpShoot()
                                    .withTimeout(1)
                                    .andThen(AmpShoot()
                                    .handleInterrupt(() -> 
                                     stopAmpShoot())));
    Constants.OperatorController.leftBumper().onTrue(m_intake.rotateIntake(-0.2)).onFalse(m_intake.stopRotate()); // Lower Intake
    Constants.OperatorController.rightBumper().onTrue(m_intake.rotateIntake(0.2)).onFalse(m_intake.stopRotate()); // Raise Intake
    Constants.OperatorController.leftTrigger().whileTrue(m_shooter.rotateShooter(-0.15).alongWith(Commands.runOnce(() -> { 
      /*if(usingVision){
        usingVision = false;
        m_shooter.stopPivot(); // Stops shooter momentarily, but will start again almost immediately
        printUsingVision();
      }*/
    }))); // Lower Shooter, Disables Vision
    Constants.OperatorController.rightTrigger().whileTrue(m_shooter.rotateShooter(0.15).alongWith(Commands.runOnce(() -> {
      /*if(usingVision){
        usingVision = false;
        m_shooter.stopPivot(); // Stops shooter momentarily, but will start again almost immediately
        printUsingVision();
      }*/
    }))); // Raise Shooter, Disables Vision
    Constants.OperatorController.povUp().whileTrue(new BothClimbers(m_climber)); // Raise Both Climbers
    Constants.OperatorController.povDown().whileTrue(new BothClimbersDown(m_climber)); // Lower Both Climbers
    Constants.OperatorController.povLeft().onTrue(Commands.runOnce(() -> { // Toggle Limelight Vision
      /*usingVision = !usingVision;
      printUsingVision();
      m_shooter.stopPivot();*/
    }));
    /*Constants.OperatorController.povRight().onTrue(Commands.runOnce(() -> 
    System.out.println(m_Limelight.yOffset)));*/ // Print Limelight's yOffset, If Valid
  }
  // Methods used during auto to use intake. Integer values have placeholder values, and will be set later.

  // Method For Grabbing Rings With Intake. This method works, but will be tuned during testing.
  public Command useIntake() {
    return Commands.parallel(m_intake.floorIntakeAuto(), m_superstructure.startFeeder()).withTimeout(1);
  }

  // Method For Lowering The Intake. This method does nothing at the moment, until we can start testing it.
  public Command lowerIntake(){
    return m_intake.rotateIntake(0).withTimeout(1).andThen(m_intake.stopRotate());
  }

  // Method For Raising The Intake. This method does nothing at the moment, until we can start testing it.
  public Command raiseIntake(){
    return m_intake.rotateIntake(0).withTimeout(1).andThen(m_intake.stopRotate());
  }

  // Method For Stopping The Intake
  public Command stopIntake(){
    return Commands.parallel(m_intake.stopIntake(), m_superstructure.stopFeeder()).withTimeout(1);
  }

  // Amp Methods, Set How Trevor Put Originally.
  public Command prepareAmpShoot() {
    return Commands.run(() -> m_shooter.prepareAmpShoot());
  }
  
  public Command AmpShoot() {
    return m_superstructure.setFeederControlled(0.6);
  }
  
  public void stopAmpShoot() {
    m_shooter.stopShooterWheels();
    m_superstructure.stopFeeder();
  }

  // Getting subsystems. This gave me a headache for no reason.
  public Limelight getLimelight() { // Could've just made the class public, but it works.
    return m_Limelight;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooter;
  }

  /*public void printUsingVision() {
    if(usingVision){
      System.out.println("Vision Enabled");
    } else {
      System.out.println("Vision Disabled");
    }
  }*/

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    Optional<DriverStation.Alliance> blue = Optional.of(DriverStation.Alliance.Blue);
    Optional<DriverStation.Alliance> red = Optional.of(DriverStation.Alliance.Red);
    String autoName = "Backup"; // Set to backup by default.
    /* Choose Auto To Be Used. If alliance is not set or "alwaysUseBackupAuto" is set to true, 
    then this will just shoot at the speaker, currently. 
    Otherwise, a four note auto will run with a separate auto for each team.
    Currently, the same auto runs for both teams. I'm not messing with that yet. */
      if(alwaysUseBackupAuto == true){
        System.out.println("[Pathplanner] Using Backup Auto!");
      } else if (DriverStation.getAlliance().equals(blue)){
        // Blue Alliance Auto Is Placed Here.
        autoName = "Blue";
        System.out.println("[Pathplanner] Using Blue Auto!");
      } else if (DriverStation.getAlliance().equals(red)){
        // Red Alliance Auto Is Placed Here. This auto doesn't currently exist, as I am waiting until I can test.
        autoName = "Red";
        System.out.println("[Pathplanner] Using Red Auto!");
      } else {
        // Backup Auto is set at the top, and this simply prints that it will be used. Alliance autos will override the backup auto.
        System.out.println("[Pathplanner] Using Backup Auto");
      }
    // Sends auto that runs in autonomous using the string name.
    return new PathPlannerAuto(autoName);
  }

  public void setDriveMode()
  {
    // Applies deadbands and inverts controls because joysticks are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation, right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -Constants.DriverController.getRightX(),
        () -> -Constants.DriverController.getRightY());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(Constants.DriverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -Constants.DriverController.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
