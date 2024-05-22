// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  private static Robot   instance;
  private        Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.usingVision = false;
    m_robotContainer.getShooterSubsystem().stopPivot();
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    // Robot should have the correct angle, and this angle should not change, so we disable vision.
    // This is more of a safeguard really.
    m_robotContainer.usingVision = false;
    m_robotContainer.getShooterSubsystem().stopPivot();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.usingVision = false;
    m_robotContainer.getShooterSubsystem().stopPivot();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic(){
    // Since we can access any subsystem from the Robot.java class, I think it makes sense to put our tracking here.
    Limelight limelight = m_robotContainer.getLimelight();
    ShooterSubsystem shooterSubsystem = m_robotContainer.getShooterSubsystem();
    if(m_robotContainer.usingVision){ // Makes sure the Limelight Never Rotates The Shooter Unless It Is Supposed To.
      if(limelight.enabled){ // Checks That The Limelight Actually Exists At This Point
        if(limelight.getTargetVisible()){ // Checks That The Limelight Is Tracking
          if(5 <= shooterSubsystem.getShooterPosition() && shooterSubsystem.getShooterPosition() <= 65){ // Hard Limit Just In Case The Limelight Rotates Too Far. Mostly For Test Stage
            if(shooterSubsystem.getShooterPosition() < limelight.angleChosen){ // Lower Than Wanted Angle
              shooterSubsystem.rotatePivot(-0.15); // Negative For Raising Shooter
            } else if(shooterSubsystem.getShooterPosition() > limelight.angleChosen){
              shooterSubsystem.rotatePivot(0.15); // Positive For Lowering Shooter
            } else if(shooterSubsystem.getShooterPosition() == limelight.angleChosen){
              shooterSubsystem.stopPivot(); // Angle Is Correct, So There Is No Need To Change It
            }
          } else {
            shooterSubsystem.stopPivot(); // Beyond Limit, So We Need To Switch To Manual
          }
        } else {
          shooterSubsystem.stopPivot(); // No Target, So No Point
      }
    } 
  }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}