// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final int teamNumber = 6934;
  private static final int limelightThrottleDisabled = 200;
  private static final int limelightThrottleEnabled = 0;

  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private HttpCamera limelightCamera;

  private Command m_autonomousCommand;
  private boolean lastEnabledState = false;
  private boolean throttleInitialized = false;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "6934RoboticsCode2026Unofficial");
    Logger.recordMetadata("Build", "AdvantageKitMigration");

    Logger.addDataReceiver(new NT4Publisher());
    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }
    Logger.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    LimelightHelpers.setStreamMode_Standard(Constants.Vision.limelightName);
    String[] limelightUrls = buildLimelightStreamUrls();
    limelightCamera = new HttpCamera("Limelight", limelightUrls, HttpCameraKind.kMJPGStreamer);
    limelightCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    CameraServer.startAutomaticCapture(limelightCamera);

    Logger.recordOutput("Vision/Camera/UrlPrimary", limelightUrls[0]);
    Logger.recordOutput("Vision/Camera/UrlSecondary", limelightUrls[1]);
    Logger.recordOutput("Vision/Camera/UrlTertiary", limelightUrls[2]);
    Logger.recordOutput("Vision/Camera/UrlQuaternary", limelightUrls[3]);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    boolean enabled = DriverStation.isEnabled();
    if (!throttleInitialized || enabled != lastEnabledState) {
      int throttleValue = enabled ? limelightThrottleEnabled : limelightThrottleDisabled;
      LimelightHelpers.setThrottle(Constants.Vision.limelightName, throttleValue);
      Logger.recordOutput("Vision/LimelightThrottleSet", throttleValue);

      lastEnabledState = enabled;
      throttleInitialized = true;
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private String[] buildLimelightStreamUrls() {
    String hostName = Constants.Vision.limelightName + ".local";
    int teamHigh = teamNumber / 100;
    int teamLow = teamNumber % 100;
    String teamIp = "10." + teamHigh + "." + teamLow + ".11";

    return new String[] {
        "http://" + hostName + ":5800/stream.mjpg",
        "http://" + teamIp + ":5800/stream.mjpg",
        "http://" + hostName + ":5800/?action=stream",
        "http://" + teamIp + ":5800/?action=stream"
    };
  }
}
