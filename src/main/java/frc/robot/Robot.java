// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDConstants;
import frc.robot.subsystems.Drive;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 /128 * 6 * Math.PI / 12;
  private Joystick joy1 = new Joystick(0);

  private final AybarsBot m_robot = new AybarsBot();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Configure default commands and condition bindings on robot startup
    m_robot.configureBindings();
    m_robot.loadAutoPaths();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Encoder value", encoder.get() * kDriveTick2Feet );
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robot.getAutonomousCommand();

    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    */
  }

  final double kP = 1.0;

  double setpoint = 0;

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    encoder.reset();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //get joystick command


    double sensorPosition = encoder.get() * kDriveTick2Feet;

    double error = setpoint - sensorPosition;

    double outputSpeed = kP * error;

    Drive.m_leftLeadMotor.set(outputSpeed);
    Drive.m_leftFollowMotor.set(outputSpeed);
    Drive.m_rightLeadMotor.set(-outputSpeed);
    Drive.m_rightFollowMotor.set(-outputSpeed);

  }

  private void onFalse(double d) {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
