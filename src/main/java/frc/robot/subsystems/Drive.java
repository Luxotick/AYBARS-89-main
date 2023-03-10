// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
  // Drivetrain motors
  public final static CANSparkMax m_leftLeadMotor = new CANSparkMax(CANIDConstants.drivebaseLeftLeadMotorID,
  MotorType.kBrushed);
  public final static CANSparkMax m_leftFollowMotor = new CANSparkMax(CANIDConstants.drivebaseLeftFollowMotorID,
  MotorType.kBrushed);
  public final static CANSparkMax m_rightLeadMotor = new CANSparkMax(CANIDConstants.drivebaseRightLeadMotorID,
  MotorType.kBrushed);
  public final static CANSparkMax m_rightFollowMotor = new CANSparkMax(CANIDConstants.drivebaseRightFollowMotorID,
  MotorType.kBrushed);
  // The robot's drive
  public final static DifferentialDrive m_drive = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);

  // The left-side drive encoder
  private final Encoder m_leftEncoder = new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // PID Controllers
  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI,
      DriveConstants.kLeftD);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kRightP, DriveConstants.kRightI,
      DriveConstants.kRightD);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      DriveConstants.kTrackWidthMeters);

  private final RamseteController m_ramseteController = new RamseteController();

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV);

  /** Creates a new Drive subsystem. */
  public Drive() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    // Reset encoders
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    // Reset motors
    m_leftLeadMotor.restoreFactoryDefaults();
    m_leftFollowMotor.restoreFactoryDefaults();
    m_rightLeadMotor.restoreFactoryDefaults();
    m_rightFollowMotor.restoreFactoryDefaults();

    m_leftLeadMotor.setInverted(DriveConstants.kLeftLeadMotorInverted);
    m_leftFollowMotor.setInverted(DriveConstants.kLeftFollowMotorInverted);
    m_rightLeadMotor.setInverted(DriveConstants.kRightLeadMotorInverted);
    m_rightFollowMotor.setInverted(DriveConstants.kRightFollowMotorInverted);

    m_leftLeadMotor.setSmartCurrentLimit(40);
    m_leftFollowMotor.setSmartCurrentLimit(40);
    m_rightLeadMotor.setSmartCurrentLimit(40);
    m_rightFollowMotor.setSmartCurrentLimit(40);

    m_leftLeadMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowMotor.setIdleMode(IdleMode.kBrake);
    m_rightLeadMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowMotor.setIdleMode(IdleMode.kBrake);

    // Make the motors on the same side follow each other
    m_leftFollowMotor.follow(m_leftLeadMotor);
    m_rightFollowMotor.follow(m_rightLeadMotor);

    m_drive.setMaxOutput(DriveConstants.kNormalMaxSpeedPercentage);
    SmartDashboard.putData(m_leftEncoder);
    SmartDashboard.putData(m_rightEncoder);
    SmartDashboard.putData("sex", m_gyro);


    m_odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(m_gyro.getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  @Override
  public void periodic() {
    // Update odometry
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Gyro 1", m_gyro.getXComplementaryAngle());
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Returns the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeadMotor.setVoltage(leftVolts);
    m_rightLeadMotor.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Sets the desired wheel speeds.
   * 
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftLeadMotor.setVoltage(leftOutput + leftFeedforward);
    m_rightLeadMotor.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public CommandBase driveCommand(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    return run(() -> setSpeeds(wheelSpeeds));
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  /**
   * Returns a command that boosts robot speed
   * 
   * @param boost make it fast / or not
   */
  public CommandBase boostCommand(boolean boost) {
    return runOnce(
        () -> {
          m_drive.setMaxOutput(
              boost ? DriveConstants.kBoostedMaxSpeedPercentage : DriveConstants.kNormalMaxSpeedPercentage);
        })
        .withName("Boost");
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a
   * specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed          The fraction of max speed at which to drive
   */
  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
        () -> {
          // Reset encoders at the start of the command
          m_leftEncoder.reset();
          m_rightEncoder.reset();
        })
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () -> Math.max(m_leftEncoder.getDistance(), m_rightEncoder.getDistance()) >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
  }

  /**
   * Returns a command that resets the field-relative position to a specific
   * location.
   *
   * @param pose The position to reset to.
   */
  public CommandBase resetOdometryCommand(Pose2d pose) {
    return runOnce(() -> {
      m_odometry.resetPosition(
          Rotation2d.fromDegrees(m_gyro.getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    })
        .withName("resetOdometry");
  }

  /**
   * 
   * @param trajectory a PathPlanner Trajectory to follow
   * @return a PPRamseteCommand that follows the given PathPlanner Trajectory
   */
  public CommandBase followTrajectoryCommand(PathPlannerTrajectory trajectory) {
    return new PPRamseteCommand(
        trajectory,
        this::getPose,
        m_ramseteController,
        m_feedforward,
        m_kinematics,
        this::getWheelSpeeds,
        m_leftPIDController,
        m_rightPIDController,
        this::tankDriveVolts,
        true,
        this // Requires this drive subsystem
    );
  }
}
