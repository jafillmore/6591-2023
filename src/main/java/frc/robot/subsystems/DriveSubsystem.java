// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontLeftDriveInversion);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kFrontRightDriveInversion);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      DriveConstants.kBackLeftDriveInversion);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      DriveConstants.kBackRightDriveInversion);

  // The gyro sensor
    /* Communicate w/navX-MXP via the USB Port.                                        */
    /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
  public AHRS m_gyro = new AHRS(Port.kUSB);
  public int resetGyroCount = 0;
  public int setXcount = 0;
  public int fieldRelativeCount = 0;


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

/* 
        /* IMU Status   
        SmartDashboard.putBoolean(  "IMU_Connected",  m_gyro.isConnected());
        SmartDashboard.putNumber(   "IMU_Temp_C",     m_gyro.getTempC());        
        SmartDashboard.putNumber(   "IMU_TotalYaw",   m_gyro.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS", m_gyro.getRate());
        SmartDashboard.putNumber(   "IMU_Pitch",      m_gyro.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",       m_gyro.getRoll());


        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) 
          
        SmartDashboard.putNumber(   "IMU_Accel_X",          m_gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          m_gyro.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         m_gyro.isMoving());
        SmartDashboard.putBoolean(  "IMU_IsRotating",       m_gyro.isRotating());


        /* Swerve Drive Module Actual Positions 
        SmartDashboard.putNumber("Swerve: LF Turn Angle - Deg", Math.toDegrees(m_frontLeft.m_turningEncoder.getPosition()));
        SmartDashboard.putNumber("Swerve: RF Turn Angle - Deg", Math.toDegrees(m_frontRight.m_turningEncoder.getPosition()));
        SmartDashboard.putNumber("Swerve: LR Turn Angle - Deg", Math.toDegrees(m_rearLeft.m_turningEncoder.getPosition()));
        SmartDashboard.putNumber("Swerve: RR Turn Angle - Deg", Math.toDegrees(m_rearRight.m_turningEncoder.getPosition()));

        /* Swerve Drive Module Desired Positions 
        SmartDashboard.putNumber("Swerve: LF Desired Angle - Deg", m_frontLeft.m_desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve: RF Desired Angle - Deg", m_frontRight.m_desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve: LR Desired Angle - Deg", m_rearLeft.m_desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve: RR Desired Angle - Deg", m_rearRight.m_desiredState.angle.getDegrees());

        /* Swerve Drive Module Optimized Positions 
        SmartDashboard.putNumber("Swerve: LF Opt. Angle - Deg", m_frontLeft.m_outputOptimizedState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve: RF Opt. Angle - Deg", m_frontRight.m_outputOptimizedState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve: LR Opt. Angle - Deg", m_rearLeft.m_outputOptimizedState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve: RR Opt. Angle - Deg", m_rearRight.m_outputOptimizedState.angle.getDegrees());

        /* command counters for debuging 
        SmartDashboard.putNumber("Count: Set X Count", setXcount);
        SmartDashboard.putNumber("Count: Reset Gyro", resetGyroCount);
        SmartDashboard.putNumber("Count: Field Relative", fieldRelativeCount);

        */

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Adjust input based on max speed
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    setXcount ++;
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    resetGyroCount ++;
  }

  /** Zeroes the heading of the robot. */
  public void toggleFieldRelative() {
    DriveConstants.driveFieldRelative = !DriveConstants.driveFieldRelative;
    fieldRelativeCount ++;
  }



  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
