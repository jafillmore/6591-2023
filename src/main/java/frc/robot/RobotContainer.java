// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private static final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Calibrate the Gyro
    m_robotDrive.m_gyro.calibrate();
    
    
    // Configure the button bindings
    configureButtonBindings();

   

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.06),
                true),
            m_robotDrive));

             
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

    public static void updateDash() {
    
        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
        
        SmartDashboard.putBoolean(  "IMU_Connected",        m_robotDrive.m_gyro.isConnected());
        SmartDashboard.putNumber(   "IMU_TotalYaw",         m_robotDrive.m_gyro.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       m_robotDrive.m_gyro.getRate());
        
        
        
            // /* Display 6-axis Processed Angle Data                                      */

            // SmartDashboard.putBoolean(  "IMU_IsCalibrating",    m_robotDrive.m_gyro.isCalibrating());
            // SmartDashboard.putNumber(   "IMU_Yaw",              m_robotDrive.m_gyro.getYaw());
            // SmartDashboard.putNumber(   "IMU_Pitch",            m_robotDrive.m_gyro.getPitch());
            // SmartDashboard.putNumber(   "IMU_Roll",             m_robotDrive.m_gyro.getRoll());
            
            // /* Display tilt-corrected, Magnetometer-based heading (requires             */
            // /* magnetometer calibration to be useful)                                   */
            
            // SmartDashboard.putNumber(   "IMU_CompassHeading",   m_robotDrive.m_gyro.getCompassHeading());
            
            // /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
            // SmartDashboard.putNumber(   "IMU_FusedHeading",     m_robotDrive.m_gyro.getFusedHeading());

            // /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
            
            // SmartDashboard.putNumber(   "IMU_Accel_X",          m_robotDrive.m_gyro.getWorldLinearAccelX());
            // SmartDashboard.putNumber(   "IMU_Accel_Y",          m_robotDrive.m_gyro.getWorldLinearAccelY());
            // SmartDashboard.putBoolean(  "IMU_IsMoving",         m_robotDrive.m_gyro.isMoving());
            // SmartDashboard.putBoolean(  "IMU_IsRotating",       m_robotDrive.m_gyro.isRotating());

            // /* Display estimates of velocity/displacement.  Note that these values are  */
            // /* not expected to be accurate enough for estimating robot position on a    */
            // /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
            // /* of these errors due to single (velocity) integration and especially      */
            // /* double (displacement) integration.                                       */
            
            // SmartDashboard.putNumber(   "Velocity_X",           m_robotDrive.m_gyro.getVelocityX());
            // SmartDashboard.putNumber(   "Velocity_Y",           m_robotDrive.m_gyro.getVelocityY());
            // SmartDashboard.putNumber(   "Displacement_X",       m_robotDrive.m_gyro.getDisplacementX());
            // SmartDashboard.putNumber(   "Displacement_Y",       m_robotDrive.m_gyro.getDisplacementY());
            
            // /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
            // /* NOTE:  These values are not normally necessary, but are made available   */
            // /* for advanced users.  Before using this data, please consider whether     */
            // /* the processed data (see above) will suit your needs.                     */
            
            // SmartDashboard.putNumber(   "RawGyro_X",            m_robotDrive.m_gyro.getRawGyroX());
            // SmartDashboard.putNumber(   "RawGyro_Y",            m_robotDrive.m_gyro.getRawGyroY());
            // SmartDashboard.putNumber(   "RawGyro_Z",            m_robotDrive.m_gyro.getRawGyroZ());
            // SmartDashboard.putNumber(   "RawAccel_X",           m_robotDrive.m_gyro.getRawAccelX());
            // SmartDashboard.putNumber(   "RawAccel_Y",           m_robotDrive.m_gyro.getRawAccelY());
            // SmartDashboard.putNumber(   "RawAccel_Z",           m_robotDrive.m_gyro.getRawAccelZ());
            // SmartDashboard.putNumber(   "RawMag_X",             m_robotDrive.m_gyro.getRawMagX());
            // SmartDashboard.putNumber(   "RawMag_Y",             m_robotDrive.m_gyro.getRawMagY());
            // SmartDashboard.putNumber(   "RawMag_Z",             m_robotDrive.m_gyro.getRawMagZ());
            // SmartDashboard.putNumber(   "IMU_Temp_C",           m_robotDrive.m_gyro.getTempC());
            
            // /* Omnimount Yaw Axis Information                                           */
            // /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
            // AHRS.BoardYawAxis yaw_axis = m_robotDrive.m_gyro.getBoardYawAxis();
            // SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
            // SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
            
            // /* Sensor Board Information                                                 */
            // SmartDashboard.putString(   "FirmwareVersion",      m_robotDrive.m_gyro.getFirmwareVersion());
            
            // /* Quaternion Data                                                          */
            // /* Quaternions are fascinating, and are the most compact representation of  */
            // /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
            // /* from the Quaternions.  If interested in motion processing, knowledge of  */
            // /* Quaternions is highly recommended.                                       */
            // SmartDashboard.putNumber(   "QuaternionW",          m_robotDrive.m_gyro.getQuaternionW());
            // SmartDashboard.putNumber(   "QuaternionX",          m_robotDrive.m_gyro.getQuaternionX());
            // SmartDashboard.putNumber(   "QuaternionY",          m_robotDrive.m_gyro.getQuaternionY());
            // SmartDashboard.putNumber(   "QuaternionZ",          m_robotDrive.m_gyro.getQuaternionZ());
            
            // /* Connectivity Debugging Support                                           */
            // SmartDashboard.putNumber(   "IMU_Byte_Count",       m_robotDrive.m_gyro.getByteCount());
            // SmartDashboard.putNumber(   "IMU_Update_Count",     m_robotDrive.m_gyro.getUpdateCount());

       


        /* Swerve Drive Module Positions */
        SmartDashboard.putNumber("Front Left Turn Angle", m_robotDrive.m_frontLeft.m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Front Right Turn Angle", m_robotDrive.m_frontRight.m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Rear Left Turn Angle", m_robotDrive.m_rearLeft.m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Rear Right Turn Angle", m_robotDrive.m_rearRight.m_turningEncoder.getPosition());
    }





}
