// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LiftSubsystem m_lift = new LiftSubsystem();

  // The driver's controller
  Joystick m_leftJoystick = new Joystick(OIConstants.kLeftControllerPort);
  Joystick m_rightJoystick = new Joystick(OIConstants.kRightControllerPort);
  Joystick m_buttonBoard = new Joystick(OIConstants.kButtonBoardPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands.  */
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
                MathUtil.applyDeadband(-(m_leftJoystick.getY()*Math.abs(m_leftJoystick.getY())), 0.03),
                MathUtil.applyDeadband(m_leftJoystick.getX()*Math.abs(m_leftJoystick.getX()), 0.03),
                MathUtil.applyDeadband(m_rightJoystick.getZ()*Math.abs(m_rightJoystick.getZ()), 0.03),
              DriveConstants.driveFieldRelative),
            m_robotDrive));
             
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(m_leftJoystick, OIConstants.kSetXButton)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_rightJoystick, OIConstants.kGyroRestButton)
        .debounce(0.05)   
        .onTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // Intake
    new JoystickButton(m_rightJoystick, OIConstants.kButtonIntake)
    .debounce(0.1)
    .onTrue(new RunCommand( () -> m_lift.setIntakeForward()))
    .onFalse (new RunCommand( () -> m_lift.setIntakeOff()));

    // Eject
    new JoystickButton(m_rightJoystick, OIConstants.kButtonEject)
    .debounce(0.1)
    .onTrue(new RunCommand( () -> m_lift.setIntakeReverse()))
    .onFalse (new RunCommand( () -> m_lift.setIntakeOff()));
    
    //Position Lift for Upper Posts
    new JoystickButton(m_buttonBoard, OIConstants.kButtonTopCone)
        .debounce(0.1)
        .whileTrue(new RunCommand(
            () -> m_lift.setPosition(PositionConstants.kTopCone[1], PositionConstants.kTopCone[2], PositionConstants.kTopCone[3]),
            m_lift));

    //Position Lift for Middle Posts
    new JoystickButton(m_buttonBoard, OIConstants.kButtonMiddleCone)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kMiddleCone[1], PositionConstants.kMiddleCone[2], PositionConstants.kMiddleCone[3]),
        m_lift));

    //Position Lift for Top Cube
    new JoystickButton(m_buttonBoard, OIConstants.kButtonTopCube)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kTopCube[1], PositionConstants.kTopCube[2], PositionConstants.kTopCube[3]),
        m_lift));

    //Position Lift for Middle Cube
    new JoystickButton(m_buttonBoard, OIConstants.kButtonMiddleCube)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kMiddleCube[1], PositionConstants.kMiddleCube[2], PositionConstants.kMiddleCube[3]),
        m_lift));

    //Position Lift for Hybrid Node (floor)
    new JoystickButton(m_buttonBoard, OIConstants.kButtonHybrid)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kHybrid[1], PositionConstants.kHybrid[2], PositionConstants.kHybrid[3]),
        m_lift));

    //Position Lift for Portal Ramp Pickup
    new JoystickButton(m_buttonBoard, OIConstants.kButtonRamp)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kPortalRamp[1], PositionConstants.kPortalRamp[2], PositionConstants.kPortalRamp[3]),
        m_lift));

    //Position Lift for Portal Shelf Pickup
    new JoystickButton(m_buttonBoard, OIConstants.kButtonShelf)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kPortalShelf[1], PositionConstants.kPortalShelf[2], PositionConstants.kPortalShelf[3]),
        m_lift));

    //Position Lift for Floor Pickup
    new JoystickButton(m_buttonBoard, OIConstants.kButtonFloor)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kFloor[1], PositionConstants.kFloor[2], PositionConstants.kFloor[3]),
        m_lift));

    //Stow Lift
    new JoystickButton(m_buttonBoard, OIConstants.kButtonStow)
    .debounce(0.1)
    .whileTrue(new RunCommand(
        () -> m_lift.setPosition(PositionConstants.kStow[1], PositionConstants.kStow[2], PositionConstants.kStow[3]),
        m_lift));



    //Manual Override for Elevator Up
    new JoystickButton(m_buttonBoard, OIConstants.kButtonElevatorUp)
    .debounce(0.1)
    .onTrue(new RunCommand(
      () -> m_lift.setPosition(LiftSubsystem.m_elevatorTargetHeight+1 , LiftSubsystem.m_armTargetAngle, LiftSubsystem.m_wristTargetAngle)));

    //Manual Override for Elevator Down
    new JoystickButton(m_buttonBoard, OIConstants.kButtonElevatorDown)
    .debounce(0.1)
    .onTrue(new RunCommand(
      () -> m_lift.setPosition(LiftSubsystem.m_elevatorTargetHeight-1, LiftSubsystem.m_armTargetAngle, LiftSubsystem.m_wristTargetAngle)
      ));

    //Manual Override for Arm Up
    new JoystickButton(m_buttonBoard, OIConstants.kButtonArmUp)
    .debounce(0.1)
    .onTrue(new RunCommand(
      () -> m_lift.setPosition(LiftSubsystem.m_elevatorTargetHeight, LiftSubsystem.m_armTargetAngle+1, LiftSubsystem.m_wristTargetAngle)
    ));

    //Manual Override for Arm Down
    new JoystickButton(m_buttonBoard, OIConstants.kButtonArmDown)
    .debounce(0.1)
    .onTrue(new RunCommand(
      () -> m_lift.setPosition(LiftSubsystem.m_elevatorTargetHeight, LiftSubsystem.m_armTargetAngle-1, LiftSubsystem.m_wristTargetAngle)
    ));

    //Manual Override Wrist Up
    new JoystickButton(m_buttonBoard, OIConstants.kButtonWristUp)
    .debounce(0.1)
    .onTrue(new RunCommand(
      () -> m_lift.setPosition(LiftSubsystem.m_elevatorTargetHeight, LiftSubsystem.m_armTargetAngle, LiftSubsystem.m_wristTargetAngle+1)
    ));

    //Manual Override Wrist Down
    new JoystickButton(m_buttonBoard, OIConstants.kButtonWristDown)
    .debounce(0.1)
    .onTrue(new RunCommand(
      () -> m_lift.setPosition(LiftSubsystem.m_elevatorTargetHeight, LiftSubsystem.m_armTargetAngle, LiftSubsystem.m_wristTargetAngle-1)
    ));
    


  }

  // public static SendableChooser<Command> mChooser = new SendableChooser<>();

  //Add commands to the autonomous command chooser

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Zero the gyro
    m_robotDrive.zeroHeading();

    // Zero the arm encoders
    m_lift.resetArmEncoders();
    
    
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
        List.of(new Translation2d(1, 0), new Translation2d(3,
         0)),
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
}
