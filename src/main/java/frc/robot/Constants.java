// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAngularSpeed = 4.0; // Was 2 pi radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians - Set to zero as encoders are zeroed facing forward
    public static final double kFrontLeftChassisAngularOffset = 0; // -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0; // Math.PI;
    public static final double kBackRightChassisAngularOffset = 0; // Math.PI / 2;

    // Motor Inversion Booleans to account for wheel orientation vs module orientation
    public static final boolean kFrontLeftDriveInversion = true;
    public static final boolean kFrontRightDriveInversion = false;
    public static final boolean kBackLeftDriveInversion = true;
    public static final boolean kBackRightDriveInversion = false;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = false;

    // Toggle for field relative driving
    public static boolean driveFieldRelative = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 40 teeth on the wheel's bevel gear, 40 teeth on the first-stage spur gear, 20 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (40.0 * 40) / (kDrivingMotorPinionTeeth * 20);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderVoltsPerTurn = 3.3; // Lamprey encoder output is 3.3V for one turn   
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kTurningEncoderVoltsPerTurn; // radians
    public static final double kTurningEncoderVelocityFactor = ((2 * Math.PI) / kTurningEncoderVoltsPerTurn) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians

    public static final double kDrivingP = 0.3;  // Example was 0.4
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 1;   // Example was 0.0
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.9; // Example was 1.0
    public static final double kTurningI = 0;
    public static final double kTurningD = 20.0 ;  // Example was 0
    public static final double kTurningFF = 0.0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class LiftConstants {
    public static final int kElevatorMotorCanId = 9 ;
    public static final int kArmMotorCanId = 10 ;
    public static final int kWristMotorCanId = 11 ;
    public static final int kLeftIntakeMotorCanId = 12 ;
    public static final int kRightIntakeMotorCanId = 13 ;

    public static final double kelevatorEncoderPositionFactor = 1 ;
    public static final double kelevatorEncoderVelocityFactor = (kelevatorEncoderPositionFactor/60.0) ;

    public static final double kArmEncoderPositionFactor = 5.3 ;
    public static final double kArmEncoderVelocityFactor = (kArmEncoderPositionFactor/60.0) ;

    public static final double kWristEncoderPositionFactor = 2.88 ;  // 360deg / 125:1 reduction ratio
    public static final double kWristEncoderVelocityFactor = (kWristEncoderPositionFactor/60.0) ;

    public static final double kArmP = 0.25 ;  // Example was 0.4
    public static final double kArmI = 0 ;
    public static final double kArmD = 5 ;   // Example was 0.0
    public static final double kArmFF = 0.05 ;
    public static final double kArmMinOutput = -0.05;
    public static final double kArmMaxOutput = 0.2;

    public static final double kWristP = 1.0; // Example was 1.0
    public static final double kWristI = 0;
    public static final double kWristD = 10.0 ;  // Example was 0
    public static final double kWristFF = 0.0;
    public static final double kWristMinOutput = -0.75;
    public static final double kWristMaxOutput = 0.5;

    public static final double kElevatorP = 1;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 1;
    public static final double kElevatorFF = 0.0;
    public static final double kElevatorMinOutput = -1;
    public static final double kElevatorMaxOutput = 1;

    public static final double kIntakeMaxSpeed = 1;

    
    public static final boolean kElevatorInversion = false;
    public static final boolean kArmInversion = false;
    public static final boolean kWristInversion = true;
    public static final boolean kLeftIntakeInversion = true;
    public static final boolean kRightIntiakeInversion = false;

    
    public static final IdleMode kElevatorNeutralMode = IdleMode.kBrake;
    public static final IdleMode kArmIdleMode = IdleMode.kBrake;
    public static final IdleMode kwristIdleMode = IdleMode.kBrake;
    public static final IdleMode kLeftIntakeIdleMode = IdleMode.kBrake;
    public static final IdleMode kRightIntakeIdleMode = IdleMode.kBrake;
  
  }


  public static final class PositionConstants {

    // Delivery Positions
    public static final double kTopCone[] = new double[] { 70.0, 97.0, 130}; // Elevator Height, Arm Angle, Wrist Angle
    public static final double kMiddleCone[] = new double[] {0, 90.0, 90.0}; // Elevator Height, Arm Angle, Wrist Angle
    public static final double kTopCube[] = new double[] { 70.0, 95.0, 130.0}; // Elevator Height, Arm Angle, Wrist Angle
    public static final double kMiddleCube[] = new double[] {0.0, 90.0, 100.0}; // Elevator Height, Arm Angle, Wrist Angle
    public static final double kHybrid[] = new double[] {6.0, 70.0, 205.0}; // Elevator Height, Arm Angle, Wrist Angle
 
    // Pickup Positions
    public static final double kPortalRamp[] = new double[] {70.0, 93, 150.0}; // Elevator Height, Arm Angle, Wrist Angle
    public static final double kPortalShelf[] = new double[] {60.0, 95.0, 205.0}; // Elevator Height, Arm Angle, Wrist Angle
    public static final double kFloor[] = new double[] {0.0, 10.0, 30.0}; // Elevator Height, Arm Angle, Wrist Angle
    


    // Stow Position
    public static final double kStow[] = new double[] {0, 0, 0}; // Elevator Height, Arm Angle, Wrist Angle


    
    



  }



  public static final class OIConstants {
    
    // Left Controller Port and Buttons
    public static final int kLeftControllerPort = 0;

    public static final int kSetXButton = 4; // Need to decide which stick and button we should use...
    public static final int kGyroRestButton = 5;  // Need to decide which stick and button we should use...
   


    // Right Controller Port and Buttons
    public static final int kRightControllerPort = 1;

    public static final int kButtonIntake = 1 ; // Need to decide which stick and button we should use...
    public static final int kButtonEject = 4 ; // Need to decide which stick and button we should use...
    

    
    
    // public static final int kFieldRelativeButton = 3;  // Not currently Used
    
    

    // Button Board Port and Buttons
    public static final int kButtonBoardPort = 2 ;

    // Placement Buttons
    public static final int kButtonTopCone = 1 ;
    public static final int kButtonMiddleCone = 2 ;
    public static final int kButtonTopCube = 3 ;
    public static final int kButtonMiddleCube = 4 ;
    public static final int kButtonHybrid = 5 ;

    // Pick-up Buttons
    public static final int kButtonRamp = 6 ;
    public static final int kButtonShelf = 7 ;
    public static final int kButtonFloor = 8 ;
    public static final int kButtonStow = 9 ;

    // Manual Lift Control Overrides
    public static final int kButtonElevatorUp = 11;
    public static final int kButtonArmUp = 13;
    public static final int kButtonWristUp = 15 ;

    // Manual Lift Control Overrides
    public static final int kButtonElevatorDown = 12;
    public static final int kButtonArmDown = 14;
    public static final int kButtonWristDown = 16 ;

    // Manual Lift Encoder Reset
    public static final int kButtonEncoderReset = 24; 









  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5; // example was 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; // example default was 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
