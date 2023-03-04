// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
  
  /** Creates a new LiftSubsystem. */


  private final CANSparkMax m_elevatorSparkMax = new CANSparkMax(LiftConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private final CANSparkMax m_armSparkMax = new CANSparkMax(LiftConstants.kArmMotorCanId, MotorType.kBrushless);
  private final CANSparkMax m_wristSparkMax = new CANSparkMax(LiftConstants.kWristMotorCanId, MotorType.kBrushless);
  private final CANSparkMax m_leftIntakeSpark = new CANSparkMax(LiftConstants.kLeftIntakeMotorCanId, MotorType.kBrushed);
  private final CANSparkMax m_rightIntakeSparkMax = new CANSparkMax(LiftConstants.kRightIntakeMotorCanId, MotorType.kBrushed);
  // add intake motor group

  private final RelativeEncoder m_elevatorEncoder = m_elevatorSparkMax.getEncoder();
  private final RelativeEncoder m_armEncoder = m_armSparkMax.getEncoder();
  private final RelativeEncoder m_wristEncoder = m_wristSparkMax.getEncoder();
  
  private final SparkMaxPIDController m_elevatorPIDController = m_elevatorSparkMax.getPIDController();
  private final SparkMaxPIDController m_armPIDController = m_armSparkMax.getPIDController();
  private final SparkMaxPIDController m_wristPIDController = m_wristSparkMax.getPIDController();
  

  public static double m_elevatorTargetHeight = 0;
  public static double m_armTargetAngle = 0;
  public static double m_wristTargetAngle = 0;

 
  
  public LiftSubsystem() {
  
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_elevatorSparkMax.restoreFactoryDefaults();
    m_armSparkMax.restoreFactoryDefaults();
    m_wristSparkMax.restoreFactoryDefaults();
    m_leftIntakeSpark.restoreFactoryDefaults();
    m_rightIntakeSparkMax.restoreFactoryDefaults();

    m_elevatorSparkMax.setInverted(LiftConstants.kElevatorInversion);
    m_armSparkMax.setInverted(LiftConstants.kArmInversion);
    m_wristSparkMax.setInverted(LiftConstants.kWristInversion);
    m_leftIntakeSpark.setInverted(LiftConstants.kLeftIntakeInversion);
    m_rightIntakeSparkMax.setInverted(LiftConstants.kRightIntiakeInversion);

    m_elevatorSparkMax.setIdleMode(LiftConstants.kElevatorNeutralMode);
    m_armSparkMax.setIdleMode(LiftConstants.kArmIdleMode);
    m_wristSparkMax.setIdleMode(LiftConstants.kwristIdleMode);
    m_leftIntakeSpark.setIdleMode(LiftConstants.kLeftIntakeIdleMode);
    m_rightIntakeSparkMax.setIdleMode(LiftConstants.kRightIntakeIdleMode);

    
    m_elevatorPIDController.setFeedbackDevice(m_elevatorEncoder);
    m_armPIDController.setFeedbackDevice(m_armEncoder);
    m_wristPIDController.setFeedbackDevice(m_wristEncoder);
  
    m_armEncoder.setPositionConversionFactor(LiftConstants.kelevatorEncoderPositionFactor);
    m_armEncoder.setPositionConversionFactor(LiftConstants.kArmEncoderPositionFactor);
    m_armEncoder.setVelocityConversionFactor(LiftConstants.kArmEncoderVelocityFactor);
  
    m_elevatorEncoder.setVelocityConversionFactor(LiftConstants.kelevatorEncoderVelocityFactor);
    m_wristEncoder.setPositionConversionFactor(LiftConstants.kWristEncoderPositionFactor);
    m_wristEncoder.setVelocityConversionFactor(LiftConstants.kWristEncoderVelocityFactor);
    


    // Set the PID gains for the elevator motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_elevatorPIDController.setP(LiftConstants.kElevatorP);
    m_elevatorPIDController.setI(LiftConstants.kElevatorI);
    m_elevatorPIDController.setD(LiftConstants.kElevatorD);
    m_elevatorPIDController.setFF(LiftConstants.kElevatorFF);
    m_elevatorPIDController.setOutputRange(LiftConstants.kElevatorMinOutput,
     LiftConstants.kElevatorMaxOutput);
    
    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_armPIDController.setP(LiftConstants.kArmP);
    m_armPIDController.setI(LiftConstants.kArmI);
    m_armPIDController.setD(LiftConstants.kArmD);
    m_armPIDController.setFF(LiftConstants.kArmFF);
    m_armPIDController.setOutputRange(LiftConstants.kArmMinOutput,
      LiftConstants.kArmMaxOutput);
  
    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_wristPIDController.setP(LiftConstants.kWristP);
    m_wristPIDController.setI(LiftConstants.kWristI);
    m_wristPIDController.setD(LiftConstants.kWristD);
    m_wristPIDController.setFF(LiftConstants.kWristFF);
    m_wristPIDController.setOutputRange(LiftConstants.kWristMinOutput,
      LiftConstants.kWristMaxOutput);
    
    

  
    m_elevatorSparkMax.burnFlash();
    m_armSparkMax.burnFlash();
    m_wristSparkMax.burnFlash();
    m_leftIntakeSpark.burnFlash();
    m_rightIntakeSparkMax.burnFlash();
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Lift Actual Positions */
   
    SmartDashboard.putNumber("Elevator Height - Actual", m_elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Arm Angle - Actual", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Angle - Actual", m_wristEncoder.getPosition());

    SmartDashboard.putNumber("Elevator Height - Target", m_elevatorTargetHeight);
    SmartDashboard.putNumber("Arm Angle - Target", m_armTargetAngle);
    SmartDashboard.putNumber("Wrist Angle - Target", m_wristTargetAngle);
  
     
  }

 
  //  Method to set the lift position
  public void setPosition(double elevatorTargetHeight, double armTargetAngle, double wristTargetAngle) {
    m_elevatorTargetHeight = elevatorTargetHeight;
    m_armTargetAngle = armTargetAngle;
    m_wristTargetAngle = wristTargetAngle;

    // m_elevatorSrx.set(ControlMode.Position, elevatorTargetHeight);
    m_armPIDController.setReference(armTargetAngle, CANSparkMax.ControlType.kPosition);  // Set the arm PID target to the desired angle
    m_wristPIDController.setReference(wristTargetAngle, CANSparkMax.ControlType.kPosition);  // Set the arm PID target to the desired angle
    

  }
 
  // STUFF THAT NEEDS TO BE ADDED (with corresponding buttons)

  // Method to turn off the intake motors
  public void setIntakeOff() {
    m_leftIntakeSpark.set(0);
    m_rightIntakeSparkMax.set(0);
  }

  // Method to turn on the intake motors in foward
  public void setIntakeForward() {
      m_leftIntakeSpark.set(LiftConstants.kIntakeMaxSpeed);
      m_rightIntakeSparkMax.set(LiftConstants.kIntakeMaxSpeed);
  }

  // Method to turn on the intake motors in Reverse
  public void setIntakeReverse() {
    m_leftIntakeSpark.set(-1*(LiftConstants.kIntakeMaxSpeed));
    m_rightIntakeSparkMax.set(-1*(LiftConstants.kIntakeMaxSpeed));
  }



}
