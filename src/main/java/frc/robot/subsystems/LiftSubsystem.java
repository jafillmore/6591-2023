// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
  
  /** Creates a new LiftSubsystem. */


  private final TalonSRX m_elevatorSrx = new TalonSRX(LiftConstants.kElevatorMotorCanId);

  private final CANSparkMax m_armSparkMax = new CANSparkMax(LiftConstants.kArmMotorCanId, MotorType.kBrushless);
  private final CANSparkMax m_wristSparkMax = new CANSparkMax(LiftConstants.kWristMotorCanId, MotorType.kBrushed);
  private final CANSparkMax m_leftIntakeSpark = new CANSparkMax(LiftConstants.kLeftIntakeMotorCanId, MotorType.kBrushed);
  private final CANSparkMax m_rightIntakeSparkMax = new CANSparkMax(LiftConstants.kRightIntakeMotorCanId, MotorType.kBrushed);

  private final RelativeEncoder m_armEncoder = m_armSparkMax.getEncoder();
  private final RelativeEncoder m_wristEncoder = m_wristSparkMax.getEncoder();

  private final SparkMaxPIDController m_armPIDController = m_armSparkMax.getPIDController();
  private final SparkMaxPIDController m_wristPIDController = m_wristSparkMax.getPIDController();

  double m_elevatorTargetHeight = 0;
  double m_armTargetAngle = 0;
  double m_wristTargetAngle = 0;

  
  
  public LiftSubsystem() {

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_armSparkMax.restoreFactoryDefaults();
    m_wristSparkMax.restoreFactoryDefaults();
    m_leftIntakeSpark.restoreFactoryDefaults();
    m_rightIntakeSparkMax.restoreFactoryDefaults();
  
    m_armPIDController.setFeedbackDevice(m_armEncoder);
    m_wristPIDController.setFeedbackDevice(m_wristEncoder);
  
    m_armEncoder.setPositionConversionFactor(LiftConstants.kArmEncoderPositionFactor);
    m_armEncoder.setVelocityConversionFactor(LiftConstants.kArmEncoderVelocityFactor);
  
    m_wristEncoder.setPositionConversionFactor(LiftConstants.kWristEncoderPositionFactor);
    m_wristEncoder.setVelocityConversionFactor(LiftConstants.kWristEncoderVelocityFactor);
    
    m_armPIDController.setPositionPIDWrappingEnabled(false);
    m_wristPIDController.setPositionPIDWrappingEnabled(false);
  
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
  
  
  
    m_armSparkMax.burnFlash();
    m_wristSparkMax.burnFlash();
    m_leftIntakeSpark.burnFlash();
    m_rightIntakeSparkMax.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Lift Actual Positions */
    
    SmartDashboard.putNumber("Elevator Height - Actual", m_elevatorSrx.getSelectedSensorPosition());
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

    m_elevatorSrx.set(ControlMode.Position, elevatorTargetHeight);
    m_armPIDController.setReference(armTargetAngle, CANSparkMax.ControlType.kPosition);  // Set the arm PID target to the desired angle
    m_wristPIDController.setReference(wristTargetAngle, CANSparkMax.ControlType.kPosition);  // Set the arm PID target to the desired angle
    

  }
 
  // STUFF THAT NEEDS TO BE ADDED

  // Method to turn off the intake motors

  // Method to turn on the intake motors in foward

  // Method to turn on the intake motors in Reverse



  



}
