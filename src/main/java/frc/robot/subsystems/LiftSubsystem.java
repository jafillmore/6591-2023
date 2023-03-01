// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {
  
  /** Creates a new LiftSubsystem. */


  //private final TalonSRX m_elevatorSrx = new TalonSRX(LiftConstants.kElevatorMotorCanId);

  //private final RelativeEncoder m_armEncoder;
  //private final RelativeEncoder m_wristEncoder;
  
  //private final CANSparkMax m_armSparkMax = new CANSparkMax(LiftConstants.kArmMotorCanId, MotorType.kBrushless);
  //private final CANSparkMax m_wristSparkMax = new CANSparkMax(LiftConstants.kWristMotorCanId, MotorType.kBrushed);
  //private final CANSparkMax m_leftIntakeSpark = new CANSparkMax(LiftConstants.kLeftIntakeMotorCanId, MotorType.kBrushed);
  //private final CANSparkMax m_rightIntakeSparkMax = new CANSparkMax(LiftConstants.kRightIntakeMotorCanId, MotorType.kBrushed);

  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  // m_armSparkMax.restoreFactoryDefaults();
  // m_armSparkMax.restoreFactoryDefaults();

  //private final SparkMaxPIDController m_armPIDController;
  //private final SparkMaxPIDController m_wristPIDController;


  

  m_armEncoder = m_armSparkMax.getEncoder();
  m_wristEncoder = m_wristSparkMax.getEncoder();

  m_armPIDController = m_armSparkMax.getPIDController();
  m_wristPIDController = m_wristSparkMax.getPIDController();

  m_armPIDController.setFeedbackDevice(m_armEncoder);
  m_wristPIDController.setFeedbackDevice(m_wristEncoder);

  m_armEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
  m_armEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

  m_wristEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
  m_wristEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
  
  m_armPIDController.setPositionPIDWrappingEnabled(false);
  m_wristPIDController.setPositionPIDWrappingEnabled(false);

  // Set the PID gains for the driving motor. Note these are example gains, and you
  // may need to tune them for your own robot!
  m_armPIDController.setP(ModuleConstants.kDrivingP);
  m_armPIDController.setI(ModuleConstants.kDrivingI);
  m_armPIDController.setD(ModuleConstants.kDrivingD);
  m_armPIDController.setFF(ModuleConstants.kDrivingFF);
  m_armPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
    ModuleConstants.kDrivingMaxOutput);

  // Set the PID gains for the turning motor. Note these are example gains, and you
  // may need to tune them for your own robot!
  m_wristPIDController.setP(ModuleConstants.kTurningP);
  m_wristPIDController.setI(ModuleConstants.kTurningI);
  m_wristPIDController.setD(ModuleConstants.kTurningD);
  m_wristPIDController.setFF(ModuleConstants.kTurningFF);
  m_wristPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
    ModuleConstants.kTurningMaxOutput);



  m_armSparkMax.burnFlash();
  m_wristSparkMax.burnFlash();


  
  public LiftSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Lift Actual Positions */
    SmartDashboard.putNumber("Arm Angle", Math.toDegrees(m_elevatorSrx.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Arm Angle", Math.toDegrees(m_armEncoder.getPosition()));
    SmartDashboard.putNumber("Wrist Angle", Math.toDegrees(m_wristEncoder.getPosition()));

     
  }

  public void setPosition(double elevatorHeight, double armAngle, double wristAngle) {

  }



}
