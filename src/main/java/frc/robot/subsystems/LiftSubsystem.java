package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ModuleConstants;

public class LiftSubsystem {
    private final CANSparkMax m_liftingSparkMax;
    private final CANSparkMax m_extendingSparkMax;

    private final RelativeEncoder m_liftingEncoder;
    private final RelativeEncoder m_extendingEncoder;

    private final SparkMaxPIDController m_liftingPIDController;
    private final SparkMaxPIDController m_extendingPIDController;

    public LiftSubsystem(int liftingCANId, int extendingCANId) {
        m_liftingSparkMax = new CANSparkMax(liftingCANId, MotorType.kBrushless);
        m_extendingSparkMax = new CANSparkMax(extendingCANId, MotorType.kBrushless);


        m_liftingSparkMax.restoreFactoryDefaults();
        m_extendingSparkMax.restoreFactoryDefaults();


        m_liftingEncoder = m_liftingSparkMax.getEncoder();
        m_extendingEncoder = m_extendingSparkMax.getEncoder();
        m_liftingPIDController = m_liftingSparkMax.getPIDController();
        m_extendingPIDController = m_extendingSparkMax.getPIDController();
        m_liftingPIDController.setFeedbackDevice(m_liftingEncoder);
        m_extendingPIDController.setFeedbackDevice(m_extendingEncoder);

        m_liftingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_liftingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        m_extendingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_extendingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        
        m_liftingPIDController.setPositionPIDWrappingEnabled(true);
        m_liftingPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_liftingPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_liftingPIDController.setP(ModuleConstants.kDrivingP);
    m_liftingPIDController.setI(ModuleConstants.kDrivingI);
    m_liftingPIDController.setD(ModuleConstants.kDrivingD);
    m_liftingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_liftingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_liftingPIDController.setP(ModuleConstants.kTurningP);
    m_liftingPIDController.setI(ModuleConstants.kTurningI);
    m_liftingPIDController.setD(ModuleConstants.kTurningD);
    m_liftingPIDController.setFF(ModuleConstants.kTurningFF);
    m_liftingPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

        // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_extendingPIDController.setP(ModuleConstants.kDrivingP);
    m_extendingPIDController.setI(ModuleConstants.kDrivingI);
    m_extendingPIDController.setD(ModuleConstants.kDrivingD);
    m_extendingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_extendingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_extendingPIDController.setP(ModuleConstants.kTurningP);
    m_extendingPIDController.setI(ModuleConstants.kTurningI);
    m_extendingPIDController.setD(ModuleConstants.kTurningD);
    m_extendingPIDController.setFF(ModuleConstants.kTurningFF);
    m_extendingPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);




        m_liftingSparkMax.burnFlash();
        m_extendingSparkMax.burnFlash();
    }

    public void setPosition(double armHeight, double armAngle, double wristAngle) {

    }
    
}
