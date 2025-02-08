// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.SparkPIDController.AccelStrategy;
// import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    private SparkMax m_topMotor;
    private SparkMax m_lowMotor;

    // private SparkPIDController m_topShooterClosedLoopConfig;
    // private SparkPIDController m_lowShooterClosedLoopConfig;
    private SparkClosedLoopController m_topShooterClosedLoopController;
    private SparkClosedLoopController m_lowShooterClosedLoopController;
    private SparkLimitSwitch m_gatewayLimitSwitch;
    private RelativeEncoder m_topEncoder;
    private RelativeEncoder m_lowEncoder;
    private SparkMaxConfig m_topShooterConfig;
    private SparkMaxConfig m_lowShooterConfig;
    private MAXMotionConfig m_topMAXMotionConfig;
    private MAXMotionConfig m_lowMAXMotionConfig;
    double p, i, d, velocity, velocity2, cP, cI, cD, cVelocity, cVelocity2, m_dial1, m_dial2, maxRPMTop, maxRPMLow,
            initialTimeTop, initialTimeLow, riseTimeTop, riseTimeLow, startOscillationTimeTop, startOscillationTimeLow,
            finalOscillationTimeTop, finalOscillationTimeLow;


    public ShooterSubsystem() {
        m_topMotor = new SparkMax(ShooterConstants.kShooterMotorTopCanID, MotorType.kBrushless);
        m_lowMotor = new SparkMax(ShooterConstants.kShooterMotorLowCanID, MotorType.kBrushless);
        m_topShooterConfig = new SparkMaxConfig();
        m_lowShooterConfig = new SparkMaxConfig();

        m_topMAXMotionConfig = new MAXMotionConfig();
        m_lowMAXMotionConfig = new MAXMotionConfig();

        //Change these inverted statements to the 2025 way to invert things (using the maxconfig method)
        // m_topMotor.setInverted(true);
        // m_lowMotor.setInverted(true);

        m_topShooterConfig.inverted(true);
        m_lowShooterConfig.inverted(true);

        
        m_lowShooterConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(true);
        // m_gatewayLimitSwitch = m_lowMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        // m_gatewayLimitSwitch.enableLimitSwitch(true);

        m_topEncoder = m_topMotor.getEncoder();
        m_lowEncoder = m_lowMotor.getEncoder();
        m_topShooterConfig.encoder.velocityConversionFactor(ShooterConstants.kShooterGearRatio);
        m_lowShooterConfig.encoder.velocityConversionFactor(ShooterConstants.kShooterGearRatio);
        // m_topEncoder.setVelocityConversionFactor(ShooterConstants.kShooterGearRatio);
        // m_lowEncoder.setVelocityConversionFactor(ShooterConstants.kShooterGearRatio);

        // m_topEncoder.setVelocityConversionFactor(1);
        // m_lowEncoder.setVelocityConversionFactor(1);
        m_lowShooterClosedLoopController = m_lowMotor.getClosedLoopController();
        m_topShooterClosedLoopController = m_topMotor.getClosedLoopController();
        // m_topShooterClosedLoopConfig = m_topMotor.getPIDController();
        // m_lowShooterClosedLoopConfig = m_lowMotor.getPIDController();
        m_topShooterConfig.idleMode(IdleMode.kCoast);
        m_lowShooterConfig.idleMode(IdleMode.kCoast);
        // m_topMotor.setIdleMode(IdleMode.kCoast);
        // m_lowMotor.setIdleMode(IdleMode.kCoast);

        cVelocity = 0;
        cVelocity2 = 0;

        // UNCOMMENT WHEN YOU RECORD THE ACTUAL MOTOR SPEED VALUES SO YOU CAN ACTUALLY
        // USE PID TO CONTROL RPM

        m_topShooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kPl0)
            .i(ShooterConstants.kIl0)
            .d(ShooterConstants.kDl0)
            .outputRange(-1, 1);
        m_lowShooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ShooterConstants.kPt0)
            .i(ShooterConstants.kIt0)
            .d(ShooterConstants.kDt0)
            .outputRange(-1, 1);
        //     pid(ShooterConstants.kPl0,ShooterConstants.kIl0,
        // ShooterConstants.kDl0);
        // m_lowShooterConfig.closedLoop.pid(ShooterConstants.kPt0,ShooterConstants.kIt0,
        // ShooterConstants.kDt0);
        setSmartMotion(m_lowShooterConfig.closedLoop.maxMotion, ShooterConstants.kShooterMaxVelocity, ShooterConstants.kShooterMaxAccel,
            ShooterConstants.kShooterAllowedErr);
        setSmartMotion(m_topShooterConfig.closedLoop.maxMotion, ShooterConstants.kShooterMaxVelocity, ShooterConstants.kShooterMaxAccel,
            ShooterConstants.kShooterAllowedErr);
        m_lowMotor.configure(m_lowShooterConfig, ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
        m_topMotor.configure(m_topShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

       // m_gatewayLimitSwitch = m_topMotor.getReverseLimitSwitch();

        // m_lowShooterClosedLoopController.
        // setSmartMotion(m_lowShooterClosedLoopConfig, ShooterConstants.kShooterMaxRPM, ShooterConstants.kShooterMinRPM,
        //         ShooterConstants.kShooterMaxAccel, ShooterConstants.kShooterAllowedErr, 0);
        // setSmartMotion(m_topShooterClosedLoopConfig, ShooterConstants.kShooterMaxRPM, ShooterConstants.kShooterMinRPM,
        //         ShooterConstants.kShooterMaxAccel, ShooterConstants.kShooterAllowedErr, 0);

        // setPID(m_lowShooterClosedLoopConfig, 100 / 917, 0, 0, 0);
        // setPID(m_topShooterClosedLoopConfig, 100 / 917, 0, 0, 0);

    }

    public boolean isIntaking() {
        if (m_lowEncoder.getVelocity() > ShooterConstants.kIntakeSpeed/2) {
            return true;
        } 
        return false;
    }

    public void setSmartMotion(MAXMotionConfig config, double maxVel, double maxAcc,
            double allowedErr) {
            config.maxAcceleration(maxAcc);
            config.maxVelocity(maxVel);
            config.allowedClosedLoopError(allowedErr);
            config.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        // controller.setSmartMotionMaxOutputAccel(maxVel, slot);
        // controller.setSmartMotionMinOutputVelocity(minVel, slot);
        // controller.setSmartMotionMaxAccel(maxAcc, slot);
        // controller.setSmartMotionAllowedClosedLoopError(allowedErr, slot);
        // controller.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, slot);
    }

    // public void setPID(ClosedLoopConfig controller, double p, double i, double d, int slot) {
    //     controller.p(p, slot);
    //     controller.i(i, slot);
    //     controller.d(d, slot);
    // }

    public boolean isSwitchPressed() {
        var isPressed = m_gatewayLimitSwitch.isPressed();

        return isPressed;
    }

    public void shootSmartVelocity(double velocity) {
        
        m_topShooterClosedLoopController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
        m_lowShooterClosedLoopController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
    }

    public void shootSmartVelocity(double velocitytop, double velocitylow) {
        m_topShooterClosedLoopController.setReference(velocitytop, ControlType.kMAXMotionVelocityControl);
        m_lowShooterClosedLoopController.setReference(velocitylow, ControlType.kMAXMotionVelocityControl);
    }

    public double voltageFF(double speed) {
        return speed / 917;
    }

    // public void initShoot() {
    // maxRPMTop = 0;
    // maxRPMLow = 0;
    // initialTimeTop = Timer.getFPGATimestamp();
    // initialTimeLow = Timer.getFPGATimestamp();
    // riseTimeTop = 0;
    // riseTimeLow = 0;
    // finalOscillationTimeTop = 0;
    // finalOscillationTimeLow = 0;
    // bOscillatingTop = false;
    // bOscillatingLow = false;
    // }

    public void shoot() {
        shootSmartVelocity(ShooterConstants.kShooterSpeedSpeakerTop, ShooterConstants.kShooterSpeedSpeakerLow);
    }

    public void shootAmp() {

        shootSmartVelocity(ShooterConstants.kShooterSpeedAmpTop, ShooterConstants.kShooterSpeedAmpLow);
    }

    // public void shootTop(double speed, double dial) {
    // m_dial1 = (dial + 1) / 2;
    // m_topMotor.set(speed * m_dial1);
    // }

    // public void shootLow(double speed, double dial) {
    // m_dial2 = (dial + 1) / 2;
    // m_lowMotor.set(speed * m_dial2);
    // }

    public double getTopMotorSpeed() {
        return m_topEncoder.getVelocity();
    }

    public double getLowMotorSpeed() {
        return m_lowEncoder.getVelocity();
    }

    public void intake() {
        
        shootSmartVelocity(ShooterConstants.kIntakeSpeed);
    }

    public void stopShoot() {
                // shootSmartVelocity(0);

        m_topMotor.set(0);
        m_lowMotor.set(0);
    }

    public boolean isRampedUp() {
        return m_topEncoder.getVelocity() > ShooterConstants.kShooterMaxVelocity;
    }

    public double lookupFF(double speed) {
        // Uncomment for real FF values once you get the actual rpms
        // double FFDutyCyclePercent = 8.48 * Math.pow(10, -5) * Math.abs(speed) +
        // .0143;
        // FFDutyCyclePercent = Math.min(1.0, FFDutyCyclePercent);
        // FFDutyCyclePercent = Math.copySign(FFDutyCyclePercent,speed);
        // return FFDutyCyclePercent;
        return 8.48 * Math.pow(10, -5) * (speed);
    }

    public int lookupSlot(double speed) {
        return 0;
    }

    // public void shootSmartDashboard() { // change name from shootsmartdashboard
    //     m_lowShooterClosedLoopConfig.setReference(cVelocity, ControlType.kSmartVelocity, 0);
    //     m_lowShooterClosedLoopConfig.setReference(cVelocity, ControlType.kSmartVelocity, 0);
    // }

    // public void shootTwoSmartDashboard() { // change from twosmart dashboard to other name
    //     m_topShooterClosedLoopConfig.setReference(cVelocity, ControlType.kSmartVelocity, 0);
    //     m_lowShooterClosedLoopConfig.setReference(cVelocity2, ControlType.kSmartVelocity, 0);
    // }

    // public void shootTwoSmartDashboardFF() {
    //     m_topShooterClosedLoopConfig.setReference(cVelocity, ControlType.kSmartVelocity, 0, lookupFF(cVelocity),
    //             ArbFFUnits.kPercentOut);
    //     m_lowShooterClosedLoopConfig.setReference(cVelocity2, ControlType.kSmartVelocity, 0, lookupFF(cVelocity),
    //             ArbFFUnits.kPercentOut);
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Percent Shooter Motor 1", m_dial1);
        // SmartDashboard.putNumber("Percent Shooter Motor 2", m_dial2);
        SmartDashboard.putNumber("Shooter Top Motor", getTopMotorSpeed());
        SmartDashboard.putNumber("Shooter Low Motor", getLowMotorSpeed());
        SmartDashboard.putNumber("Shooter Output Voltage Low motor",
                m_lowMotor.getAppliedOutput() * m_lowMotor.getBusVoltage());
        // SmartDashboard.putNumber("Shooter Output Voltage Low motor",
        // m_lowMotor.getAppliedOutput()*m_lowMotor.getBusVoltage());
        // SmartDashboard.putNumber("maxRPMTop", maxRPMTop);
        // SmartDashboard.putNumber("maxRPMLow", maxRPMLow);
        //SmartDashboard.putBoolean("NOTE IN?", m_gatewayLimitSwitch.isPressed());
        SmartDashboard.putBoolean("INTAKE SPINNING", isIntaking());
        // SmartDashboard.putNumber("Rise Time Top", riseTimeTop);
        // SmartDashboard.putNumber("Rise Time Low", riseTimeLow);
        // SmartDashboard.putNumber("Stable Oscillation Time Top",
        // finalOscillationTimeTop);
        // SmartDashboard.putNumber("Stable Oscillation Time Low",
        // finalOscillationTimeLow);
        // if (getLowMotorSpeed() > maxRPMLow) {
        // maxRPMLow = getLowMotorSpeed();
        // }

        // if (getLowMotorSpeed() >= cVelocity2 && riseTimeLow == 0) {
        // riseTimeLow = Timer.getFPGATimestamp() - initialTimeLow;
        // startOscillationTimeLow = Timer.getFPGATimestamp();
        // oscillationStopWatchLow.start();
        // bOscillatingLow = true;
        // }
        // if (bOscillatingLow) {
        // if (getLowMotorSpeed() < cVelocity2 + 10 && getLowMotorSpeed() > cVelocity2 -
        // 10) {
        // oscillationStopWatchLow.start();
        // if (oscillationStopWatchLow.get() >= .5) {
        // finalOscillationTimeLow = Timer.getFPGATimestamp() - startOscillationTimeLow
        // - .5;
        // bOscillatingLow = false;
        // }
        // } else {
        // oscillationStopWatchLow.stop();
        // oscillationStopWatchLow.reset();
        // }
        // }

        // if (getTopMotorSpeed() > maxRPMTop) {
        // maxRPMTop = getTopMotorSpeed();
        // }

        // if (getTopMotorSpeed() >= cVelocity && riseTimeTop == 0) {
        // riseTimeTop = Timer.getFPGATimestamp() - initialTimeTop;
        // startOscillationTimeTop = Timer.getFPGATimestamp();
        // oscillationStopWatchTop.start();
        // bOscillatingTop = true;
        // }
        // if (bOscillatingTop) {
        // if (getTopMotorSpeed() < cVelocity + 10 && getTopMotorSpeed() > cVelocity -
        // 10) {
        // oscillationStopWatchTop.start();
        // if (oscillationStopWatchTop.get() >= .5) {
        // finalOscillationTimeTop = Timer.getFPGATimestamp() - startOscillationTimeTop
        // - .5;
        // bOscillatingTop = false;
        // }
        // } else {
        // oscillationStopWatchTop.stop();
        // oscillationStopWatchTop.reset();
        // }
        // }

        // p = SmartDashboard.getNumber("P Gain", 0);
        // i = SmartDashboard.getNumber("I Gain", 0);
        // d = SmartDashboard.getNumber("D Gain", 0);
        // velocity = SmartDashboard.getNumber("Set Velocity", 0);
        // velocity2 = SmartDashboard.getNumber("Set Velocity2", 0);

        // if (p != cP) {
        //     m_topShooterClosedLoopConfig.setP(p);
        //     m_lowShooterClosedLoopConfig.setP(p);
        //     cP = p;
        // }
        // if (i != cI) {
        //     m_topShooterClosedLoopConfig.setI(i);
        //     m_lowShooterClosedLoopConfig.setI(i);
        //     cI = i;
        // }
        // if (d != cD) {
        //     m_topShooterClosedLoopConfig.setD(d);
        //     m_lowShooterClosedLoopConfig.setD(d);
        //     cD = d;
        // }
        if (velocity != cVelocity) {
            cVelocity = velocity;
        }
        if (velocity2 != cVelocity2) {
            cVelocity2 = velocity2;
        }

    }
}