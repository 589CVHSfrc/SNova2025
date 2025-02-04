// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class GatewaySubsystem extends SubsystemBase {
  private SparkMax m_gatewayMotor;
  private SparkMaxConfig m_gatewayConfig;
  private SparkLimitSwitch m_gatewayLimitSwitch;
  private SparkLimitSwitch m_gatewayReverseLimitSwitch;

  public GatewaySubsystem() {
    m_gatewayConfig = new SparkMaxConfig();
    m_gatewayMotor = new SparkMax(ShooterConstants.kTopGatewayWheelMotorID, MotorType.kBrushless);
    
    //m_gatewayMotor.setInverted(false);
    m_gatewayConfig.inverted(true);
    m_gatewayConfig.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(false)
      .reverseLimitSwitchType(Type.kNormallyOpen);
    // m_gatewayLimitSwitch.enableLimitSwitch(true);
    // m_gatewayReverseLimitSwitch = m_gatewayMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    // m_gatewayLimitSwitch.enableLimitSwitch(false);
    m_gatewayConfig.idleMode(IdleMode.kBrake);
    // m_gatewayMotor.setIdleMode(IdleMode.kBrake);
    // m_gatewayMotor.burnFlash();
    m_gatewayMotor.configure(m_gatewayConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    
  }

  public boolean isSwitchPressed() {
    return m_gatewayLimitSwitch.isPressed();
  }


  public void stopGateway() {
    m_gatewayMotor.set(0);
  }

  public void IDLEGateway() {
    m_gatewayMotor.set(0);
  }
  public void intakeGateway() {
    m_gatewayMotor.set(ShooterConstants.kGatewayMotorSpeed);
  }

  public void shootGateway() {
    //System.out.println("Setting");
    m_gatewayMotor.set(-ShooterConstants.kGatewayMotorSpeed);
  }

  @Override
  public void periodic() {
    
  }
}