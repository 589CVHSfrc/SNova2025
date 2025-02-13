// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_ARM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStowEXP extends Command {
  ArmSubsystem m_arm;
  DoubleSupplier m_speed;
  boolean m_limitHit;
  Timer m_timer;
  public ArmStowEXP(ArmSubsystem arm, DoubleSupplier speed) {
    m_timer = new Timer();
    m_arm = arm;
    m_speed = speed;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_limitHit = false;
    m_arm.armRelease();
    m_timer.reset();
  }

  @Override
  public void execute() {
    var isReversed = m_arm.isReverselimitReached();

    if(isReversed){
      m_limitHit = true;
      m_timer.start();
    }

    if(m_arm.getAbsoluteAngle()<.25){
      // System.out.println("slow....");
      m_arm.moveArmSpeed(.01);
    }

     else if (m_arm.getAbsoluteAngle()<.30){
      m_arm.moveArmSpeed(-.05);
    }

    else if (m_arm.getAbsoluteAngle()<.45){
      m_arm.moveArmSpeed(-.15);
    }
    else{
      m_arm.moveArmSpeed(-m_speed.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.stopArm();
    m_arm.armBrake();
    m_timer.reset();
  }

  @Override
  public boolean isFinished() {
    return m_limitHit ;//&& m_timer.hasElapsed(1);
  }
}
