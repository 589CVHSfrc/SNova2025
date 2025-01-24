// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_PARALLEL;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.COMMAND_ARM.ArmFloor;
import frc.robot.commands.COMMAND_SHOOTER.IntakeLimitSwitch;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GatewaySubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class IntakeDown extends ParallelCommandGroup {
  public IntakeDown(ArmSubsystem arm, ShooterSubsystem shooter, GatewaySubsystem gate) {
    
    addCommands(
      new ArmFloor(arm, ()-> .1),// new MoveArmSpeed(arm, ()->.1),
      new IntakeLimitSwitch(shooter, gate)
    );
  }
}
