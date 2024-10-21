// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSystem;

public class RelativeDrive extends Command {
  
  SwerveSystem swerveSystem;
  PS5Controller controller;
  double modifier;

  public RelativeDrive(SwerveSystem swerveSystem, PS5Controller controller) {
    this.swerveSystem = swerveSystem;
    this.controller = controller;
    addRequirements(swerveSystem);
  }

  @Override
  public void initialize() {
    modifier = 0.5;
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = new ChassisSpeeds(controller.getLeftY() * Constants.MAX_SWERVE_SPEED * modifier, controller.getLeftX() * Constants.MAX_SWERVE_SPEED * modifier, controller.getRightX() * Constants.MAX_SWERVE_SPEED); // -y * 4.4196 * modifier, -x * 4.4196 * modifier, -rot * 4.4196
    swerveSystem.driveFromChassisSpeeds(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
