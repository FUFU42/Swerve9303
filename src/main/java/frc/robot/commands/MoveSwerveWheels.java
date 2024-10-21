// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSystem;

public class MoveSwerveWheels extends Command {
  
  SwerveSystem swerveSystem;
  PS5Controller controller;

  public MoveSwerveWheels(SwerveSystem swerveSystem, PS5Controller controller) {
    this.swerveSystem = swerveSystem;
    this.controller = controller;
    addRequirements(swerveSystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveSystem.move(controller.getLeftY(), controller.getLeftX());
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
