// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSystem;

public class DriveModeSwitch extends Command {
  
  SwerveSystem swerveSystem;
  PS5Controller controller;

  public DriveModeSwitch(SwerveSystem swerveSystem, PS5Controller controller) {
    this.swerveSystem = swerveSystem;
    this.controller = controller;
    addRequirements(swerveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command currentDriveState = swerveSystem.getDefaultCommand();
    if (currentDriveState.getClass() == DriveWithNavX.class) {
      swerveSystem.setDefaultCommand(new RelativeDrive(swerveSystem, controller));
    }else{
      swerveSystem.setDefaultCommand(new DriveWithNavX(swerveSystem, controller));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
