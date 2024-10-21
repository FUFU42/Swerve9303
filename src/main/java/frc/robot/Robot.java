// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveModeSwitch;
import frc.robot.commands.DriveWithNavX;
import frc.robot.subsystems.SwerveSystem;

public class Robot extends TimedRobot {

  SwerveSystem swerveSystem;
  PS5Controller controller;

  @Override
  public void robotInit() {
    swerveSystem = new SwerveSystem();
    controller = new PS5Controller(0);

    swerveSystem.setDefaultCommand(new DriveWithNavX(swerveSystem, controller));

    new JoystickButton(controller, PS5Controller.Button.kR2.value).onTrue(new DriveModeSwitch(swerveSystem, controller));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
