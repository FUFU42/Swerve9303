// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSystem extends SubsystemBase {

    private SwerveModule[] modulesArray = new SwerveModule[4];
    private SwerveDriveKinematics kinematics;
    private AHRS navX;

    public SwerveSystem() {
        // TODO: Update IDs
        modulesArray[0] = new SwerveModule(1, 2);  // Front-left
        modulesArray[1] = new SwerveModule(3, 4);  // Front-right
        modulesArray[2] = new SwerveModule(5, 6);  // Back-left
        modulesArray[3] = new SwerveModule(7, 8);  // Back-right

        kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVE_OFFSET_FROM_CENTER, Constants.DRIVE_OFFSET_FROM_CENTER),
            new Translation2d(Constants.DRIVE_OFFSET_FROM_CENTER, -Constants.DRIVE_OFFSET_FROM_CENTER),
            new Translation2d(-Constants.DRIVE_OFFSET_FROM_CENTER, Constants.DRIVE_OFFSET_FROM_CENTER),
            new Translation2d(-Constants.DRIVE_OFFSET_FROM_CENTER, -Constants.DRIVE_OFFSET_FROM_CENTER)
        );

        navX = new AHRS(SPI.Port.kMXP);
    }

    public void stop() {
        for (int i = 0; i < modulesArray.length; i++){
            modulesArray[i].stop();
        }
    }

    public void move(double drive, double rotation) {
        for (int i = 0; i < modulesArray.length; i++){
            modulesArray[i].move(drive, rotation);
        }
    }

    public void setDesiredStates(SwerveModuleState[] states) {
        for (int i = 0; i < modulesArray.length; i++){
            modulesArray[i].setDesiredState(states[i]);
        }
    }

    // drive in relation to the robot.
    public void driveRobotRelative(double speedY, double speedX, double rotation) {
        driveFromChassisSpeeds(new ChassisSpeeds(speedY, speedX, Math.toRadians(rotation)));
    }

    public void driveFromChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SWERVE_SPEED); // max speed m/s
        setDesiredStates(states);
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = new SwerveModuleState[modulesArray.length];
        for (int i = 0; i < modulesArray.length; i++) {
          states[i] = modulesArray[i].getState();
        }
        return kinematics.toChassisSpeeds(states);
    }

    public void driveWithNavX(double speedY, double speedX, double rotation) {
        double yaw = navX.getYaw();

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(speedY, speedX, rotation, new Rotation2d(Math.toRadians(yaw))));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SWERVE_SPEED);
        
        setDesiredStates(states);
    }
}
