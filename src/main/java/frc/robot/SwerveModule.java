package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  
  private CANSparkMax driveMotor;
  private CANSparkMax steerMotor;
  private RelativeEncoder driveEncoder;
  private RelativeEncoder steerEncoder;
  private PIDController steerPID;
  private PIDController drivePID;


  public SwerveModule(int driveMotorID, int steerMotorID) {
      
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
        
    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();

    driveEncoder.setPosition(0);
    steerEncoder.setPosition(0);

    steerPID = new PIDController(1, 0.0, 0.0);  // Tune later
    steerPID.enableContinuousInput(-180, 180);

    drivePID = new PIDController(1, 0.0, 0.0);  // Tune later
    drivePID.enableContinuousInput(-180, 180);
  }

  public void stop() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }
    
  public void move(double drive, double rotation) {
    driveMotor.set(drive);
    steerMotor.set(rotation);
  }
    
  public double getHeadingDegrees() {
    return (steerEncoder.getPosition() / steerEncoder.getCountsPerRevolution() * Constants.SWERVE_STEER_GEAR_RATIO * 360) % 360;
  }

  public double getVelocityRPM() {
    return driveEncoder.getVelocity();
    // return driveEncoder.getVelocity() * 600 / Constants.TALONFX_CPR * Constants.SWERVE_DRIVE_GEAR_RATIO;
  }
    
  public void setDesiredState(SwerveModuleState desiredState) {

    SwerveModuleState optimizedDesiredState = optimize(desiredState, new Rotation2d(Math.toRadians(getHeadingDegrees())));

    double desiredSpeedRPM = (optimizedDesiredState.speedMetersPerSecond * 60 / (2 * Math.PI * Constants.DRIVE_WHEEL_RADIUS_METERS)) / Constants.SWERVE_DRIVE_GEAR_RATIO;
    double calculatedDesiredSpeedRPM = drivePID.calculate(getVelocityRPM(), desiredSpeedRPM);
    driveMotor.set(calculatedDesiredSpeedRPM);

    double desiredAngleDegrees = desiredState.angle.getDegrees();
    double calculatedDesiredAngleDegrees = steerPID.calculate(getHeadingDegrees(), desiredAngleDegrees);
    steerMotor.set(calculatedDesiredAngleDegrees);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState((getVelocityRPM() * Math.PI  / 30.0) * Constants.DRIVE_WHEEL_RADIUS_METERS, Rotation2d.fromDegrees(getHeadingDegrees()));
  }

  public SwerveModulePosition getModulePosition() {
    SwerveModulePosition pos = new SwerveModulePosition(getDistanceTraveledMeters(),new Rotation2d(Math.toRadians(getHeadingDegrees())));
    return pos;
  }

  public double getDistanceTraveledMeters() {
    return driveEncoder.getPosition() / Constants.SWERVE_DRIVE_GEAR_RATIO * (2 * Math.PI * Constants.DRIVE_WHEEL_RADIUS_METERS);
  }

  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
      }
    
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }
}