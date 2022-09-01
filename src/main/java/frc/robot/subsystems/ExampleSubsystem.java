// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExampleSubsystem extends SubsystemBase {
  private final SwerveModule m_testSwerveModule;

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Front right
    new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );
  
  ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
  DoubleSupplier m_steerAngleSupplier;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    m_testSwerveModule = Mk4iSwerveModuleHelper.createNeo(
      m_tab.getLayout("Test Swerve Module", BuiltInLayouts.kList)
           .withSize(2, 4)
           .withPosition(0, 0),
      Mk4iSwerveModuleHelper.GearRatio.L2,
      Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
      Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
      99,
      0);

    m_steerAngleSupplier = () -> { return m_testSwerveModule.getSteerAngle(); };
  }

  public void directSwerve(double voltage, double angleRadians) {
    m_testSwerveModule.set(voltage, angleRadians);
  }

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    m_tab.addNumber("Steer Angle", m_steerAngleSupplier);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
