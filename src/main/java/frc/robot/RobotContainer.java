// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private ShuffleboardTab m_drivetrainTab;
  private XboxController m_driveController;

  private DigitalInput m_encoderInput;
  private DutyCycle m_encoderDutyCycle;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_encoderInput = new DigitalInput(0);
    m_encoderDutyCycle = new DutyCycle(m_encoderInput);

    m_drivetrainTab = Shuffleboard.getTab("Drivetrain");

    m_drivetrainTab.addNumber("TranslationX", () -> getSwerveTranslationX());
    m_drivetrainTab.addNumber("TranslationY", () -> getSwerveTranslationY());
    m_drivetrainTab.addNumber("Rotation", () -> getSwerveRotation());
    m_drivetrainTab.addNumber("EncoderValue", () -> getAbsoluteEncoderPosition());
    m_drivetrainTab.addNumber("EncoderRadians", () -> getAbsoluteEncoderPosition());

    m_driveController = new XboxController(0);

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_swerveDriveSubsystem.setDefaultCommand(new SwerveDriveCommand(
      m_swerveDriveSubsystem,
      () -> getSwerveTranslationX(),
      () -> getSwerveTranslationY(),
      () -> getSwerveRotation()
    ));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public int getAbsoluteEncoderPosition() {
    int position = (int)Math.round(m_encoderDutyCycle.getOutput());

    if(position < 0) {
        position = 0;
    } else if (position > 4095) {
        position = 4095;
    }

    position -= 2048;

    if(position > 2047) {
        position -=4096;
    } else if(position < -2048) {
        position += 4096;
    }

    return position;
  }

  public double getAbsoluteEncoderAngle() {
    int position = getAbsoluteEncoderPosition();
    double angle = (360.0 / 4096.0) * position + 180.0;
    return angle;
  }

  public double getSwerveTranslationX() {
    return -modifyAxis(m_driveController.getLeftY()) * SwerveDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
  }
  
  public double getSwerveTranslationY() {
    return -modifyAxis(m_driveController.getLeftX()) * SwerveDriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
  }

  public double getSwerveRotation() {
    return -modifyAxis(m_driveController.getRightX()) * SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
