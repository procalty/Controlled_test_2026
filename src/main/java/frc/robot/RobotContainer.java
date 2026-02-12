// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.constants.DriveConstants;

public class RobotContainer {


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband( DriveConstants.MaxSpeed* 0.15).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.15) // Add a  deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  public final  double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	private final CommandXboxController joystick = new CommandXboxController(0);
    
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

      // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Swerve.get().setDefaultCommand(
            // Drivetrain will execute this command periodically
            Swerve.get().applyRequest(() ->
                drive.withVelocityX((-joystick.getLeftY() / 2) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY((-joystick.getLeftX() / 2) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
