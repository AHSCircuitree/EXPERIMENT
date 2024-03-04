// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ChoreoPath;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  //wbecol;vb edwo;lujdbvw oeWSB 

  // Variables
  private static final double MaxSpeed = 3; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
 
  // Controllers
  private final CommandXboxController Player1 = new CommandXboxController(0);
 
  // Subsystems
  public final Drivetrain drivetrain = TunerConstants.DriveTrain;  

  // Commands
 
  // Objects for Tele-op Drive
  public SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.05) // Small deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I DON'T want field-centric
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    // Basic Drive Command
    drivetrain.setDefaultCommand( 
        drivetrain.applyRequest(() -> driveFieldCentric
        .withVelocityX(Player1.getLeftY() * MaxSpeed * .85)  
        .withVelocityY(Player1.getLeftX()* MaxSpeed * .85) 
        .withRotationalRate((-Player1.getRightX() * MaxAngularRate)) 
    ));

    // Registers the Telemetry
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    
    // Set the button bindings
    configureBindings();
 
  }
  //Field Orientation Reset
  public Command getAutonomousCommand() {

    return new ChoreoPath(drivetrain, "CenterShoot", true);
    
  }

}
