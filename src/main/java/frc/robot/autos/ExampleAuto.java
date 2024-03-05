// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ExampleAuto extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;

  public ExampleAuto(Drivetrain Drivetrain) {
  
    m_drivetrain = Drivetrain;

    addCommands(

      ChoreoPathing("CenterShoot", false),
      ChoreoPathing("CenterShoot2", false)
      
    );

  }

  private Command ChoreoPathing(String Trajectory, boolean IsRed) {

    return Choreo.choreoSwerveCommand(

      Choreo.getTrajectory(Trajectory), 
      () -> (m_drivetrain.getState().Pose), 
      Constants.AutoDrivePID, Constants.AutoDrivePID, Constants.AutoTurnPID, 
      (ChassisSpeeds speeds) -> m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> IsRed, 
      m_drivetrain

    );

  }

}
