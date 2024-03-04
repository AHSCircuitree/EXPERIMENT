// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;

public class ChoreoPath extends Command {
 
  // Variables
  Drivetrain m_drivetrain;
  String m_trajectory;
  boolean m_isRed;

  public ChoreoPath(Drivetrain Drivetrain, String Trajectory, boolean IsRed) {

    m_drivetrain = Drivetrain;
    m_trajectory = Trajectory;
    m_isRed = IsRed;

    addRequirements(Drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_isRed == true) {

      m_drivetrain.seedFieldRelative(Choreo.getTrajectory(m_trajectory).flipped().getInitialPose());

    } else {

      m_drivetrain.seedFieldRelative(Choreo.getTrajectory(m_trajectory).getInitialPose());

    }
   
    CommandScheduler.getInstance().schedule(Choreo.choreoSwerveCommand(

      Choreo.getTrajectory(m_trajectory), 
      () -> (m_drivetrain.getState().Pose), 
      Constants.AutoDrivePID, Constants.AutoDrivePID, Constants.AutoTurnPID, 
      (ChassisSpeeds speeds) -> m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> m_isRed, 
      m_drivetrain

    ));

  }
 
}
