// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ManualShoot extends Command {
 
  double m_currentVelocity;
  double m_targetVelocity;

  Timer moveNoteDown;
  Timer shootNote;
  
  Arm m_arm;

  public ManualShoot(Arm Arm, double Velocity) {

    m_arm = Arm;
    m_targetVelocity = Velocity;

    moveNoteDown = new Timer();
    shootNote = new Timer();

    addRequirements(Arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    moveNoteDown.restart();
    shootNote.stop();

    m_currentVelocity = m_arm.ReturnVelocity();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_arm.Spinup(m_targetVelocity);

    if (moveNoteDown.get() < .5) {

      m_arm.RunBottom(-.1);

    } else if (Math.abs(m_currentVelocity - m_targetVelocity) > 100) {

      m_arm.RunBottom(0);

    } else {

      shootNote.restart();

      m_arm.RunBottom(.2);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_arm.RunBottom(0);
    m_arm.Spinup(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (shootNote.get() > .5) {

      return true;

    } else {

      return false;

    }

  }

}
