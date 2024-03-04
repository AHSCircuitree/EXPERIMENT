// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
 
  TalonFX AngleMotor;
  TalonFX BottomShootingMotor;
  TalonFX CentralShootingMotor;
  TalonFX TopShootingMotor;
 
  DutyCycleEncoder AngleEncoder;

  double CurrentTicks;
  double CurrentAngle;
  double TargetAngle;
 
  public Arm() {
 
    // Motors
    AngleMotor = new TalonFX(Constants.CAN_IDs.AngleID,"FRC 1599");
    BottomShootingMotor = new TalonFX(Constants.CAN_IDs.BottomShootingID,"FRC 1599");
    CentralShootingMotor = new TalonFX(Constants.CAN_IDs.CentralShootingID,"FRC 1599");
    TopShootingMotor = new TalonFX(Constants.CAN_IDs.TopShootingID,"FRC 1599");

    AngleMotor.setNeutralMode(NeutralModeValue.Brake);

    AngleEncoder = new DutyCycleEncoder(6);

    AngleEncoder.setPositionOffset(.828);
 
    SmartDashboard.putNumber("Custom Angle", 0);
    SmartDashboard.putNumber("Custom Speed",0);
 
  }

  @Override
  public void periodic() {
 
    if (AngleEncoder.getAbsolutePosition() < .2) {

      CurrentTicks = AngleEncoder.getAbsolutePosition() + 1;
      
    } else {

      CurrentTicks = AngleEncoder.getAbsolutePosition();

    }

    CurrentAngle = -(CurrentTicks / (.072 / 28) - 328) - 14;
 
    SmartDashboard.putNumber("Angle Encoder Degrees", CurrentAngle);
    SmartDashboard.putNumber("Arm Speed", AnglePID.calculate(CurrentAngle, TargetAngle));

  }
 
  public void RunAngleWithLimits(double Speed) {

    if (Speed > 0 && CurrentAngle >= Constants.UpperArmLimit) {


      
    }
    AngleMotor.set(speed);
 
  }

  public void ChangeAngleThroughPID() {

    RunAngleWithLimits(Constants.AnglePID.calculate(CurrentAngle, TargetAngle));
 
  }

  public void ChangeTarget(double Target) {

    TargetAngle = Target;

  }

  public void RunShooter(double Speed) {
 
    BottomShootingMotor.set(-Speed);
    CentralShootingMotor.set(-Speed);
    TopShootingMotor.set(-Speed);

  }

  public void Spinup(double speed) {

    TopShootingMotor.set(-speed);
    CentralShootingMotor.set(-speed);
  }
 
  public void RunBottom(double speed) {

    BottomShootingMotor.set(-speed);

  }

}
