// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constElevator;
import frc.robot.RobotMap.mapElevator;

public class Elevator extends SubsystemBase {
  private TalonFX leftMotorFollower;
  private TalonFX rightMotorLeader;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotorFollower = new TalonFX(mapElevator.LEFT_ELEVATOR_CAN);
    rightMotorLeader = new TalonFX(mapElevator.RIGHT_ELEVATOR_CAN);

    rightMotorLeader.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);
    leftMotorFollower.getConfigurator().apply(constElevator.ELEVATOR_CONFIG);

  }

  public void setPosition(double setpoint) {
    rightMotorLeader.setControl(new PositionVoltage(setpoint));
    leftMotorFollower.setControl(new Follower(rightMotorLeader.getDeviceID(), true));
  }

  public void resetSensorPosition(double setpoint) {
    rightMotorLeader.setPosition(setpoint);
    leftMotorFollower.setPosition(setpoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Left/Pos", leftMotorFollower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/CLO", leftMotorFollower.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Left/Ouptu", leftMotorFollower.get());
    SmartDashboard.putBoolean("Elevator/Left/Inverted", leftMotorFollower.getInverted());
        SmartDashboard.putNumber("Elevator/Left/Current", leftMotorFollower.getSupplyCurrent().getValueAsDouble());


    SmartDashboard.putNumber("Elevator/Right/Pos", rightMotorLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/CLO", rightMotorLeader.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Right/Ouptu", rightMotorLeader.get());
    SmartDashboard.putBoolean("Elevator/Right/Inverted", rightMotorLeader.getInverted());
    SmartDashboard.putNumber("Elevator/Right/Current", rightMotorLeader.getSupplyCurrent().getValueAsDouble());

  }
}
