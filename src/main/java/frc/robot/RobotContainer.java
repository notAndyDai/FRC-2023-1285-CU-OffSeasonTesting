// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.DriveLoop.DriveStates;

public class RobotContainer {
  private DriveLoop driveLoop;

  public RobotContainer() {
    driveLoop = DriveLoop.getInstance();
  }

  public void setAuto() {
    driveLoop.resetOdometry(new Pose2d());
    driveLoop.setBrake();
  }

  public void setTeleop() {
    driveLoop.setDrive();
    driveLoop.setState(DriveStates.TELEOP);
  }

  public void setDisabled() {
    driveLoop.resetGyro();
    driveLoop.resetEncoders();
    driveLoop.setCoast();
    driveLoop.stopPower();
  }

}
