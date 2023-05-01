// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.DriveLoop.DriveStates;
import frc.robot.util.NumberConstants;
import frc.robot.util.Trajectories;

public class FollowPath extends SequentialCommandGroup {
  private DriveLoop driveLoop;

  /** Creates a new FollowPath. */
  public FollowPath(Trajectory path) {
    driveLoop = DriveLoop.getInstance();

    addCommands(
        // set state
        new InstantCommand(() -> driveLoop.setState(DriveStates.PATH_FOLLOWING)),
        new InstantCommand(() -> driveLoop.resetOdometry(Trajectories.testTraj.getInitialPose())),

        // follow path
        new RamseteCommand(path,
            driveLoop::getDrivePose,
            new RamseteController(NumberConstants.Ram_b, NumberConstants.Ram_zeta),
            driveLoop.getFeedForward(),
            driveLoop.getKinematics(),
            driveLoop::getWheelSpeeds,
            driveLoop.getLeftPID(),
            driveLoop.getRightPID(),
            driveLoop::driveVolts),

        // set state
        new InstantCommand(() -> driveLoop.setState(DriveStates.DISABLED)));
  }
}
