package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.loops.DriveLoop;

public class Trajectories {
    private static DriveLoop driveLoop = DriveLoop.getInstance();

    // Create a voltage constraint to ensure we don't accelerate too fast
    private static DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            driveLoop.getFeedForward(),
            driveLoop.getKinematics(),
            10);

    // Create config for trajectory
    private static TrajectoryConfig config = new TrajectoryConfig(
            NumberConstants.kMaxSpeedMetersPerSecond,
            NumberConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(driveLoop.getKinematics())
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    public static final Trajectory testTraj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)), config);

}
