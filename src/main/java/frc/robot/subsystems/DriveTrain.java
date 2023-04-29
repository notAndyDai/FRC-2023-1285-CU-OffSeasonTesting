// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ElectricalConstants;
import frc.robot.util.NumberConstants;

public class DriveTrain extends SubsystemBase {
  protected AHRS gyro;

  private CANSparkMax leftMain;
  private CANSparkMax leftFollowFront;
  private CANSparkMax leftFollowBack;

  private RelativeEncoder leftMainEncoder;
  private RelativeEncoder leftFollowFrontEncoder;
  private RelativeEncoder leftFollowBackEncoder;

  private CANSparkMax rightMain;
  private CANSparkMax rightFollowFront;
  private CANSparkMax rightFollowBack;

  private RelativeEncoder rightMainEncoder;
  private RelativeEncoder rightFollowFrontEncoder;
  private RelativeEncoder rightFollowBackEncoder;

  protected final MotorControllerGroup leftMotors;
  protected final MotorControllerGroup rightMotors;

  protected DifferentialDriveKinematics kinematics;
  protected DifferentialDriveOdometry odometry;
  protected Pose2d pose;

  private SimpleMotorFeedforward feedforward;
  private PIDController leftPID;
  private PIDController rightPID;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Left Motors
    leftMain = new CANSparkMax(ElectricalConstants.LEFT_MAIN, MotorType.kBrushless);
    leftMain.setInverted(false);

    leftFollowFront = new CANSparkMax(ElectricalConstants.LEFT_FOLLOW_FRONT, MotorType.kBrushless);
    leftFollowFront.follow(leftMain);
    leftFollowFront.setInverted(false);

    leftFollowBack = new CANSparkMax(ElectricalConstants.LEFT_FOLLOW_BACK, MotorType.kBrushless);
    leftFollowBack.follow(leftMain);
    leftFollowBack.setInverted(false);

    leftMainEncoder = leftMain.getEncoder();
    leftFollowFrontEncoder = leftFollowFront.getEncoder();
    leftFollowBackEncoder = leftFollowBack.getEncoder();

    // Right motors
    rightMain = new CANSparkMax(ElectricalConstants.RIGHT_MAIN, MotorType.kBrushless);
    rightMain.setInverted(true);

    rightFollowFront = new CANSparkMax(ElectricalConstants.RIGHT_FOLLOW_FRONT, MotorType.kBrushless);
    rightFollowFront.follow(rightMain);
    rightFollowFront.setInverted(true);

    rightFollowBack = new CANSparkMax(ElectricalConstants.RIGHT_FOLLOW_BACK, MotorType.kBrushless);
    rightFollowBack.follow(rightMain);
    rightFollowBack.setInverted(true);

    rightMainEncoder = rightMain.getEncoder();
    rightFollowFrontEncoder = rightFollowFront.getEncoder();
    rightFollowBackEncoder = rightFollowBack.getEncoder();

    // motor groups
    leftMotors = new MotorControllerGroup(leftMain, leftFollowBack, leftFollowFront);
    rightMotors = new MotorControllerGroup(rightMain, rightFollowBack, rightFollowFront);

    // thing
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(NumberConstants.ROBOT_WIDTH));
    odometry = new DifferentialDriveOdometry(getHeading(), getLeftDistance(), getRightDistance());
    pose = new Pose2d();

    // feedforward
    feedforward = new SimpleMotorFeedforward(NumberConstants.driveKs, NumberConstants.driveKv, NumberConstants.driveKa);

    // PID
    leftPID = new PIDController(NumberConstants.driveKp, 0, 0);
    rightPID = new PIDController(NumberConstants.driveKp, 0, 0);
  }

  // ******************* Odometry ********************
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Pose2d getDrivePose() {
    return odometry.getPoseMeters();
  }

  // ******************* Encoders ********************
  public double getLeftEncoderValue() {
    return leftMainEncoder.getPosition();
  }

  public double getRightEncoderValue() {
    return rightMainEncoder.getPosition();
  }

  public double getLeftDistance() {
    return (getLeftEncoderValue() * Math.PI * NumberConstants.WHEELDIAMETER)
        / (NumberConstants.NEO_ENCODER_TPR * NumberConstants.DRIVE_GEAR_RATIO);
  }

  public double getRightDistance() {
    return (getRightEncoderValue() * Math.PI * NumberConstants.WHEELDIAMETER)
        / (NumberConstants.NEO_ENCODER_TPR * NumberConstants.DRIVE_GEAR_RATIO);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMainEncoder.getVelocity() / NumberConstants.DRIVE_GEAR_RATIO * 2 * Math.PI
            * Units.inchesToMeters(NumberConstants.WHEELDIAMETER) / 60,
        rightMainEncoder.getVelocity() / NumberConstants.DRIVE_GEAR_RATIO * 2 * Math.PI
            * Units.inchesToMeters(NumberConstants.WHEELDIAMETER) / 60);
  }

  // ******************* Kinematics ********************
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  // ******************* Feedforward and PID ********************
  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public PIDController getLeftPID() {
    return leftPID;
  }

  public PIDController getRightPID() {
    return rightPID;
  }

  // ******************* The ********************
  public void setBrake() {
    rightMain.setIdleMode(IdleMode.kBrake);
    rightFollowBack.setIdleMode(IdleMode.kBrake);
    rightFollowFront.setIdleMode(IdleMode.kBrake);
    leftMain.setIdleMode(IdleMode.kBrake);
    leftFollowBack.setIdleMode(IdleMode.kBrake);
    leftFollowFront.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    rightMain.setIdleMode(IdleMode.kCoast);
    rightFollowBack.setIdleMode(IdleMode.kCoast);
    rightFollowFront.setIdleMode(IdleMode.kCoast);
    leftMain.setIdleMode(IdleMode.kCoast);
    leftFollowBack.setIdleMode(IdleMode.kCoast);
    leftFollowFront.setIdleMode(IdleMode.kCoast);
  }

  public void stopPower() {
    leftMotors.setVoltage(0);
    rightMotors.setVoltage(0);
  }

  public void driveVolts(double left, double right) {
    leftMotors.setVoltage(left / 12);
    rightMotors.setVoltage(right / 12);
  }

}
