// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnSimplePID extends CommandBase {
  private final DriveSubsystem m_drive;
  PIDController m_turnCtrl = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
  private double m_output;
  private double m_heading;
  private double m_goalAngle;

  /** Creates a new TurnSimplePID. */
  public TurnSimplePID(double targetAngleDegrees, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);

    m_goalAngle = targetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(1.0);
    m_turnCtrl.setSetpoint(m_goalAngle);
    m_turnCtrl.reset();

    // Debug information
    // double kP = SmartDashboard.getNumber("kP", DriveConstants.kTurnP);
    // double kI = SmartDashboard.getNumber("kI", DriveConstants.kTurnI);
    // double kD = SmartDashboard.getNumber("kD", DriveConstants.kTurnD);
    // m_turnCtrl.setPID(kP, kI, kD);
    // Things to remember:
    //   input squaring in arcade drive
    //   changing values in shuffleboard PID works - but robot must be disabled

    SmartDashboard.putData("PID", m_turnCtrl);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_heading = m_drive.getHeading();
    m_output = MathUtil.clamp(m_turnCtrl.calculate(m_heading), -1.0, 1.0);
    // Send PID output to drivebase
    m_drive.arcadeDrive(0.0, -m_output, false);
    // m_drive.tankDriveVolts(m_output, -m_output);

    // Debug information
    SmartDashboard.putNumber("PID setpoint", m_goalAngle);
    SmartDashboard.putNumber("PID output", m_output);
    SmartDashboard.putNumber("PID setpoint error", m_turnCtrl.getPositionError());
    SmartDashboard.putNumber("PID velocity error", m_turnCtrl.getVelocityError());
    SmartDashboard.putNumber("PID measurement", m_heading);

    // SmartDashboard.putNumber("kP", m_turnCtrl.getP());
    // SmartDashboard.putNumber("kI", m_turnCtrl.getI());
    // SmartDashboard.putNumber("kD", m_turnCtrl.getD());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turnCtrl.atSetpoint();
  }
}
