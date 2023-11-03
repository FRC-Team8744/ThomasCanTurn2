// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnSimplePID extends CommandBase {
  private final DriveSubsystem m_drive;
  PIDController m_turnCtrl = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
  private double m_output;

  /** Creates a new TurnSimplePID. */
  public TurnSimplePID(double targetAngleDegrees, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(10.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_output = m_turnCtrl.calculate(m_drive.getHeading());
    SmartDashboard.putNumber("PID output", m_output);
    // Send PID output to drivebase
    m_drive.arcadeDrive(0.0, m_output);
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
