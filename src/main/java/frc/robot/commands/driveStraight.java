// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class driveStraight extends CommandBase {

  private final DrivetrainSubsystem m_drive;
  private final double m_distance;
  private final double startEncoder;

  /** Creates a new driveStraight. */
  double startTime;
  public driveStraight(DrivetrainSubsystem drive, double inches) {
    m_drive = drive;
    m_distance = inches;
    startEncoder = m_drive.getAverageEncoderDistance();
    addRequirements(m_drive);
    // m_drive.resetEncoders();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    // m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0.25);
    System.out.println(m_drive.getAverageEncoderDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0);
    // m_drive.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Finished");
    return Math.abs((m_drive.getAverageEncoderDistance() - startEncoder) * 6) >= m_distance;
    // return Timer.getFPGATimestamp() - startTime > 0.5;
  }
}
