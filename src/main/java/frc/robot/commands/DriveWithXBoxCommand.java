// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */

public class DriveWithXBoxCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Declaring the drivetrainsubsystem variable
  private final DrivetrainSubsystem drivetrainSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */


   // Constructor
  public DriveWithXBoxCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting joystick drive command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Where everything happens
  @Override
  public void execute() {
        
    double forwardSpeed = -RobotContainer.joystick.getRawAxis(1);
    double turningSpeed = -RobotContainer.joystick.getRawAxis(4);

    

    drivetrainSubsystem.arcadeDrive(forwardSpeed * 0.5, turningSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
