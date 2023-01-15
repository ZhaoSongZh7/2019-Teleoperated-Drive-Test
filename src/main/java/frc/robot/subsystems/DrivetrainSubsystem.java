// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem
   *. */

  CANSparkMax leftFrontNeoMotor = new CANSparkMax(Constants.OperatorConstants.leftNeoMotorOneCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBackNeoMotor = new CANSparkMax(Constants.OperatorConstants.leftNeoMotorTwoCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
  WPI_TalonSRX leftSimMotor = new WPI_TalonSRX(Constants.OperatorConstants.leftSimMotorCANID);

  CANSparkMax rightFrontNeoMotor = new CANSparkMax(Constants.OperatorConstants.rightNeoMotorOneCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBackNeoMotor = new CANSparkMax(Constants.OperatorConstants.rightNeoMotorTwoCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
  WPI_TalonSRX rightSimMotor = new WPI_TalonSRX(Constants.OperatorConstants.rightSimMotorCANID);

  RelativeEncoder leftEncoder = leftFrontNeoMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontNeoMotor.getEncoder();

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontNeoMotor, leftBackNeoMotor, leftSimMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontNeoMotor, rightBackNeoMotor, rightSimMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  public DrivetrainSubsystem() {
    leftFrontNeoMotor.restoreFactoryDefaults();
    leftBackNeoMotor.restoreFactoryDefaults();
    leftSimMotor.configFactoryDefault();
    rightFrontNeoMotor.restoreFactoryDefaults();
    rightBackNeoMotor.restoreFactoryDefaults();
    rightSimMotor.configFactoryDefault();

    leftFrontNeoMotor.setIdleMode(IdleMode.kCoast);
    leftBackNeoMotor.setIdleMode(IdleMode.kCoast);
    leftSimMotor.setNeutralMode(NeutralMode.Brake);

    rightFrontNeoMotor.setIdleMode(IdleMode.kCoast);
    rightBackNeoMotor.setIdleMode(IdleMode.kCoast);
    rightSimMotor.setNeutralMode(NeutralMode.Brake);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftSimMotor.setInverted(false);
    rightSimMotor.setInverted(true);
    rightFrontNeoMotor.setInverted(true);
    rightBackNeoMotor.setInverted(true);

  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void drive(double percentOutput) {
    leftControllerGroup.set(percentOutput);
    rightControllerGroup.set(percentOutput);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    System.out.println("Resetting " + leftEncoder.getPosition());
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }  
}
