// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ReverseWristAutoCommand;

public class WristIntake extends SubsystemBase {
  private CANSparkMax spark1 = new CANSparkMax(6, MotorType.kBrushless);
  private final RelativeEncoder m_intakeEncoder = spark1.getEncoder();
  private double intakeError;
  private double AutointakeSetpointB = -168;
  private double intakeSetpointB = -168;
  private double intakeSetpointA = 0;
  private double AutointakeSetpointA = -0;
  private final double PscaleFactor = 475;
  public boolean reversed = false;
  private State currentState = State.ShootPos;
  enum State {
    ShootPos,
    IntakePos,
    NoMovePos
  }

  final double wheelCircumference = 0.478778720406;

  public double getAnglePosition() {
    return m_intakeEncoder.getPosition()*3.4;
  }

  public void wrist(boolean Bpressed, boolean Apressed, boolean Rightstickpressed, boolean AutoOn) {
    
    if(Apressed) {
    currentState = State.ShootPos;
  }
  else if (Bpressed) {
    currentState = State.IntakePos;
  }

    if (currentState == State.IntakePos) {
      if(AutoOn == true) {
      intakeError = getAnglePosition() - AutointakeSetpointB;
      spark1.set(-intakeError/PscaleFactor);
      }
      else {
      intakeError = getAnglePosition() - intakeSetpointB;
      spark1.set(-intakeError/PscaleFactor);
      }
      if(intakeError < 15 && intakeError > -15) {
      reversed = true;
    }
    else {
      reversed = false;
    }
    } else if (currentState == State.ShootPos) {   
      if(AutoOn == true) {   
      intakeError = getAnglePosition() - AutointakeSetpointA;
      spark1.set(-intakeError/PscaleFactor);
      }else {
      intakeError = getAnglePosition() - intakeSetpointA;
      spark1.set(-intakeError/PscaleFactor);
      }
      if(intakeError < 15 && intakeError > -15) {
      reversed = true;
    } else {
      reversed = false;
    }
  }
  else if (currentState == State.NoMovePos) {
    spark1.stopMotor();
  }
    SmartDashboard.putNumber("intake error", intakeError);
    SmartDashboard.putNumber("spark input (-1 to 1)", intakeError/180);
    SmartDashboard.putNumber("Intake Encoder Position", getAnglePosition());
    SmartDashboard.putNumber("Intake Encoder Velocity", m_intakeEncoder.getVelocity());
  }
  
  public WristIntake() {
    m_intakeEncoder.setPosition(0);
    spark1.setIdleMode(IdleMode.kBrake);
    spark1.setSmartCurrentLimit(40);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}