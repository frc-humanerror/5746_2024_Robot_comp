// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends SubsystemBase {
  private WPI_TalonSRX climberleft = new WPI_TalonSRX(10);
  private WPI_TalonSRX climberright = new WPI_TalonSRX(9);
  edu.wpi.first.wpilibj.Encoder lclimberEncoder = new edu.wpi.first.wpilibj.Encoder(6,7);
  edu.wpi.first.wpilibj.Encoder rclimberEncoder = new edu.wpi.first.wpilibj.Encoder(4,5);
  
  /** Creates a new ExampleSubsystem. */
  public void Climb(boolean RightBumper, boolean LeftBumper) {
    if(RightBumper && getRightClimberEncoder() < 710) {
    climberleft.set(.3);
    climberright.set(-.3);
    }
    else if(LeftBumper) {
      climberleft.set(-.3);
      climberright.set(.3);
    }
    else {
      climberleft.stopMotor();
      climberright.stopMotor();
    }

    SmartDashboard.putNumber("Left Climber Encoder", getLeftClimberEncoder());
    SmartDashboard.putNumber("Right Climber Encoder", getRightClimberEncoder());
  }
  public double getLeftClimberEncoder() {
    return lclimberEncoder.getDistance()/10;
  }
  public double getRightClimberEncoder() {
    return rclimberEncoder.getDistance()/10;
  }

  public Climber() {
    lclimberEncoder.reset();
    rclimberEncoder.reset();
    climberleft.setNeutralMode(NeutralMode.Brake);
    climberright.setNeutralMode(NeutralMode.Brake);
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
