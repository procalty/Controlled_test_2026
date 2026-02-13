
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.util.BotConstants;

public class Intake extends SubsystemBase {

  private static Intake intake = null;

  //Prevents the need of duplicate objects
  public static synchronized Intake get(){
    if(intake == null){
      intake = new Intake();
    } 
    return intake;
  }

  /** Creates a new Intake. */

  //Enum to determin state, values are temporary
  public enum State{
    IDLE(0.0),
    INTAKE(-15),
    OUTTAKE(15);

    public double roller_velocity;  // Renamed
    State(double roller_velocity){
        this.roller_velocity = roller_velocity;
    }
}
  //Enum to determin pivot position, values are temporary
  public enum Pivot{
    STOW(0.01),
    DEPLOY(-2.3);

    public double position;

    Pivot(double position){
      this.position = position;
    }
  }
  

  //Motors
  private final TalonFX m_IntakePivot = new TalonFX(BotConstants.Intake.pivotID, BotConstants.Canivore);
  private final TalonFX m_IntakeRoller = new TalonFX(BotConstants.Intake.intakeID, BotConstants.Canivore);
  //Motor Controller
  private final MotionMagicVoltage PivotPositionControl = new MotionMagicVoltage(0);
  private final VelocityVoltage intakeVelocityController  = new VelocityVoltage(0);
//   private final VelocityDutyCycle intakeVelocityController = new VelocityDutyCycle(0);
  //Variables getting the values
  private State mState = State.IDLE;
  private Pivot mPivot = Pivot.STOW;

  //Constructor, just sets up the config
  public Intake() {
    m_IntakeRoller.getConfigurator().apply(BotConstants.Intake.cfg_Roller);
    m_IntakePivot.getConfigurator().apply(BotConstants.Intake.cfg_Pivot);
    m_IntakePivot.setPosition(0.0);
  }


public Command setRollerState(State state){
    return runEnd(
        () -> mState = state,           // While running
        () -> mState = State.IDLE       // When stopped/interrupted
    );
}

public Command setPivotState(Pivot pivot){
    return runOnce(() -> mPivot = pivot);
}

// public Command intake_Command(){
//     return setPivotState(Pivot.DEPLOY).andThen(setRollerState(State.INTAKE));
// }

public Command intake_pivot(){
    return setPivotState(Pivot.DEPLOY);
}

public Command intake_run(){
    return setRollerState(State.INTAKE);
}

public Command stow(){
    return runOnce(() -> {
        mPivot = Pivot.STOW;
        mState = State.IDLE;
    });
}
public Command Outtake_Command(){
    return setPivotState(Pivot.DEPLOY).andThen(setRollerState(State.OUTTAKE));
}

@Override
public void periodic() {
    m_IntakeRoller.setControl(intakeVelocityController.withVelocity(mState.roller_velocity));
   
    m_IntakePivot.setControl(PivotPositionControl.withPosition(mPivot.position));

    SmartDashboard.putString("Pivot State", mPivot.toString());
    SmartDashboard.putString("Roller State", mState.toString());
    SmartDashboard.putNumber("Pivot Position", m_IntakePivot.getPosition().getValueAsDouble());
}
}