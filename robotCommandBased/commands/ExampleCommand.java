package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {

  private final int commandID;
  private int printCount;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem, int commandID) {
    this.commandID = commandID;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    System.out.println("created ExampleCommand " + commandID);
  }

/**
 * example of overriding the default method that normally returns false
 */
  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
   {
     System.out.println(" commandID " + commandID + " initializing");
     printCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    printCount++;
    System.out.println(" commandID " + commandID + " printed " + printCount);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) // interrupted true for interrupted or false if the isFinished had been set to true
  {
    System.out.println(" commandID " + commandID + " ended by interrupted " + interrupted);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(printCount >= 100)
    { System.out.println(" commandID " + commandID + " printed limit reached so isFinished");
      return true;
    }
    else return false;
  }
}

/*
cycle commands from
https://raw.githubusercontent.com/FRC-Team-1405/TankDrive/master/src/main/java/frc/robot/lib/CycleCommands.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class CycleCommands {
    private final CommandBase[] commands;
    private final String label;
    private int activeCommand=0;
    public CycleCommands(String label, CommandBase[] commands, JoystickButton previous, JoystickButton next){
        this.label=label;
        this.commands=commands;
        previous.whenPressed( this::previousCommand );
        next.whenPressed( this::previousCommand ); //FIXME ?? nextCommand??
        SmartDashboard.putString(label, commands[activeCommand].getName());
        commands[activeCommand].schedule();
    }

    private void nextCommand(){
        activeCommand += 1;
        if (activeCommand >= commands.length){
            activeCommand=0;
        }
        SmartDashboard.putString(label, commands[activeCommand].getName());
        commands[activeCommand].schedule();
    }

    private void previousCommand(){
        activeCommand -= 1;
        if (activeCommand <0 ){
            activeCommand = commands.length-1;
        }
        SmartDashboard.putString(label, commands[activeCommand].getName());
        commands[activeCommand].schedule();
    }
}*/