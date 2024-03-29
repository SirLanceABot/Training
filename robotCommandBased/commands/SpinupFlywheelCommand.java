package frc.robot.commands;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.FlywheelSubsystem;

public class SpinupFlywheelCommand extends CommandBase {
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  private FlywheelSubsystem flywheelSubsystem;
  private double speed;
  private boolean flywheelHappy = true; // happy is the base state

  // command does not run if DISABLED
  
  public SpinupFlywheelCommand(FlywheelSubsystem flywheelSubsystem, double speed)
    {
      this.flywheelSubsystem = flywheelSubsystem;
      System.out.println("Spin Flywheel to speed command " + speed);
      this.speed = speed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        flywheelHappy = true;
        System.out.println("SpinupFlywheel speed " + speed);   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
      {
        System.out.println("SpinupFlywheelCommand execute " + speed);
        flywheelSubsystem.setFlywheelSpeed.accept(speed);
      }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) // interrupted true for interrupted or false if the isFinished had been set to true
      {
        System.out.println(
          "end SpinupFlywheel speed " + speed + (interrupted?" interrupted or cancelled - I'm dead":" stopped"));
        flywheelSubsystem.setFlywheelPctVBus.accept(0.); // stop motor
        flywheelHappy = false; // return code
      }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() { // stub for now
      // if(at speed) return true;
      // else return false;
      return false; // keep running (until others cancel command)
    }

    public SpinupFlywheelCommand spinAtSpeed(double speed)
    {
      this.speed = speed;
      return this;
    }

    public BooleanSupplier isFlywheelHappy()
    {
      return () -> flywheelHappy; // supply return code
    }
}
