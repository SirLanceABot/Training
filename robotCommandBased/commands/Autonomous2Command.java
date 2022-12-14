package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.FlywheelSubsystem;

public class Autonomous2Command extends SequentialCommandGroup
{
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }
  
  private final TestSeqGrpCommand testSeqGrpCommand = new TestSeqGrpCommand();
  private final FlywheelSubsystem m_flywheelSubsystem;
  private final SpinupFlywheelCommand testit;

  public Autonomous2Command(FlywheelSubsystem flywheelSubsystem)
  {
    m_flywheelSubsystem = flywheelSubsystem;
    addRequirements(m_flywheelSubsystem); //all subsystems that might be used in any command in the group
    testit = new SpinupFlywheelCommand(m_flywheelSubsystem, 0.);
  }

  // build command from other commands and return the single big command
  public Command get()
  {
   return new SequentialCommandGroup
        (
          new SpinupFlywheelCommand( m_flywheelSubsystem, 300. ) .withTimeout(10.)
         ,new InstantCommand(()->System.out.println("IC 1"))
         ,testSeqGrpCommand.raceWith(new WaitCommand(5.))
         ,new PrintCommand("IC 2")
         ,new SpinupFlywheelCommand( m_flywheelSubsystem, 1000. ) .withTimeout(8.)
         ,testit.spinAtSpeed(500.) .withTimeout(4.)
        );
  }
}