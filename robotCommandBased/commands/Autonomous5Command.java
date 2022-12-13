package frc.robot.commands;

/**
 * Autonomous command to print awhile
 * WHAT NOT TO DO!!
 */

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous5Command extends SequentialCommandGroup
{
  static
  {
      System.out.println("Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName());
  }

  int count;
  double timeout;

  /**
   * Print a bunch of times for awhile
   * @param count
   * @param timeout
   */
  public Autonomous5Command(int count, double timeout)
  {
    this.count = count;
    this.timeout = timeout;

    addCommands( // PART A
          new InstantCommand
          //FIXME Instant commands run once to completion so most decorators don't have a chance to work
          //FIXME and it won't be interrupted.
          // If you want periodic execution code, then code the whole command with an execute() method
          // since Runnables only have the run() method (implied).
          // You have to do your own checking of when to finish.
          (
            ()->
            {
              var message = "command 5A printing " + count + " times or " + timeout + " seconds which ever expires first";
              for(int i = 1; i <= count; i++) // bad idea to have long loops
              {
                System.out.println(message + " " + i + " " + System.currentTimeMillis());
              }
            }
            /*no other subsystems required to display the message*/
          ) .withTimeout(timeout) //FIXME worse idea - this timeout does not work on InstantCommands
    );

    addCommands( // PART B
          new InstantCommand
          // Instant commands run once to completion so most decorators don't have a chance to work
          // and it won't be interrupted.
          // If you want periodic execution code, then code the whole command with an execute() method
          // since Runnables only have the run() method (implied).
          // You have to do your own checking of when to finish.
          (
            ()->
            {
              var message = "command 5B printing " + count + " times or " + timeout + " seconds which ever expires first";
              for(int i = 1; i <= count; i++) // bad idea to have long loops
              {
                System.out.println(message + " " + i + " " + System.currentTimeMillis());
              }
            }
            /*no other subsystems required to display the message*/
          ).withTimeout(timeout) //FIXME worse idea - this timeout does not work on InstantCommands
    );
  }
}
