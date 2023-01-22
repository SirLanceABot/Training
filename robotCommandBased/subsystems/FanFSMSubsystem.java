/**
 * Finite State Machine for a three-speed ceiling.
 * The fan has a pull chain (Xbox button A) that switches off - high - medium - low - off.
 * The fan has an off switch (Xbox button B) that goes to off no matter the current speed.
 * The buttons only respond in teleop enabled otherwise the fan is off.
 */

package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.FanFSM.*;

public class FanFSMSubsystem  extends Subsystem4237 {

//////////////////////////////////////////////////////////////////////////////
//////////// SETUP FSM ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
  /**
   * define all the I/O to be read and written at once periodically
   * by robotPeriodic()
   */
  private PeriodicIO periodicIO;
  private XboxController driverController;
  
  @Override
  public void readPeriodicInputs()
  {
      // populate each input variable needed for states (commands) and events (triggers)
      periodicIO.AButtonPressed = driverController.getAButtonPressed(); // for trigger for next state
      periodicIO.BButtonPressed = driverController.getBButtonPressed(); // for trigger for stop
  }

  public void writePeriodicOutputs()
  {
      // act on (put out) data others have populated
      // System.out.println(periodicIO.speed);
      SmartDashboard.putNumber("fan speed", periodicIO.speed);
  } 
    
  private class PeriodicIO
  {
  // INPUTS
    private boolean AButtonPressed; // for trigger for next state
    private boolean BButtonPressed; // for trigger for stop
  // OUTPUTS
    private double speed; // fan speed
  }
  /**
   * end define I/O
   */

    // Define names of the states. States are implemented by Commands
    Command offState ;
    Command highState;
    Command medState;
    Command lowState;

    Command initialState;
    Command currentState;
    
    // the triggers (events) to change states
    public NextStateTrigger nst;
    public OffStateTrigger ost;

    static ArrayList<Trigger> triggers = new ArrayList<Trigger>(); // redundant but so much easier to loop if all are also in a list

    // Store Transitions defined for this FSM
    static ArrayList<Transition> transitions = new ArrayList<Transition>(10); // specify size as number of transitions or more

    private void initializeFSM()
    {
      // instantiate states (as Commands)
        offState = new OffState();
        highState = new HighState();
        medState = new MedState();
        lowState = new LowState();

        initialState = offState;
        currentState = initialState;

      // instantiate events (Triggers)
        nst = new NextStateTrigger();
        ost = new OffStateTrigger();
    
      // The FSM's Transition Table
      //
      // A transition is made for the first currently triggered event found for the current state;
      // Thus transition added order is important if there is more than one transition out of a state.
      //
      // Add Current State (Command), Event (Trigger), Next State (Next Command)
        transitions.add( new Transition(  offState,   nst,  highState  ) );
        transitions.add( new Transition(  highState,  nst,  medState   ) );
        transitions.add( new Transition(  medState,   nst,  lowState   ) );
        transitions.add( new Transition(  lowState,   nst,  offState   ) );

        transitions.add( new Transition(  highState,  ost,  offState   ) );
        transitions.add( new Transition(  medState,   ost,  offState   ) );
        transitions.add( new Transition(  lowState,   ost,  offState   ) );

        System.out.println( dumpFSM() );

      // schedule the Command for the initial state
      // but may not matter depending on commands and triggers that may run only ENABLED
        initialState.schedule(); // start at initialState
    }

//STATES
//STATES
//STATES

// These states all have:
// initialize() to set the speed for the state (may not be needed if an execute takes care of it soon thereafter)
// execute() to refresh the state as needed
// end() to stop the fan but the next state should override that with a new speed

// No isFinished() but maybe useful in some circumstances but can't think of any
// for this fan except maybe hard stop the FSM until a trigger revives the FSM.

// The states all run until interrupted by the scheduling of a new state.

//___________________________________________________________________________________________________________

    // OffState
    public class OffState extends CommandBase
    {
      // the command by default is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior may happen
      public OffState() { addRequirements( FanFSMSubsystem.this ); }

      @Override
      public void initialize() { periodicIO.speed = kOffSpeed; } // set the speed for the state if not in execute
      @Override
      public void execute() { periodicIO.speed = kOffSpeed; } // refresh state as needed
      @Override
      public void end( boolean interrupted ) {
         periodicIO.speed = kOffSpeed; // leaving state so turn off
         if ( DriverStation.isDisabled() ) currentState = initialState; // safety - no surprise  restarts
      }
    }    // end OffState

//___________________________________________________________________________________________________________

    public class HighState extends CommandBase
    {
      // the command by default is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior may happen
      public HighState() { addRequirements( FanFSMSubsystem.this ); }

      @Override
      public void initialize() { periodicIO.speed = kHighSpeed; } // set the speed for the state if not in execute
      @Override
      public void execute() { periodicIO.speed = kHighSpeed; } // refresh state as needed
      @Override
      public void end( boolean interrupted ) {
         periodicIO.speed = kOffSpeed; // leaving state so turn off
         if ( DriverStation.isDisabled() ) currentState = initialState; // safety - no surprise  restarts
      }
    }    // end HighState

//___________________________________________________________________________________________________________

    // MedState
    public class MedState extends CommandBase
    {
      // the command by default is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior may happen
      public MedState() { addRequirements( FanFSMSubsystem.this ); }

      @Override
      public void initialize() { periodicIO.speed = kMediumSpeed; } // set the speed for the state if not in execute
      @Override
      public void execute() { periodicIO.speed = kMediumSpeed; } // refresh state as needed
      @Override
      public void end( boolean interrupted ) { periodicIO.speed = kOffSpeed; // leaving state so turn off
        if ( DriverStation.isDisabled() ) currentState = initialState; // safety - no surprise  restarts
      }
    }    // end MedState

//___________________________________________________________________________________________________________

    // LowState
    public class LowState extends CommandBase
    {
      // the command by default is interruptible and does NOT run when DISABLED
      // but the subsystem is still running and processing inputs so slightly odd behavior may happen
      public LowState() { addRequirements( FanFSMSubsystem.this ); }

      @Override
      public void initialize() { periodicIO.speed = kLowSpeed; } // set the speed for the state if not in execute
      @Override
      public void execute() { periodicIO.speed = kLowSpeed; } // refresh state as needed
      @Override
      public void end( boolean interrupted ) {
         periodicIO.speed = kOffSpeed; // leaving state so turn off
         if ( DriverStation.isDisabled() ) currentState = initialState; // safety - no surprise  restarts
      }
    }   // end LowState

//___________________________________________________________________________________________________________
//end STATES
//end STATES
//end STATES

//TRIGGERS
//TRIGGERS
//TRIGGERS
 /**
 * Press A button to change to next state (like pulling the fan chain)
 */
public class NextStateTrigger extends Trigger
{
    NextStateTrigger()
    {
      super
      (
        () ->
          { 
            if ( DriverStation.isTeleopEnabled() )
              return FanFSMSubsystem.this.periodicIO.AButtonPressed;
            else
              return false; // ignore events (triggers) unless
          } 
      );

      triggers.add( this ); // add to the list of triggers
    }
}

/**
 * Press the B button to turn off from any state (like the off switch)
 */
public class OffStateTrigger extends Trigger
{
    OffStateTrigger()
    {
      super
      (
        () ->
          { 
            if ( DriverStation.isTeleopEnabled() )
              return FanFSMSubsystem.this.periodicIO.BButtonPressed;
            else
              return false; // ignore events (triggers) unless
          } 
      );

      triggers.add( this ); // add to the list of triggers
    }
}
//end TRIGGERS
//end TRIGGERS
//end TRIGGERS

/////////////////////////////////////////////////////////////////////////////////////////
//////// HELPER FUNCTIONS - LARGELY FSM INDEPENDENT except the class constructor name
/////////////////////////////////////////////////////////////////////////////////////////

public FanFSMSubsystem( XboxController driverController ) // pass in all the stuff this class needs from caller
  {  
      this.driverController = driverController;
      periodicIO = new PeriodicIO(); // all the inputs and outputs appear here
      initializeFSM();
  }

static
{
    System.out.println( "Loading: " + MethodHandles.lookup().lookupClass().getCanonicalName() );
}
 
@Override
public void periodic() {
  // This method will be called once per scheduler run without regard to DISABLED or ENABLED
    checkStateChange();
    }

@Override
public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
}

  /**
   * Transitions of the FSM
   * 
   * each Transition is composed of 2 independent variables - current state and event - 
   * and a dependent variable - new state
   */
  private class Transition
  {
    private Command currentState;
    private final Trigger event;
    private final Command nextState;
    
    /**
     * Create a transition that can be added to the FSM transition table
     * @param currentState - independent variable
     * @param event - independent variable
     * @param nextState - dependent variable
     */
    Transition(Command currentState, Trigger event, Command nextState)
    {
      this.currentState = currentState;
      this.event = event;
      this.nextState = nextState;
    }
  }

  /**
   * Check if a transition to the next state was triggered this iteration
   * If there is a transition then schedule the new state
   */
  public void checkStateChange()
  {
    Command newState = null;

    newState = findNextState ( currentState ); // see if a transition was triggered to the next state

    if ( newState != null ) // found in transition table
    {
      // System.out.println(newState.getClass().getSimpleName());
      currentState = newState; // transition to next state even if it's the same since it was in the transition table
      currentState.schedule(); // new state interrupts previous state, end old and init new methods are run if available
      // System.out.println("new state scheduled " + currentState.isScheduled());
    }
  }

  /**
   * Table lookup to determine new state triggered given the current state and an event.
   * If state transition is not in the transition table then return null.
   * 
   * @param currentState
   * @param event
   * @return new state or null if no transition was triggered
   */
  private static Command findNextState ( Command currentState )
  {
    for ( Transition transition : transitions ) // loop through transition table
    {
      if ( transition.currentState == currentState ) // found an entry for the current state
      {
          if ( transition.event.getAsBoolean() ) // check if this entry was triggered this iteration
          {
            // System.out.println(" transition change to " + transition.nextState.toString());
            return transition.nextState; // return the new state
          }
        }
    }
    return null; // no new state triggered
  }

  /**
   * print information about the FSM
   * @return String of information about the FSM
   */
  public String dumpFSM()
  {
    StringBuilder sb = new StringBuilder(500);
    sb.append("\nFSM Transition Table for " + this.getClass().getSimpleName() + "\nFrom State   +   Event   ->   Next State\n");
    for ( Transition transition : transitions )
    {
      sb.append( String.format("%-12s + %-12s -> %-12s\n",
        transition.currentState.getClass().getSimpleName(),
        transition.event.getClass().getSimpleName(),
        transition.nextState.getClass().getSimpleName() ) );
    }
    sb.append( "initial state: " + initialState.getClass().getSimpleName() );
    sb.append( ", current state: " + currentState.getClass().getSimpleName() + "\n" );
    
    return sb.toString();
  }
}
