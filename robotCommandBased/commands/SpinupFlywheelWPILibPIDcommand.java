package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.FlywheelSubsystem;

public class SpinupFlywheelWPILibPIDcommand extends PIDCommand {

  /**
   * Turns to robot to the specified angle.
   *
   * @param setpoint The velocity to achieve
   * @param drive The drive subsystem to use
   */
  public SpinupFlywheelWPILibPIDcommand(double setpoint, FlywheelSubsystem drive) {
      
    super(
        new PIDController(kP, kI, kD, 1.), //PIDController controller

        ()->
        {
            double speed = drive.getFlywheelSpeed.get();
            System.out.println("current speed " + speed);
            return speed;
        }, // Close loop on speed DoubleSupplier measurementSource

         setpoint, // Set reference to target double or DoubleSupplier setpointSource

        output -> {System.out.println("output " + output);drive.setFlywheelPctVBus.accept(output+kF*setpoint);}, // Pipe output to spin motor DoubleConsumer useOutput

        (Subsystem)drive);        // Require the drive Subsystem... requirements

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(kPositionTolerance, kVelocityTolerance);
  }

  @Override
  public boolean isFinished() {
    // // End when the controller is at the reference.
    // System.out.println("WPILib PID isFinished " + getController().atSetpoint());
    System.out.println(getController().getVelocityError() + " " + getController().getPositionError());
    // return getController().atSetpoint();
    return false;
  }

    public static final double kP = 0.00005;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.000051;

    public static final double kPositionTolerance = 20.;
    public static final double kVelocityTolerance = Double.POSITIVE_INFINITY;

}