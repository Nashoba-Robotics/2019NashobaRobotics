package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.wpi.first.wpilibj.command.Command;

public class ElevatorZeroCommand extends NRCommand {
  public ElevatorZeroCommand() {
    super(Elevator.getInstance());
  }

  @Override
  protected void onStart() {
    Elevator.getInstance().zeroElevEncoder();
  }

  @Override
  protected boolean isFinishedNR() {
    return true;
  }

}
