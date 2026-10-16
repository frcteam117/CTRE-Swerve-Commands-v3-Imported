package first.robot.subsystems.can_range;

import java.util.List;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

import first.robot.subsystems.can_range.RangeIO.RangeIOInputs;

public class RangeSubsystem implements Mechanism {
  RangeIO rangeIO = null;

  public RangeSubsystem(RangeIO rangeIO) {
    this.rangeIO = rangeIO;
  }

  public void periodic() {
    RangeIOInputs inputs = new RangeIOInputs();
    rangeIO.updateInputs(inputs);

    // List<Command> runningCommands = this.getRunningCommands();
    // for (int i = 0; i < 4; i++) {
    //   Logger.recordOutput("Range/RunningCommand_"+i,"N/A");
    // }

    // for (Command cmd : runningCommands) {
    //   Logger.recordOutput("Range/RunningCommand_"+runningCommands.indexOf(cmd),cmd.name());
    // }
  }
}
