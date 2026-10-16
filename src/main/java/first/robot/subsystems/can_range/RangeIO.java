package first.robot.subsystems.can_range;

import org.littletonrobotics.junction.AutoLog;

public interface RangeIO {
  @AutoLog
  public static class RangeIOInputs {
    public boolean connected = false;
    public double distance = 0;
  }

  public default void updateInputs(RangeIOInputs inputs) {}
}
