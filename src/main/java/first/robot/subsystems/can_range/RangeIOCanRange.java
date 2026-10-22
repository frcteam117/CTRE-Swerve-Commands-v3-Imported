package first.robot.subsystems.can_range;

import com.ctre.phoenix6.hardware.CANrange;
import first.robot.generated.TunerConstants;

public class RangeIOCanRange implements RangeIO {
  CANrange canRange = new CANrange(15, TunerConstants.kCANBus1);

  public RangeIOCanRange() {}

  @Override
  public void updateInputs(RangeIOInputs inputs) {
    inputs.distance = canRange.getDistance().getValueAsDouble();
    inputs.connected = canRange.isConnected();
  }
}
