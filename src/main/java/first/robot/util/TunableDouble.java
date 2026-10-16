package first.robot.util;

import org.wpilib.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableDouble implements DoubleSupplier {
  private final String key;
  private final BooleanSupplier shouldPublish;
  private DoubleConsumer onChange;
  private double value;
  private LoggedNetworkNumber networkNumber = null;

  public TunableDouble(
      String key, double defaultValue, BooleanSupplier shouldPublish, DoubleConsumer onChange) {
    this.key = key;
    this.shouldPublish = shouldPublish;
    this.onChange = onChange;
    value = defaultValue;
    LogUtil.getInstance().registerUpdateMethod(this::update);
  }

  public TunableDouble(String key, double defaultValue) {
    this(key, defaultValue, () -> true, (value) -> {});
  }

  public TunableDouble(String key, double defaultValue, BooleanSupplier shouldPublish) {
    this(key, defaultValue, shouldPublish, (value) -> {});
  }

  public void update() {
    if (shouldPublish.getAsBoolean()) {
      if (networkNumber == null) {
        networkNumber = new LoggedNetworkNumber(key, value);
      }
      if (value != networkNumber.get()) {
        value = networkNumber.get();
        onChange.accept(value);
      }
    } else {
      if (networkNumber != null) {
        networkNumber = null;
        NetworkTableInstance.getDefault().getDoubleTopic(key).getEntry(value).unpublish();
      }
    }
  }

  public void runOnChange(DoubleConsumer onChange) {
    this.onChange = onChange;
  }

  @Override
  public double getAsDouble() {
    return value;
  }
}
