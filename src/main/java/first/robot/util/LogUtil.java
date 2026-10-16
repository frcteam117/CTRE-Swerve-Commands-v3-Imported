package first.robot.util;

import java.util.ArrayList;
import java.util.List;

public class LogUtil {
  private static LogUtil instance = null;
  private final List<Runnable> updateMethods = new ArrayList<>();

  public static LogUtil getInstance() {
    if (instance == null) {
      instance = new LogUtil();
    }
    return instance;
  }

  public void registerUpdateMethod(Runnable updateMethod) {
    updateMethods.add(updateMethod);
  }

  public void runUpdateMethods() {
    for (Runnable runnable : updateMethods) {
      runnable.run();
    }
  }
}
