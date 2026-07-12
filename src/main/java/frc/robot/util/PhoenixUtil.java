package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

public class PhoenixUtil {
  /**
   * Attempts to run the command until no error is produced, up to
   * maxAttempts. CAN config calls fail transiently at startup (device still
   * booting, bus busy), so a few retries make configuration reliable.
   *
   * <p>NOTE: gives up SILENTLY after maxAttempts — the device may be left
   * unconfigured with no log trace. Worth revisiting (log/alert on failure).
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }
}