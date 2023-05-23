package frc.robot;

import frc.robot.util.controllers.ButtonMap;
import frc.robot.util.controllers.Logitech;
import frc.robot.util.controllers.Playstation;
import frc.robot.util.controllers.Xbox;
import frc.robot.util.controllers.Logitech.Version;

public class Config {
  public class Subsystems {
    public static final boolean SWERVE_ENABLED = true;
    public static final boolean INTAKE_MOTOR_ENABLED = true;
    public static final boolean ELEVATOR_ARM_ENABLED = true;
    public static final boolean MOTION_PROFILE_ENABLED = false;
  }

  public class Controllers {
    public static final boolean DRIVER_ENABLED = true;
    public static final boolean OPERATOR_ENABLED = true;
    public static final boolean TESTER_ENABLED = false;
  }

  public static ButtonMap getDriverController() {
    return new Playstation();
  }

  public static ButtonMap getOperatorController() {
    return new Xbox();
  }


}
