package frc.robot;

import frc.robot.layout.TwoJoyStickDriverMap;
import frc.robot.layout.TwoJoyStickOperatorMap;

import frc.robot.util.controllers.GameController;
import frc.robot.Config.Controllers;

public class OI {
  private static OI instance;

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private GameController driver;

  public GameController getDriver() {
    return driver;
  }

  private GameController operator;

  public GameController getOperator() {
    return operator;
  }

  private GameController tester;

  public GameController getTester() {
    return tester;
  }

  public void registerCommands() {
    if (Controllers.DRIVER_ENABLED) new TwoJoyStickDriverMap(driver).registerCommands();
    if (Controllers.OPERATOR_ENABLED) new TwoJoyStickOperatorMap(operator).registerCommands();
   
  }

  private OI() {
    driver = new GameController(
        RobotMap.ControllerMap.DRIVER_JOYSTICK, Config.getDriverController());
    operator = new GameController(RobotMap.ControllerMap.OPERATOR_JOYSTICK, Config.getOperatorController());
  }
}
