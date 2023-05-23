package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.ButtonMap.Trigger;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    var multiplier = (getRightTrigger() > 0 ) ? 0.3 : 1;
    var x = Math.pow(controller.getAxis(Axis.AXIS_LEFT_X), 1) * RobotMap.DriveMap.MAX_VELOCITY * multiplier;
    var y = Math.pow(controller.getAxis(Axis.AXIS_LEFT_Y), 1) * RobotMap.DriveMap.MAX_VELOCITY * multiplier;
    var rot = controller.getAxis(Axis.AXIS_RIGHT_X) * RobotMap.DriveMap.MAX_ANGULAR_VELOCITY * 0.7 * multiplier;

    var swerve = Swerve.getInstance();
    return ChassisSpeeds.fromFieldRelativeSpeeds(-y, -x, -rot, swerve.getYaw());
  }

  @Override
  public double getRightTrigger(){
    return controller.getAxis(Axis.AXIS_RIGHT_TRIGGER);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
