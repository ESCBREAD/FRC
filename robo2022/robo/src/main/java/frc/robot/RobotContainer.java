package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//add by yaoxing, joystick controller
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;


public class RobotContainer {
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

    private final XboxController controller = new XboxController(0);
     private final Joystick controllerJ = new Joystick(0);

    public RobotContainer() {
        drivetrain.register();

        // drivetrain.setDefaultCommand(new DriveCommand(
        //         drivetrain,
        //         () -> -modifyAxis(controller.getLeftY()), // Axes are flipped here on purpose
        //         () -> -modifyAxis(controller.getLeftX()),
        //         () -> -modifyAxis(controller.getRightX())
        // ));

        // new Button(controller::getBackButtonPressed)
        //         .whenPressed(drivetrain::zeroGyroscope);

        
        //modify by yaoxing, joystick controller
        drivetrain.setDefaultCommand(new DriveCommand(
                drivetrain,
                () -> -modifyAxis(controllerJ.getY()), // Axes are flipped here on purpose
                () -> -modifyAxis(controllerJ.getX()),
                () -> -modifyAxis(controllerJ.getZ())
        ));
        
        
        new Button(controllerJ::getTriggerPressed)
                .whenPressed(drivetrain::zeroGyroscope);


//System.out.printf("@@@@@@ getLeftY: %f   getLeftX: %f   getRightX: %f    %n", controller.getLeftY() , controller.getLeftX(),  controller.getRightX() );
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
