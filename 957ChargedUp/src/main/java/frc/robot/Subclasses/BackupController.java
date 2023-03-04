package frc.robot.Subclasses;

import edu.wpi.first.wpilibj.Joystick;

public class BackupController extends Joystick{

    public BackupController(int port) {
        super(port);
    }
    
    final int leftStickY = 1;
    final int leftStickX = 0;
    final int rightStickY = 5;
    final int rightStickX = 4;
}
