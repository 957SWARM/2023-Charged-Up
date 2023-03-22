package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DroneDrive {

    double joystickAngle = 0;
    double roundedAngle = 0;
    double previousOutput = 0;
    //double robotAngle = -425; //manually setting robot angle

    private double convertToUnitCircleAngle(double joystickAngle) {
        double unitCircleAngle = 90 - joystickAngle;
        if (unitCircleAngle < 0) {
            unitCircleAngle += 360;
        }
        return unitCircleAngle;
    }

    private double findClosestPath(double robotAngle, double targetAngle){

        double diff = Math.toRadians(robotAngle - targetAngle);
        double closestPath = Math.round(Math.toDegrees(Math.atan2(Math.sin(diff), Math.cos(diff))));

        return closestPath;
    }

    public double calculateAngle(double x, double y, double robotAngle){ 
        //find joystick angle maybee??
        if (0.7 < Math.abs(x) || 0.7 < Math.abs(y)) {
			joystickAngle = Math.toDegrees(Math.atan2(x, -y));
            roundedAngle = Math.round(joystickAngle / 90) * 90;
		}
        double newJoyAngle = convertToUnitCircleAngle(roundedAngle);
        
        double remainder = (newJoyAngle + (robotAngle * 360)) % 360;
        int multiplier = (int)robotAngle / 360;
        double targetAngle = remainder + (360 * multiplier);

        double closestPath = findClosestPath(robotAngle, targetAngle);
        /*if(closestPath > 0){
            System.out.print("closest path: " + Math.abs(closestPath) + " clockwise || ");
        }
        else{
            System.out.print("closest path: " + Math.abs(closestPath) + " counter-clockwise || ");
        }

        System.out.print(" robotAngle " + robotAngle + " || "); */

        //previousOutput = targetAngle;
        return closestPath;
    }
}
