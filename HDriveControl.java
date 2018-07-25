package org.firstinspires.ftc.teamcode.FTC10251_Test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class HDriveControl {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    double scale;
    public HDriveControl(DcMotorEx left, DcMotorEx right, DcMotorEx middle){
        leftMotor = left;
        rightMotor = right;
        middleMotor = middle;
        scale = 1;
    }
    public void calculateMovement (double joystickX, double joystickY, double turnStickX, double angle) {
        double robotAngle = -angle/360 * 2 * Math.PI;
        double tempVal = joystickY * Math.cos(robotAngle) - joystickX * Math.sin(robotAngle);
        double driveMiddle = joystickX * Math.cos(-robotAngle) - joystickY * Math.sin(-robotAngle);
        double driveLeft = tempVal + (3*turnStickX/4);
        double driveRight = tempVal - (3*turnStickX/4);
        if (driveLeft > 1) {
            driveRight = driveRight * (1 / driveLeft);
            driveMiddle = driveMiddle * (1 / driveLeft);
            driveLeft = 1;
        } else if (driveRight > 1) {
            driveLeft = driveLeft * (1 / driveRight);
            driveMiddle = driveMiddle * (1 / driveRight);
            driveRight = 1;
        }
        leftMotor.setPower(scale * driveLeft);
        rightMotor.setPower(scale * driveRight);
        middleMotor.setPower(scale*-driveMiddle);
    }
}
