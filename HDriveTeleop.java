package org.firstinspires.ftc.teamcode.FTC10251_Test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name= "Basic Teleop")
public class HDriveTeleop extends OpMode{
    double angleOffset;
    BNO055IMU imu;
    Orientation angles;
    String angleDouble = "";
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    HDriveControl driveBase;
    public HDriveTeleop(){

    }
    public void init(){
        //gyro initialize
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //motor initialize
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftMotor");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "middleMotor");

        //subsystem initialize
        driveBase = new HDriveControl(leftMotor, rightMotor,middleMotor);
    }
    public void loop(){
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        float rightX = gamepad1.right_stick_x;
        float rightY = gamepad1.right_stick_y;
        float left = gamepad1.left_trigger;
        float right = gamepad1.right_trigger;
        float leftTrigger = gamepad2.left_trigger;
        float rightTrigger = gamepad2.right_trigger;
        boolean buttonAPressed = gamepad1.a;
        boolean buttonXPressed = gamepad1.x;
        boolean buttonAPressed2 = gamepad2.a;
        boolean buttonXPressed2 = gamepad2.x;
        boolean dPadUp = gamepad1.dpad_up;
        boolean dPadDown = gamepad1.dpad_down;
        boolean dPadLeft = gamepad1.dpad_left;
        boolean dPadRight = gamepad1.dpad_right;

        //HDrive Code:
        if(gamepad1.b == true){
            angleOffset = -Double.parseDouble(angleDouble);
        }
        driveBase.calculateMovement(leftX, leftY, rightX, Double.parseDouble(angleDouble) + angleOffset);
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
