package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name="Mecanum Drive Example", group="Examples")
public class MyFIRSTJavaOpMode extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    ColorSensor color1;
    DistanceSensor distance1;
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        color1 = hardwareMap.get(ColorSensor.class, "color1");
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {


            driveMecanum(0, 1, 0);  //1 = foward, -1 = back, 0 = stop
            sleep(2000);


            driveMecanum(0, 0, 0);
            sleep(1000);


            driveMecanum(0.5, 0, 0); //first is right, second is both, last is left
            sleep(676);


            driveMecanum(0, 0, 0);
        }
    }


    private void driveMecanum(double x, double y, double rotation) {
        double frontLeftPower = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;


        double max = Math.max(1.0, Math.max(
        Math.abs(frontLeftPower),
        Math.max(Math.abs(frontRightPower),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))
        ));

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);
    }
}
