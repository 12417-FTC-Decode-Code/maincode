package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Debugger", group = "Test")
public class MotorDebug extends LinearOpMode {

    // Define motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    @Override
    public void runOpMode() {
        // Initialize motors - CHANGE THESE NAMES to match your Driver Hub Config!
        leftFront  = hardwareMap.get(DcMotor.class, "front_left_motor");
        rightFront = hardwareMap.get(DcMotor.class, "front_right_motor");
        leftRear   = hardwareMap.get(DcMotor.class, "back_left_motor");
        rightRear  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Set motors to run without encoders for simple testing
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse left motors if your config usually requires it
        // Note: For this specific test, direction matters less than PORT MAPPING,
        // but keeping it consistent helps.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Waiting for start...");
        telemetry.addLine("Press buttons to test individual motors:");
        telemetry.addLine("X (Square) -> Front Left");
        telemetry.addLine("Y (Triangle) -> Front Right");
        telemetry.addLine("A (Cross)    -> Back Left");
        telemetry.addLine("B (Circle)   -> Back Right");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Stop all motors by default every loop
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Check buttons and run specific motor
            // We use 'else if' to ensure only one runs at a time

            if (gamepad1.x) {
                leftFront.setPower(0.5);
                telemetry.addData("Running Motor:", "FRONT LEFT (leftFront)");
            }
            else if (gamepad1.y) {
                rightFront.setPower(0.5);
                telemetry.addData("Running Motor:", "FRONT RIGHT (rightFront)");
            }
            else if (gamepad1.a) {
                leftRear.setPower(0.5);
                rightFront.setPower(0.5);
                leftFront.setPower(0.5);
                rightRear.setPower(0.5);
                telemetry.addData("Running Motor:", "BACK LEFT (leftRear)");
            }
            else if (gamepad1.b) {
                rightRear.setPower(0.5);
                telemetry.addData("Running Motor:", "BACK RIGHT (rightRear)");
            }
            else {
                telemetry.addLine("No buttons pressed.");
            }

            telemetry.update();
        }
    }
}
