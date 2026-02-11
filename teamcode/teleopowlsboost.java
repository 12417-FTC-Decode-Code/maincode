package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="teleOpOwls (BOOST)", group="TeleOp")
public class teleopowlsboost extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private DcMotor outakeMotorLeft;
    private DcMotor outakeMotorRight;

    private Servo rampServo;

    private Servo lever;

    private double cfl = 1.0;
    private double cfr = 1.0;
    private double cbl = 1.0;
    private double cbr = 1.0;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Initialize other motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_m");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_m");
        outakeMotorLeft = hardwareMap.get(DcMotor.class, "outake_l");
        outakeMotorRight = hardwareMap.get(DcMotor.class, "outake_r");

        rampServo = hardwareMap.get(Servo.class, "ramp_servo");
        lever = hardwareMap.get(Servo.class, "lever_servo");

        /* Servo transServo = hardwareMap.get(Servo.class, "intake_servo");
        Servo outrightservo = hardwareMap.get(Servo.class, "outake_right_servo");
        Servo outleftServo = hardwareMap.get(Servo.class, "outake_left_servo"); */

        // Set motor directions for mecanum drive
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set run modes
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Optional: Set brake behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double yleft  = gamepad1.left_stick_y;
            double yright = gamepad1.right_stick_y;
            double strafe =  -gamepad1.left_stick_x;
            double rotate =  gamepad1.right_stick_x;

            double speedMultiplier = 0.6;

            double forward = (-yleft - yright) * speedMultiplier;

            double frontLeftPower  = cfl * (forward + strafe - rotate);
            double frontRightPower = cfr * (forward - strafe + rotate);
            double backLeftPower   = cbl * (forward - strafe - rotate);
            double backRightPower  = cbr * (forward + strafe + rotate);

            double maxMag = Math.max(1.0,
                    Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(frontRightPower),
                                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

            frontLeftPower  /= maxMag;
            frontRightPower /= maxMag;
            backLeftPower   /= maxMag;
            backRightPower  /= maxMag;

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);



            /* frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad */


            // Apply powers to drive motors
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            // Intake
            if (gamepad2.a) {
                intakeMotor.setPower(1.0);
            } else if (gamepad2.b) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            /* if (gamepad2.right_bumper) { //COMMENT OUT START
                servoIntake.setPower(1.0);
            } else if (gamepad2.left_bumper) {
                servoIntake.setPower(-1.0);
            } else {
                servoIntake.setPower(0.0);
            } */

           /*double servoPower = -gamepad2.left_stick_y; // +Y up, -Y down (Raw)
            servoIntake.setPower(servoPower); */

            // Transfer
            if (gamepad2.y) {
                transferMotor.setPower(0.7);
            } else if (gamepad2.x) {
                transferMotor.setPower(-0.7);
            } else {
                transferMotor.setPower(0.0);
            }

            // Outake (dual motors)
            /* if (gamepad1.right_bumper) {
                outakeMotorLeft.setPower(0.65);
                outakeMotorRight.setPower(-0.65);
            } else {
                outakeMotorLeft.setPower(0.0);
                outakeMotorRight.setPower(0.0);
            } */

            // Ramp Servo
            if (gamepad2.dpad_up) {
                rampServo.setPosition(0.75);
            } else if (gamepad2.dpad_down) {
                rampServo.setPosition(0);
            }

            if (gamepad1.dpad_right) {
                lever.setPosition(1.0);
                sleep(1500);
                lever.setPosition(0.0);
            } else if (gamepad1.dpad_left) {
                lever.setPosition(0.0);
            }

            if (gamepad1.right_bumper) {
                outakeMotorLeft.setPower(0.65);
                outakeMotorRight.setPower(-0.65);
            } else if (gamepad1.left_bumper) {
                outakeMotorLeft.setPower(0.75);
                outakeMotorRight.setPower(-0.75);
            } else {
                outakeMotorLeft.setPower(0.0);
                outakeMotorRight.setPower(0.0);
            }

            telemetry.addData("Drive", "LF: %.2f LB: %.2f RF: %.2f RB: %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }

    /* private double getSpeedMultiplier() {
        double leftY = Math.abs(gamepad1.left_stick_y);
        double rightY = Math.abs(gamepad1.right_stick_y);

        // Both joysticks pushed up significantly
        if (leftY > 0.5 && rightY > 0.5) {
            return 1.0;
        }
        // Only one joystick pushed
        else if (leftY > 0.5 || rightY > 0.5) {
            return 0.6;
        }
        // Neither joystick pushed significantly
        else {
            return 0.0;
        }
    } */
}

