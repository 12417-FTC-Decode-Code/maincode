package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="teleOpOwls", group="TeleOp")
public class teleopowls extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private DcMotor outakeMotorLeft;
    private DcMotor outakeMotorRight;

    private CRServo servoIntake;
    private Servo outrightservo;

    private Servo outleftservo;

    private Servo transferservo;

    // Coefficients for weight distribution adjustments
    private double cfl = 1.0;  // front left coefficient
    private double cfr = 1.0;  // front right coefficient
    private double cbl = 1.0;  // back left coefficient
    private double cbr = 1.0;  // back right coefficient

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        outakeMotorLeft = hardwareMap.get(DcMotor.class, "outake_motor_left");
        outakeMotorRight = hardwareMap.get(DcMotor.class, "outake_motor_right");

        outrightservo = hardwareMap.get(Servo.class, "outake_right_servo");
        outleftservo = hardwareMap.get(Servo.class, "outake_left_servo");
        transferservo = hardwareMap.get(Servo.class, "transfer_servo");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            // Replace your power calculation section with this
            double yleft  = -gamepad1.left_stick_y;
            double yright = -gamepad1.right_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double rotate =  -gamepad1.right_stick_x;

            double forward = yleft + yright;

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


            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            // Intake
            if (gamepad2.a) {
                intakeMotor.setPower(1.0);
            } else if (gamepad2.y) {
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

            /* double servoPower = -gamepad2.left_stick_y; // +Y up, -Y down (Raw)
            servoIntake.setPower(servoPower); */

            // Transfer
            if (gamepad1.a) {
                transferMotor.setPower(1.0);
            } else if (gamepad1.y) {
                transferMotor.setPower(-1.0);
            } else {
                transferMotor.setPower(0.0);
            }

            // Outake (dual motors)
            if (gamepad1.right_bumper) {
                outakeMotorLeft.setPower(-0.65);
                outakeMotorRight.setPower(0.65);
            } else {
                outakeMotorLeft.setPower(0.0);
                outakeMotorRight.setPower(0.0);
            }

            // Outake Ramp Servos
            if (gamepad1.dpad_up) {
                outrightservo.setPosition(1);
                outleftservo.setPosition(1);
            } else {
                outrightservo.setPosition(0);
                outleftservo.setPosition(0);
            }

            // Transfer Lever Servo
            if (gamepad2.x) {
                transferservo.setPosition(-1);
            } else if (gamepad2.b) {
                transferservo.setPosition(1);
            } else {
                transferservo.setPosition(0);
            }

            telemetry.addData("Drive", "LF: %.2f LB: %.2f RF: %.2f RB: %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}

