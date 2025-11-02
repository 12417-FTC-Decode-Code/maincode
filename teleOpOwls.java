package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="teleOpOwls", group="TeleOp")
public class teleOpOwls extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    private DcMotor intakeMotor;
    private DcMotor transferMotorLeft;

    private DcMotor outakeMotorLeft;
    private DcMotor outakeMotorRight;

    private double cfl = 1.0, cfr = 1.0, cbl = 1.0, cbr = 1.0;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotorLeft = hardwareMap.get(DcMotor.class, "transfer_motor_left");
        outakeMotorLeft = hardwareMap.get(DcMotor.class, "outake_motor_left");
        outakeMotorRight = hardwareMap.get(DcMotor.class, "outake_motor_right");

        // Typical mecanum directions (flip if needed for your build)
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

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
            // Inputs: up on sticks is positive forward
            double yleft  = -gamepad1.left_stick_y;
            double yright = -gamepad1.right_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double rotate =  gamepad1.right_stick_x; // FIX: use right stick X for rotation

            // Dual-stick forward/back "boost" term
            double forward = yleft + yright;

            // Mecanum mixing with per-wheel coefficients
            double frontLeftPower  = cfl * (forward - strafe - rotate);
            double frontRightPower = cfr * (forward + strafe + rotate);
            double backLeftPower   = cbl * (forward + strafe - rotate);
            double backRightPower  = cbr * (forward - strafe + rotate);

            // Normalize to keep within [-1, 1]
            double maxMag = Math.max(1.0,
                    Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(frontRightPower),
                                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

            frontLeftPower  /= maxMag;
            frontRightPower /= maxMag;
            backLeftPower   /= maxMag;
            backRightPower  /= maxMag;

            // Apply
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Intake
            if (gamepad1.y) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.a) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            // Transfer
            if (gamepad1.b) {
                transferMotorLeft.setPower(1.0);
            } else if (gamepad1.x) {
                transferMotorLeft.setPower(-1.0);
            } else {
                transferMotorLeft.setPower(0.0);
            }

            // Outake (dual motors)
            if (gamepad1.right_bumper) {
                outakeMotorLeft.setPower(1.0);
                outakeMotorRight.setPower(1.0);
            } else {
                outakeMotorLeft.setPower(0.0);
                outakeMotorRight.setPower(0.0);
            }

            telemetry.addData("Drive", "LF: %.2f LB: %.2f RF: %.2f RB: %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}
