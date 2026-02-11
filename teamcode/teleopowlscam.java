package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;

@TeleOp(name="teleOpOwls (CAM FIXED)", group="TeleOp")
public class teleopowlscam extends LinearOpMode {

    // --- CONFIGURATION ---
    // If camera is to the RIGHT of robot center, make this POSITIVE (e.g., 5.0 inches)
    // If camera is to the LEFT, make this NEGATIVE.
    private static final double CAMERA_OFFSET_X_INCHES = 0.0;

    // Motor Declarations
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intakeMotor, transferMotor, outakeMotorLeft, outakeMotorRight;

    // Servo Declarations
    private CRServo servoIntake;
    private Servo rampServo, lever;

    // Vision Variables
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final int GOAL_TAG_ID = 24;

    // Alignment State
    private boolean isAutoAligning = false;

    // Coefficients
    private double cfl = 1.0, cfr = 1.0, cbl = 1.0, cbr = 1.0;

    @Override
    public void runOpMode() {
        // --- 1. HARDWARE MAPPING ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_m");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_m");
        outakeMotorLeft = hardwareMap.get(DcMotor.class, "outake_l");
        outakeMotorRight = hardwareMap.get(DcMotor.class, "outake_r");

        rampServo = hardwareMap.get(Servo.class, "ramp_servo");
        lever = hardwareMap.get(Servo.class, "lever_servo");

        // Directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Brakes
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeVision();

        telemetry.addData("Status", "Initialized");
        lever.setPosition(0);
        telemetry.update();

        waitForStart();

        // --- 2. MAIN LOOP ---
        while (opModeIsActive()) {

            // --- DRIVE INPUTS ---
            double yleft  = gamepad1.left_stick_y;
            double yright = gamepad1.right_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Check for manual override (if driver touches sticks, cancel auto-align)
            if (Math.abs(yleft) > 0.1 || Math.abs(yright) > 0.1 || Math.abs(strafe) > 0.1 || Math.abs(rotate) > 0.1) {
                isAutoAligning = false;
            }

            // Toggle Auto-Align with 'X'
            // We use a simple latch logic or just set true when holding,
            // here simply pressing X enables the mode until aligned or cancelled.
            if (gamepad1.x) {
                isAutoAligning = true;
            }

            // --- DRIVE LOGIC ---
            if (isAutoAligning) {
                // RUN AUTO ALIGN LOGIC (Non-blocking)
                runAutoAlign();
            } else {
                // RUN MANUAL DRIVE LOGIC
                runManualDrive(yleft, yright, strafe, rotate);
            }

            // --- MECHANISMS (Now work continuously!) ---
            runMechanisms();

            telemetry.addData("Mode", isAutoAligning ? "AUTO-ALIGNING" : "MANUAL");
            telemetry.update();
        }
    }

    // --- LOGIC METHODS ---

    private void runManualDrive(double yleft, double yright, double strafe, double rotate) {
        double speedMultiplier = getSpeedMultiplier();
        double forward = (-yleft - yright) * speedMultiplier;

        double frontLeftPower  = cfl * (forward + strafe - rotate);
        double frontRightPower = cfr * (forward - strafe + rotate);
        double backLeftPower   = cbl * (forward - strafe - rotate);
        double backRightPower  = cbr * (forward + strafe + rotate);

        // Normalize
        double maxMag = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftDrive.setPower(frontLeftPower / maxMag);
        frontRightDrive.setPower(frontRightPower / maxMag);
        backLeftDrive.setPower(backLeftPower / maxMag);
        backRightDrive.setPower(backRightPower / maxMag);
    }

    private void runAutoAlign() {
        // 1. Vision Parameters
        // CHANGED: Gain set to negative.
        // If it was "running away", reversing this makes it run "toward".
        final double BEARING_GAIN = 0.01;

        final double MIN_TURN_POWER = 0.15; // Lowered slightly for smoother start
        final double MAX_TURN_POWER = 0.45;
        final double BEARING_TOLERANCE = 1.0; // Tighter tolerance for better aim

        AprilTagDetection targetTag = null;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Find the tag
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == GOAL_TAG_ID) {
                targetTag = detection;
                break;
            }
        }

        if (targetTag == null) {
            telemetry.addData("Align", "Tag Lost");
            stopDrivetrain();
            // Optional: Uncomment below to cancel auto-align if tag is lost
            // isAutoAligning = false;
            return;
        }

        // 2. Calculate Bearing (Angle to tag)
        double range = targetTag.ftcPose.range;
        double bearing = targetTag.ftcPose.bearing;

        // --- OFFSET CALCULATION FIX ---
        // Basic Trig:
        // If Camera is to the RIGHT of Robot Center (Positive Offset),
        // The Robot Center is to the LEFT of the Camera.
        // Therefore, the Target is further to the RIGHT relative to the Robot Center.
        // We should SUBTRACT the angle if the Camera is Right.
        double offsetAdjustment = Math.toDegrees(Math.atan2(6.25, range));

        // This math assumes standard FTC coordinate systems
        double adjustedBearing = bearing - offsetAdjustment;

        telemetry.addData("Align", "Err: %.2f", adjustedBearing);

        // 3. Check Alignment
        if (Math.abs(adjustedBearing) < BEARING_TOLERANCE) {
            stopDrivetrain();
            isAutoAligning = false; // We are done!
            telemetry.addData("Align", "DONE");
            return;
        }

        // 4. Calculate Turn Power
        double turnPower = adjustedBearing * BEARING_GAIN;

        // Apply Minimum Power (Friction Feedforward)
        // Only apply if the calculated power is too low to move the robot
        if (Math.abs(turnPower) < MIN_TURN_POWER) {
            turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
        }

        // Clamp Maximum Power
        turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));

        // 5. Apply to Motors
        setDrivetrainTurn(turnPower);
    }

    private void runMechanisms() {
        // Intake
        if (gamepad2.a) intakeMotor.setPower(1.0);
        else if (gamepad2.b) intakeMotor.setPower(-1.0);
        else intakeMotor.setPower(0.0);

        // Transfer
        if (gamepad2.y) transferMotor.setPower(0.7);
        else if (gamepad2.x) transferMotor.setPower(-0.7);
        else transferMotor.setPower(0.0);

        // Ramp
        if (gamepad2.dpad_up) rampServo.setPosition(0.5);
        else if (gamepad2.dpad_down) rampServo.setPosition(0);

        // Lever (Non-blocking timer logic could be added here, currently simple toggle)
        if (gamepad1.dpad_right) {
            lever.setPosition(1.0);
            sleep(1500);
            lever.setPosition(0.0);
        } else if (gamepad1.dpad_left) {
            lever.setPosition(0.0);
        }

        // Outtake
        if (gamepad1.right_bumper) {
            outakeMotorLeft.setPower(0.58);
            outakeMotorRight.setPower(-0.58);
        } else if (gamepad1.left_bumper) {
            outakeMotorLeft.setPower(0.7);
            outakeMotorRight.setPower(-0.7);
        } else {
            outakeMotorLeft.setPower(0.0);
            outakeMotorRight.setPower(0.0);
        }
    }

    // --- HELPERS ---

    private double getSpeedMultiplier() {
        double leftY = Math.abs(gamepad1.left_stick_y);
        double rightY = Math.abs(gamepad1.right_stick_y);
        if (leftY > 0.5 && rightY > 0.5) return 1.0;
        else return 0.6;
    }

    private void setDrivetrainTurn(double turnPower) {
        // To Turn RIGHT (Positive Power): Left side Forward, Right side Backward
        frontLeftDrive.setPower(turnPower);
        backLeftDrive.setPower(turnPower);
        frontRightDrive.setPower(-turnPower);
        backRightDrive.setPower(-turnPower);
    }

    private void stopDrivetrain() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void initializeVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}