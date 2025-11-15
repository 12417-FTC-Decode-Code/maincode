package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.util.ReadWriteFile;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// import java.io.File;
// import java.util.List;
// import java.util.Map;

@Config
@Autonomous(name = "RED GATE SIDE", group = "Autonomous")
public class owlsredgate extends LinearOpMode {

    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotorLeft;
    private DcMotorEx outtakeMotorRight;
    private DcMotorEx transferMotor;
    private CRServo intakeServo;

    // Vision variables commented out
    // private VisionPortal visionPortal;
    // private AprilTagProcessor aprilTag;
    // private int detectedTagId = -1;
    // private String detectedTagName = "UNKNOWN";
    //
    // private final Map<Integer, String> tagNames = Map.of(
    //         20, "BLUE",
    //         21, "GPP",
    //         22, "PGP",
    //         23, "PPG",
    //         24, "RED"
    // );

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("====== INITIALIZING ======");
        telemetry.update();

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outake_motor_left");
        outtakeMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outake_motor_right");
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor_left");
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        Pose2d startPose = new Pose2d(-55, 55, Math.toRadians(-45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Vision initialization commented out
        // aprilTag = new AprilTagProcessor.Builder().build();
        // visionPortal = new VisionPortal.Builder()
        //         .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        //         .addProcessor(aprilTag)
        //         .build();

        telemetry.addLine("Auto ready — All thanks to Kusa 💪");
        telemetry.addLine("✓ Initialized");
        telemetry.update();

        // ===== BUILD MOVEMENTS =====
        Action moveForward = drive.actionBuilder(startPose)
                .lineToX(-10)
                .build();

        telemetry.addLine("✓ Trajectories built");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("\n========== RUNNING ==========");
        telemetry.update();

        try {
            // ===== PHASE 1: Move forward (4 seconds) =====
            telemetry.addLine("[PHASE 1] Moving forward + Intake (4s)");
            telemetry.update();

            Actions.runBlocking(moveForward);

            // ===== PHASE 3: Outtake (2 seconds) =====
            telemetry.addLine("[PHASE 3] Running outtake (2s)");
            telemetry.update();

            outtakeMotorLeft.setPower(-0.7);
            outtakeMotorRight.setPower(0.7);
            transferMotor.setPower(-1);
            // intakeServo.setPower(1.0);
            // intakeMotor.setPower(-1.0);
            sleep(10000);

            outtakeMotorLeft.setPower(0.0);
            outtakeMotorRight.setPower(0.0);
            transferMotor.setPower(0);
            // intakeServo.setPower(0.0);
            // intakeMotor.setPower(0.0);

            // ===== PHASE 4: Execute tag-specific path =====
            // COMMENTED OUT FOR NOW - USING FALLBACK PATH ONLY

            telemetry.addLine("[PHASE 4] Executing fallback path (no vision)");
            telemetry.update();

            Pose2d currentPose = drive.localizer.getPose();

            // Fallback path (default)
            telemetry.addLine("Running default fallback path");
            telemetry.update();

            // Spline movement
            Actions.runBlocking(
                    drive.actionBuilder(currentPose)
                            .splineTo(new Vector2d(-25,50), Math.toRadians(0))
                            .build()
            );

            telemetry.addLine("✓ Phase 4 Complete");
            telemetry.update();

            telemetry.addData("Status", "✓ COMPLETE");

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
        }
        telemetry.update();

        // Stop all motors
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        outtakeMotorLeft.setPower(0);
        outtakeMotorRight.setPower(0);
        intakeServo.setPower(0);

        // Vision cleanup commented out
        // if (visionPortal != null) visionPortal.close();

        Pose2d finalPose = drive.localizer.getPose();
        telemetry.addData("Final X", String.format("%.2f", finalPose.position.x));
        telemetry.addData("Final Y", String.format("%.2f", finalPose.position.y));
        telemetry.addData("Final Heading (deg)", String.format("%.2f", Math.toDegrees(finalPose.heading.toDouble())));
        telemetry.addLine("\n========== AUTO COMPLETE ==========");
        telemetry.update();
    }
}