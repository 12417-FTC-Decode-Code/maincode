package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "VisionPortal AprilTag Demo")
public class VisionPortalAprilTagDemo extends LinearOpMode {

    private VisionPortal portal;
    private AprilTagProcessor tag;

    // runtime knobs
    private int decimation = 2;            // 1 = more detail, 2–3 = faster
    private boolean streaming = true;
    private boolean exposureLocked = false;

    @Override
    public void runOpMode() {
        // 1) Build AprilTag processor: use the current game's tag library.

        tag.setDecimation(decimation);

        // 2) Build VisionPortal
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))   // must match RC config
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tag)
                .build();

        telemetry.addLine("Init OK. A=toggle stream  X/B=decimation  Y=lock exposure");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> dets = tag.getDetections();
            if (!dets.isEmpty()) {
                AprilTagDetection d = dets.get(0);
                telemetry.addData("Detections", dets.size());
                telemetry.addData("ID", d.id);

                if (d.ftcPose != null) {
                    telemetry.addData("Range",   "%.1f in", d.ftcPose.range);
                    telemetry.addData("Bearing", "%.1f deg", d.ftcPose.bearing);
                    telemetry.addData("Yaw",     "%.1f deg", d.ftcPose.yaw);
                    telemetry.addData("X,Y,Z",   "(%.1f, %.1f, %.1f) in",
                            d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);

                    // Example drive suggestions
                    double turnCmd = 0.02 * d.ftcPose.bearing;
                    double fwdCmd  = 0.05 * Math.max(0, d.ftcPose.range - 12);
                    telemetry.addData("Drive", "fwd=%.2f turn=%.2f", fwdCmd, turnCmd);
                } else {
                    telemetry.addLine("Seen tag, but no pose. Check tag library and lighting.");
                }
            } else {
                telemetry.addLine("No tag");
            }

            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.addData("Decimation", decimation);
            telemetry.addData("Streaming", streaming ? "on" : "off");
            telemetry.addData("Exposure locked", exposureLocked);
            telemetry.update();

            // Controls
            if (gamepad1.a) {
                if (streaming) portal.stopStreaming(); else portal.resumeStreaming();
                streaming = !streaming;
                sleep(200);
            }
            if (gamepad1.x) {
                decimation = Math.max(1, decimation - 1);
                tag.setDecimation(decimation);
                sleep(200);
            }
            if (gamepad1.b) {
                decimation = Math.min(3, decimation + 1);
                tag.setDecimation(decimation);
                sleep(200);
            }
            if (gamepad1.y && !exposureLocked) {
                lockExposureAndWhiteBalance();
                exposureLocked = true;
                sleep(200);
            }

            idle();
        }

        portal.close();
    }

    private void lockExposureAndWhiteBalance() {
        try {
            ExposureControl exp = portal.getCameraControl(ExposureControl.class);
            if (exp != null) {
                exp.setMode(ExposureControl.Mode.Manual);
                exp.setExposure(12, TimeUnit.MILLISECONDS); // tune on-field
            }
            WhiteBalanceControl wb = portal.getCameraControl(WhiteBalanceControl.class);
            if (wb != null) {
                wb.setMode(WhiteBalanceControl.Mode.MANUAL);
                wb.setWhiteBalanceTemperature(4000); // try 3500–5000 under event lights
            }
        } catch (Throwable ignored) {
            // not all webcams support programmatic control. It’s fine.
        }
    }
}