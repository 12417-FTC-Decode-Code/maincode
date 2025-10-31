
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Driving", group="12417")

public class TestTele extends LinearOpMode {

    //declaration
    //DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;
    DcMotor Slide;
    //CRServo slide;
    //DcMotor slide;
    CRServo sweep;


    int FREEZE_SPEED=0;
//    private float cfl = 2.7f;
//    private float cfr = 1.5f;
//    private float cbl = 3.0f;
//    private float cbr = 3.0f;
    private float cfl = 0.2f;
    private float cfr = 0.2f; //was 0.8f
    private float cbl = 0.2f; //was 1.3f
    private float cbr = 0.2f; //was 1.4f

    @Override

    public void runOpMode() throws InterruptedException {
        //initialization
        FRMotor = hardwareMap.dcMotor.get("FR_Motor");
        FLMotor = hardwareMap.dcMotor.get("FL_Motor");
        BLMotor = hardwareMap.dcMotor.get("BL_Motor");
        BRMotor = hardwareMap.dcMotor.get("BR_Motor");

        Slide = hardwareMap.dcMotor.get("S_Motor");
        //slide = hardwareMap.crservo.get("slide");
        //slide = hardwareMap.dcMotor.get("slide");
        sweep = hardwareMap.crservo.get("sweep");


//speeds
        float SPwr=1.5f, APwr, FLPwr,FRPwr,BLPwr,BRPwr,BPwr=10f;

//booleans
        boolean isBeamBroke;
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE); // Forward initially
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);



//let's gooo
//
// ooooooooooooeooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
        waitForStart();
//        Slide.setPower(-0.5f);
//        sleep(2000);
//        Slide.setPower(0);

        while (opModeIsActive()) {
            //slide.setPower(10);
            DcMotor[] motors = {FRMotor, FLMotor, BLMotor, BRMotor/*,Slide*/};
            for (int i = 0; i < 4; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               //   motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

//            sweepRight.setDirection(DcMotor.Direction.FORWARD);
//            sweepLeft.setDirection(DcMotor.Direction.FORWARD);

            float yleft, yright,strafe,rotate;

            yleft = gamepad1.left_stick_y;
            yright = gamepad1.right_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

//            FLPwr = (cfl)*(yleft + yright - strafe + rotate)/4f; //cfl for possible coeffs to deal with weight distr
//            FRPwr = (cfr)*(yleft + yright + strafe - rotate)/4f; //cfr for possible coeffs to deal with weight distr
//            BLPwr = (cbl)*(yleft + yright + strafe + rotate)/4f; //cbl for possible coeffs to deal with weight distr
//            BRPwr = (cbr)*(yleft + yright - strafe - rotate)/4f; //cbr for possible coeffs to deal with weight distr
            FLPwr= cfl*(yleft+yright-strafe-rotate);
            FRPwr= cfr*(yleft+yright+strafe+rotate);
            BLPwr= cbl*(yleft+yright+strafe-rotate);
            BRPwr= cbr*(yleft+yright-strafe+rotate);

            FLMotor.setPower(FLPwr);
            FRMotor.setPower(FRPwr);
            BLMotor.setPower(BLPwr);
            BRMotor.setPower(BRPwr);



            if(gamepad1.left_trigger>0.1)
            {
                Slide.setTargetPosition(-200);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.3);
                telemetry.addData("NO", Slide.getCurrentPosition());
                telemetry.update();

            }
            if(gamepad1.right_trigger>0.1)
            {
                Slide.setTargetPosition(0);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.3);
                telemetry.addData("NO", Slide.getCurrentPosition());
                telemetry.update();
            }



            if(gamepad1.left_bumper)
            {
                Slide.setTargetPosition(-500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(-0.3);
                telemetry.addData("NO", Slide.getCurrentPosition());
                telemetry.update();
                //Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               // Slide.setPower(0);

            }
            if(gamepad1.right_bumper)
            {

                Slide.setTargetPosition(-1100);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(-0.3);
                //Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // Slide.setPower(0);

            }
            if(gamepad1.dpad_down)
            {
                Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER  );
                Slide.setPower(0.3);
                telemetry.addData("STOP PLEASE WORK ALREADY", "??");
                telemetry.update();
            }
            while(gamepad1.y)
            {
                //slide.setPower(1);
            }
            while(gamepad1.a)
            {
                //slide.setPower(-1);
                sleep(500);
            }
            while(gamepad1.b)
            {
                sweep.setPower(0.5);
                telemetry.addData("hello", "hello");
                telemetry.update();
            }
            while(gamepad1.x)
            {
                sweep.setPower(-0.5);
                telemetry.addData("goodbye", "goodbye");
                telemetry.update();
            }
//            Slide.setTargetPosition(Slide.getCurrentPosition());
//            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Slide.setPower(-0.9);


           // Slide.setPower(0);
            //slide.setPower(0);
            sweep.setPower(0);
            //slide.setPower(0);
            //sweep.setPower(0   );
            telemetry.addData("BR", BRMotor.getPower());
            telemetry.addData("BL", BLMotor.getPower());
            telemetry.addData("FR", FRMotor.getPower());
            telemetry.addData("FL", FLMotor.getPower());
            telemetry.addData("Slide", Slide.getCurrentPosition());
            telemetry.update();
        }

    }

}