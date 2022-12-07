package org.firstinspires.ftc.teamcode.drive.TeleOp;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Driving")
public class TudorTeleOp extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;
    DcMotorEx liftMotor1,liftMotor2, plateMotor;
    Servo catcher;
    double suppress1;
    NormalizedRGBA colors;
    RevColorSensorV3 sensor;
    int cp1 = 0,cp2 = 0,pp = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }
    private void initialization(){
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        catcher = hardwareMap.get(Servo.class, "catcherServo");
        sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");
        sensor.enableLed(true);
        catcher.setPosition(0);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void controlWheels(){
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y*suppress1,
                        -gamepad1.left_stick_x*suppress1
                ).rotated(-poseEstimate.getHeading());
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                )
        );
    }

    private void controlArm(){
        if(gamepad2.left_stick_y!=0){
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cp1 = liftMotor1.getCurrentPosition();
            cp2 = liftMotor2.getCurrentPosition();
            liftMotor1.setPower( 1f * gamepad2.left_stick_y);
            liftMotor2.setPower( 1f * gamepad2.left_stick_y);
        }
        else {
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setPower(1f);
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setPower(1f);
        }
        if (gamepad2.right_trigger!=0 || gamepad2.left_trigger!=0){
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pp = plateMotor.getCurrentPosition();
            plateMotor.setPower ( 1f * (gamepad2.right_trigger-gamepad2.left_trigger));
        }
        else {
            plateMotor.setTargetPosition(pp);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plateMotor.setPower(1f);
        }
    }
    boolean lastPressedCatch = false;
    private void controlCatcher(){
        boolean left_bumper1_pressed = gamepad1.left_bumper;
        if(left_bumper1_pressed&&!lastPressedCatch) {
            if(catcher.getPosition()==0)
                catcher.setPosition(.4f);
            else
                catcher.setPosition(0);
        }
        lastPressedCatch = left_bumper1_pressed;
    }
    private void setPlateLevel(){
        if(gamepad2.right_bumper){
            pp = - 1000;
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            plateMotor.setTargetPosition(-1000);
        }
        else if (gamepad2.left_bumper){
            pp = 1000;
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            plateMotor.setTargetPosition(1000);
        }
        else if(gamepad2.dpad_up) {
            pp = 0;
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            plateMotor.setTargetPosition(0);
        }
    }
    private void setLiftLevel(){
        if (gamepad2.a) {
            pp = 0;
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            plateMotor.setTargetPosition(0);
            cp1 = 0;
            liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = 0;
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
        }
        else if (gamepad2.b) {
            cp1 = -702;
            liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -702;
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
        }
        else if (gamepad2.x) {
            cp1 = -1235;
            liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -1234;
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
        }
        else if (gamepad2.y) {
            cp1 = -1740;
            liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setTargetPosition(cp1);
            liftMotor1.setPower(1f);
            cp2 = -1737;
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setTargetPosition(cp2);
            liftMotor2.setPower(1f);
        }
    }

    private void suppressWheels() {
        if(gamepad1.right_bumper)
            suppress1 = 0.5f;
        else
            suppress1 = 1f;
    }

    private void colorDetect(){
        if(catcher.getPosition()==0f)
            if(sensor.red()>sensor.blue() && sensor.red()>sensor.green() && sensor.getDistance(DistanceUnit.METER)< 0.045)
                catcher.setPosition(.4f);
    }

    private void run(){
        suppressWheels();
        setPlateLevel();
        controlWheels();
        setLiftLevel();
        controlArm();
        controlCatcher();
        debugTelemetry();
        colorDetect();
    }

    private void debugTelemetry(){
        telemetry.addData("lift1", liftMotor1.getCurrentPosition());
        telemetry.addData("lift2", liftMotor2.getCurrentPosition());
        telemetry.addData("plate", plateMotor.getCurrentPosition());
        telemetry.addData("claw",catcher.getPosition());
        telemetry.addLine("Colors ").addData("red", sensor.red()).addData("green", sensor.green()).addData("blue", sensor.blue());
        telemetry.addData("colorDistance", sensor.getDistance(DistanceUnit.METER));
        telemetry.update();
    }

}
