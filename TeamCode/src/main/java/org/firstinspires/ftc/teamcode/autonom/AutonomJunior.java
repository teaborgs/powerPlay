package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonom")
public class AutonomJunior extends LinearOpMode {
    // variabile necesare
    Autonom autonom;
    Servo claw; // ghruta
    DcMotorEx liftMotor, plateMotor; // lift, ???
    double surp; // jeg

    TrajectorySequence tryMove;
    Trajectory traj1;

    public static final double MAX_RPM = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();
        waitForStart();
        TrajectoryInitialConstructor();

        ControlArm(30); // setam unghi de pornire la brat

        // suppressWheels(); // ca sa dam rotile la viteza mica

        // asteapta oprirea din driver hub
        while (!isStopRequested()) {

            autonom.mecanumDrive.followTrajectory(traj1);
       //     autonom.mecanumDrive.followTrajectorySequence(tryMove);



            sleep(30000);
        }


    }

    //
    private void ControlArm(int unghi) {
        plateMotor.setTargetPosition(unghi);
        plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        plateMotor.setPower(0.5f);
    }

    private void robotInit() { // initierea robotolui si a variabilelor necesare
        // petru roti
        autonom.mecanumDrive = new SampleMecanumDrive(hardwareMap);
        autonom.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));

        // pentru gheara
        claw = hardwareMap.get(Servo.class, "catcherServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        claw.setPosition(.4f); // inchidem gheara
    }

    /*
    private void suppressWheels() {
        //if(gamepad1.right_bumper)
            surp = 0.5f;
        //else
        //    surp = 1f;
    }
     */

    private void TrajectoryInitialConstructor()
    {
       // tryMove = autonom.mecanumDrive.trajectorySequenceBuilder(new Pose2d(0,0)).strafeRight(69).build();

        traj1 = autonom.mecanumDrive.trajectoryBuilder(new Pose2d(0, 0)).forward(-50)
                .addDisplacementMarker(() -> {
                    liftMotor.setTargetPosition(-2650);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(0.5f);
                }).build();
    }



}
