package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonomRed{
    Autonom auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone1, deliverCone2, catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5;

    public AutonomRed(Autonom auto){
        this.auto = auto;
        initializeTrajectories();
    }

    private void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .strafeRight(55)
                .back(11.5)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverPreload.end())
                .strafeLeft(2)
                .forward(30)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
                .addDisplacementMarker(() -> AutoUtil.relaxMotor(auto.liftMotor1, auto.liftMotor2))
                .build();
        deliverCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone1.end())
                .strafeRight(1.5)
                .back(31.3)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone1.end())
                .strafeRight(0.1)
                .forward(30.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE4))
                .addDisplacementMarker(() -> AutoUtil.relaxMotor(auto.liftMotor1, auto.liftMotor2))
                .build();
        deliverCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone2.end())
                .back(31.2)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone2.end())
                .strafeRight(0.3)
                .forward(30.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE3))
                .addDisplacementMarker(() -> AutoUtil.relaxMotor(auto.liftMotor1, auto.liftMotor2))
                .build();
        deliverCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone3.end())
                .back(30)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.6, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone3.end())
                .strafeRight(0.5)  ///
                .forward(31.2)
                .addTemporalMarker( 0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2))
                .addDisplacementMarker(() -> AutoUtil.relaxMotor(auto.liftMotor1, auto.liftMotor2))
                .build();
        deliverCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone4.end())
                .back(30)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.6, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverCone4.end())
                .forward(30.6)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .addDisplacementMarker(() -> AutoUtil.relaxMotor(auto.liftMotor1, auto.liftMotor2))
                .build();
        deliverCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(catchCone5.end())
                .back(31)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.6, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
    }

    public void runAuto(){
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(200);
        auto.mecanumDrive.followTrajectorySequence(catchCone1);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone1);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone2);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone2);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone3);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone3);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.mecanumDrive.followTrajectorySequence(catchCone4);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone4);
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(150);
        auto.mecanumDrive.followTrajectorySequence(catchCone5);
        auto.sleep(150);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone5);
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(30000);
    }
}
