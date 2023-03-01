package org.firstinspires.ftc.teamcode.autonom.Traiectorii;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.AutoUtil;
import org.firstinspires.ftc.teamcode.autonom.AutonomDreaptaHighMijloc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TraiectoriiDreaptaHighMijloc {
    AutonomDreaptaHighMijloc auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone1, deliverCone2, catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5mid,deliverCone5high,deliverCone5,park1,park2,park3;

    public TraiectoriiDreaptaHighMijloc(AutonomDreaptaHighMijloc auto){
        this.auto = auto;
        initializeTrajectories();
    }

    double MAX_ANG_VEL = Math.toRadians(308.7320082135523);
    double TRACK_WIDTH = 13.2; // in

    public void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .strafeLeft(52.9)
                .back(7)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MID))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,2080))
                .addTemporalMarker(.7, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(24.8)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
                .build();
        deliverCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(25.3)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,2070))
                .addTemporalMarker(.5, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(25.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE4))
                .build();
        deliverCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(49.9)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHother))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,2050))
                .addTemporalMarker(.5, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                ///.strafeLeft(0.1)
                .forward(50.8)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE3))
                .build();
        deliverCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(51.1)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHother))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,2040))
                .addTemporalMarker(.5, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(52)
                .addTemporalMarker( 0, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2))
                .build();
        deliverCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(51.6)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHother))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,2030))
                .addTemporalMarker(.5, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(53.1)
                .strafeRight(0.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        deliverCone5mid = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(25.9)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.MIDother))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,2000))
                .addTemporalMarker(.5, () -> auto.adjuster.setPosition(0.8))
                .build();
        deliverCone5high = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(52.2)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHother))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,2010))
                .addTemporalMarker(.5, () -> auto.adjuster.setPosition(0.8))
                .build();
        park3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(11.7)
                .turn(Math.toRadians(-80))
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(7.6)
                .turn(Math.toRadians(-80))
                .addTemporalMarker(.3, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(1.7, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(30.6)
                .turn(Math.toRadians(-80))
                .addTemporalMarker(.3, () -> AutoUtil.platePosition(auto.plateMotor,1745))
                .addTemporalMarker(2, () -> AutoUtil.platePosition(auto.plateMotor,1045))
                .addTemporalMarker(2.2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
    }

    public void runAuto(int detected){
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(250);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone1);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone1);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone2);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone2);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone3);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone3);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone4);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone4);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone5);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(150);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(200);
        if(detected==1) {
            auto.mecanumDrive.followTrajectorySequence(deliverCone5high);
            auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
            auto.sleep(200);
            auto.AutoUtil.setClaw(auto.catcher,true);
            auto.sleep(150);
            auto.adjuster.setPosition(0f);
            auto.mecanumDrive.followTrajectorySequence(park3);
        }
        else if(detected==2) {
            auto.mecanumDrive.followTrajectorySequence(deliverCone5mid);
            auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
            auto.sleep(200);
            auto.AutoUtil.setClaw(auto.catcher,true);
            auto.sleep(150);
            auto.adjuster.setPosition(0f);
            auto.mecanumDrive.followTrajectorySequence(park2);
        }
        else {
            auto.mecanumDrive.followTrajectorySequence(deliverCone5mid);
            auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
            auto.sleep(200);
            auto.AutoUtil.setClaw(auto.catcher,true);
            auto.sleep(150);
            auto.adjuster.setPosition(0f);
            auto.mecanumDrive.followTrajectorySequence(park1);
        }
    }
}
