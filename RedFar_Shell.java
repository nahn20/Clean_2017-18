package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="RedFar_Charlotte", group="Charlotte Autonomae")
public class RedFar_Charlotte extends LinearOpMode{
    NathanPushboat boat = new NathanPushboat();
    private ElapsedTime runtime = new ElapsedTime();
    VuforiaLocalizer vuforia;
    double rotateAngle = 0;
    double PI = Math.PI;
    String correctCryptoSlot = "Left";
    @Override
    public void runOpMode(){
        telemetry.addData("uwu", "Wait");
        telemetry.update();
        boat.init(hardwareMap);
        AutoTransitioner.transitionOnStop(this, "RedFar_AngelOp");
        
        boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQ6M0kj/////AAAAGYsMfGvfjkGGlaBJBo84DksXdDgs4AmpEDWNkoag1HAlZ93v7JEK967tDDswHjp6gNrANoJqgPDZCawn6YEnlYzDTzaoufvvImFMlSa94j0OW28rElHTniEc0hbRblm1qEX5pHri02/FTyEAmdbKNW324ljfpHWZnwHp65Bwr8WR9vB6QxkAPJBf+0R3f9H0MuHdKVQkMBC2E97MJVy9fBc3huI5zBOrdEYIvZCf32ktKrw6uTenPZGdpJF4x4VqS4VXFrJ2w+tpWU6pHn2JZM+wLGDy8gYtKKMXKmX2Jfz1U6THFBxlFiXOojOuaFIBN9iPlWvG2AIBRNvbLw8sOW0jmhSWRvOx0bSc/QH3l8By";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        
        telemetry.addData("uwu", "Loading IMU. If stuck in this stage for too long, use the emergency non-imu autonomous.");
        telemetry.update();
        boat.imu();
        
        telemetry.addData("uwu", "Good");
        telemetry.update();
        while(!opModeIsActive()){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", vuMark);
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                correctCryptoSlot = "Right";
            } else if (vuMark == RelicRecoveryVuMark.CENTER) { //7.63
                correctCryptoSlot = "Center";
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                correctCryptoSlot = "Left";
            }
            telemetry.addData("uwu", correctCryptoSlot);
            telemetry.update();
        }
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        // S T A R T /\/\ S T A R T /\/\ S T A R T /\/\ S T A R T \\
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        
        double startTime = runtime.milliseconds();
        boat.jewel_arm.setPosition(1);
        boat.glyph_stacker.setPosition(.27);
        boat.relic_flipper.setPosition(1);
        
        int success = 1;
        for(int i = 0; i < 3 && success == 1; i++){
            success = detect_color("RED", "FAR");
            telemetry.addData("Success", success);
            telemetry.update();
        }
        
        boat.glyph_aligner.setPosition(.55);
        boat.glyph_aligner2.setPosition(.55);
        drive(PI/2, 10, .4); //Drives off platform
        boat.winch.setPower(-0.3);
        drive(PI/2, 17, 1);
        switch(correctCryptoSlot){
            case "Left":
                drive(PI, 15 + 8, 1);
                break;
            case "Center":
                drive(PI, 15, 1);
                break;
            case "Right":
                drive(PI, 15 - 8, 1);
                break;
        }
        drive(PI/2, 2, 1);
        findColumn(6, runtime.milliseconds() - startTime - 1000);
        fancyOuttake(1);
        drive(-PI/2, 1, 1);
        switch(correctCryptoSlot){
            case "Right":
                drive(PI, 8, 1);
                break;
            case "Left":
                drive(0, 8, 1);
                break;
        }
        ////////////////////////////////////////////////////////////
        // S E C O N D    G L Y P H /\/\ S E C O N D    G L Y P H \\
        // S T A R T /\/\ S T A R T /\/\ S T A R T /\/\ S T A R T \\
        // S E C O N D    G L Y P H /\/\ S E C O N D    G L Y P H \\
        ////////////////////////////////////////////////////////////
        
//        if(correctCryptoSlot != "Center"){
//            drive(PI, 24, 1);
//            rotate(0);
//            rotate_arc(-(PI + PI* 25/180), 1);
//            drive(PI/2, 18, 1);
//            for(int i = 0; i < 3 && !(boat.glyph_distance_sensor.getDistance(DistanceUnit.MM) < 100); i++){
//                getGlyph(i);
//                sleep(400);
//            }
//            drive(-PI/2, 19, 1);
//            rotate_arc(PI + PI* 25/180, 1);
//            
//            switch(correctCryptoSlot){
//                case "Right":
//                    drive(0, 10, 1);
//                    drive(PI/2, 10, 1);
//                    findColumn(6, runtime.milliseconds() - startTime - 1000);
//                    break;
//                case "Left":
//                    drive(0, 26, 1);
//                    drive(PI/2, 10, 1);
//                    findColumn(6, runtime.milliseconds() - startTime - 1000);
//                    break;
//            }
//            fancyOuttake(1);
//            drive(-PI/2, 2, 0.3);
//        }

//        if(runtime.milliseconds() - startTime < 28000){
//            switch(correctCryptoSlot){
//                case "Right":
//                    drive(0, 6, 1);
//                    break;
//                case "Left":
//                    drive (PI, 6, 1);
//                    break;
//            }
//        }
        if(runtime.milliseconds() - startTime < 28000){
            rotate(0);
        }
        if(runtime.milliseconds() - startTime < 29500){
            sleep(400);
        }
        
        /*
        */
    }
    public void drive(double angle, double distance, double power){ //"24 inches" off the balancing stone gives 22.5 inches in actuality
        /*
                 -PI/2
               ==========
               |        |
               |        |
 jewel arm  0  |        | PI
               |        |
               | INTAKE |
                  PI/2

         */

        // DEFINE DIRECTION: front of the ROBOT IS THE INTAKE, define right and left from there
        boat.front_left_motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        boat.back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        boat.front_right_motor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        boat.back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("uwu", "Drive Angle: " + angle);
        telemetry.update();
        int Dx = (int) Math.round(Math.sin(angle + Math.PI/4));
        int Dy = (int) Math.round(Math.sin(angle - Math.PI/4));
        double Px = Math.sin(angle + Math.PI/4);
        double Py = Math.sin(angle - Math.PI/4);
        int conversion_factor = 1120 * 2/35; //find THROUGH EXPERIMENTATOIN,  1120*5 rotations should give 2pi4, what does it actually equal?
        int rotations = (int) Math.round(distance * conversion_factor);

        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boat.front_left_motor.setTargetPosition(Dx*rotations + boat.front_left_motor.getCurrentPosition()); //1120 for andy mark
        boat.front_right_motor.setTargetPosition(Dy*rotations + boat.front_right_motor.getCurrentPosition());
        boat.back_left_motor.setTargetPosition(Dy*rotations + boat.back_left_motor.getCurrentPosition());
        boat.back_right_motor.setTargetPosition(Dx*rotations + boat.back_right_motor.getCurrentPosition());

        boat.back_left_motor.setPower(Py*power);
        boat.front_right_motor.setPower(Py*power);
        boat.front_left_motor.setPower(Px*power);
        boat.back_right_motor.setPower(Px*power);


        busy();
        boat.back_left_motor.setPower(0);
        boat.front_right_motor.setPower(0);
        boat.front_left_motor.setPower(0);
        boat.back_right_motor.setPower(0);
    }
    public void driveWithoutBusy(double angle, double distance, double power){ //"24 inches" off the balancing stone gives 22.5 inches in actuality
        /*
                 -PI/2
               ==========
               |        |
               |        |
 jewel arm  0  |        | PI
               |        |
               | INTAKE |
                  PI/2

         */

        // DEFINE DIRECTION: front of the ROBOT IS THE INTAKE, define right and left from there
        boat.front_left_motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        boat.back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        boat.front_right_motor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        boat.back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("uwu", "Drive Angle: " + angle);
        telemetry.update();
        int Dx = (int) Math.round(Math.sin(angle + Math.PI/4));
        int Dy = (int) Math.round(Math.sin(angle - Math.PI/4));
        double Px = Math.sin(angle + Math.PI/4);
        double Py = Math.sin(angle - Math.PI/4);
        int conversion_factor = 1120 * 2/35; //find THROUGH EXPERIMENTATOIN,  1120*5 rotations should give 2pi4, what does it actually equal?
        int rotations = (int) Math.round(distance * conversion_factor);

        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boat.front_left_motor.setTargetPosition(Dx*rotations + boat.front_left_motor.getCurrentPosition()); //1120 for andy mark
        boat.front_right_motor.setTargetPosition(Dy*rotations + boat.front_right_motor.getCurrentPosition());
        boat.back_left_motor.setTargetPosition(Dy*rotations + boat.back_left_motor.getCurrentPosition());
        boat.back_right_motor.setTargetPosition(Dx*rotations + boat.back_right_motor.getCurrentPosition());

        boat.back_left_motor.setPower(Py*power);
        boat.front_right_motor.setPower(Py*power);
        boat.front_left_motor.setPower(Px*power);
        boat.back_right_motor.setPower(Px*power);


    }
    public void findColumn(double blep, double bop){
        double range = blep; //Amount it goes left and right to find the thingy
        double time = bop;
        int fuckthisissuchastupidwayofdoingthis = 0;
        boolean stupiditycount = false;
        double columnStartTime = runtime.milliseconds();
        //double fuckme = runtime.milliseconds();
        while(!(boat.wall_distance_sensor.getDistance(DistanceUnit.MM) < 999) && runtime.milliseconds() - columnStartTime < time && opModeIsActive()){
            telemetry.addData("uwu", boat.front_left_motor.isBusy());
            telemetry.update();
            if(fuckthisissuchastupidwayofdoingthis == 0){
                driveWithoutBusy(PI, range/2, .5);
                fuckthisissuchastupidwayofdoingthis = 1;
            }
            if(!boat.front_left_motor.isBusy()){
                if(stupiditycount == true){
                    drive(PI/2, 2, 1);
                    stupiditycount = false;
                }
                if(fuckthisissuchastupidwayofdoingthis == 1){
                    driveWithoutBusy(0, range, .5);
                    fuckthisissuchastupidwayofdoingthis = 2;
                    stupiditycount = true;
                }
                else if(fuckthisissuchastupidwayofdoingthis == 2){
                    driveWithoutBusy(PI, range, .5);
                    fuckthisissuchastupidwayofdoingthis = 1;
                    stupiditycount = true;
                }
                //fuckme = runtime.milliseconds();
            }
        }
        if(fuckthisissuchastupidwayofdoingthis == 1){
            drive(PI, 1, 1);
        }
        boat.back_left_motor.setPower(0);
        boat.front_right_motor.setPower(0);
        boat.front_left_motor.setPower(0);
        boat.back_right_motor.setPower(0);
    }
    public void evenMoreStupidFindColumn(){
        double columnStartTime = runtime.milliseconds();
        int fml = 0;
        //double fuckme = runtime.milliseconds();
        while(!(boat.wall_distance_sensor.getDistance(DistanceUnit.MM) < 999) && runtime.milliseconds() - columnStartTime < 4000 && opModeIsActive()){
            if(fml == 0){
                driveWithoutBusy(0, 5, .5);
                fml = 1;
            }
            if(fml == 1){
                driveWithoutBusy(PI, 5, .5);
                drive(PI/2, 1, .3);
                fml = 0;
            }
        }
        boat.back_left_motor.setPower(0);
        boat.front_right_motor.setPower(0);
        boat.front_left_motor.setPower(0);
        boat.back_right_motor.setPower(0);
    }
    public void getGlyph(int i){
        double shift = 0;
        int front_left_motor_original_pos = boat.front_left_motor.getCurrentPosition();
        int front_right_motor_original_pos = boat.front_right_motor.getCurrentPosition();
        int back_left_motor_original_pos = boat.back_left_motor.getCurrentPosition();
        int back_right_motor_original_pos = boat.back_right_motor.getCurrentPosition();
        
        telemetry.addData("uwu", ":(");
        telemetry.update();
//        if(i == 0){
//            boat.right_intake.setPower(-.3);
//            boat.left_intake.setPower(-.3);
//        }
//        if(i == 1){
//            boat.right_intake.setPower(-.5);
//            boat.left_intake.setPower(-1);
//        }
//        if(i == 2){
//            boat.right_intake.setPower(-.8);
//            boat.left_intake.setPower(-1);
//        }
        boat.right_intake.setPower(-1);
        boat.left_intake.setPower(-1);
        driveWithoutBusy(PI/2, 30, 0.3);
        double blep = runtime.milliseconds();
        while(!(boat.glyph_distance_sensor.getDistance(DistanceUnit.MM) < 100) && opModeIsActive() && runtime.milliseconds() - blep < 8000){
            if(boat.front_left_motor.getCurrentPosition() > 3000){
                telemetry.addData("uwu", "driving back! woo");
                telemetry.update();
                sleep(500);
                drive(PI/2, -(boat.front_left_motor.getCurrentPosition() - front_left_motor_original_pos) * 35/2240, 1);

                front_left_motor_original_pos = boat.front_left_motor.getCurrentPosition();
                front_right_motor_original_pos = boat.front_right_motor.getCurrentPosition();
                back_left_motor_original_pos = boat.back_left_motor.getCurrentPosition();
                back_right_motor_original_pos = boat.back_right_motor.getCurrentPosition();
                driveWithoutBusy(PI/2, 30, 0.3);
            }
        }
        while(boat.glyph_distance_sensor.getDistance(DistanceUnit.MM) < 100){}
        boat.right_intake.setPower(0);
        boat.left_intake.setPower(0);
        telemetry.addData("uwu", shift);
        telemetry.update();
        drive(PI/2, -(boat.front_left_motor.getCurrentPosition() - front_left_motor_original_pos) * 35/2240, 1);
    }
    public void fuckGetGlyph(){
        double shift = 0;
        int front_left_motor_original_pos = boat.front_left_motor.getCurrentPosition();
        boat.right_intake.setPower(-1);
        boat.left_intake.setPower(-1);
        driveWithoutBusy(PI/2, 30, 0.3);
        double blep = runtime.milliseconds();
        while(!(boat.glyph_distance_sensor.getDistance(DistanceUnit.MM) < 999) && opModeIsActive() && runtime.milliseconds() - blep < 8000){
            boat.right_intake.setPower(-1);
            boat.left_intake.setPower(-1);
            if(boat.front_left_motor.getCurrentPosition() > 3000){
                boat.right_intake.setPower(0);
                boat.left_intake.setPower(0);
                telemetry.addData("uwu", "driving back! woo");
                telemetry.update();
                sleep(500);
                drive(PI/2, -(boat.front_left_motor.getCurrentPosition() - front_left_motor_original_pos) * 35/2240, 1);

                front_left_motor_original_pos = boat.front_left_motor.getCurrentPosition();
                driveWithoutBusy(PI/2, 30, 0.3);
            }
        }
        boat.right_intake.setPower(0);
        boat.left_intake.setPower(0);
        drive(PI/2, -(boat.front_left_motor.getCurrentPosition() - front_left_motor_original_pos) * 35/2240, 1);
    }
    public void printInt(int text){
        telemetry.addData("uwu", Integer.toString(text));
        telemetry.update();
    }
    public void printHeading(){
        telemetry.addData("uwu", getHeading());
        telemetry.update();
    }
    public void outtake(){
        boat.right_intake.setPower(1);
        boat.left_intake.setPower(1);
        sleep(300);
        boat.right_intake.setPower(.4);
        boat.left_intake.setPower(.1);
        sleep(1000);
        drive(-PI / 2, 10, 0.6);
        boat.right_intake.setPower(0);
        boat.left_intake.setPower(0);
    }
    public void outtakePush(){
        telemetry.addData("uwu", boat.glyph_distance_sensor.getDistance(DistanceUnit.MM));
        telemetry.update();
        boat.right_intake.setPower(.4);
        boat.left_intake.setPower(.4);
        sleep(10);
        boat.right_intake.setPower(.4);
        boat.left_intake.setPower(.1);
        sleep(1000);
        boat.right_intake.setPower(0);
        boat.left_intake.setPower(0);
        drive(PI/2, 10, 0.3);
        drive(-PI / 2, 10, 0.6);
        while(boat.glyph_distance_sensor.getDistance(DistanceUnit.MM) < 100 && opModeIsActive()){
            boat.right_intake.setPower(.6);
            boat.left_intake.setPower(.15);
            sleep(1000);
            boat.right_intake.setPower(0);
            boat.left_intake.setPower(0);
            drive(PI/2, 16, 0.3);
            drive(-PI / 2, 10, 0.6);
        }
    }
    public void fancyOuttake(double power){
        boat.right_intake.setPower(power);
        boat.left_intake.setPower(power);
        sleep(200);
        driveWithoutBusy(-PI/2, 300, 0.2);
        while(boat.glyph_distance_sensor.getDistance(DistanceUnit.MM) < 999){
        }
        drive(-PI/2, 4, 0.2);
        boat.right_intake.setPower(0);
        boat.left_intake.setPower(0);
    }
    public void absolutelyRetardedOuttake(){
        boat.winch.setPower(0);
        boat.right_intake.setPower(1);
        boat.left_intake.setPower(.2);
        sleep(500);
    }
    public void autoDetectColumn(){
        boat.jewel_arm.setPosition(0.4);
        sleep(800);
        double column_distance = boat.distance_sensor.getDistance(DistanceUnit.MM);
        double target_distance = 80.0; //this distance is how far you want the distance sensor to get to the column
        double autoDetectStartTime = runtime.milliseconds();
        while(Math.abs(column_distance - target_distance)>2.0 && runtime.milliseconds() - autoDetectStartTime < 2000 && opModeIsActive()){
            boat.front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boat.front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boat.back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            boat.back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double power = 0;
            power = .2;
            if (column_distance-target_distance < 0) {
                power = power;
            }
            else if (column_distance-target_distance > 0){
                power = -power;
            }
            boat.back_left_motor.setPower(power);
            boat.front_right_motor.setPower(power);
            boat.front_left_motor.setPower(power);
            boat.back_right_motor.setPower(power);
            column_distance = boat.distance_sensor.getDistance(DistanceUnit.MM);
        }
        boat.back_left_motor.setPower(0);
        boat.front_right_motor.setPower(0);
        boat.front_left_motor.setPower(0);
        boat.back_right_motor.setPower(0);
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(200);
        boat.jewel_arm.setPosition(1);
        sleep(300);
    }
    public void reset_drive(){
        telemetry.addData("uwu", "~ Resetting Encoders ~");
        telemetry.update();

        boat.front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boat.back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("uwu", "~ Done :D ~");
        telemetry.update();
    }
    public void busy(){ //Use to check if motors are running
        //Good to make sure it doesn't continue onto next function while previous function is still running
        double busyStartTime = runtime.milliseconds();

        while(boat.back_left_motor.isBusy() && boat.back_right_motor.isBusy() && boat.front_left_motor.isBusy() && boat.front_right_motor.isBusy() && opModeIsActive()){
//            telemetry.addData("uwu", "Busy " + (runtime.milliseconds() - busyStartTime));
//            telemetry.addData("uwu", "~~~ Motors ~~~");
//            telemetry.addData("uwu", "Front_Left_Motor: " + boat.front_left_motor.getCurrentPosition() / 1120);
//            telemetry.addData("uwu", "Front_Right_Motor: " + boat.front_right_motor.getCurrentPosition() / 1120);
//            telemetry.addData("uwu", "Back_Left_Motor: " + boat.back_left_motor.getCurrentPosition() / 1120);
//            telemetry.addData("uwu", "Back_Right_Motor: " + boat.back_right_motor.getCurrentPosition() / 1120);
//            telemetry.addData("uwu", "Busy List <");
//            if(boat.front_left_motor.isBusy()){
//                telemetry.addData("uwu", "Front Left Motor");
//            }
//            if(boat.front_right_motor.isBusy()){
//                telemetry.addData("uwu", "Front Right Motor");
//            }
//            if(boat.back_left_motor.isBusy()){
//                telemetry.addData("uwu", "Back Left Motor");
//            }
//            if(boat.back_right_motor.isBusy()){
//                telemetry.addData("uwu", "Back Right Motor");
//            }
//            telemetry.update();
            telemetry.addData("uwu", "Buuuusy");
            
        }
    }
    public void rotate(double angle) {
        double threshold = 1;
        double power = 0;
        double angleDist = 0;
        sleep(200);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("uwu", angle);
        telemetry.update();
        double rotateStartTime = runtime.milliseconds();
        while (((Math.abs(angle - getHeading()) > threshold) || ((Math.abs(angle) > 180 - threshold) && (Math.abs(Math.abs(angle) - Math.abs(getHeading())) > threshold))) && runtime.milliseconds() - rotateStartTime < 6000) {
            angleDist =  Math.abs(angle - getHeading());
            if (angleDist > 180) {
                angleDist = 360 - angleDist;
            }
            if(Math.abs(angle - getHeading()) > 100){
                power = 1;
            }
            else if(Math.abs(angle - getHeading()) > 50){
                power = 0.2;
            }
            else{
                power = 0.12;
            }
        //TURN COUNTER CLOCKWISE
            if (angle - getHeading() > 0 || angle - getHeading() < -180) {
                boat.back_left_motor.setPower(-1*power);
                boat.front_right_motor.setPower(power);
                boat.front_left_motor.setPower(-1*power);
                boat.back_right_motor.setPower(power);
                telemetry.addData("uwu", "If");
            } else {
        //TURN CLOCKWISE
                boat.back_left_motor.setPower(power);
                boat.front_right_motor.setPower(-1*power);
                boat.front_left_motor.setPower(power);
                boat.back_right_motor.setPower(-1*power);
                telemetry.addData("uwu", "Else");
            }
            telemetry.addData("uwu", angle - getHeading());
            telemetry.addData("uwu", runtime.milliseconds() - rotateStartTime);
            telemetry.update();
        }
        boat.back_left_motor.setPower(0);
        boat.front_right_motor.setPower(0);
        boat.front_left_motor.setPower(0);
        boat.back_right_motor.setPower(0);
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public float getHeading(){
        Orientation angles = boat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;
        float fakeHeading = heading;// - rotate_angle;
        return fakeHeading;
    }
    public int detect_color(String teamColor, String location){
        //Assumes color sensor is on the back of jewel arm
        boat.jewel_arm.setPosition(0);
        sleep(1200);
        double higherPosition = 0;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double scale_factor = 255;
        Color.RGBToHSV((int) (boat.color_sensor.red() * scale_factor),
                (int) (boat.color_sensor.green() * scale_factor),
                (int) (boat.color_sensor.blue() * scale_factor),
                hsvValues);
        telemetry.addData("Values", hsvValues[0]);
        if (hsvValues[0] <= 250 && hsvValues[0] >= 130) {
            telemetry.addData("uwu", "Blue");
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLUE);
                }
            });
            switch(teamColor){
                case "BLUE":
                    if(location == "FAR"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(50);
                    }
                    rotate_arc(PI / 8, 0.4);
                    boat.jewel_arm.setPosition(1);
                    sleep(200);
                    rotate_arc(-PI / 8, 0.4);
                    break;
                case "RED":
                    if(location == "FAR"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(50);
                    }
                    rotate_arc(-PI / 8, 0.4);
                    boat.jewel_arm.setPosition(1);
                    sleep(200);
                    rotate_arc(PI / 8, 0.4);
                    break;
            }
        } else if (hsvValues[0] <= 25 || (hsvValues[0] >= 300 && hsvValues[0] <= 400)) {
            telemetry.addData("uwu", "Red");
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.RED);
                }
            });
            switch (teamColor) {
                case "BLUE":
                    if(location == "CLOSE"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(50);
                    }
                    rotate_arc(-PI / 8, 0.4);
                    boat.jewel_arm.setPosition(1);
                    sleep(200);
                    rotate_arc(PI / 8, 0.4);
                    break;
                case "RED":
                    if(location == "CLOSE"){
                        boat.jewel_arm.setPosition(higherPosition);
                        sleep(50);
                    }
                    rotate_arc(PI / 8, 0.4);
                    boat.jewel_arm.setPosition(1);
                    sleep(200);
                    rotate_arc(-PI / 8, 0.4);
                    break;
            }
        } else {
            telemetry.addData("uwu", "Unknown");
            telemetry.update();
            boat.jewel_arm.setPosition(1);
            sleep(200);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.BLACK);
                }
            });
            return 1;
        }
        telemetry.update();
        return 0;
    }
public void rotate_arc (double angle, double power) {
        ///CCW IS POSITIVE

        boat.front_left_motor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        boat.back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        boat.front_right_motor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        boat.back_right_motor.setDirection(DcMotor.Direction.FORWARD);//TAKE RADIANS

        double radius_chassis = 21.45/2; //19.5
        double distance = radius_chassis * angle; //distance you want to travel HAS BUILT IN DISTANCE SIDE NEEDED BECAUSE OF THE SIGN OF ANGLE
        double P = -1;
        int conversion_factor = 1120 * 2/35 * 9/8; //9/8 = conversion for ROTATION find THROUGH EXPERIMENTATOIN,  1120*5 rotations should give 2pi4, what does it actually equal?
        int rotations = (int) Math.round(distance * conversion_factor);

        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boat.front_left_motor.setTargetPosition(-rotations + boat.front_left_motor.getCurrentPosition()); //1120 for andy mark
        boat.front_right_motor.setTargetPosition(rotations + boat.front_right_motor.getCurrentPosition());
        boat.back_left_motor.setTargetPosition(-rotations + boat.back_left_motor.getCurrentPosition() );
        boat.back_right_motor.setTargetPosition(rotations + boat.back_right_motor.getCurrentPosition());

        boat.back_left_motor.setPower(-P*power);
        boat.front_right_motor.setPower(P*power);
        boat.front_left_motor.setPower(-P*power);
        boat.back_right_motor.setPower(P*power);
        busy();
    }
}