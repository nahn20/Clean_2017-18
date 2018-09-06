package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="BlueFar_AngelOp", group="Pushbot")
public class BlueFar_AngelOp extends LinearOpMode {
    NathanPushboat boat = new NathanPushboat();
    private ElapsedTime runtime = new ElapsedTime();
    
    float rotate_angle = -90; //Change this
    // Close_AngelOp -> 180
    // RedFar_AngelOp -> 90
    // BlueFar_AngelOp -> -90
    
    
    //////////////////////
   /* TOGGLE VARIABLES */
    //////////////////////

    Boolean gamepad2_y_toggle = false;
    Boolean gamepad2_a_toggle = false;
    Boolean gamepad2_b_toggle = false;
    Boolean gamepad2_x_toggle = false;
    Boolean gamepad2_right_bumper_toggle = false;
    Boolean killUrself = false;
    Boolean doKillUrself = true;
    boolean open = true;
    boolean pause = false;
    //:)
    boolean pause2 = false;
    /////////////////////////////
   /* DRIVE RELATED VARIABLES */
    /////////////////////////////

    boolean stall = false;
    boolean switchServo = false;
    boolean pButtonSwitch = false;
    boolean cButtonSwitch = false;
    static final double SERVO_SWITCH_CLOSE = .35;//165.0/180.0;
    static final double SERVO_SWITCH_OPEN = 0.0/180.0;

    boolean moveShovelSafe = false;
    boolean switchServo2 = false;
    boolean pButtonSwitch2 = false;
    boolean cButtonSwitch2 = false;
    static final double SERVO_SWITCH_CLOSE2 = .65;
    static final double SERVO_SWITCH_OPEN2 = .39;
    double wait = 0;
    float stick_x = 0;
    float stick_y = 0;
    double gyroAngle = 0;
    double theta = 0;
    double Px = 0;
    double gamepad1_a_last_press_time = 0;

    double Py = 0;
    int dpadRotate = 0;
    double threshold = 0.05;
    double Protate = 0;
    double maxRotate = 1.25;
    int moveAngle = 9001;
    double currentTimeForAngle = 0;
    double servo_angle;
    boolean applePie = true;
    double lastErrorGlyph = 0;
    boolean Shovelness = false;
    double coefficient = 1.0;
    double ProtateMultiplier = 1.0;
    boolean lowhigh = false;
    boolean glyphLoadMvmt = false;
    //////////////////////////////////
   /* GLYPH FLIP RELATED VARIABLES */
    //////////////////////////////////

    double switchangle = .52;
    double switchangle2 = .39; //for grabber
    double glyph_flip_last_press_time = 0;
    double intake_last_press  = 0;
    double glyph_grab_last_press_time2 = 0;
    double servo_adjust_last_time = 0;
    double glyph_flipper_last_time = 0;
    double glyph_flip_stomp_last_time = 0;
    Boolean glyph_grabber_openness = false;
    double glyph_grab_last_press_time = 0;
    double errorServo = .835; //could be .26
    double firstError;
    double shovel_time = 0;
    double lastPosition = 0;
    /////////////////////////////////
   /* RELIC RELATED VARIABLES */
    //////////////////////////////////
    boolean relicMode = false;
    boolean gamepad1_b_relic_toggle = true;
    double gamepad1_b_relic_toggle_time = 0;
    double gamepad1_start_relic_toggle_time = 0;
    double lastFlipTime = 0;
    @Override
    public void runOpMode() {
        boat.init(hardwareMap);
        //boat.glyph_aligner.setPosition(boat.glyph_aligner.getPosition());
        boat.front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boat.relic_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("uwu", "Nathan is stupid, loading imu :(");
        telemetry.update();
        boat.relic_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boat.imu();
        telemetry.addData("NO", "Nathan Boat 1.0 without crossbow is ready to be sailed!");
        telemetry.update();

        while(!opModeIsActive()){
            //boat.jewel_arm.setPosition(56.0/180.0);
            // boat.glyph_aligner.setPosition(.32);
            boat.winch.setPower(-.005);
            boat.glyph_grabber.setPosition(.5);
//            boat.glyph_aligner.setPosition(.79); //CHANGE
//            boat.glyph_aligner2.setPosition(.79);
            //boat.glyph_stacker.setPosition(0.25);
            boat.relic_flipper.setPosition(1.0);
        }
        boat.jewel_arm.setPosition(1.0); //should be 0
        while(opModeIsActive()){
            telemetry.addData(">", "Relic Encoder" + boat.relic_extender.getCurrentPosition());

            //boat.glyph_stacker.setPosition(.93); //.5 should be to stuff glyphs in


//            if (killUrself == false) {
//                boat.winch.setPower(-.005);
//            } else {
//                boat.winch.setPower(0);
//            }
            boat.winch.setPower(-.01);

            relicModeToggle();

            //relicFlipperNoRelicMode();
            if(relicMode == true){
                relic();
                if (!gamepad2.start) {
                    relicSlide();
                }
                else {
                    relicSimple();
                }
            } else{
                glyph_flip();
            }
            intake(); // this encompasses all intake functions necessary
            jewel_arm();
            telemetry.addData(">", "Relic Grabber Angle" + boat.relic_grabber.getPosition());
            telemetry.addData(">", "Relic Flopper Angle" + boat.relic_flipper.getPosition());
            telemetry.addData(">", "Current Angle: " + boat.glyph_aligner.getPosition());
            telemetry.addData("Distance (mm)",
                    String.format( "%.02f", boat.distance_sensor.getDistance(DistanceUnit.MM)));
            getPotValues();
            drive();
//            if (gamepad2.left_stick_button){
//                boat.back_right_motor.setPower(1);
//                boat.back_left_motor.setPower(1);
//                boat.front_right_motor.setPower(1);
//                boat.front_left_motor.setPower(1);
//            }
            //killUrself();
            telemetry.update();//THIS GOES AT THE END
        }
    }

    /////////////////////////////
    /////////////////////////////
   /* DRIVE RELATED FUNCTIONS */
    /////////////////////////////
    /////////////////////////////

    public void drive() {
        stick_x = gamepad1.left_stick_x;
        stick_y = gamepad1.left_stick_y;
        stick_x = stick_x / 2;
        stick_y = stick_y / 2;

        if (gamepad1.start && gamepad1.right_trigger > .2) {
            setCurrentHeadingToZero();
        }

        gyroAngle = getHeading() * Math.PI / 180;
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;
        //MOVEMENT
        if (gamepad1.left_trigger >.2&& (boat.armPotentiometer.getVoltage()<2.0 || relicMode)){
            stick_x = stick_x * 1/2;
            stick_y = stick_y * 1/2;
//            if (gamepad1.left_bumper) {
//                stick_x = stick_x / 3;
//                stick_y = stick_y / 3;
//            }
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
        }
        else if (gamepad1.right_trigger < 0.5) {
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
            coefficient = 1.0;
        } else if (gamepad1.right_trigger > 0.5 ) {
            //Runs Px Py independent of gyro (old drive) -> DOUBLE SPEED GYRO
            coefficient = 1.5;
            stick_x = stick_x * 2;
            stick_y = stick_y * 2;
//            if (gamepad1.left_bumper) {
//                stick_x = stick_x / 3;
//                stick_y = stick_y / 3;
//            }
            theta = (Math.atan2(stick_y, stick_x) - gyroAngle) - (Math.PI / 2);
            Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));

            // theta = (Math.atan2(stick_y, stick_x)) - 0; //burt is an idiot
            // Px = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta + Math.PI / 4));
            // Py = Math.sqrt(stick_x * stick_x + stick_y * stick_y) * (Math.sin(theta - Math.PI / 4));
        }
        //ROTATION
        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            if (Px == 0 && Py == 0) {
                Protate = ProtateMultiplier*gamepad1.right_stick_x * .4;
            } else {
                Protate = ProtateMultiplier*gamepad1.right_stick_x * 3 / 2 * 0.35; //* 3 / 2 * 0.42
                Px = Px / 3 * 2;
                Py = Py / 3 * 2;
            }
        } else {
            Protate = 0;
        }
        //SPINNING LEFT IS NEGATIVE POWER

        rotateToAngle();

        // cardinal directions

        if (gamepad1.x) {
            Protate = -0.15;
        } else if (gamepad1.b) {
            Protate = .15;
        }

        if (gamepad1.right_stick_button && runtime.milliseconds() > (gamepad1_a_last_press_time + 500)) {
            gamepad1_a_last_press_time = runtime.milliseconds();
            dpadRotate = 1;
            if (gamepad1.start) {
                dpadRotate = 0;
            }
        } else if (gamepad1.left_stick_button && runtime.milliseconds() > (gamepad1_a_last_press_time + 500)) {
            gamepad1_a_last_press_time = runtime.milliseconds();
            dpadRotate = 2;
            if (gamepad1.start) {
                dpadRotate = 0;
            }
        }



        if (gamepad1.left_trigger>.2&& (boat.armPotentiometer.getVoltage()<2.2 || relicMode == true)) {
            ProtateMultiplier = .3;
        } else {

            ProtateMultiplier = 1.0;
        }


        if (gamepad1.dpad_down && !gamepad1.right_bumper && !gamepad1.right_bumper) {
            if (dpadRotate == 0) {
                boat.back_left_motor.setPower(coefficient * -.5 + Protate);
                boat.front_right_motor.setPower(coefficient * -.5 - Protate);
                boat.front_left_motor.setPower(coefficient * -.5 + Protate);
                boat.back_right_motor.setPower(coefficient * -.5 - Protate);
            } else if (dpadRotate ==1) {
                boat.back_left_motor.setPower(coefficient * .3 + Protate);
                boat.front_right_motor.setPower(coefficient * .3 - Protate);
                boat.front_left_motor.setPower(coefficient * -.3 + Protate);
                boat.back_right_motor.setPower(coefficient * -.3 - Protate);
            } else {
                boat.back_left_motor.setPower(coefficient * -.3 + Protate);
                boat.front_right_motor.setPower(coefficient * -.3 - Protate);
                boat.front_left_motor.setPower(coefficient * .3 + Protate);
                boat.back_right_motor.setPower(coefficient * .3 - Protate);
            }
            boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.dpad_left && !gamepad1.right_bumper && !gamepad1.right_bumper ) {
            if (dpadRotate == 0) {
                boat.back_left_motor.setPower(coefficient * .3 + Protate);
                boat.front_right_motor.setPower(coefficient * .3 - Protate);
                boat.front_left_motor.setPower(coefficient * -.3 + Protate);
                boat.back_right_motor.setPower(coefficient * -.3 - Protate);
            } else if (dpadRotate ==1) {
                boat.back_left_motor.setPower(coefficient * .2 + Protate);
                boat.front_right_motor.setPower(coefficient * .2 - Protate);
                boat.front_left_motor.setPower(coefficient * .2 + Protate);
                boat.back_right_motor.setPower(coefficient * .2 - Protate);
            } else {
                boat.back_left_motor.setPower(coefficient * -.5 + Protate);
                boat.front_right_motor.setPower(coefficient * -.5 - Protate);
                boat.front_left_motor.setPower(coefficient * -.5 + Protate);
                boat.back_right_motor.setPower(coefficient * -.5 - Protate);
            }
        } else if (gamepad1.dpad_right && !gamepad1.right_bumper && !gamepad1.right_bumper) {
                if (dpadRotate == 0) {
                    boat.back_left_motor.setPower(coefficient * -.3 + Protate);
                    boat.front_right_motor.setPower(coefficient * -.3 - Protate);
                    boat.front_left_motor.setPower(coefficient * .3 + Protate);
                    boat.back_right_motor.setPower(coefficient * .3 - Protate);
                } else if (dpadRotate ==1) {
                    boat.back_left_motor.setPower(coefficient * -.5 + Protate);
                    boat.front_right_motor.setPower(coefficient * -.5 - Protate);
                    boat.front_left_motor.setPower(coefficient * -.5 + Protate);
                    boat.back_right_motor.setPower(coefficient * -.5 - Protate);
                } else {
                    boat.back_left_motor.setPower(coefficient * .2 + Protate);
                    boat.front_right_motor.setPower(coefficient * .2 - Protate);
                    boat.front_left_motor.setPower(coefficient * .2 + Protate);
                    boat.back_right_motor.setPower(coefficient * .2 - Protate);
                }
        } else if (gamepad1.dpad_up && !gamepad1.right_bumper && !gamepad1.right_bumper) {
            if (dpadRotate == 0) {
                boat.back_left_motor.setPower(coefficient * .2 + Protate);
                boat.front_right_motor.setPower(coefficient * .2 - Protate);
                boat.front_left_motor.setPower(coefficient * .2 + Protate);
                boat.back_right_motor.setPower(coefficient * .2 - Protate);
            } else if (dpadRotate ==1) {
                boat.back_left_motor.setPower(coefficient * -.3 + Protate);
                boat.front_right_motor.setPower(coefficient * -.3 - Protate);
                boat.front_left_motor.setPower(coefficient * .3 + Protate);
                boat.back_right_motor.setPower(coefficient * .3 - Protate);
            } else {
                boat.back_left_motor.setPower(coefficient * .3 + Protate);
                boat.front_right_motor.setPower(coefficient * .3 - Protate);
                boat.front_left_motor.setPower(coefficient * -.3 + Protate);
                boat.back_right_motor.setPower(coefficient * -.3 - Protate);
            }
                    boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
                    boat.back_left_motor.setPower(-(Px - Protate));
                    boat.front_right_motor.setPower(-(Px + Protate));
                    boat.front_left_motor.setPower(-(Py - Protate));
                    boat.back_right_motor.setPower(-(Py + Protate));
                    boat.front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    boat.front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    boat.back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    boat.back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }


        //telemetry.addData(">", "Protate: " + Protate);
        //telemetry.addData(">", "theta: " + theta);
        //telemetry.addData(">", "Px: " + Px);
        //telemetry.addData(">", "Py: " + Py);
        //telemetry.addData(">", "gyroAngle" + gyroAngle);
        //telemetry.addData(">", "leftsticky" + gamepad1.left_stick_y);

    public void rotateToAngle() { //updated move to angle
        int threshold = 1;
        double desiredRunTime = 5000;
        float incorrectHeading = getHeading();
//        if (gamepad1.a) {
//            moveAngle = 180;
//            currentTimeForAngle = runtime.milliseconds();
//            // } else if (gamepad1.x) {
//            // moveAngle = 90;
//            //    currentTimeForAngle = runtime.milliseconds();
//            //  } else if (gamepad1.b) {
//            //    moveAngle = -90;
//            //  currentTimeForAngle = runtime.milliseconds();
//        }
        if (gamepad1.left_bumper && gamepad1.left_trigger<.2) {
            moveAngle = 0;
            currentTimeForAngle = runtime.milliseconds();
        }
        if (moveAngle != 9001 && runtime.milliseconds() > currentTimeForAngle && runtime.milliseconds() < currentTimeForAngle + desiredRunTime) {
            if (moveAngle == 0) {
                if (incorrectHeading <= 0 - threshold) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, incorrectHeading);
                } else if (incorrectHeading > 0 + threshold) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, incorrectHeading);
                } else {
                    moveAngle = 9001;
                }
            }
            if (moveAngle == -90) {
                if (-90 - threshold < incorrectHeading && incorrectHeading < -90 + threshold) {
                    moveAngle = 9001;
                } else if (Math.abs(incorrectHeading) >= 90) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, incorrectHeading);
                } else if (Math.abs(incorrectHeading) < 90) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, incorrectHeading);
                }
            }
            if (moveAngle == 90) {
                if (90 - threshold < incorrectHeading && incorrectHeading < 90 + threshold) {
                    moveAngle = 9001;
                } else if (Math.abs(incorrectHeading) <= 90) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, incorrectHeading);
                } else if (Math.abs(incorrectHeading) > 90) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, incorrectHeading);
                }
            }
            if (moveAngle == 180) {
                if (180 - threshold < incorrectHeading || incorrectHeading < -180 + threshold) {
                    moveAngle = 9001;
                } else if (incorrectHeading >= 0) {
                    //rotate left
                    Protate = -1 * scaleProtate(moveAngle, incorrectHeading);
                } else if (incorrectHeading < 0) {
                    //rotate right
                    Protate = scaleProtate(moveAngle, incorrectHeading);
                }
            }
        }
    }
    public double scaleProtate(int moveAngle, float fakeHeading) {
        double ProtatePower;
        int angleMove = moveAngle;
        float angleDist;
        float incorrectHeading = fakeHeading;
        angleDist =  Math.abs(angleMove - incorrectHeading);
        if (angleDist > 180) {
            angleDist = 360 - angleDist;
        }
        ProtatePower = (1.4*((maxRotate * (angleDist / 180)) + 0.07));
        return ProtatePower;
    }

    public void setCurrentHeadingToZero(){
        if (applePie == true) {
            rotate_angle = rotate_angle + getHeading();
            applePie = false;
        }
    }
    public float getHeading(){
        Orientation angles = boat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;
        float fakeHeading = heading - rotate_angle;
        if (fakeHeading < -180) {
            fakeHeading = fakeHeading + 360;
        } else if (fakeHeading > 180) {
            fakeHeading = fakeHeading - 360;
        }
        return fakeHeading;
    }

    //////////////////////////////////
    //////////////////////////////////
   /* GLYPH FLIP RELATED FUNCTIONS */
    //////////////////////////////////
    //////////////////////////////////
    public void glyph_flip(){ //Includes toggle and calling all the glyph_flip functions
        double desiredTime = 1000;
        if(gamepad2.y && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //used to be gamepad1.right bumper
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_y_toggle = toggle(gamepad2_y_toggle);
            gamepad2_a_toggle = false;
            gamepad2_b_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad2.a && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_low();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_a_toggle = toggle(gamepad2_a_toggle);
            gamepad2_y_toggle = false;
            gamepad2_b_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad1.x && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used for glyph_flip_hover();
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_b_toggle = toggle(gamepad2_b_toggle);
            gamepad2_y_toggle = false;
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
        }
        else if(gamepad2.x && runtime.milliseconds() > (glyph_flip_last_press_time + desiredTime)){ //Used to be gamepad1.x
            glyph_flip_last_press_time = runtime.milliseconds();
            gamepad2_x_toggle = toggle(gamepad2_x_toggle);
            gamepad2_y_toggle = false;
            gamepad2_a_toggle = false;
            gamepad2_b_toggle = false;
        }
        telemetry.addData(">", "Arm Potentiometer: " + boat.armPotentiometer.getVoltage());
        //glyph_flipper_threshold_check();
        //   glyph_flip_high();
        //   glyph_flip_low();
        shovelFlip();
//        glyph_flip_hover();
//        glyph_flip_stomp();
        //glyph_grab();

//        glyph_flick();
        // drive();
    }
// FLIPPER FIRST SERVO SECOND
//    glyph_flipper_runToPosition(desired_glyph_position);
//    if(Math.abs(potValue - desired_glyph_position) <= .01){
//        glyph_aligner_servo_runToPosition(desired_servo_position);
//    }


//SERVO FIRST FLIPPER SECOND
//    glyph_aligner_servo_runToPosition(desired_servo_position);
//    if(Math.abs(servoValue - desired_servo_position) <= .015){
//        glyph_flipper_runToPosition(desired_glyph_position);
//    }




    //.854, 2.096., 0.096

    public void glyph_flipper_runToPosition(double desiredPosition, double coefficient){ //Replace later with Josh's PID loop
        double desiredTime = 20;
        double potValue = boat.armPotentiometer.getVoltage();
        double kp = .9;   //.8 3/6/18         // proportional konstant
        double ki = 0.345;    //.345 3/6/18         //konstant of integration
        double kd = 0.45;     //.2 4/21 //1.7 and 3.0?      //konstant of derivation
        double current = 0;        //value to be sent to shooter motors
        double integralActiveZone = 2.0;    // zone of error values in which the total error for the                    integral term accumulates
        double errorT = 0;                        // total error accumulated
        //double lastError = 0;                    // last error recorded by the controller
        double proportion;                        // the proportional term
        double integral;                            // the integral term
        double derivative;
        double error = potValue - desiredPosition;
        shovel_time = runtime.milliseconds();
// the derivative term
/*////////////////////////////////////////////////////////
  NOTE:
  Error is a float declared at global level, it represents the difference between target velocity and current velocity
  power is a float declared at global level, it represents target velocity
  velocity is a float declared at global level, it is the current measured velocity of the shooter wheels
  /////////////////////////////////////////////*/
//if (current >.2 && Math.abs(error-lastErrorGlyph) < .1 && runtime.milliseconds() > (shovel_time + 500)){
//    shovel_time = runtime.milliseconds();
//    boat.glyph_flipper.setPower(0);
//}
//else {
//        if (boat.armPotentiometer.getVoltage() < .002 || boat.armPotentiometer.getVoltage() > 2.7){
//            boat.glyph_flipper.setPower(0);
//        }else {
        if (Math.abs(desiredPosition - potValue) > .001 && opModeIsActive()) {

            //  drive();
            if (error < integralActiveZone && error != 0) {// total error only accumulates where        /                                                                                            //there is error, and when the error is
                //within the integral active zone
                //DON'T
                // Check for integral until you're within a certain limit
                errorT += error;// adds error to the total each time through the loop
            } else {
                errorT = 0;// if error = zero or error is not withing the active zone, total       /                                                    //error is set to zero
            }
            if (errorT > 50 / ki) { //caps total error at 50
                errorT = 50 / ki;
            }
            if (error == 0) {
                derivative = 0; // if error is zero derivative term is zero
            }
            proportion = error * kp; // sets proportion term
            integral = errorT * ki;// sets integral term
            derivative = (error - lastErrorGlyph) * kd;// sets derivative term
            lastErrorGlyph = error; // sets the last error to current error so we can use it in the next loop
            current = proportion + integral + derivative;// sets value current as total of all terms

            boat.glyph_flipper.setPower(current * coefficient);

            sleep(20);
            //glyph_flipper_last_time = runtime.milliseconds();// waits so we dont hog all our CPU power or cause loop instability
        } else {
            telemetry.addData(">", "Glyph_flipper PID complete.");
        }
    }
    //}
    //}


    public void servoHorizontal(double startValue){ // this function is supposed to keep the plate
        // attached to the glyphs horizontal even as the glyph_flipper moves by
        //errorServo = startValue;// THIS IS FOR STARTING POSITION // used to say start at .26
        errorServo = errorServo + (boat.armPotentiometer.getVoltage()-firstError)/1.7; // need to convert values later

        if (Math.abs(boat.armPotentiometer.getVoltage()-firstError) > .01) {
            boat.glyph_aligner.setPosition(errorServo);
        }
        //increment the glyph aligner servo by the fraction of error that the glyph arm has moved
        firstError = boat.armPotentiometer.getVoltage();
        //sleep(10);
        if (errorServo < 0.001){
            boat.glyph_aligner.setPosition(.004);
        }
// store the last potentiometer value    //80.83/230                                    // changing the servo angle according to the armPotentiometer error bound
        // function will always be declared in a toggle such that whenever
    }
    public void Playglyph_flipper_runToPosition(double desiredPosition, double coefficient){ //Replace later with Josh's PID loop
        double desiredTime = 20;
        double potValue = boat.armPotentiometer.getVoltage();
        double kp = .82;   //.8              // proportional konstant
        double ki = 0.5;    //.4         //konstant of integration
        double kd = 0.8;     //0.3 4/10/18 //1.7 and 3.0?      //konstant of derivation
        double current = 0;        //value to be sent to shooter motors
        double integralActiveZone = 2.0;    // zone of error values in which the total error for the                    integral term accumulates
        double errorT = 0;                        // total error accumulated
        //double lastError = 0;                    // last error recorded by the controller
        double proportion;                        // the proportional term
        double integral;                            // the integral term
        double derivative;
        double error = potValue - desiredPosition;

// the derivative term
/*////////////////////////////////////////////////////////
  NOTE:
  Error is a float declared at global level, it represents the difference between target velocity and current velocity
  power is a float declared at global level, it represents target velocity
  velocity is a float declared at global level, it is the current measured velocity of the shooter wheels
  /////////////////////////////////////////////*/
//        if (boat.armPotentiometer.getVoltage() < .002 || boat.armPotentiometer.getVoltage() > 2.7){
//            boat.glyph_flipper.setPower(0);
//        }else {

        if (Math.abs(desiredPosition - potValue) > .001 && opModeIsActive()) {

            //  drive();
            if (error < integralActiveZone && error != 0) {// total error only accumulates where        /                                                                                            //there is error, and when the error is
                //within the integral active zone
                //DON'T
                // Check for integral until you're within a certain limit
                errorT += error;// adds error to the total each time through the loop
            } else {
                errorT = 0;// if error = zero or error is not withing the active zone, total       /                                                    //error is set to zero
            }
            if (errorT > 50 / ki) { //caps total error at 50
                errorT = 50 / ki;
            }
            if (error == 0) {
                derivative = 0; // if error is zero derivative term is zero
            }
            proportion = error * kp; // sets proportion term
            integral = errorT * ki;// sets integral term
            derivative = (error - lastErrorGlyph) * kd;// sets derivative term
            lastErrorGlyph = error; // sets the last error to current error so we can use it in the next loop
            current = proportion + integral + derivative;// sets value current as total of all terms

            boat.glyph_flipper.setPower(current * coefficient);

            sleep(20);
            //glyph_flipper_last_time = runtime.milliseconds();// waits so we dont hog all our CPU power or cause loop instability
        } else {
            telemetry.addData(">", "Glyph_flipper PID complete.");
        }
    }
    //}
    public void lowPlayglyph_flipper_runToPosition(double desiredPosition, double coefficient){ //Replace later with Josh's PID loop
        double desiredTime = 20;
        double potValue = boat.armPotentiometer.getVoltage();
        double kp = .8;   //.8              // proportional konstant
        double ki = 0.0;    //.4         //konstant of integration
        double kd = 1.0;     //0.3 4/10/18 //1.7 and 3.0?      //konstant of derivation
        double current = 0;        //value to be sent to shooter motors
        double integralActiveZone = 2.0;    // zone of error values in which the total error for the                    integral term accumulates
        double errorT = 0;                        // total error accumulated
        //double lastError = 0;                    // last error recorded by the controller
        double proportion;                        // the proportional term
        double integral;                            // the integral term
        double derivative;
        double error = potValue - desiredPosition;

// the derivative term
/*////////////////////////////////////////////////////////
  NOTE:
  Error is a float declared at global level, it represents the difference between target velocity and current velocity
  power is a float declared at global level, it represents target velocity
  velocity is a float declared at global level, it is the current measured velocity of the shooter wheels
  /////////////////////////////////////////////*/
//        if (boat.armPotentiometer.getVoltage() < .002 || boat.armPotentiometer.getVoltage() > 2.7){
//            boat.glyph_flipper.setPower(0);
//        }else {

        if (Math.abs(desiredPosition - potValue) > .001 && opModeIsActive()) {

            //  drive();
            if (error < integralActiveZone && error != 0) {// total error only accumulates where        /                                                                                            //there is error, and when the error is
                //within the integral active zone
                //DON'T
                // Check for integral until you're within a certain limit
                errorT += error;// adds error to the total each time through the loop
            } else {
                errorT = 0;// if error = zero or error is not withing the active zone, total       /                                                    //error is set to zero
            }
            if (errorT > 50 / ki) { //caps total error at 50
                errorT = 50 / ki;
            }
            if (error == 0) {
                derivative = 0; // if error is zero derivative term is zero
            }
            proportion = error * kp; // sets proportion term
            integral = errorT * ki;// sets integral term
            derivative = (error - lastErrorGlyph) * kd;// sets derivative term
            lastErrorGlyph = error; // sets the last error to current error so we can use it in the next loop
            current = proportion + integral + derivative;// sets value current as total of all terms

            boat.glyph_flipper.setPower(current * coefficient);

            sleep(20);
            //glyph_flipper_last_time = runtime.milliseconds();// waits so we dont hog all our CPU power or cause loop instability
        } else {
            telemetry.addData(">", "Glyph_flipper PID complete.");
        }
    }


    public void lowstagePlayglyph_flipper_runToPosition(double desiredPosition, double coefficient){ //Replace later with Josh's PID loop
        double desiredTime = 20;
        double potValue = boat.armPotentiometer.getVoltage();
        double kp = .82;   //.8              // proportional konstant
        double ki = 0.1;    //.4         //konstant of integration
        double kd = .8;     //0.3 4/10/18 //1.7 and 3.0?      //konstant of derivation
        double current = 0;        //value to be sent to shooter motors
        double integralActiveZone = 2.0;    // zone of error values in which the total error for the                    integral term accumulates
        double errorT = 0;                        // total error accumulated
        //double lastError = 0;                    // last error recorded by the controller
        double proportion;                        // the proportional term
        double integral;                            // the integral term
        double derivative;
        double error = potValue - desiredPosition;

// the derivative term
/*////////////////////////////////////////////////////////
  NOTE:
  Error is a float declared at global level, it represents the difference between target velocity and current velocity
  power is a float declared at global level, it represents target velocity
  velocity is a float declared at global level, it is the current measured velocity of the shooter wheels
  /////////////////////////////////////////////*/
//        if (boat.armPotentiometer.getVoltage() < .002 || boat.armPotentiometer.getVoltage() > 2.7){
//            boat.glyph_flipper.setPower(0);
//        }else {

        if (Math.abs(desiredPosition - potValue) > .001 && opModeIsActive()) {

            //  drive();
            if (error < integralActiveZone && error != 0) {// total error only accumulates where        /                                                                                            //there is error, and when the error is
                //within the integral active zone
                //DON'T
                // Check for integral until you're within a certain limit
                errorT += error;// adds error to the total each time through the loop
            } else {
                errorT = 0;// if error = zero or error is not withing the active zone, total       /                                                    //error is set to zero
            }
            if (errorT > 50 / ki) { //caps total error at 50
                errorT = 50 / ki;
            }
            if (error == 0) {
                derivative = 0; // if error is zero derivative term is zero
            }
            proportion = error * kp; // sets proportion term
            integral = errorT * ki;// sets integral term
            derivative = (error - lastErrorGlyph) * kd;// sets derivative term
            lastErrorGlyph = error; // sets the last error to current error so we can use it in the next loop
            current = proportion + integral + derivative;// sets value current as total of all terms

            boat.glyph_flipper.setPower(current * coefficient);

            sleep(20);
            //glyph_flipper_last_time = runtime.milliseconds();// waits so we dont hog all our CPU power or cause loop instability
        } else {
            telemetry.addData(">", "Glyph_flipper PID complete.");
        }
    }



    public void shovelFlip() {
        pButtonSwitch2 = cButtonSwitch2;
        cButtonSwitch2 = gamepad1.right_bumper;
//        if(gamepad1.right_bumper) {
//            gamepad2_a_toggle = false;
//            gamepad2_x_toggle = false;
//            gamepad2_y_toggle = false;
//
//            if (cButtonSwitch2 && !pButtonSwitch2) {
//                switchServo2 = switchServo2 ? false : true;
//            }
//
//            if (switchServo2) {
//                boat.glyph_grabber.setPosition(SERVO_SWITCH_OPEN2);
//            } else if (!switchServo2) {
//                boat.glyph_grabber.setPosition(SERVO_SWITCH_CLOSE2);
//
//            }
//        }
        double desiredTime = 300;

        if (gamepad2.dpad_left && runtime.milliseconds() > (glyph_grab_last_press_time2 + desiredTime)) { //glyph flicking
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
            gamepad2_y_toggle = false;

            if (switchangle2 == .39 && Shovelness == false) {
                boat.glyph_grabber.setPosition(.60); //.12
                switchangle2 = .65;
            } else if (switchangle2 == .65 && Shovelness == false) {
                boat.glyph_grabber.setPosition(.30); //.36 3/6
                switchangle2 = .39;
            }

            glyph_grab_last_press_time2 = runtime.milliseconds();

        }

        if ((gamepad1.y|| gamepad2.dpad_left) && runtime.milliseconds() > (glyph_grab_last_press_time2 + desiredTime)) { //glyph flicking
            gamepad2_a_toggle = false;
            gamepad2_x_toggle = false;
            gamepad2_y_toggle = false;

            if (switchangle2 == .39 && Shovelness == true) {
                boat.glyph_grabber.setPosition(.65);
                //boat.glyph_aligner.setPosition(boat.glyph_aligner.getPosition() - .05);//added new line
                switchangle2 = .65;
            } else if (switchangle2 == .65 && Shovelness == true) {
                boat.glyph_grabber.setPosition(.4);
                switchangle2 = .39;
            }
            glyph_grab_last_press_time2 = runtime.milliseconds();
        }
        if ((gamepad2_x_toggle == false || gamepad2_y_toggle == false) && gamepad2_a_toggle == false){
            if (gamepad2.dpad_down) {
                boat.glyph_flipper.setPower(.36);
            } else if (gamepad2.dpad_up) {
                boat.glyph_flipper.setPower(-.3);
            }
            else{
                boat.glyph_flipper.setPower(0);
            }
        }


        if (gamepad2_a_toggle == true) { //glyph load
            //toggle
            pause = false;
            pause2 = false;
            stall = false;
            if (boat.armPotentiometer.getVoltage() > 3.3) {
                boat.glyph_grabber.setPosition(.30); //.42 4/17/18
                boat.glyph_aligner.setPosition(.72); //.64 4/14   CHANGE
                boat.glyph_aligner2.setPosition(.72);
            }
//            else if (boat.armPotentiometer.getVoltage()> .9){
//                boat.glyph_aligner.setPosition(.95);
//            }

//            else if (boat.armPotentiometer.getVoltage()>.7){
//                boat.glyph_aligner.setPosition(.38);
//            }
            else if (boat.armPotentiometer.getVoltage() > 1.0) {
                boat.glyph_aligner.setPosition(.92);
                boat.glyph_aligner2.setPosition(.92);

            } else {
                boat.glyph_aligner.setPosition(.6);
                boat.glyph_aligner2.setPosition(.6);

            }
//            else if (boat.armPotentiometer.getVoltage()>0.0){
//                boat.glyph_aligner.setPosition(.15);
//            }


            if (lowhigh == true) {
                glyph_flipper_runToPosition(3.345, 0.09);//2.29 3/6/18
            } else {
                glyph_flipper_runToPosition(3.345, 0.04);//.11 3/6/18
            }


            moveShovelSafe = false;
            switchangle2 = .39;
            Shovelness = false;


        }
//        if (gamepad2_y_toggle == true) {
//            boat.glyph_aligner.setPosition(.33);
//        }


        boolean disarm = false;
        if (gamepad2_y_toggle == true) { //glyph high
//            boat.left_intake.setPower(-.3);
//            boat.right_intake.setPower(-.3);
            lowhigh = true;
//            if (boat.armPotentiometer.getVoltage() <.55){
//                stallShovel(.39);
//            }
//            else if () { // CHANGE THE IF STATEMENT DIM WIT
//                Playglyph_flipper_runToPosition(.4, .8); //.45 3/7/18 OLD ONE FROM FEB 15 THAT WORKS //.438 SHOULD BE RIGHT 3/5
//                //Playglyph_flipper_runToPosition(.25, 1.0); // OLD ONE FROM FEB 15 THAT WORKS
//            }

//            if (moveShovelSafe == true){
//                Playglyph_flipper_runToPosition(); // CHANGE
//
//            }


////
//        if (boat.armPotentiometer.getVoltage() <2.2) {
            boat.glyph_grabber.setPosition(.65);
            // }
            if (gamepad2.dpad_down) {
                boat.glyph_flipper.setPower(.36);
                stall = true;
            } else if (gamepad2.dpad_up) {
                boat.glyph_flipper.setPower(-.4);
                stall = true;

            } else {


                //else {
                if (moveShovelSafe == true && stall == false) {
//                    if (boat.armPotentiometer.getVoltage() < 1.0) { //used to be 1.0 4/17
//                        stallShovel(.85);
//                        //boat.glyph_flipper.setPower(-.15);
//                    } else {
                    Playglyph_flipper_runToPosition(.9, .20); //.9, .19 4/18
                    //}
                } else {
                    boat.glyph_flipper.setPower(0);
                }
            }
            if (boat.armPotentiometer.getVoltage() < 2.8) { //used to be 2.45 4/17
                boat.glyph_aligner.setPosition(.43);
                boat.glyph_aligner2.setPosition(.43);

            } else if (boat.armPotentiometer.getVoltage() > 2.9) {
                boat.glyph_aligner.setPosition(1.0);
                boat.glyph_aligner2.setPosition(1.0);


                if (boat.glyph_aligner.getPosition() == 1.0 || boat.glyph_aligner.getPosition() == .43) {
                    moveShovelSafe = true; // CHANGE THE IF STATEMENT DIM WIT             // .07 3/6 march

                } else {

                }
                // }

                if (boat.glyph_aligner.getPosition() == 1.0 && pause == false) {
                    sleep(350);
                    pause = true;
                }

                boat.glyph_grabber.setPosition(.65);
                switchangle2 = .65;
                switchangle = .65;
//            if (boat.armPotentiometer.getVoltage()<.50){
//                boat.glyph_aligner.setPosition(.09);  //.11 3/8/18
//            }
                // else

                Shovelness = true;
//            if (boat.armPotentiometer.getVoltage()< 1.90) {
//                boat.glyph_grabber.setPosition(.65);
//                switchangle2 = .65;
//            }
//            if (boat.armPotentiometer.getVoltage()<1.7){
//                boat.glyph_aligner.setPosition(.08);                // .15, 9.0 was for hover
//            }
//            else if (boat.armPotentiometer.getVoltage()>2.0){
//                boat.glyph_aligner.setPosition(.7);         //NEED TO FIND THIS       // .15, 9.0 was for hover
//
//            }


//            if (boat.armPotentiometer.getVoltage() < 1.8){
//                boat.glyph_aligner.setPosition(.20);
//
//            }
//////            else{
//            if (boat.armPotentiometer.getVoltage() < 1.0){
//                servoSlow(0.07, 8.0); //.18
//
//            }
//            else if (boat.armPotentiometer.getVoltage() < 2.3) {
//                servoSlow(.07, 14.0); //.18
//            }


            }

            //double desiredTime = 100;

//        if (gamepad1.b == true && runtime.milliseconds() > (glyph_grab_last_press_time + desiredTime)){ //glyph flicking
//            gamepad2_a_toggle = false;
//            gamepad2_x_toggle = false;
//            gamepad2_y_toggle = false;
//            if (Shovelness == true){
//                if (lowhigh == true){
//                    if (switchangle == .52){
//                        boat.glyph_aligner.setPosition(.05);
//                        switchangle = .65;
//                    }
//                    else if (switchangle == .65){
//                        boat.glyph_aligner.setPosition(.23);
//                        switchangle = .52;
//                    }
//                }
//                else{
//                    if (switchangle == .52){
//                        boat.glyph_aligner.setPosition(.01);
//                        switchangle = .65;
//                    }
//                    else if (switchangle == .65){
//                        boat.glyph_aligner.setPosition(.09);
//                        switchangle = .52;
//                    }
//                }
//
//            }
//            else {
//                if (switchangle == .52){
//                    boat.glyph_aligner.setPosition(.32);
//                    switchangle = .65;
//                }
//                else if (switchangle == .65){
//                    boat.glyph_aligner.setPosition(.5);
//                    switchangle = .52;
//                }
//            }
//
//            glyph_grab_last_press_time = runtime.milliseconds();
//            boat.glyph_flipper.setPower(0);
//
//        }
        }
//THIS WAS FOR GLYPH STACKER
        if (gamepad2.b && runtime.milliseconds() - gamepad1_b_relic_toggle_time > 350) { //pushing the glyphs in
            gamepad1_b_relic_toggle_time = runtime.milliseconds();
            gamepad1_b_relic_toggle = toggle(gamepad1_b_relic_toggle);
        }
        if (gamepad2.right_stick_y<-.1){
            boat.glyph_stacker.setPosition(.25);
        }
        else if (gamepad2_x_toggle == true || gamepad2_y_toggle == true) {
            boat.glyph_stacker.setPosition(.8);
        } else {
            if (gamepad1_b_relic_toggle) {
                boat.glyph_stacker.setPosition(.64); //Open used to be .72
                open = true;
            } else if (!gamepad1_b_relic_toggle) {
//                if (boat.glyph_stacker.getPosition() == .5|| open == false){
//                    boat.glyph_stacker.setPosition(.93);
//                    open = false;
//                }
//                else {
                boat.glyph_stacker.setPosition(1.0); //Closed
                sleep(300);
                gamepad1_b_relic_toggle = true;               // }
            }
        }

        boolean burton = true;
        if (gamepad2_x_toggle == true) {
            lowhigh = false;
            lowhigh = true;
            telemetry.addData("> ","x toggle");

//            if (boat.armPotentiometer.getVoltage() <.55){
//                stallShovel(.39);
//            }
//            else if () { // CHANGE THE IF STATEMENT DIM WIT
//                Playglyph_flipper_runToPosition(.4, .8); //.45 3/7/18 OLD ONE FROM FEB 15 THAT WORKS //.438 SHOULD BE RIGHT 3/5
//                //Playglyph_flipper_runToPosition(.25, 1.0); // OLD ONE FROM FEB 15 THAT WORKS
//            }

//            if (moveShovelSafe == true){
//                Playglyph_flipper_runToPosition(); // CHANGE
//
//            }


////
//        if (boat.armPotentiometer.getVoltage() <2.2) {
            boat.glyph_grabber.setPosition(.65);
            // }
            if (gamepad2.dpad_down) {
                boat.glyph_flipper.setPower(.36);
                stall = true;
            } else if (gamepad2.dpad_up) {
                boat.glyph_flipper.setPower(-.4);
                stall = true;

            } else {


                //else {
                if (moveShovelSafe == true && stall == false) {
//                        if (boat.armPotentiometer.getVoltage() < 1.1) { //used to be 1.0 4/17
//                            stallShovel(.85);
//                            //boat.glyph_flipper.setPower(-.15);
//                        } else {
                    Playglyph_flipper_runToPosition(.9, .20); //.8, .19 4/18
                    //}
                } else {
                    boat.glyph_flipper.setPower(0);
                }
            }
            if (boat.armPotentiometer.getVoltage() < 2.4) { //used to be 2.45 4/17
                boat.glyph_aligner.setPosition(.52);
                boat.glyph_aligner2.setPosition(.52);

            } else if (boat.armPotentiometer.getVoltage() > 2.8) {
                boat.glyph_aligner.setPosition(1.0);
                boat.glyph_aligner2.setPosition(1.0);


                if (boat.glyph_aligner.getPosition() == 1.0 || boat.glyph_aligner.getPosition() == .52) {
                    moveShovelSafe = true; // CHANGE THE IF STATEMENT DIM WIT             // .07 3/6 march

                } else {

                }
                // }

                if (boat.glyph_aligner.getPosition() == 1.0 && pause == false) {
                    sleep(350);
                    pause = true;
                }

                boat.glyph_grabber.setPosition(.65);
                switchangle2 = .65;
                switchangle = .65;
//            if (boat.armPotentiometer.getVoltage()<.50){
//                boat.glyph_aligner.setPosition(.09);  //.11 3/8/18
//            }
                // else

                Shovelness = true;
//            if (boat.armPotentiometer.getVoltage()< 1.90) {
//                boat.glyph_grabber.setPosition(.65);
//                switchangle2 = .65;
//            }
//            if (boat.armPotentiometer.getVoltage()<1.7){
//                boat.glyph_aligner.setPosition(.08);                // .15, 9.0 was for hover
//            }
//            else if (boat.armPotentiometer.getVoltage()>2.0){
//                boat.glyph_aligner.setPosition(.7);         //NEED TO FIND THIS       // .15, 9.0 was for hover
//
//            }


//            if (boat.armPotentiometer.getVoltage() < 1.8){
//                boat.glyph_aligner.setPosition(.20);
//
//            }
//////            else{
//            if (boat.armPotentiometer.getVoltage() < 1.0){
//                servoSlow(0.07, 8.0); //.18
//
//            }
//            else if (boat.armPotentiometer.getVoltage() < 2.3) {
//                servoSlow(.07, 14.0); //.18
//            }


            }

//        else{
//            errorServo = .835;
//             firstError = boat.armPotentiometer.getVoltage();

            //boat.glyph_flipper.setPower(0);

            //}


//        else{
//            errorServo = .835;
//             firstError = boat.armPotentiometer.getVoltage();

            //boat.glyph_flipper.setPower(0);

            //}

        }
    }

    public double servoAdjustAligner(double servo_angle) {

        if (gamepad1.right_trigger>.3) { //CONFLICTING CONTROL KEYS

            servo_angle = boat.glyph_aligner.getPosition() - 4.0 / 180.0;

            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.glyph_aligner.setPosition(servo_angle);
            boat.glyph_aligner2.setPosition(servo_angle);
        }

        if (gamepad1.left_trigger>.3) {
            servo_angle = boat.glyph_aligner.getPosition() + 4.0 / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.glyph_aligner.setPosition(servo_angle);
            boat.glyph_aligner2.setPosition(servo_angle);

        }

        telemetry.addData(">", "Current Angle: " + servo_angle);
        return servo_angle;
    }


    public void stallShovel( double target){
        double currentPos = boat.armPotentiometer.getVoltage();

        boat.glyph_flipper.setPower(Math.signum(currentPos - target)*((-.27/Math.sqrt(1+2000*(currentPos-target)*(currentPos-target)))+.27)); //used to be .3

    }

    public double servoSlow(double shovelServo, double increment) {
        //double servo_angle = .84;
        servo_angle = boat.glyph_aligner.getPosition();
        if (boat.glyph_aligner.getPosition() < .12){
            boat.glyph_aligner.setPosition(.1); //used to be .3
        }
        else if ((boat.glyph_aligner.getPosition() - shovelServo) < - (increment+1.0) / 180.0) { //CONFLICTING CONTROL KEYS
            servo_angle = boat.glyph_aligner.getPosition() + increment / 180.0; // used to be 10
            // boat.glyph_aligner.setPosition(151.0/180.0);

            boat.glyph_aligner.setPosition(servo_angle);
        }

        else  if ((boat.glyph_aligner.getPosition() - shovelServo) >  (increment+1.0 )/ 180.0)  {
            servo_angle = boat.glyph_aligner.getPosition() - increment / 180.0;
            //boat.glyph_aligner.setPosition(39.0/180.0);
            boat.glyph_aligner.setPosition(servo_angle);
        }


        telemetry.addData(">", "Current Angle: " + servo_angle*180);
        return servo_angle;
    }


    //////////////////////////
    //////////////////////////
   /* INTAKE RELATED STUFF */
    //////////////////////////
    //////////////////////////

    public void killUrself(){
        if (gamepad2.start && doKillUrself == true) {
            if (killUrself == true) {
                killUrself = false;
            } else {
                killUrself = true;
            }
            doKillUrself = false;
        } else if (!gamepad2.start) {
            doKillUrself = true;
        }
    }
    boolean gamepad2_leftbumper_toggle = false;

    public void intake(){
        intakeSimple();
        int desiredTime = 1000;
        if(gamepad2.left_bumper && runtime.milliseconds() > (intake_last_press + desiredTime)){ //used to be gamepad1.right bumper
            intake_last_press = runtime.milliseconds();
            gamepad2_leftbumper_toggle = toggle(gamepad2_leftbumper_toggle);
        }
    }


    public void intakeSimple() {
        if (relicMode == false) {
            if (gamepad2.right_trigger > .2) { //intake
                boat.right_intake.setPower(1);
                boat.left_intake.setPower(1);
                boat.glyph_stacker.setPosition(.25);
            } else if (gamepad1.left_trigger > .2 && gamepad1.left_bumper) {
                boat.right_intake.setPower(1.0);
                boat.left_intake.setPower(1.0);
            } else if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .05) {
                boat.right_intake.setPower(-gamepad2.left_trigger);
                boat.left_intake.setPower(-gamepad2.left_trigger);
//
//            boat.right_intake.setPower(-.9);
//            boat.left_intake.setPower(-1.0);
            } else if (gamepad2_leftbumper_toggle == true) {
                boat.glyph_stacker.setPosition(.25);
                boat.right_intake.setPower(1);
                boat.left_intake.setPower(1);
                sleep(400);
                boat.right_intake.setPower(-1.0);
                boat.left_intake.setPower(-1.0);
                sleep(400);
                boat.right_intake.setPower(0.0);
                boat.left_intake.setPower(0.0);
                gamepad2_leftbumper_toggle = false;
            } else {
                boat.right_intake.setPower(0);
                boat.left_intake.setPower(0);

            }
        }
        else{
            if (gamepad1.right_bumper) { //intake
                boat.right_intake.setPower(1);
                boat.left_intake.setPower(1);
                boat.glyph_stacker.setPosition(.25);
            } else if (gamepad1.left_trigger > .2) {
                boat.right_intake.setPower(-1.0);
                boat.left_intake.setPower(-1.0);
            }
            else {
                boat.right_intake.setPower(0);
                boat.left_intake.setPower(0);
            }
        }

    }



    /////////////////////////////
    /////////////////////////////
   /* MISCELLANEOUS FUNCTIONS */
    /////////////////////////////
    /////////////////////////////



    public void winch() {
        if(Math.abs(gamepad2.right_stick_y)>.2) {
            boat.winch.setPower(gamepad2.right_stick_y);
        }
    }

    public void jewel_arm() {

        //deciding the state if the switch servo is at pos 1 || 0
        pButtonSwitch = cButtonSwitch;
        cButtonSwitch = gamepad2.left_bumper;
//        if(gamepad2.left_bumper && !gamepad2.dpad_up && !gamepad2.dpad_down) {
//            if (cButtonSwitch && !pButtonSwitch) {
//                switchServo = switchServo ? false : true;
//            }
//
//            if (switchServo) {
//                boat.jewel_arm.setPosition(SERVO_SWITCH_OPEN);
//            } else if (!switchServo) {
//                boat.jewel_arm.setPosition(SERVO_SWITCH_CLOSE);
//
//            }
//        }
        if(gamepad2.dpad_right){
            boat.jewel_arm.setPosition(boat.jewel_arm.getPosition() - 3/180.0);

        }
        else if(gamepad2.dpad_left){
            boat.jewel_arm.setPosition(boat.jewel_arm.getPosition() + 3/180.0);
        }
        else{
            boat.jewel_arm.setPosition(boat.jewel_arm.getPosition());
        }
    }



    public void getPotValues(){
        double value = boat.armPotentiometer.getVoltage();
        //double intakevalue = boat.intakePotentiometer.getVoltage();// 0 - 3.34
        telemetry.addData("Arm Pot",  value);
        //telemetry.addData("intake Pot", intakevalue);
    }

    /////////////////////
// RELIC ////////////
/////////////////////
    public void relicModeToggle(){
        if(gamepad2.right_stick_button && runtime.milliseconds() - gamepad1_start_relic_toggle_time > 200){
            gamepad1_start_relic_toggle_time = runtime.milliseconds();
            relicMode = toggle(relicMode);
        }
        if(relicMode){
            telemetry.addData(">", "RELIC MODE | A C T I V A T E D |");
            telemetry.addData(">", "THE POWERS OF THE RELICS HAVE BESTOWED THEIR POWER UPON YOU. USE IT WISELY.");
        }
        else if(!relicMode){
            telemetry.addData(">", "GLYPH MODE | A C T I V A T E D |");
            telemetry.addData(">", "USE YOUR INVESTIGATION SKILLS TO UNLOCK THE MYSTERY BEHIND THE GLYPHS.");
        }
        telemetry.addData("relicMode", relicMode);
    }
    public void relic(){
        relicGrabber();
        relicFlipper();
    }
    boolean extended = false;
    public void relicSimple() {
        if (gamepad2.left_stick_y > .1) {

            boat.relic_extender.setPower(gamepad2.left_stick_y * .3);

        }

        else if (gamepad2.left_stick_y < -0.1) {

            boat.relic_extender.setPower(gamepad2.left_stick_y * .3);

        }

        else {
            boat.relic_extender.setPower(0);
        }
    }
    public void relicSlide(){ //Sliiiiides
        // up on right stick is extend
        telemetry.addData("y stick", gamepad2.left_stick_y);
        if (boat.relic_extender.getCurrentPosition() > 1000) {
            if (gamepad2.left_stick_y >= 0) {
                if (gamepad2.left_trigger > .2) {
                    boat.relic_extender.setPower(gamepad2.left_stick_y * .44);
                    telemetry.addData("hi extend", "hi");

                } else {
                    boat.relic_extender.setPower(gamepad2.left_stick_y);

                }
            }
        }
        else if (gamepad2.left_stick_y >=0){
            boat.relic_extender.setPower(0);
        }

        if (boat.relic_extender.getCurrentPosition() < 7000) {
            if (gamepad2.left_stick_y <= 0) {
                if (gamepad2.left_trigger > .2) {
                    boat.relic_extender.setPower(gamepad2.left_stick_y * .44);
                    telemetry.addData("hi retract", "hi");

                } else {
                    boat.relic_extender.setPower(gamepad2.left_stick_y);
                }
            }
        }
        else if (gamepad2.left_stick_y <=0){
            boat.relic_extender.setPower(0);
        }

    }

    public void relicGrabber(){ //Opens and closes the relic grabber
        if(gamepad2.b && relicMode && runtime.milliseconds() - gamepad1_b_relic_toggle_time > 350){
            gamepad1_b_relic_toggle_time = runtime.milliseconds();
            gamepad1_b_relic_toggle = toggle(gamepad1_b_relic_toggle);
        }
        if(gamepad1_b_relic_toggle){
            boat.relic_grabber.setPosition(0.19); //Open
        }
        else if(!gamepad1_b_relic_toggle){
            boat.relic_grabber.setPosition(0.7); //Closed
        }
    }
    public void relicFlipper(){ //Has the ability to adjust and set to specific positions
        if(gamepad2.a && runtime.milliseconds() - lastFlipTime > 25){
            lastFlipTime = runtime.milliseconds();
            boat.relic_flipper.setPosition(boat.relic_flipper.getPosition() + 0.02);
        }
        else if(gamepad2.y && runtime.milliseconds() - lastFlipTime > 25){
            lastFlipTime = runtime.milliseconds();
            boat.relic_flipper.setPosition(boat.relic_flipper.getPosition() - 0.02);
        }
        if(gamepad2.right_trigger>.1){
                boat.relic_flipper.setPosition(0.02); //.1 High
        }
        if(gamepad2.left_bumper){
            boat.relic_flipper.setPosition(0.85); //Low//.87
        }
        if(gamepad2.x){
            boat.relic_flipper.setPosition(.77);
        }
        telemetry.addData("uwu", boat.relic_flipper.getPosition());
    }


    public boolean toggle(boolean variable){ //Takes a boolean variable then swaps its trueness
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }

}


