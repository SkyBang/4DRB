package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Accelerare2", group="Linear Opmode")

public class Accelerare2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor TleftDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor TrightDrive = null;
    private DcMotor BrightDrive = null;
    private Servo ServoRight = null;
    private Servo ServoLeft = null;
    private DcMotor ColectorE = null;
    private DcMotor ColectorV = null;
    private Servo Mascota = null;
    private CRServo Parcare = null;
    double power;
    double mersbine = 0;
    int ok = 1;
    int i = 1;
    int n = 1;
    double FranaS;
    double FranaD;
    public Servo ServomotorE = null;
    public Servo ServomotorV = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        TleftDrive = hardwareMap.get(DcMotor.class, "Tleft_drive");
        TrightDrive = hardwareMap.get(DcMotor.class, "Tright_drive");
        BleftDrive = hardwareMap.get(DcMotor.class, "Bleft_drive");
        BrightDrive = hardwareMap.get(DcMotor.class, "Bright_drive");
        ColectorE = hardwareMap.get(DcMotor.class, "ColectorE");
        ColectorV = hardwareMap.get(DcMotor.class, "ColectorV");
        ServoLeft = hardwareMap.get(Servo.class, "Servo1");
        ServoRight = hardwareMap.get(Servo.class, "Servo2");
        Mascota = hardwareMap.get(Servo.class, "Mascota");
        Parcare = hardwareMap.get(CRServo.class, "Parcare");
        ServomotorE = hardwareMap.servo.get("Servo_spateE");
        ServomotorV = hardwareMap.servo.get("Servo_spateV");

        TleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColectorE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ColectorV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BleftDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            FranaD=gamepad1.right_trigger;
            FranaS=gamepad1.left_trigger;
            power = 0.2;
            if (ok == 1) {
                ServomotorE.setPosition(0.2);
                ServomotorV.setPosition(0.2);
                ok = 0;
            }
            while (mersbine == 0) {
                i = 1;
                power = 0.2;
                n = 1;
                if (gamepad2.dpad_down)
                    Mascota.setPosition(1);
                if (gamepad2.dpad_up)
                    Mascota.setPosition(-1);
                if (gamepad2.a) {
                    ServomotorE.setPosition(0.5);
                    ServomotorV.setPosition(0.6);
                }
                if (gamepad2.y) {
                    ServomotorE.setPosition(0.2);
                    ServomotorV.setPosition(0.2);
                }
                while (gamepad2.dpad_left) {
                    Parcare.setPower(1);
                    sleep(100);
                    if (!(gamepad2.dpad_down)) {
                        Parcare.setPower(0);
                    }
                }
                while (gamepad2.dpad_right) {
                    Parcare.setPower(-1);
                    sleep(100);
                    if (!(gamepad2.dpad_down)) {
                        Parcare.setPower(0);
                    }
                }
                ColectorE.setPower(0.7);
                ColectorV.setPower(0.7);
                while (gamepad1.dpad_up && i <= 9) {
                    if (power < 1.5) {
                        power = n * 0.2;
                    } else power = 1;
                    TleftDrive.setPower(-power);
                    BleftDrive.setPower(-power);
                    TrightDrive.setPower(-power);
                    BrightDrive.setPower(-power);

                    CurbaFata(1,FranaD,FranaS);

                    n++;
                    sleep(250);
                    i++;
                    if (gamepad1.left_trigger == 0)
                        power = 1;
                    if (gamepad1.right_trigger == 0)
                        power = 1;
                    if (!gamepad1.dpad_up) {
                        TleftDrive.setPower(0);
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        BrightDrive.setPower(0);
                        break;
                    }
                }
                i = 1;


                while (gamepad1.dpad_down && i <= 9) {
                    if (power < 1.5) {
                        power = n * 0.2;
                    } else power = 1;
                    TleftDrive.setPower(power);
                    BleftDrive.setPower(power);
                    TrightDrive.setPower(power);
                    BrightDrive.setPower(power);

                    CurbaSpate(1,FranaD,FranaS);

                    n++;
                    sleep(250);
                    i++;
                    if (gamepad1.left_trigger == 0)
                        power = 1;
                    if (gamepad1.right_trigger == 0)
                        power = 1;

                    if (!gamepad1.dpad_down) {
                        TleftDrive.setPower(0);
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        BrightDrive.setPower(0);
                        power = 1;
                        break;
                    }
                }

                while (gamepad1.dpad_left) {
                    TleftDrive.setPower(power);
                    BleftDrive.setPower(-power);
                    TrightDrive.setPower(-power);
                    BrightDrive.setPower(power);
                    if (gamepad1.left_trigger != 0)
                        power = 0.5;
                    if (gamepad1.left_trigger == 0)
                        power = 1;
                    if (!gamepad1.dpad_left) {
                        TleftDrive.setPower(0);
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        BrightDrive.setPower(0);
                        power = 1;
                        break;
                    }
                }

                while (gamepad1.dpad_right) {
                    TleftDrive.setPower(-power);
                    BleftDrive.setPower(power);
                    TrightDrive.setPower(power);
                    BrightDrive.setPower(-power);
                    if (gamepad1.left_trigger != 0)
                        power = 0.2;
                    if (gamepad1.left_trigger == 0)
                        power = 1;
                    if (!gamepad1.dpad_right) {
                        TleftDrive.setPower(0);
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        BrightDrive.setPower(0);
                        power = 1;
                        break;
                    }
                }

                while (gamepad1.y) {
                    TleftDrive.setPower(1);
                    BrightDrive.setPower(1);
                    if (!(gamepad1.y)) {
                        TleftDrive.setPower(0);
                        BrightDrive.setPower(0);
                        break;
                    }
                }
                while (gamepad1.a) {
                    BleftDrive.setPower(1);
                    TrightDrive.setPower(1);
                    if (!(gamepad1.a)) {
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        break;
                    }
                }
                while (gamepad1.x) {
                    TleftDrive.setPower(-1);
                    BrightDrive.setPower(-1);
                    if (!(gamepad1.x)) {
                        TleftDrive.setPower(0);
                        BrightDrive.setPower(0);
                        break;
                    }
                }
                while (gamepad1.b) {
                    BleftDrive.setPower(-1);
                    TrightDrive.setPower(-1);
                    if (!(gamepad1.b)) {
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        break;
                    }
                }
                while (gamepad1.left_bumper) {
                    TleftDrive.setPower(1);
                    BleftDrive.setPower(1);
                    TrightDrive.setPower(-1);
                    BrightDrive.setPower(-1);
                    if (!gamepad1.left_bumper) {
                        TleftDrive.setPower(0);
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        BrightDrive.setPower(0);
                        break;
                    }
                }
                while (gamepad1.right_bumper) {
                    TleftDrive.setPower(-1);
                    BleftDrive.setPower(-1);
                    TrightDrive.setPower(1);
                    BrightDrive.setPower(1);
                    if (!gamepad1.right_bumper) {
                        TleftDrive.setPower(0);
                        BleftDrive.setPower(0);
                        TrightDrive.setPower(0);
                        BrightDrive.setPower(0);
                        break;
                    }
                }
                if (gamepad1.start) {
                    ColectorE.setPower(0);
                    ColectorV.setPower(0);
                    sleep(100);
                    mersbine = 1;
                }
            }
            while (gamepad2.dpad_left) {
                Parcare.setPower(1);
                if (!(gamepad2.dpad_down)) {
                    Parcare.setPower(0);
                }
            }
            if (gamepad2.dpad_down)
                Mascota.setPosition(1);
            if (gamepad2.dpad_up)
                Mascota.setPosition(-1);
            if (gamepad2.a) {
                ServomotorE.setPosition(0.5);
                ServomotorV.setPosition(0.6);
            }
            if (gamepad2.y) {
                ServomotorE.setPosition(0.2);
                ServomotorV.setPosition(0.2);
            }
            while (gamepad2.dpad_right) {
                Parcare.setPower(-1);
                if (!(gamepad2.dpad_down)) {
                    Parcare.setPower(0);
                }
            }
            //am iesit din while!!!!!!!!!!
            if (gamepad1.back) {
                ColectorE.setPower(-0.7);
                ColectorV.setPower(-0.7);
                sleep(1000);
                mersbine = 0;
            }
            i = 1;
            power = 0.2;
            n = 1;
            while (gamepad1.dpad_up && i <= 9) {
                if (power < 1.5) {
                    power = n * 0.2;
                } else power = 1;
                TleftDrive.setPower(-power);
                BleftDrive.setPower(-power);
                TrightDrive.setPower(-power);
                BrightDrive.setPower(-power);

                CurbaFata(1,FranaD,FranaS);

                n++;
                sleep(250);
                i++;
                if (gamepad1.left_trigger == 0)
                    power = 1;
                if (gamepad1.right_trigger == 0)
                    power=1;
                if (!gamepad1.dpad_up) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            i = 1;


            while (gamepad1.dpad_down && i <= 9) {
                if (power < 1.5) {
                    power = n * 0.2;
                } else power = 1;
                TleftDrive.setPower(power);
                BleftDrive.setPower(power);
                TrightDrive.setPower(power);
                BrightDrive.setPower(power);

                CurbaSpate(1,FranaD,FranaS);

                n++;
                sleep(250);
                i++;
                if (gamepad1.left_trigger == 0)
                    power = 1;
                if (gamepad1.right_trigger == 0)
                    power=1;
                if (!gamepad1.dpad_down) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 1;
                    break;
                }
            }

            while (gamepad1.dpad_left) {
                TleftDrive.setPower(power);
                BleftDrive.setPower(-power);
                TrightDrive.setPower(-power);
                BrightDrive.setPower(power);
                if (gamepad1.left_trigger != 0)
                    power = 0.3;
                if (gamepad1.left_trigger == 0)
                    power = 1;
                if (!gamepad1.dpad_left) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 1;
                    break;
                }
            }

            while (gamepad1.dpad_right) {
                TleftDrive.setPower(-power);
                BleftDrive.setPower(power);
                TrightDrive.setPower(power);
                BrightDrive.setPower(-power);
                if (gamepad1.left_trigger != 0)
                    power = 0.3;
                if (gamepad1.left_trigger == 0)
                    power = 1;
                if (!gamepad1.dpad_right) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    power = 1;
                    break;
                }
            }

            while (gamepad1.y) {
                TleftDrive.setPower(1);
                BrightDrive.setPower(1);
                if (!(gamepad1.y)) {
                    TleftDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.a) {
                BleftDrive.setPower(1);
                TrightDrive.setPower(1);
                if (!(gamepad1.a)) {
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.x) {
                TleftDrive.setPower(-1);
                BrightDrive.setPower(-1);
                if (!(gamepad1.x)) {
                    TleftDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.b) {
                BleftDrive.setPower(-1);
                TrightDrive.setPower(-1);
                if (!(gamepad1.b)) {
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.left_bumper) {
                TleftDrive.setPower(1);
                BleftDrive.setPower(1);
                TrightDrive.setPower(-1);
                BrightDrive.setPower(-1);
                if (!gamepad1.left_bumper) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }
            while (gamepad1.right_bumper) {
                TleftDrive.setPower(-1);
                BleftDrive.setPower(-1);
                TrightDrive.setPower(1);
                BrightDrive.setPower(1);
                if (!gamepad1.right_bumper) {
                    TleftDrive.setPower(0);
                    BleftDrive.setPower(0);
                    TrightDrive.setPower(0);
                    BrightDrive.setPower(0);
                    break;
                }
            }

        }

    }

    public void CurbaFata(double power, double FranaD, double FranaS) {
        FranaD = gamepad1.right_trigger;
        FranaS = gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(-(power - FranaS));
            BleftDrive.setPower(-(power - FranaS));
            TrightDrive.setPower(-power);
            BrightDrive.setPower(-power);
        }
        if (gamepad1.right_trigger != 0) {
            TrightDrive.setPower(-(power - FranaD));
            BrightDrive.setPower(-(power - FranaD));
            TleftDrive.setPower(-power);
            BleftDrive.setPower(-power);
        }

    }

    public void CurbaSpate(double power, double FranaD, double FranaS) {
        FranaD = gamepad1.right_trigger;
        FranaS = gamepad1.left_trigger;

        if (gamepad1.left_trigger != 0) {
            TleftDrive.setPower(power - FranaS);
            BleftDrive.setPower(power - FranaS);
            TrightDrive.setPower(power);
            BrightDrive.setPower(power);
        }
        if (gamepad1.right_trigger != 0) {
            TrightDrive.setPower(power - FranaD);
            BrightDrive.setPower(power - FranaD);
            TleftDrive.setPower(power);
            BleftDrive.setPower(power);
        }
    }
}