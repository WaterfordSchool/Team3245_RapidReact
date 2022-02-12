package frc.robot;

public class RobotMap {
    //can drive motors; CAN IDS CORRECT
    public static final int R1CANID = 1;
    public static final int R2CANID = 2;
    public static final int L1CANID = 3;
    public static final int L2CANID = 4;
    

    //can doing things motors, place holders
    public static final int SORTINGID = 7;
    public static final int DRINTAKEID = 11;

    //pwm ports
    public static final int BLINKINPORT = 1;

    //auto; change names once designate function
    public static final double AUTOTIME1 = 1.0;
    public static final double AUTOTIME2 = 1.0;
    public static final double AUTOTIME3 = 1.0;
    public static final double AUTODRIVESPEED1 = .5;
    public static final double AUTODRIVETURN1 = 0.0;

    //operator controls
    public static final int OPERATORINDEXERBUTTON = 1;
    public static final int SHOOTERBUTTON = 3;
    public static final int DEPLOYINTAKEAXIS = 3;
    public static final int RETRACTINTAKEAXIS = 4;
    public static final int INTAKEBUTTON = 4;

    //driver controls 
    public static final int DRIVERINDEXERSORTINGBUTTON = 2;
    
    //pneumatics
    public static final int SOLFORCHANNEL1 = 1;
    public static final int SOLFORCHANNEL2 = 2;

    //pneumatics controls
    public static final int SOLFOROFFBUTTON = 5;
    public static final int SOLFORONBUTTON = 2;
    
    //shooter motor controller IDs
    public static final int INDEXID = 8;
    public static final int SHOOTID = 5;
    public static final int INTAKEID = 10;
}
