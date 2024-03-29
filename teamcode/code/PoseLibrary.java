package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseLibrary {

    public static Pose2d startRedDuck =		            new Pose2d(-37.8125,-64.75,     Math.toRadians(270));
    public static Pose2d startRedBland =		        new Pose2d(9.125,   -64.75,     Math.toRadians(270));
    public static Pose2d startblueDuck =		        new Pose2d(-37.8125,64.75,      Math.toRadians(90));
    public static Pose2d startblueBland =		        new Pose2d(9.125,   64.75,      Math.toRadians(90));
    public static Pose2d duckRed = 				        new Pose2d(-51.75,  -58.5,      Math.toRadians(270));
    public static Pose2d duckblue = 		            new Pose2d(-51.75,  59.5,       Math.toRadians(90));
    public static Pose2d redParking0 =                  new Pose2d(-60,     -38,        Math.toRadians(0));
    public static Pose2d redE1 =                        new Pose2d(-44.5,   -49,        Math.toRadians(270));
    public static Pose2d redE2 =                        new Pose2d(-36,     -49,        Math.toRadians(270));
    public static Pose2d redE3 =                        new Pose2d(-27.5,   -48.5,      Math.toRadians(270));
    public static Pose2d redE4 =                        new Pose2d(3.5,     -48,        Math.toRadians(270));
    public static Pose2d redE5 =                        new Pose2d(12,      -48,        Math.toRadians(270));
    public static Pose2d redE6 =                        new Pose2d(20.5,    -48,        Math.toRadians(270));
    public static Pose2d redGoalAlliance =              new Pose2d(-10,     -39,        Math.toRadians(270));
    public static Pose2d redGoalAlliance2 =             new Pose2d(-11,     -40,        Math.toRadians(270));
    public static Pose2d redGoalParking =               new Pose2d(-23,     -26,        Math.toRadians(180));
    public static Pose2d redGoalWarehouse =             new Pose2d(12,      -24,        Math.toRadians(0));
    public static Pose2d redGoalOpposite =              new Pose2d(-12,     -12,        Math.toRadians(90));
    public static Pose2d redParking1 =                  new Pose2d(36,      -65,        Math.toRadians(0));
    public static Pose2d redParking2 =                  new Pose2d(36,      -36,        Math.toRadians(315));
    public static Pose2d redParking3 =                  new Pose2d(62,      -40,        Math.toRadians(270));
    public static Pose2d redLoopBreak =                 new Pose2d(12,      -18,        Math.toRadians(90));
    public static Pose2d redPause =                     new Pose2d(-40,     -63,        Math.toRadians(0));
    public static Pose2d redOutOfWay =                  new Pose2d(-56,     -24,        Math.toRadians(270));
    public static Pose2d redBehindElement1 =            new Pose2d(-51.75,  -14,        Math.toRadians(270));
    public static Pose2d redBehindElement2 =            new Pose2d(-41.75,  -14,        Math.toRadians(270));
    public static Pose2d redBehindElement3 =            new Pose2d(-34,     -14,        Math.toRadians(270));
    public static Pose2d redManeuverAvoidMiddle=        new Pose2d(0,       -36,        Math.toRadians(270));
    public static Pose2d redWarehouseOut =              new Pose2d(12,      -65.75,     Math.toRadians(0));
    public static Pose2d redWarehouseOut3 =             new Pose2d(12,      -67,        Math.toRadians(0));
    public static Pose2d redWarehouseOut2 =             new Pose2d(12,      -65,        Math.toRadians(360));
    public static Pose2d redWarehouseIn =               new Pose2d(38,      -65,        Math.toRadians(0));
    public static Pose2d redReading =                   new Pose2d(40,      -65,        Math.toRadians(0));

    public static Pose2d blueParking0 =                 new Pose2d(-63,     39,         Math.toRadians(0));
    public static Pose2d blueE1 =                       new Pose2d(-44.5,   49,         Math.toRadians(90));
    public static Pose2d blueE2 =                       new Pose2d(-36,     49,         Math.toRadians(90));
    public static Pose2d blueE3 =                       new Pose2d(-27.5,   48.5,       Math.toRadians(90));
    public static Pose2d blueE4 =                       new Pose2d(3.5,     48,         Math.toRadians(90));
    public static Pose2d blueE5 =                       new Pose2d(12,      48,         Math.toRadians(90));
    public static Pose2d blueE6 =                       new Pose2d(20.5,    48,         Math.toRadians(90));
    public static Pose2d blueGoalAlliance =             new Pose2d(-10,     37.5,       Math.toRadians(90));
    public static Pose2d blueGoalParking =              new Pose2d(-26,     26,         Math.toRadians(180));
    public static Pose2d blueGoalWarehouse =            new Pose2d(12,      24,         Math.toRadians(0));
    public static Pose2d blueGoalOpposite =             new Pose2d(-12,     12,         Math.toRadians(90));
    public static Pose2d blueParking1 =                 new Pose2d(36,      65,         Math.toRadians(0));
    public static Pose2d blueParking2 =                 new Pose2d(36,      34,         Math.toRadians(45));
    public static Pose2d blueParking3 =                 new Pose2d(62,      36,         Math.toRadians(90));
    public static Pose2d blueLoopBreak =                new Pose2d(12,      18,         Math.toRadians(90));
    public static Pose2d bluePause =                    new Pose2d(-40,     63,         Math.toRadians(0));
    public static Pose2d blueOutOfWay =                 new Pose2d(-56,     24,         Math.toRadians(90));
    public static Pose2d blueBehindElement1 =           new Pose2d(-51.75,  14,         Math.toRadians(90));
    public static Pose2d blueBehindElement2 =           new Pose2d(-41.75,  14,         Math.toRadians(90));
    public static Pose2d blueBehindElement3 =           new Pose2d(-34,     14,         Math.toRadians(90));
    public static Pose2d blueManeuverAvoidMiddle=       new Pose2d(0,       36,         Math.toRadians(90));
    public static Pose2d blueWarehouseOut =             new Pose2d(12,      65.25,      Math.toRadians(0));
    public static Pose2d blueWarehouseOut3 =            new Pose2d(12,      67,         Math.toRadians(0));
    public static Pose2d blueWarehouseOut2 =            new Pose2d(12,      65,         Math.toRadians(360));
    public static Pose2d blueWarehouseIn =              new Pose2d(38,      65,         Math.toRadians(0));
    public static Pose2d blueReading =                  new Pose2d(40,      65,         Math.toRadians(0));
    //---------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * These would be used if we used spline paths in roadrunner but we have not found it efficient just yet
     */
    public static Vector2d startRedDuckV =		    new Vector2d(-41,     -65);
    public static Vector2d duckRedV = 				new Vector2d(-41,     -60);
    public static Vector2d redParking0V = 		    new Vector2d(-60,     -36);
    public static Vector2d redAwayFromElements = 	new Vector2d(-55,     -36);
    public static Vector2d redE1V = 				new Vector2d(-44.5,   -49);
    public static Vector2d redE2V = 				new Vector2d(-36,     -49);
    public static Vector2d redE3V = 				new Vector2d(-27.5,   -48.5);
    public static Vector2d redE4V =			 	    new Vector2d(3.5,     -48);
    public static Vector2d redE5V =				    new Vector2d(12,      -48);
    public static Vector2d redE6V = 				new Vector2d(20.5,    -48);
    public static Vector2d redGoalAllianceV = 	    new Vector2d(-12,     -36);
    public static Vector2d redGoalParkingV = 	    new Vector2d(-27,     -24);
    public static Vector2d redGoalWarehouseV = 	    new Vector2d(12,      -24);
    public static Vector2d redParking1V = 		    new Vector2d(36,      -65);
    public static Vector2d redParking2V = 		    new Vector2d(36,      -36);
    public static Vector2d redParking3V = 		    new Vector2d(60,      -36);
    public static Vector2d redOutOfWayV = 		    new Vector2d(-24,     -60 );
    public static Vector2d redWarehouseOutV = 	    new Vector2d(12,      -65);
    public static Vector2d redWarehouseInV = 	    new Vector2d(38,      -65);
    public static Vector2d blueParking0V = 		    new Vector2d(-60,     36);
    public static Vector2d blueE1V = 			    new Vector2d(-44,     51);
    public static Vector2d blueE2V = 			    new Vector2d(-36,     51);
    public static Vector2d blueE3V = 			    new Vector2d(-26,     51);
    public static Vector2d blueE4V = 		    	new Vector2d(3.5,     48);
    public static Vector2d blueE5V = 	    		new Vector2d(12,      48);
    public static Vector2d blueE6V = 		    	new Vector2d(20.5,    48);
    public static Vector2d blueGoalAllianceV = 	    new Vector2d(-12,     36);
    public static Vector2d blueGoalParkingV = 	    new Vector2d(-27,     24);
    public static Vector2d blueGoalWarehouseV = 	new Vector2d(12,      24);
    public static Vector2d blueParking1V = 		    new Vector2d(36,      65);
    public static Vector2d blueParking2V = 		    new Vector2d(36,      36);
    public static Vector2d blueParking3V = 		    new Vector2d(60,      36);
    public static Vector2d blueWarehouseOutV = 	    new Vector2d(12,      65);
    public static Vector2d blueWarehouseInV = 	    new Vector2d(36,      65);


}