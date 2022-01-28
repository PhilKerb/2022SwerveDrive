package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve {

    public static SwerveOutput convertControllerToSwerve(double x1, double y1, double x2, double theta_radians) {
        if (Math.abs(x1) < .17) x1 = 0; //sets up deadzone 
        if (Math.abs(y1) < .17) y1 = 0; //sets up deadzone
        if (Math.abs(x2) < .17) x2 = 0; //sets up deadzone

        SwerveOutput output = new SwerveOutput();
        double L = 23; //length between axis of two wheels
        double W = 23; //width between axis of two wheels
        double R = Math.sqrt(Math.pow(L,2) + Math.pow(W,2)); //hypotenuse length between the axis of two wheels
        
        double temp = (y1 * Math.cos(theta_radians)) + (-x1 * Math.sin(theta_radians)); 
        x1 = (y1 * Math.sin(theta_radians)) + (x1 * Math.cos(theta_radians));
        y1 = temp;
        //} 

        x1 = -x1;
        y1 = -y1;
        x2 = -x2;

        SmartDashboard.putNumber("x1", x1); //displays values 
        SmartDashboard.putNumber("y1", y1); //displays values 
        SmartDashboard.putNumber("x2", x2); //displays values

        double A = x1 - x2 * (L/R);
        double B = x1 + x2 * (L/R);
        double C = y1 - x2 * (W/R);
        double D = y1 + x2 * (W/R);
        double pi = 3.14159265358979323846264338;

        double ws1 = Math.sqrt(Math.pow(B,2) + Math.pow(C,2)); 
        double ws2 = Math.sqrt(Math.pow(B,2) + Math.pow(D,2)); 
        double ws3 = Math.sqrt(Math.pow(A,2) + Math.pow(D,2)); 
        double ws4 = Math.sqrt(Math.pow(A,2) + Math.pow(C,2));

        double wa1 = Math.atan2(B,C) * 180/pi;
        double wa2 = Math.atan2(B,D) * 180/pi;
        double wa3 = Math.atan2(A,D) * 180/pi;
        double wa4 = Math.atan2(A,C) * 180/pi;

        output.wheelSpeeds[0] = ws1;
        output.wheelSpeeds[1] = ws2;
        output.wheelSpeeds[2] = ws3;
        output.wheelSpeeds[3] = ws4;
        
        output.wheelAngles[0] = wa1 / 360;
        output.wheelAngles[1] = wa2 / 360;
        output.wheelAngles[2] = wa3 / 360;
        output.wheelAngles[3] = wa4 / 360;


        return output;
    }

}