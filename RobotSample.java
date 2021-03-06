import ShefRobot.*;

/**
 *
 * @author sdn
 */
public class RobotSample {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        
        //Create a robot object to use and connect to it
        Robot myRobot = new Robot("dia-lego-a1");
       
        //The robot is made of components which are themselves objects.
        //Create references to them as useful shortcuts
        Motor leftMotor = myRobot.getLargeMotor(Motor.Port.B);
        Motor rightMotor = myRobot.getLargeMotor(Motor.Port.C);
        Speaker speaker = myRobot.getSpeaker();
        UltrasonicSensor ultrasonicSensor = myRobot.getUltrasonicSensor(Sensor.Port.S2);

        boolean test = true;
        while (test) {
            myRobot.sleep(1);
            System.out.println(ultrasonicSensor.getDistance());
        }
        
      
        
        // //Go Forwards
        // leftMotor.setSpeed(150);
        // rightMotor.setSpeed(150);
        // leftMotor.forward();
        // rightMotor.forward();

        // //Keep going for 5 seconds
        // myRobot.sleep(5000);

        // //Stop
        // leftMotor.stop();
        // rightMotor.stop();

        // //Beep at 1000Hz for half a second
        // speaker.playTone(1000, 500);

        // //Go Backwards
        // leftMotor.backward();
        // rightMotor.backward();

        //Keep going for 5 seconds
        myRobot.sleep(5000);

        //Stop
        leftMotor.stop();
        rightMotor.stop();
        
        //Disconnect from the Robot
        myRobot.close();

    }

}