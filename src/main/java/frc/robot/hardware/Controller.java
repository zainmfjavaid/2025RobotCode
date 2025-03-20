package frc.robot.hardware;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OperatorConstants;

public class Controller {

    public static class DriverController {
        private final Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);

        //TODO: Tune curves to driver preference
        private static final ControlCurve kXTranslationCurve = new ControlCurve(0.85,0.05,0.85,0.1);
        private static final ControlCurve kYTranslationCurve = new ControlCurve(0.85,0.05,0.85,0.1);
        private static final ControlCurve kRotationCurve = new ControlCurve(0.8,0,1,0.1);

        public enum Button
        {
            A(1),
            B(2),
            X(3),
            Y(4),
            LB(5),
            RB(6),
            Back(7),
            Start(8),
            LJ(9),
            RJ(10);

            private int port;

            Button(int port)
            {
                this.port = port;
            }
            public int getPort()
            {
                return port;
            }
        }

        public JoystickButton getButton(Button button) {
            return new JoystickButton(joystick, button.getPort());
        }

        public double getLeftTrigger() {
            return joystick.getRawAxis(2);
        }
        public double getRightTrigger() {
            return joystick.getRawAxis(3);
        }

        // Left is positive
        public double getLeftStickX() {
            return -joystick.getRawAxis(0);
        }
        // Up is positive
        public double getLeftStickY() {
            return -joystick.getRawAxis(1);
        }
        // Left is positive
        public double getRightStickX() {
            return -joystick.getRawAxis(4);
        }
        // Up is positive
        public double getRightStickY() {
            return -joystick.getRawAxis(5);
        }

        public Supplier<Double> getXTranslationSupplier(){
            //This axis is inverted
            return () -> kXTranslationCurve.calculate(-joystick.getRawAxis(0));
        }

        public Supplier<Double> getYTranslationSupplier(){
            //This axis is inverted
            return () -> kYTranslationCurve.calculate(-joystick.getRawAxis(1));
        }

        public Supplier<Double> getRotationSupplier(){
            //This axis is inverted
            return () -> kRotationCurve.calculate(-joystick.getRawAxis(4));
        }
    }

    public static class OperatorController {
        private final Joystick joystick = new Joystick(OperatorConstants.kOperatorControllerPort);

        public static enum Button {
            X(1),
            A(2),
            B(3),
            Y(4),
            LB(5), // Left Bumper
            RB(6), // Right Bumper
            LT(7), // Left Trigger
            RT(8), // Right Trigger
            Back(9),
            Start(10),
            LJ(11), // Left Joystick Button
            RJ(12),  // Right Joystick Button
            POVUP(0),
            POVDOWN(180),
            POVLEFT(270),
            POVRIGHT(90);
            
            private final int port; 
            
            private Button(int port) {
                this.port = port;
            }
            
            public int getPort() {
                return port;
            }
        }

        public JoystickButton getButton(Button button) {
            return new JoystickButton(joystick, button.getPort());
        }
    }

    public static class ControlCurve{
        private double ySaturation; // Maximum output, in percentage of possible output
        private double yIntercept; // Minimum output, in percentage of saturation
        private double curvature; // Curvature shift between linear and cubic
        private double deadzone; // Range of input that will always return zero output

        public ControlCurve(double ySaturation, double yIntercept, double curvature, double deadzone){
            this.ySaturation = ySaturation;
            this.yIntercept = yIntercept;
            this.curvature = curvature;
            this.deadzone = deadzone;
        }

        public double calculate(double input){
            /* https://www.desmos.com/calculator/w6ovblmmqj
            Two equations, separated by a ternary
            The first is the deadzone
            y = 0 {|x| < d}
            The second is the curve
            y = a(sign(x) * b + (1 - b) * (c * x^3 + (1 - c) * x)) {|x| >= d}
            Where
            x = input
            y = output
            a = ySaturation
            b = yIntercept
            c = curvature
            d = deadzone
            and 0 <= a,b,c,d < 1 
            */
            return Math.abs(input) <  deadzone ? 0 : 
            ySaturation * (Math.signum(input) * yIntercept + 
            (1 - yIntercept) * (curvature * Math.pow(input, 3) +
            (1 - curvature) * input));
        }
    }
}