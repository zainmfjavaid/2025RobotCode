package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller {
    public enum Button
    {
        X(1),
        A(2),
        B(3),
        Y(4),
        LB(5),
        RB(6),
        LT(7),
        RT(8),
        Back(9),
        Start(10),
        LJ(11),
        RJ(12);

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

    private final Joystick joystick;
    
    public Controller(int port) {
        joystick = new Joystick(port);
    }

    public JoystickButton getButton(Button button) {
        return new JoystickButton(joystick, button.getPort());
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
        return -joystick.getRawAxis(2);
    }
    
    // Up is positive
    public double getRightStickY() {
        return -joystick.getRawAxis(3);
    }
}
