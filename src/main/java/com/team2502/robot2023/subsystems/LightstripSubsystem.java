package com.team2502.robot2023.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.Optional;
import java.util.function.Function;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Leds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightstripSubsystem extends SubsystemBase {
    /*
    public class Lightstrip {
        AddressableLED strip;
        public AddressableLEDBuffer buffer;

        public Lightstrip(int port, int count) {
            strip = new AddressableLED(port);
            strip.setLength(count);
            buffer = new AddressableLEDBuffer(count);

            strip.setData(buffer);
            strip.start();
        }
    }

    public class Animations {
        public void setRGB(int r, int g, int b) {
            for (var i = 0; i < Leds.LED_COUNT; i++) {
                buffer.setRGB(i, r, g, b);
            }
        }
    }

    //private Lightstrip strip;
    private AddressableLEDBuffer buffer;
    private AddressableLED strip;
    private Animations animations;

    public LightstripSubsystem() {
        buffer = new AddressableLEDBuffer(Leds.LED_COUNT);
        strip.setLength(Leds.LED_COUNT);
        strip.setData(buffer);
        strip.start();
        //strip = new Lightstrip(Leds.PORT, Leds.LED_COUNT);
    }

    @Override
    public void periodic() {
        animations.setRGB(255, 0 ,0);

        //strip.strip.setData(strip.buffer);
        for (var i = 0; i < Leds.LED_COUNT; i++) {
            buffer.setRGB(i, 255, 0, 0);
        }
        strip.setData(buffer);
    }*/
}
