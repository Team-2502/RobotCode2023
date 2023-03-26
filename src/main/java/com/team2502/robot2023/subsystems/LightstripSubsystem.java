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

    /** Class for static animations not tied to a subsystem
     *
     * {@link Animation}s that need to accept data inputs should be defined elsewhere
     * */
    public static final class Animations {
        public static final Animation off = ((s,f)->{
            for (int i = 0; i < Leds.LED_COUNT; i++) {
                s.buffer.setRGB(i,55,0,0);
            }
            return false;
        });
        public static final Animation request_cube = ((s,f)->{
            s.fillColor(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(90),Color.kPurple);
            return false;
        });
        public static final Animation request_cone = ((s,f)->{
            s.fillColor(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(90),Color.kYellow);
            return false;
        });
        public static final Animation orbit_demo_simple = ((s,f)->{
            s.buffer.setRGB((int) (f/8)%Leds.LED_COUNT,255,0,0);
            return false;
        });
        public static final Animation orbit_demo = ((s,f)->{
            Rotation2d angle = Rotation2d.fromDegrees(f*Leds.FRAME_TIME*90);
            Rotation2d offset = Rotation2d.fromDegrees(30);
            s.fillColor(angle.minus(offset),angle.plus(offset),Color.kPeru);
            return false;
        });
        public static final Animation disabled = ((s,f) -> {
            /*for (var i = 0; i < Leds.LED_COUNT; i += 2) {
                s.buffer.setHSV(i, 0, 255, 255);
            }
             */

/*
            double rainbowFirstPixelRed = 0;
            for (var i = 0; i < Leds.LED_COUNT; i++) {
                final var red = (rainbowFirstPixelRed + (i * 180 / s.buffer.getLength())) % 180;
                s.buffer.setRGB(i, (int) red, 0, 0);
            }
            rainbowFirstPixelRed += 3;
            rainbowFirstPixelRed %= 180;*/

            for (var i = 0; i < s.buffer.getLength(); i++) {
                s.buffer.setRGB(i, 255, 0, 0);
            }

            return false;
        });
        public static final Animation rainbow = ((s,f) -> {
            double rainbowFirstPixelHue = 0;
            for (var i = 0; i < Leds.LED_COUNT; i++) {
                final var hue = (rainbowFirstPixelHue + (i * 180 / s.buffer.getLength())) % 180;
                s.buffer.setHSV(i, (int) hue, 255, 128);
            }
            rainbowFirstPixelHue += 3;
            rainbowFirstPixelHue %= 180;
            return false;
        });
        public static final Animation intake = ((s,f) -> {
            for (var i = 0; i < Leds.LED_COUNT; i++) {
                s.buffer.setRGB(i, 180, 180, 180);
            }
            return false;
        });
    };

    @FunctionalInterface
    public interface Animation {
        /** preform one tick of an animation
         *
         * @param strip led strip to set colors on
         * @param frame execution count, starts at 1
         */
        boolean tick(Lightstrip strip, double frame);
    }

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
