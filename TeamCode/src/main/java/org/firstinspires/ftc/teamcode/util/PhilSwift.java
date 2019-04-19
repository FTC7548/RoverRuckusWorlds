package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.media.MediaPlayer;

import java.io.IOException;

// That's a lotta damage

public class PhilSwift {
    //The player handling the audio
    private static MediaPlayer mediaPlayer = null;

    //Start the wubs
    public static void start(Context context, int file) {
        if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(context, file);
        mediaPlayer.seekTo(0);
        mediaPlayer.start();
    }

    //Stop the wubs
    public static void stop() {
        if (mediaPlayer != null) {
            mediaPlayer.stop();
            try { mediaPlayer.prepare(); }
            catch (IOException e) {}
        }
    }
}
/*
public class SoundOpMode extends OpMode {
    private boolean pressedLast;
    public void init() {
        this.pressedLast = false;
    }
    public void loop() {
        if (this.gamepad1.a && !this.pressedLast) {
            CenaPlayer.start(this.hardwareMap.appContext);
            this.pressedLast = true;
        }
        else if (!this.gamepad1.a && this.pressedLast) {
            CenaPlayer.stop();
            this.pressedLast = false;
        }
    }
    public void stop() {
        CenaPlayer.stop();
    }

    */