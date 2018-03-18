package com.vrjco.v.os_project;

import android.content.Intent;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.RelativeLayout;

public class TouchMouseActivity extends AppCompatActivity implements View.OnTouchListener {

    private RelativeLayout touch_area;
    private ImageButton gyro_start;
    private Button left, right;

    private float x = 0, y = 0, lastX = 0, lastY = 0;
    private static int dx = 0, dy = 0, key = 0;


    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.touch_mouse_layout);
        initialize();

        touch_area.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {

                x = motionEvent.getX();
                y = motionEvent.getY();

                dx = (-1) * Math.round((x - lastX));
                dy = Math.round(y - lastY);

                //printing change in x and y
                Log.d(MouseControllerActivity.TAG, dx + " " + dy + " " + key);

                lastX = x;
                lastY = y;
                return true;
            }
        });

        left.setOnTouchListener(this);

        right.setOnTouchListener(this);


        gyro_start.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent gyro = new Intent(TouchMouseActivity.this, MouseControllerActivity.class);
                startActivity(gyro);
            }
        });

    }

    private void initialize() {
        touch_area = (RelativeLayout) findViewById(R.id.touch_area);
        left = (Button) findViewById(R.id.button_left_touch);
        right = (Button) findViewById(R.id.button_right_touch);
        gyro_start = (ImageButton) findViewById(R.id.go_gyro_mouse);
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        if (view.getId() == R.id.button_left_touch
                || view.getId() == R.id.button_right_touch) {
            if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                if (view.getId() == R.id.button_left_touch) {
                    key = 3;
                } else if (view.getId() == R.id.button_right_touch) {
                    key = 1;
                }
                Log.d(MouseControllerActivity.TAG, dx + " " + dy + " " + key);
            } else if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
                key = 0;
            }
        }
        return true;
    }
}
