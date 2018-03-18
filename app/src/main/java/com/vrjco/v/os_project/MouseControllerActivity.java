package com.vrjco.v.os_project;

import android.animation.Animator;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewAnimationUtils;
import android.view.Window;
import android.view.WindowManager;
import android.view.animation.AccelerateDecelerateInterpolator;
import android.widget.ImageView;
import android.widget.Toast;

public class MouseControllerActivity extends AppCompatActivity implements View.OnTouchListener, SensorEventListener {

    public static final String TAG = "OSP";
    public static final String LEFT = "LEFT_CLICK";
    public static final String RIGHT = "RIGHT_CLICK";
    public static final String MOUSE_MOVE = "D";
    public static final int sensitivity = 5;
    private ImageView left, right;

    private SensorManager sensorManager;
    private Sensor accMeter, gyroScope, magMeter;
    public static final float EPSILON = 0.000000001f;

    // private static final float NS2S = 1.0f / 10000.0f;
    // Nano-second to second conversion
    private static final float NS2S = 1.0f / 1000000000.0f;

    private boolean hasOrientation = false;

    // The coefficient for the filter... 0.5 = means it is averaging the two
    // transfer functions (rotations from the gyroscope and
    // acceleration/magnetic, respectively).
    public float filterCoefficient = 0.5f;

    private float dT = 0;

    private float omegaMagnitude = 0;

    private float thetaOverTwo = 0;
    private float sinThetaOverTwo = 0;
    private float cosThetaOverTwo = 0;

    private float[] components = new float[3];

    // angular speeds from gyro
    private float[] gyroscope = new float[3];

    // rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];

    // magnetic field vector
    private float[] magnetic = new float[3];

    // accelerometer vector
    private float[] acceleration = new float[3];

    // final orientation angles from sensor fusion
    private float[] fusedOrientation = new float[3];

    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];

    private float[] linearAcceleration = new float[3];

    // copy the new gyro values into the gyro array
    // convert the raw gyro data into a rotation vector
    private float[] deltaVector = new float[4];

    // convert rotation vector into rotation matrix
    private float[] deltaMatrix = new float[9];

    private long timeStamp;

    private static int key = 0;


    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.mouse_controller_layout);
        initializeViews();

        left.setOnTouchListener(this);
        right.setOnTouchListener(this);

        loadSensors();
        registerSupportedSensors();
    }

    private void initializeViews() {
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        left = (ImageView) findViewById(R.id.left_click);
        right = (ImageView) findViewById(R.id.right_click);
    }


    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {

        float pointerX = motionEvent.getX();
        float pointerY = motionEvent.getY();

        switch (view.getId()) {
            case R.id.left_click:
                if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                    key = 3;
                    Log.d(TAG, sensitivity * Math.round(linearAcceleration[0]) + " " + sensitivity * Math.round(linearAcceleration[2]) + " " + key);
                    return true;
                } else if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
                    if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.LOLLIPOP) {
                        Animator left_circular = ViewAnimationUtils.createCircularReveal(left, (int) (pointerX), (int) (pointerY), 0, left.getWidth());
                        left_circular.setInterpolator(new AccelerateDecelerateInterpolator());
                        left_circular.setDuration(400);
                        left_circular.start();
                    }
                    key = 0;
                    return true;
                }
            case R.id.right_click:
                if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                    key = 1;
                    Log.d(TAG, sensitivity * Math.round(linearAcceleration[0]) + " " + sensitivity * Math.round(linearAcceleration[2]) + " " + key);
                    return true;
                } else if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
                    if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.LOLLIPOP) {
                        Animator right_circular = ViewAnimationUtils.createCircularReveal(right, (int) (pointerX), (int) (pointerY), 0, right.getWidth());
                        right_circular.setInterpolator(new AccelerateDecelerateInterpolator());
                        right_circular.setDuration(400);
                        right_circular.start();
                    }
                    key = 0;
                    return true;
                }
            default:
                return false;
        }

    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        this.finish();
    }

    private void loadSensors() {
        accMeter = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroScope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magMeter = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    }

    private void registerSupportedSensors() {

        if (accMeter != null) {
            sensorManager.registerListener(this, accMeter, SensorManager.SENSOR_DELAY_NORMAL);
        } else {
            Toast.makeText(this, "No Accelerometer detected!", Toast.LENGTH_SHORT).show();
        }

        if (gyroScope != null) {
            sensorManager.registerListener(this, gyroScope, SensorManager.SENSOR_DELAY_NORMAL);
        } else {
            //Toast.makeText(this, "No Gyroscope detected!", Toast.LENGTH_SHORT).show();
        }

        if (magMeter != null) {
            sensorManager.registerListener(this, magMeter, SensorManager.SENSOR_DELAY_NORMAL);
        } else {
            Toast.makeText(this, "No Magnetic field detected!", Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor == accMeter) {
            acceleration = sensorEvent.values.clone();
            setAcceleration(acceleration);
        }
        if (sensorEvent.sensor == gyroScope) {
            gyroscope = sensorEvent.values.clone();
            setGyroscope(gyroscope, sensorEvent.timestamp);
        }
        if (sensorEvent.sensor == magMeter) {
            magnetic = sensorEvent.values.clone();
            setMagnetic(magnetic);
        }

        getLinearAcceleration();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    public void setAcceleration(float[] acceleration) {
        // Get a local copy of the raw magnetic values from the device sensor.
        System.arraycopy(acceleration, 0, this.acceleration, 0,
                acceleration.length);

        // We fuse the rotation of the magnetic and acceleration sensor based
        // on acceleration sensor updates. It could be done when the magnetic
        // sensor updates or when they both have updated if you want to spend
        // the resources to make the checks.
        calculateRotationAccelMag();
    }

    public void setGyroscope(float[] gyroscope, long timeStamp) {
        // don't start until first accelerometer/magnetometer orientation has
        // been acquired
        if (!hasOrientation) {
            return;
        }

        if (this.timeStamp != 0) {
            dT = (timeStamp - this.timeStamp) * NS2S;

            System.arraycopy(gyroscope, 0, this.gyroscope, 0, 3);
            getRotationVectorFromGyro(dT);
        }

        // measurement done, save current time for next interval
        this.timeStamp = timeStamp;

        // Get the rotation matrix from the gyroscope
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // Apply the new rotation interval on the gyroscope based rotation
        // matrix to form a composite rotation matrix. The product of two
        // rotation matricies is a rotation matrix...
        // Multiplication of rotation matrices corresponds to composition of
        // rotations... Which in this case are the rotation matrix from the
        // fused orientation and the rotation matrix from the current gyroscope
        // outputs.
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        calculateFusedOrientation();
    }

    public void setMagnetic(float[] magnetic) {
        // Get a local copy of the raw magnetic values from the device sensor.
        System.arraycopy(magnetic, 0, this.magnetic, 0, magnetic.length);

    }

    private void calculateFusedOrientation() {

        // Create our scalar matrix for the gyroscope
        float[] alphaGyro = new float[]
                {filterCoefficient, 0, 0, 0, filterCoefficient, 0, 0, 0,
                        filterCoefficient};

        float oneMinusCoeff = (1.0f - filterCoefficient);

        // Create our scalar matrix for the acceleration/magnetic
        float[] alphaRotation = new float[]
                {oneMinusCoeff, 0, 0, 0, oneMinusCoeff, 0, 0, 0, oneMinusCoeff};

        // Apply the complementary filter. We multiply each rotation by their
        // coefficients (scalar matrices) and then add the two rotations
        // together.
        // output[0] = alpha * output[0] + (1 - alpha) * input[0];
        gyroMatrix = matrixAddition(
                matrixMultiplication(gyroMatrix, alphaGyro),
                matrixMultiplication(rotationMatrix, alphaRotation));

        // Finally, we get the fused orientation
        SensorManager.getOrientation(gyroMatrix, fusedOrientation);
    }

    /**
     * Calculates orientation angles from accelerometer and magnetometer output.
     */
    private void calculateRotationAccelMag() {
        // To get the orientation vector from the acceleration and magnetic
        // sensors, we let Android do the heavy lifting. This call will
        // automatically compensate for the tilt of the compass and fail if the
        // magnitude of the acceleration is not close to 9.82m/sec^2. You could
        // perform these steps yourself, but in my opinion, this is the best way
        // to do it.
        SensorManager.getRotationMatrix(rotationMatrix, null, acceleration,
                magnetic);

        if (!hasOrientation) {
            gyroMatrix = rotationMatrix;
        }

        hasOrientation = true;
    }


    private void getRotationVectorFromGyro(float timeFactor) {

        // Calculate the angular speed of the sample
        omegaMagnitude = (float) Math.sqrt(Math.pow(gyroscope[0], 2)
                + Math.pow(gyroscope[1], 2) + Math.pow(gyroscope[2], 2));

        // Normalize the rotation vector if it's big enough to get the axis
        if (omegaMagnitude > EPSILON) {
            gyroscope[0] /= omegaMagnitude;
            gyroscope[1] /= omegaMagnitude;
            gyroscope[2] /= omegaMagnitude;
        }

        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        thetaOverTwo = omegaMagnitude * timeFactor / 2.0f;
        sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
        cosThetaOverTwo = (float) Math.cos(thetaOverTwo);

        deltaVector[0] = sinThetaOverTwo * gyroscope[0];
        deltaVector[1] = sinThetaOverTwo * gyroscope[1];
        deltaVector[2] = sinThetaOverTwo * gyroscope[2];
        deltaVector[3] = cosThetaOverTwo;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    private float[] matrixAddition(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] + B[0];
        result[1] = A[1] + B[1];
        result[2] = A[2] + B[2];
        result[3] = A[3] + B[3];
        result[4] = A[4] + B[4];
        result[5] = A[5] + B[5];
        result[6] = A[6] + B[6];
        result[7] = A[7] + B[7];
        result[8] = A[8] + B[8];

        return result;
    }

    public void getLinearAcceleration() {
        // values[0]: azimuth, rotation around the Z axis.
        // values[1]: pitch, rotation around the X axis.
        // values[2]: roll, rotation around the Y axis.

        // Find the gravity component of the X-axis
        // = g*-cos(pitch)*sin(roll);
        components[0] = (float) (SensorManager.GRAVITY_EARTH
                * -Math.cos(fusedOrientation[1]) * Math
                .sin(fusedOrientation[2]));

        // Find the gravity component of the Y-axis
        // = g*-sin(pitch);
        components[1] = (float) (SensorManager.GRAVITY_EARTH * -Math
                .sin(fusedOrientation[1]));

        // Find the gravity component of the Z-axis
        // = g*cos(pitch)*cos(roll);
        components[2] = (float) (SensorManager.GRAVITY_EARTH
                * Math.cos(fusedOrientation[1]) * Math.cos(fusedOrientation[2]));

        // Subtract the gravity component of the signal
        // from the input acceleration signal to get the
        // tilt compensated output.
        linearAcceleration[0] = (this.acceleration[0] - components[0]);
        linearAcceleration[1] = (this.acceleration[1] - components[1]);
        linearAcceleration[2] = (this.acceleration[2] - components[2]);

        // Log.d(TAG, "X: " +  linearAcceleration[0] + " Z: " + linearAcceleration[2]);

        Log.d(TAG, sensitivity * Math.round(linearAcceleration[0]) + " " + sensitivity * Math.round(linearAcceleration[2]) + " " + key);
    }


    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }


    @Override
    protected void onResume() {
        super.onResume();
        loadSensors();
        registerSupportedSensors();
    }

}


