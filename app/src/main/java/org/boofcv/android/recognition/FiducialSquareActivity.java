package org.boofcv.android.recognition;

import android.content.Intent;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Toast;
import android.widget.ToggleButton;

import org.boofcv.android.DemoMain;
import org.boofcv.android.DemoVideoDisplayActivity;
import org.boofcv.android.R;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.image.GrayU8;
import georegression.struct.se.Se3_F64;

/**
 * Base class for square fiducials
 *
 * @author Peter Abeles
 */
public abstract class FiducialSquareActivity extends DemoVideoDisplayActivity
		implements View.OnTouchListener
{
	public static final String TAG = "FiducialSquareActivity";

	final Object lock = new Object();
	volatile boolean changed = true;
	volatile boolean robust = true;
	volatile int binaryThreshold = 100;

	Se3_F64 targetToCamera = new Se3_F64();
	CameraPinholeRadial intrinsic;

	Class help;

	// this text is displayed
	String drawText = "";

	// true for showinginput image or false for debug information
	boolean showInput = true;

	protected boolean disableControls = false;

	FiducialSquareActivity(Class help) {
		this.help = help;
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		LayoutInflater inflater = getLayoutInflater();
		LinearLayout controls = (LinearLayout)inflater.inflate(R.layout.fiducial_controls,null);

		LinearLayout parent = getViewContent();
		parent.addView(controls);

		FrameLayout iv = getViewPreview();
		iv.setOnTouchListener(this);

		final ToggleButton toggle = (ToggleButton) controls.findViewById(R.id.toggle_robust);
		final SeekBar seek = (SeekBar) controls.findViewById(R.id.slider_threshold);

		if( disableControls ) {
			toggle.setEnabled(false);
			seek.setEnabled(false);
		} else {
			robust = toggle.isChecked();
			binaryThreshold = seek.getProgress();

			seek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {					// thresholder change
				@Override
				public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
					synchronized (lock) {
						changed = true;
						binaryThreshold = progress;
					}
				}

				@Override
				public void onStartTrackingTouch(SeekBar seekBar) {
				}

				@Override
				public void onStopTrackingTouch(SeekBar seekBar) {
				}
			});
			toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
				@Override
				public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
					synchronized (lock) {
						changed = true;
						robust = isChecked;
						if (robust) {
							seek.setEnabled(false);
						} else {
							seek.setEnabled(true);
						}
					}
				}
			});
		}
	}

	public void pressedHelp( View view ) {
		Intent intent = new Intent(this, help );
		startActivity(intent);
	}

	@Override
	protected void onResume() {
		super.onResume();
		changed = true;
		intrinsic = null;
		startDetector();
		if( DemoMain.preference.intrinsic == null ) {
			Toast.makeText(FiducialSquareActivity.this, "Calibrate camera for better results!", Toast.LENGTH_LONG).show();
		}
	}

	@Override
	public boolean onTouch(View v, MotionEvent event) {
		if( MotionEvent.ACTION_DOWN == event.getActionMasked()) {
			showInput = !showInput;
			return true;
		}
		return false;
	}

	protected void startDetector() {
		setProcessing(new FiducialImageProcessor(this) );
	}

	protected abstract FiducialDetector<GrayU8> createDetector();

}
