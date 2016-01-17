package com.app.vision;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.ColorMatrix;
import android.graphics.ColorMatrixColorFilter;
import android.graphics.Paint;
import android.hardware.Camera;
import android.hardware.Camera.AutoFocusCallback;
import android.hardware.Camera.PictureCallback;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup.LayoutParams;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.Toast;

/**
 * Created by aravind on 23/10/15.
 */

public class MainActivity extends Activity {
	private static final String TAG = "CamTestActivity";
	Preview preview;
	Button buttonClick;
	Camera camera;
	Activity act;
	Context ctx;

	static {
		//System.loadLibrary("png");
		//Log.e("app", "loaded png");
		//System.loadLibrary("lept");
		//System.loadLibrary("tess");
        System.loadLibrary("opencv_java3");
        System.loadLibrary("text");
	}

    public native int[] text(long addrGray);

	public static final String DATA_PATH = Environment
			.getExternalStorageDirectory().toString() + "/SeeSharp/";

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		String lang = "eng";
		
		String[] paths = new String[] { DATA_PATH, DATA_PATH + "tessdata/" };
        Log.i("path", paths[1]);

		for (String path : paths) {
			File dir = new File(path);
			if (!dir.exists()) {
				if (!dir.mkdirs()) {
					Log.v(TAG, "ERROR: Creation of directory " + path + " on sdcard failed");
					return;
				} else {
					Log.v(TAG, "Created directory " + path + " on sdcard");
				}
			}

		}

		if (!(new File(DATA_PATH + "tessdata/" + lang + ".traineddata"))
				.exists()) {
			try {

				AssetManager assetManager = getAssets();
				InputStream in = assetManager.open("tessdata/" + lang
						+ ".traineddata");
				// GZIPInputStream gin = new GZIPInputStream(in);
				OutputStream out = new FileOutputStream(DATA_PATH + "tessdata/"
						+ lang + ".traineddata");

				// Transfer bytes from in to out
				byte[] buf = new byte[1024];
				int len;
				// while ((lenf = gin.read(buff)) > 0) {
				while ((len = in.read(buf)) > 0) {
					out.write(buf, 0, len);
				}
				in.close();
				// gin.close();
				out.close();

				Log.v(TAG, "Copied " + lang + " traineddata");
			} catch (IOException e) {
				Log.e(TAG,
						"Was unable to copy " + lang + " traineddata "
								+ e.toString());
			}
		}

		ctx = this;
		act = this;
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);

		setContentView(R.layout.activity_main);

		preview = new Preview(this,
				(SurfaceView) findViewById(R.id.surfaceView));
		preview.setLayoutParams(new LayoutParams(LayoutParams.MATCH_PARENT,
				LayoutParams.MATCH_PARENT));
		((FrameLayout) findViewById(R.id.layout)).addView(preview);
		preview.setKeepScreenOn(true);

		preview.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View arg0) {
				camera.autoFocus(new AutoFocusCallback() {

					@Override
					public void onAutoFocus(boolean success, Camera camera) {
						camera.takePicture(null, null, new PictureCallback() {
							public void onPictureTaken(byte[] data,
									Camera camera) {
								Bitmap bmpT = toGrayscale(BitmapFactory
										.decodeByteArray(data, 0, data.length));

								Bitmap bmp = bmpT.copy(Bitmap.Config.ARGB_8888,
										true);

								Mat imgMat = new Mat(bmp.getHeight(), bmp
										.getWidth(), CvType.CV_8UC1);
								Utils.bitmapToMat(bmp, imgMat);

                                // TODO: Process mat image here
                                int[] boxes = text(imgMat.getNativeObjAddr());
                                Toast.makeText(MainActivity.this, "" + boxes.length, Toast.LENGTH_SHORT).show();
								resetCam();
							}
						});
					}
				});
			}
		});
	}

	public Bitmap toGrayscale(Bitmap bmpOriginal) {
		int width, height;
		height = bmpOriginal.getHeight();
		width = bmpOriginal.getWidth();

		Bitmap bmpGrayscale = Bitmap.createBitmap(width, height,
				Bitmap.Config.ARGB_8888);
		Canvas c = new Canvas(bmpGrayscale);
		Paint paint = new Paint();
		ColorMatrix cm = new ColorMatrix();
		cm.setSaturation(0);
		ColorMatrixColorFilter f = new ColorMatrixColorFilter(cm);
		paint.setColorFilter(f);
		c.drawBitmap(bmpOriginal, 0, 0, paint);
		return bmpGrayscale;
	}

	@Override
	protected void onResume() {
		super.onResume();
		int numCams = Camera.getNumberOfCameras();
		if (numCams > 0) {
			try {
				camera = Camera.open(0);
				camera.startPreview();
				preview.setCamera(camera);
			} catch (RuntimeException ex) {
				Toast.makeText(ctx, "Camera not found.", Toast.LENGTH_LONG)
						.show();
			}
		}
	}

	@Override
	protected void onPause() {
		if (camera != null) {
			camera.stopPreview();
			preview.setCamera(null);
			camera.release();
			camera = null;
		}
		super.onPause();
	}

	private void resetCam() {
		camera.startPreview();
		preview.setCamera(camera);
	}

	private void refreshGallery(File file) {
		Intent mediaScanIntent = new Intent(
				Intent.ACTION_MEDIA_SCANNER_SCAN_FILE);
		mediaScanIntent.setData(Uri.fromFile(file));
		sendBroadcast(mediaScanIntent);
	}

	class SaveImageTask extends AsyncTask<byte[], Void, Void> {

		@Override
		protected Void doInBackground(byte[]... data) {
			FileOutputStream outStream = null;

			// Write to SD Card
			try {
				File sdCard = Environment.getExternalStorageDirectory();
				File dir = new File(sdCard.getAbsolutePath() + "/camtest");
				dir.mkdirs();

				String fileName = String.format("%d.jpg",
						System.currentTimeMillis());
				File outFile = new File(dir, fileName);

				outStream = new FileOutputStream(outFile);
				outStream.write(data[0]);
				outStream.flush();
				outStream.close();

				Log.d(TAG, "onPictureTaken - wrote bytes: " + data.length
						+ " to " + outFile.getAbsolutePath());

				refreshGallery(outFile);
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} finally {
			}
			return null;
		}

	}

}