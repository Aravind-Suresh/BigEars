package misc;

import android.graphics.Bitmap;

import com.googlecode.tesseract.android.TessBaseAPI;

/**
 * Created by aravind on 24/10/15.
 */

public class OCR {
	private String DATA_PATH;
	private String LANG;

	public OCR(String DATA_PATH, String LANG) {
		this.DATA_PATH = DATA_PATH;
		this.LANG = LANG;
	}

	public String getText(Bitmap bitmap) {
		bitmap = bitmap.copy(Bitmap.Config.ARGB_8888, true);

		TessBaseAPI baseApi = new TessBaseAPI();
		baseApi.setDebug(true);
		baseApi.init(DATA_PATH, LANG);
		baseApi.setImage(bitmap);

		String recognizedText = baseApi.getUTF8Text();

		baseApi.end();

		if (LANG.equalsIgnoreCase("eng")) {
			recognizedText = recognizedText.replaceAll("[^a-zA-Z0-9]+", " ");
		}

		return recognizedText.trim();
	}
}
