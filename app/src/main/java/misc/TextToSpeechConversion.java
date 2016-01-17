package misc;

import android.content.Context;
import android.speech.tts.TextToSpeech;
import android.widget.Toast;

import java.util.Locale;

/**
 * Created by aravind on 23/10/15.
 */

public class TextToSpeechConversion {
    TextToSpeech textToSpeech;
    Context context;

    public TextToSpeechConversion(final Context context, final String text) {
        this.context = context;
        textToSpeech = new TextToSpeech(context, new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                if(status == TextToSpeech.SUCCESS) {
                    // Toast.makeText(context, "No Error TTS", Toast.LENGTH_SHORT).show();
                    textToSpeech.setLanguage(Locale.US);
                    speakText(text);
                }
                else {
                    Toast.makeText(context, "Error in TTS", Toast.LENGTH_SHORT).show();

                }
            }
        });
    }

    public void speakText(String text) {
        textToSpeech.speak(text, TextToSpeech.QUEUE_FLUSH, null);
    }

}