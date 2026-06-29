package org.firstinspires.ftc.teamcode.summer26.computerVision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import org.tensorflow.lite.Interpreter;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * FTC DECODE Artifact Detector
 * Detects green and purple artifacts using YOLOv8n TFLite model.
 *
 * SETUP:
 *   1. assets/ftc_artifact_detector.tflite
 *   2. assets/labels.txt  (green / purple)
 *   3. build.gradle (TeamCode) dependencies:
 *        implementation 'com.google.ai.edge.litert:litert:1.0.1'
 *   4. Webcam named "Webcam 1" in robot config
 */
@TeleOp(name = "Artifact Detector", group = "Vision")
public class ArtifactDetectionOpMode extends LinearOpMode {

    private static final String MODEL_FILE  = "ftc_artifact_detector.tflite";
    private static final String LABELS_FILE = "labels.txt";
    private static final int    INPUT_SIZE  = 320;
    private static final float  CONF_THRESH = 0.40f;
    private static final float  IOU_THRESH  = 0.40f;
    private static final String WEBCAM_NAME = "Webcam 1";

    private ArtifactProcessor processor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        try {
            processor = new ArtifactProcessor(
                    hardwareMap.appContext, MODEL_FILE, LABELS_FILE,
                    INPUT_SIZE, CONF_THRESH, IOU_THRESH
            );
        } catch (IOException e) {
            telemetry.addData("ERROR", "Model load failed: " + e.getMessage());
            telemetry.update();
            return;
        }

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(processor)
                .build();

        telemetry.addData("Status", "Ready — waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<ArtifactProcessor.Detection> detections = processor.getLatestDetections();

            telemetry.addData("Artifacts detected", detections.size());

            int greenCount = 0, purpleCount = 0;
            for (int i = 0; i < detections.size(); i++) {
                ArtifactProcessor.Detection d = detections.get(i);
                int cx = (int) d.centerX;
                int cy = (int) d.centerY;

                // Normalized position: 0,0 = center; -1/+1 = edges
                float normX = (d.centerX / processor.getFrameWidth()  - 0.5f) * 2f;
                float normY = (d.centerY / processor.getFrameHeight() - 0.5f) * 2f;

                telemetry.addData(
                        String.format("[%d] %s", i + 1, d.label),
                        String.format("conf=%.2f  px=(%d,%d)  norm=(%.2f,%.2f)",
                                d.confidence, cx, cy, normX, normY)
                );

                if (d.label.equalsIgnoreCase("green"))  greenCount++;
                if (d.label.equalsIgnoreCase("purple")) purpleCount++;
            }

            telemetry.addData("Green",  greenCount);
            telemetry.addData("Purple", purpleCount);

            // Example: steer toward nearest artifact
            if (!detections.isEmpty()) {
                ArtifactProcessor.Detection nearest = detections.get(0);
                float errorX = (nearest.centerX / processor.getFrameWidth()) - 0.5f;
                telemetry.addData("Steer error (L<0 R>0)", String.format("%.3f", errorX));
            }

            telemetry.addData("FPS", String.format("%.1f", processor.getFps()));
            telemetry.update();
        }

        visionPortal.close();
    }

    // =========================================================================
    //  VisionProcessor
    // =========================================================================
    public static class ArtifactProcessor implements VisionProcessor {

        public static class Detection {
            public final String label;
            public final float  confidence;
            public final float  centerX, centerY;
            public final float  left, top, right, bottom;

            Detection(String label, float conf, float cx, float cy,
                      float l, float t, float r, float b) {
                this.label = label; this.confidence = conf;
                this.centerX = cx;  this.centerY = cy;
                this.left = l; this.top = t; this.right = r; this.bottom = b;
            }
        }

        private final Interpreter   tflite;
        private final List<String>  labels;
        private final int           inputSize;
        private final float         confThresh, iouThresh;

        private volatile List<Detection> latestDetections = new ArrayList<>();
        private volatile float frameWidth  = 640;
        private volatile float frameHeight = 480;
        private volatile float fps = 0f;
        private long lastFrameTime = 0;

        private ByteBuffer inputBuffer;
        private int numOutputRows, numOutputCols;

        public ArtifactProcessor(Context ctx, String modelFile, String labelsFile,
                                 int inputSize, float confThresh, float iouThresh)
                throws IOException {
            this.inputSize  = inputSize;
            this.confThresh = confThresh;
            this.iouThresh  = iouThresh;

            labels = loadLabels(ctx, labelsFile);

            // Load model from assets
            MappedByteBuffer modelBuffer = loadModelFile(ctx, modelFile);
            Interpreter.Options options = new Interpreter.Options();
            options.setNumThreads(4);
            tflite = new Interpreter(modelBuffer, options);

            // Input: [1, inputSize, inputSize, 3] float32
            inputBuffer = ByteBuffer.allocateDirect(1 * inputSize * inputSize * 3 * 4);
            inputBuffer.order(ByteOrder.nativeOrder());

            // Output shape: [1, 4+nc, anchors]
            int[] outShape = tflite.getOutputTensor(0).shape();
            numOutputRows = outShape[1]; // 4 + numClasses
            numOutputCols = outShape[2]; // num anchors (e.g. 2100 for 320px)
        }

        @Override
        public void init(int width, int height, CameraCalibration cal) {
            frameWidth  = width;
            frameHeight = height;
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            long now = System.currentTimeMillis();
            if (lastFrameTime != 0) fps = 1000f / (now - lastFrameTime);
            lastFrameTime = now;

            // Resize frame to model input size
            Mat resized = new Mat();
            Imgproc.resize(frame, resized, new Size(inputSize, inputSize));

            // Convert BGR Mat → float32 RGB ByteBuffer (normalized 0-1)
            inputBuffer.rewind();
            int rows = resized.rows(), cols = resized.cols();
            byte[] rowBytes = new byte[cols * 3];
            for (int r = 0; r < rows; r++) {
                resized.get(r, 0, rowBytes);
                for (int c = 0; c < cols; c++) {
                    int base = c * 3;
                    // OpenCV is BGR — swap to RGB
                    inputBuffer.putFloat(((rowBytes[base + 2] & 0xFF)) / 255.0f); // R
                    inputBuffer.putFloat(((rowBytes[base + 1] & 0xFF)) / 255.0f); // G
                    inputBuffer.putFloat(((rowBytes[base + 0] & 0xFF)) / 255.0f); // B
                }
            }
            resized.release();

            // Run inference
            float[][][] output = new float[1][numOutputRows][numOutputCols];
            tflite.run(inputBuffer, output);

            // Parse + NMS
            latestDetections = parseAndNms(output[0], (int)frameWidth, (int)frameHeight);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasPxToDisplayPx,
                                Object userContext) {
            Paint boxPaint  = new Paint();
            boxPaint.setStyle(Paint.Style.STROKE);
            boxPaint.setStrokeWidth(4f);

            Paint textPaint = new Paint();
            textPaint.setTextSize(32f);
            textPaint.setStyle(Paint.Style.FILL);

            Paint bgPaint = new Paint();
            bgPaint.setStyle(Paint.Style.FILL);
            bgPaint.setColor(0xAA000000);

            float scale = scaleBmpPxToCanvasPx;

            for (Detection d : latestDetections) {
                int color = d.label.equalsIgnoreCase("green") ? 0xFF00CC44 : 0xFFAA00FF;
                boxPaint.setColor(color);
                textPaint.setColor(color);

                canvas.drawRect(d.left*scale, d.top*scale, d.right*scale, d.bottom*scale, boxPaint);

                String txt = String.format("%s %.0f%% (%d,%d)",
                        d.label, d.confidence*100, (int)d.centerX, (int)d.centerY);
                float tx = d.left * scale;
                float ty = d.top  * scale - 8f;
                canvas.drawRect(tx, ty - 34f, tx + txt.length()*18f, ty + 4f, bgPaint);
                canvas.drawText(txt, tx, ty, textPaint);
            }
        }

        // ── Helpers ──────────────────────────────────────────────────────────

        private List<Detection> parseAndNms(float[][] raw, int imgW, int imgH) {
            // raw shape: [4+nc, numAnchors]
            int numAnchors = raw[0].length;
            int nc = labels.size();

            List<float[]> boxes    = new ArrayList<>();
            List<Float>   scores   = new ArrayList<>();
            List<Integer> classIds = new ArrayList<>();

            for (int a = 0; a < numAnchors; a++) {
                float bestScore = 0f;
                int   bestCls   = 0;
                for (int c = 0; c < nc; c++) {
                    float s = raw[4 + c][a];
                    if (s > bestScore) { bestScore = s; bestCls = c; }
                }
                if (bestScore < confThresh) continue;

                float cx = raw[0][a] / inputSize * imgW;
                float cy = raw[1][a] / inputSize * imgH;
                float bw = raw[2][a] / inputSize * imgW;
                float bh = raw[3][a] / inputSize * imgH;

                boxes.add(new float[]{cx - bw/2f, cy - bh/2f, cx + bw/2f, cy + bh/2f});
                scores.add(bestScore);
                classIds.add(bestCls);
            }

            List<Detection> result = new ArrayList<>();
            for (int cls = 0; cls < nc; cls++) {
                List<Integer> idx = new ArrayList<>();
                for (int i = 0; i < classIds.size(); i++)
                    if (classIds.get(i) == cls) idx.add(i);
                idx.sort((a, b) -> Float.compare(scores.get(b), scores.get(a)));

                boolean[] suppressed = new boolean[idx.size()];
                for (int i = 0; i < idx.size(); i++) {
                    if (suppressed[i]) continue;
                    float[] b = boxes.get(idx.get(i));
                    float   s = scores.get(idx.get(i));
                    float cx = (b[0]+b[2])/2f, cy = (b[1]+b[3])/2f;
                    result.add(new Detection(labels.get(cls), s, cx, cy, b[0], b[1], b[2], b[3]));
                    for (int j = i+1; j < idx.size(); j++) {
                        if (!suppressed[j] && iou(b, boxes.get(idx.get(j))) > iouThresh)
                            suppressed[j] = true;
                    }
                }
            }
            result.sort((a, b) -> Float.compare(b.confidence, a.confidence));
            return result;
        }

        private float iou(float[] a, float[] b) {
            float ix1 = Math.max(a[0],b[0]), iy1 = Math.max(a[1],b[1]);
            float ix2 = Math.min(a[2],b[2]), iy2 = Math.min(a[3],b[3]);
            float inter = Math.max(0,ix2-ix1) * Math.max(0,iy2-iy1);
            return inter / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter + 1e-6f);
        }

        private List<String> loadLabels(Context ctx, String filename) throws IOException {
            List<String> list = new ArrayList<>();
            try (InputStream is = ctx.getAssets().open(filename);
                 BufferedReader br = new BufferedReader(new InputStreamReader(is))) {
                String line;
                while ((line = br.readLine()) != null)
                    if (!line.trim().isEmpty()) list.add(line.trim());
            }
            return list;
        }

        private MappedByteBuffer loadModelFile(Context ctx, String filename) throws IOException {
            android.content.res.AssetFileDescriptor afd = ctx.getAssets().openFd(filename);
            FileInputStream fis = new FileInputStream(afd.getFileDescriptor());
            FileChannel fc = fis.getChannel();
            return fc.map(FileChannel.MapMode.READ_ONLY, afd.getStartOffset(), afd.getDeclaredLength());
        }

        public List<Detection> getLatestDetections() { return latestDetections; }
        public float getFrameWidth()  { return frameWidth;  }
        public float getFrameHeight() { return frameHeight; }
        public float getFps()         { return fps; }
    }
}