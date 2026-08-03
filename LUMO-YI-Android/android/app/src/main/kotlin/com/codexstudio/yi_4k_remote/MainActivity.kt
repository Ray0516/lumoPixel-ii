package com.codexstudio.yi_4k_remote

import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.ColorMatrix
import android.graphics.ColorMatrixColorFilter
import android.graphics.Paint
import android.graphics.RadialGradient
import android.graphics.Shader
import android.media.ExifInterface
import android.net.ConnectivityManager
import android.net.NetworkCapabilities
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.provider.Settings
import android.provider.MediaStore
import android.util.Log
import com.xiaoyi.action.ActionCamera
import com.xiaoyi.action.ActionCameraCommandCallback
import com.xiaoyi.action.ActionCameraCommandCallback1
import com.xiaoyi.action.ActionCameraListener
import com.xiaoyi.action.ActionCameraSettings
import com.xiaoyi.action.CameraStatus
import com.xiaoyi.action.ColorMode
import com.xiaoyi.action.FieldOfView
import com.xiaoyi.action.ISO
import com.xiaoyi.action.Logger
import com.xiaoyi.action.Platform
import com.xiaoyi.action.PhotoResolution
import com.xiaoyi.action.Quality
import com.xiaoyi.action.Sharpness
import com.xiaoyi.action.SystemMode
import com.xiaoyi.action.ToggleState
import com.xiaoyi.action.VideoResolution
import com.xiaoyi.action.WhiteBalance
import com.xiaoyi.action.YICameraSDKDispatchQueue
import com.xiaoyi.action.YICameraSDKError
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.embedding.android.FlutterActivity
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.atomic.AtomicBoolean

class MainActivity : FlutterActivity(), MethodChannel.MethodCallHandler {
    companion object {
        private const val METHOD_CHANNEL = "yi4k.remote/control"
        private const val EVENT_CHANNEL = "yi4k.remote/events"
        private const val DEFAULT_ENDPOINT = "tcp:192.168.42.1:7878"
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private val sdkThread = HandlerThread("yi-camera-sdk").apply { start() }
    private val sdkHandler = Handler(sdkThread.looper)
    private var camera: ActionCamera? = null
    private var eventSink: EventChannel.EventSink? = null
    private var pendingConnect: MethodChannel.Result? = null
    private var connected = false

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, METHOD_CHANNEL)
            .setMethodCallHandler(this)
        EventChannel(flutterEngine.dartExecutor.binaryMessenger, EVENT_CHANNEL)
            .setStreamHandler(object : EventChannel.StreamHandler {
                override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
                    eventSink = events
                    emit("connection", mapOf("connected" to connected))
                }

                override fun onCancel(arguments: Any?) {
                    eventSink = null
                }
            })
        initializeCamera()
    }

    private fun initializeCamera() {
        Platform.initialize(object : Logger() {
            override fun verbose(message: String) {
                Log.v("YI4K", message)
            }

            override fun info(message: String) {
                Log.i("YI4K", message)
            }

            override fun warning(message: String) {
                Log.w("YI4K", message)
            }

            override fun error(message: String) {
                Log.e("YI4K", message)
            }
        })

        camera = ActionCamera(object : ActionCameraListener() {
            override fun onConnected() {
                connected = true
                emit("connection", mapOf("connected" to true))
                mainHandler.post {
                    pendingConnect?.success(true)
                    pendingConnect = null
                }
                refreshSettings()
            }

            override fun onClosed(error: YICameraSDKError?) {
                connected = false
                emit("connection", mapOf("connected" to false, "detail" to error?.detail))
                mainHandler.post {
                    pendingConnect?.error("connect_failed", errorText(error), null)
                    pendingConnect = null
                }
            }

            override fun onRecordStarted() = emit("recording", mapOf("active" to true))
            override fun onRecordStopped() = emit("recording", mapOf("active" to false))
            override fun onCaptureStarted() = emit("capture", mapOf("active" to true))
            override fun onCaptureStopped() = emit("capture", mapOf("active" to false))
            override fun onViewFinderStarted() = emit(
                "viewfinder",
                mapOf("active" to true, "url" to camera?.rtspURL)
            )
            override fun onViewFinderStopped() = emit("viewfinder", mapOf("active" to false))
            override fun onBatteryLifeChanged(restBattery: Int) =
                emit("battery", mapOf("percent" to restBattery))

            override fun onSettingChanged(newSettings: ActionCameraSettings?) {
                if (newSettings != null) emit("settings", settingsMap(newSettings))
            }
        }, YICameraSDKDispatchQueue { task -> sdkHandler.post(task) })
    }

    override fun onMethodCall(call: MethodCall, result: MethodChannel.Result) {
        when (call.method) {
            "openWifiSettings" -> {
                startActivity(Intent(Settings.ACTION_WIFI_SETTINGS))
                result.success(null)
            }
            "connect" -> connect(call.argument<String>("ip") ?: "192.168.42.1", result)
            "disconnect" -> {
                camera?.disconnect()
                connected = false
                result.success(null)
            }
            "getSettings" -> getSettings(result)
            "getBattery" -> cameraOrError(result)?.getBatteryQuantity(
                { value -> mainHandler.post { result.success(value) } },
                { error -> sdkError(result, error) }
            )
            "startRecording" -> command(result) { ok, fail -> camera!!.startRecording(ok, fail) }
            "stopRecording" -> command(result) { ok, fail -> camera!!.stopRecording(ok, fail) }
            "capturePhoto" -> command(result) { ok, fail -> camera!!.capturePhoto(ok, fail) }
            "startViewFinder" -> command(result) { ok, fail -> camera!!.startViewFinder(ok, fail) }
            "stopViewFinder" -> command(result) { ok, fail -> camera!!.stopViewFinder(ok, fail) }
            "getRtspUrl" -> result.success(camera?.rtspURL)
            "setMode" -> setMode(call.argument<String>("mode"), result)
            "setPhotoResolution" -> setPhotoResolution(call.argument<String>("value"), result)
            "setVideoResolution" -> setVideoResolution(call.argument<String>("value"), result)
            "setQuality" -> setQuality(call.argument<String>("value"), result)
            "setFieldOfView" -> setFieldOfView(call.argument<String>("value"), result)
            "setWhiteBalance" -> setWhiteBalance(call.argument<String>("value"), result)
            "setIso" -> setIso(call.argument<String>("value"), result)
            "setStabilization" -> setStabilization(call.argument<Boolean>("enabled") == true, result)
            "setMicrophone" -> setMicrophone(call.argument<Boolean>("enabled") == true, result)
            "setSharpness" -> setSharpness(call.argument<String>("value"), result)
            "setColorMode" -> setColorMode(call.argument<String>("value"), result)
            "getFiles" -> getFiles(result)
            "getThumbnail" -> getThumbnail(call.argument<String>("name"), result)
            "downloadFile" -> downloadFile(
                call.argument<String>("name"),
                call.argument<Boolean>("crop2x") == true,
                call.argument<String>("filter") ?: "none",
                call.argument<Boolean>("enhance") == true,
                call.argument<List<Number>>("matrix")?.map { it.toFloat() },
                (call.argument<Number>("vignette")?.toFloat() ?: 0f),
                result
            )
            "getPreferences" -> result.success(readPreferences())
            "setPreference" -> setPreference(call, result)
            "deleteFile" -> deleteFile(call.argument<String>("name"), result)
            else -> result.notImplemented()
        }
    }

    private fun connect(ip: String, result: MethodChannel.Result) {
        if (pendingConnect != null) {
            result.error("busy", "正在連線相機", null)
            return
        }
        bindToWifi()
        pendingConnect = result
        camera?.connect("tcp:$ip:7878", 10_000)
    }

    private fun bindToWifi() {
        val manager = getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        val wifi = manager.allNetworks.firstOrNull { network ->
            manager.getNetworkCapabilities(network)
                ?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
        }
        if (wifi != null) manager.bindProcessToNetwork(wifi)
    }

    private fun getSettings(result: MethodChannel.Result) {
        val active = cameraOrError(result) ?: return
        active.getSettings(
            { settings -> mainHandler.post { result.success(settingsMap(settings)) } },
            { error -> sdkError(result, error) }
        )
    }

    private fun refreshSettings() {
        camera?.getSettings(
            { settings -> emit("settings", settingsMap(settings)) },
            { error -> emit("error", mapOf("message" to errorText(error))) }
        )
        camera?.getBatteryQuantity(
            { value -> emit("battery", mapOf("percent" to value)) },
            { error -> emit("error", mapOf("message" to errorText(error))) }
        )
    }

    private fun setMode(value: String?, result: MethodChannel.Result) {
        val mode = if (value == "photo") SystemMode.Capture else SystemMode.Record
        command(result) { ok, fail -> camera!!.setSystemMode(mode, ok, fail) }
    }

    private fun setVideoResolution(value: String?, result: MethodChannel.Result) {
        val resolution = try {
            VideoResolution.valueOf(value ?: "")
        } catch (_: IllegalArgumentException) {
            result.error("invalid_resolution", "不支援的錄影解析度", value)
            return
        }
        command(result) { ok, fail -> camera!!.setVideoResolution(resolution, ok, fail) }
    }

    private fun setPhotoResolution(value: String?, result: MethodChannel.Result) {
        val resolution = try {
            PhotoResolution.valueOf(value ?: "")
        } catch (_: IllegalArgumentException) {
            result.error("invalid_resolution", "不支援的照片解析度", value)
            return
        }
        command(result) { ok, fail -> camera!!.setPhotoResolution(resolution, ok, fail) }
    }

    private fun readPreferences(): Map<String, Any> {
        val preferences = getSharedPreferences("lumo_preferences", Context.MODE_PRIVATE)
        return mapOf(
            "crop2x" to preferences.getBoolean("crop2x", true),
            "previewCrop2x" to preferences.getBoolean("previewCrop2x", true),
            "enhance" to preferences.getBoolean("enhance", true),
            "autoDownload" to preferences.getBoolean("autoDownload", true),
            "filter" to (preferences.getString("filter", "leicaAuthentic") ?: "leicaAuthentic")
            ,"customContrast" to preferences.getFloat("customContrast", 1f).toDouble()
            ,"customSaturation" to preferences.getFloat("customSaturation", 1f).toDouble()
            ,"customWarmth" to preferences.getFloat("customWarmth", 0f).toDouble()
            ,"customFade" to preferences.getFloat("customFade", 0f).toDouble()
            ,"customVignette" to preferences.getFloat("customVignette", 0.1f).toDouble()
        )
    }

    private fun setPreference(call: MethodCall, result: MethodChannel.Result) {
        val key = call.argument<String>("key")
        val value = call.argument<Any>("value")
        if (key !in setOf(
                "crop2x", "previewCrop2x", "enhance", "autoDownload", "filter",
                "customContrast", "customSaturation", "customWarmth", "customFade",
                "customVignette"
            )) {
            result.error("invalid_preference", "不支援的設定", key)
            return
        }
        val editor = getSharedPreferences("lumo_preferences", Context.MODE_PRIVATE).edit()
        when (value) {
            is Boolean -> editor.putBoolean(key, value)
            is String -> editor.putString(key, value)
            is Number -> editor.putFloat(key, value.toFloat())
            else -> {
                result.error("invalid_preference", "設定值格式錯誤", value)
                return
            }
        }
        editor.apply()
        result.success(null)
    }

    private fun setQuality(value: String?, result: MethodChannel.Result) {
        val setting = enumValue<Quality>(value, result) ?: return
        command(result) { ok, fail -> camera!!.setVideoQuality(setting, ok, fail) }
    }

    private fun setFieldOfView(value: String?, result: MethodChannel.Result) {
        val setting = enumValue<FieldOfView>(value, result) ?: return
        command(result) { ok, fail -> camera!!.setVideoFieldOfView(setting, ok, fail) }
    }

    private fun setWhiteBalance(value: String?, result: MethodChannel.Result) {
        val setting = enumValue<WhiteBalance>(value, result) ?: return
        command(result) { ok, fail -> camera!!.setVideoWhiteBalance(setting, ok, fail) }
    }

    private fun setIso(value: String?, result: MethodChannel.Result) {
        val setting = enumValue<ISO>(value, result) ?: return
        command(result) { ok, fail -> camera!!.setVideoISO(setting, ok, fail) }
    }

    private fun setStabilization(enabled: Boolean, result: MethodChannel.Result) {
        command(result) { ok, fail ->
            camera!!.setElectronicImageStabilizationState(
                if (enabled) ToggleState.On else ToggleState.Off,
                ok,
                fail
            )
        }
    }

    private fun setMicrophone(enabled: Boolean, result: MethodChannel.Result) {
        command(result) { ok, fail ->
            camera!!.setVideoMuteState(
                if (enabled) ToggleState.Off else ToggleState.On,
                ok,
                fail
            )
        }
    }

    private fun setSharpness(value: String?, result: MethodChannel.Result) {
        val setting = enumValue<Sharpness>(value, result) ?: return
        command(result) { ok, fail -> camera!!.setVideoSharpness(setting, ok, fail) }
    }

    private fun setColorMode(value: String?, result: MethodChannel.Result) {
        val setting = enumValue<ColorMode>(value, result) ?: return
        command(result) { ok, fail -> camera!!.setVideoColorMode(setting, ok, fail) }
    }

    private inline fun <reified T : Enum<T>> enumValue(
        value: String?,
        result: MethodChannel.Result
    ): T? = try {
        enumValueOf<T>(value ?: "")
    } catch (_: IllegalArgumentException) {
        result.error("invalid_setting", "相機不支援這個設定", value)
        null
    }

    private fun getFiles(result: MethodChannel.Result) {
        val active = cameraOrError(result) ?: return
        active.getFileList(
            { files ->
                mainHandler.post {
                    result.success(files.map { file ->
                        mapOf(
                            "name" to file.name,
                            "size" to file.size,
                            "time" to file.time?.time
                        )
                    })
                }
            },
            { error -> sdkError(result, error) }
        )
    }

    private fun deleteFile(name: String?, result: MethodChannel.Result) {
        if (name.isNullOrBlank()) {
            result.error("missing_name", "缺少檔案名稱", null)
            return
        }
        command(result) { ok, fail -> camera!!.deleteFile(name, ok, fail) }
    }

    private fun getThumbnail(name: String?, result: MethodChannel.Result) {
        if (name.isNullOrBlank()) {
            result.success(null)
            return
        }
        sdkHandler.post {
            try {
                val cleanName = name.substringAfterLast('/').substringAfterLast('\\')
                val thumbnailRoot = File(cacheDir, "lumo_thumbnails").apply { mkdirs() }
                val partial = File(thumbnailRoot, "$cleanName.partial")
                var thumbnail = if (partial.exists()) {
                    try { ExifInterface(partial.absolutePath).thumbnail } catch (_: Throwable) { null }
                } else null
                if (thumbnail == null) {
                    val connection = URL(
                        "http://192.168.42.1/DCIM/100MEDIA/" +
                            java.net.URLEncoder.encode(cleanName, "UTF-8").replace("+", "%20")
                    ).openConnection() as HttpURLConnection
                    connection.connectTimeout = 1800
                    connection.readTimeout = 2200
                    connection.setRequestProperty("Range", "bytes=0-262143")
                    connection.inputStream.use { input ->
                        FileOutputStream(partial, false).use { output ->
                            val buffer = ByteArray(8192)
                            var total = 0
                            while (total < 262144) {
                                val read = input.read(buffer, 0, minOf(buffer.size, 262144 - total))
                                if (read <= 0) break
                                output.write(buffer, 0, read)
                                total += read
                            }
                        }
                    }
                    connection.disconnect()
                    thumbnail = try {
                        ExifInterface(partial.absolutePath).thumbnail
                    } catch (_: Throwable) {
                        null
                    }
                }
                mainHandler.post { result.success(thumbnail) }
            } catch (error: Throwable) {
                Log.d("LUMO", "Thumbnail unavailable for $name", error)
                mainHandler.post { result.success(null) }
            }
        }
    }

    private fun downloadFile(
        name: String?,
        crop2x: Boolean,
        filter: String,
        enhance: Boolean,
        matrix: List<Float>?,
        vignette: Float,
        result: MethodChannel.Result
    ) {
        if (name.isNullOrBlank()) {
            result.error("missing_name", "缺少檔案名稱", null)
            return
        }
        val active = cameraOrError(result) ?: return
        val downloadRoot = File(
            getExternalFilesDir(android.os.Environment.DIRECTORY_DOWNLOADS),
            "YI4K"
        ).apply { mkdirs() }
        val destination = File(downloadRoot, name.substringAfterLast('/'))
        if (destination.exists()) destination.delete()
        val replySubmitted = AtomicBoolean(false)
        var lastProgressPercent = -2
        emit("download", mapOf("name" to name, "active" to true))
        active.downloadFile(
            name,
            destination.absolutePath,
            success@{ task ->
                if (task.totalBytes <= 0L || task.downloadedBytes < task.totalBytes) {
                    val progress = if (task.totalBytes > 0L) {
                        ((task.downloadedBytes * 100L) / task.totalBytes).toInt()
                    } else {
                        0
                    }
                    if (progress >= lastProgressPercent + 2) {
                        lastProgressPercent = progress
                        emit(
                            "download",
                            mapOf(
                                "name" to name,
                                "active" to true,
                                "bytes" to task.downloadedBytes,
                                "total" to task.totalBytes
                            )
                        )
                    }
                    return@success
                }
                if (!replySubmitted.compareAndSet(false, true)) return@success
                Thread {
                    Thread.sleep(120)
                    val processed = processPhoto(
                        destination, crop2x, filter, enhance, matrix, vignette
                    )
                    val publicLocation = publishToGallery(destination)
                    emit(
                        "download",
                        mapOf(
                            "name" to name,
                            "active" to false,
                            "complete" to true,
                            "processed" to processed,
                            "crop2x" to crop2x,
                            "bytes" to task.downloadedBytes,
                            "total" to task.totalBytes
                        )
                    )
                    mainHandler.post {
                        result.success(
                            mapOf(
                                "name" to name,
                                "path" to (publicLocation ?: destination.absolutePath),
                                "processed" to processed,
                                "crop2x" to crop2x,
                                "bytes" to task.downloadedBytes
                            )
                        )
                    }
                }.start()
            },
            failure@{ error ->
                if (!replySubmitted.compareAndSet(false, true)) return@failure
                emit("download", mapOf("name" to name, "active" to false, "complete" to false))
                sdkError(result, error)
            }
        )
    }

    private fun processPhoto(
        source: File,
        crop2x: Boolean,
        filter: String,
        enhance: Boolean,
        matrix: List<Float>?,
        vignette: Float
    ): Boolean {
        val lower = source.name.lowercase()
        if (!lower.endsWith(".jpg") && !lower.endsWith(".jpeg")) return false
        var original: Bitmap? = null
        var cropped: Bitmap? = null
        var working: Bitmap? = null
        var output: Bitmap? = null
        return try {
            val bitmap = BitmapFactory.decodeFile(
                source.absolutePath,
                BitmapFactory.Options().apply {
                    inPreferredConfig = Bitmap.Config.RGB_565
                }
            ) ?: return false
            original = bitmap
            working = if (crop2x) {
                val cropWidth = (bitmap.width / 2).coerceAtLeast(1)
                val cropHeight = (bitmap.height / 2).coerceAtLeast(1)
                val crop = Bitmap.createBitmap(
                    bitmap,
                    (bitmap.width - cropWidth) / 2,
                    (bitmap.height - cropHeight) / 2,
                    cropWidth,
                    cropHeight
                )
                cropped = crop
                Bitmap.createScaledBitmap(crop, bitmap.width, bitmap.height, true)
            } else {
                bitmap
            }
            val target = Bitmap.createBitmap(bitmap.width, bitmap.height, Bitmap.Config.RGB_565)
            output = target
            val paint = Paint(Paint.ANTI_ALIAS_FLAG or Paint.FILTER_BITMAP_FLAG)
            val chosenMatrix = if (matrix?.size == 20) {
                ColorMatrix(matrix.toFloatArray())
            } else {
                filmMatrix(filter, enhance)
            }
            paint.colorFilter = ColorMatrixColorFilter(chosenMatrix)
            Canvas(target).drawBitmap(working!!, 0f, 0f, paint)
            if (enhance) applyLightSharpen(target)
            if (vignette > 0f) applyVignette(target, vignette.coerceIn(0f, 0.65f))
            FileOutputStream(source, false).use { output ->
                check(target.compress(Bitmap.CompressFormat.JPEG, 98, output))
            }
            true
        } catch (error: Throwable) {
            Log.e("LUMO", "Could not process photo", error)
            false
        } finally {
            if (output !== working && output !== original) output?.recycle()
            if (working !== cropped && working !== original) working?.recycle()
            if (cropped !== original) cropped?.recycle()
            original?.recycle()
        }
    }

    private fun filmMatrix(filter: String, enhance: Boolean): ColorMatrix {
        val matrix = when (filter) {
            "classic" -> ColorMatrix(floatArrayOf(
                1.08f, -0.03f, -0.02f, 0f, 4f,
                -0.02f, 1.03f, -0.01f, 0f, 2f,
                -0.04f, 0.02f, 0.92f, 0f, -2f,
                0f, 0f, 0f, 1f, 0f
            ))
            "gold" -> ColorMatrix(floatArrayOf(
                1.12f, 0.02f, -0.04f, 0f, 5f,
                0.01f, 1.02f, -0.03f, 0f, 2f,
                -0.06f, 0.02f, 0.88f, 0f, -4f,
                0f, 0f, 0f, 1f, 0f
            ))
            "cine" -> ColorMatrix(floatArrayOf(
                0.95f, 0.03f, 0.02f, 0f, -2f,
                -0.02f, 1.06f, 0.01f, 0f, 1f,
                0.01f, 0.04f, 1.08f, 0f, 1f,
                0f, 0f, 0f, 1f, 0f
            ))
            "mono" -> ColorMatrix().apply { setSaturation(0f) }
            else -> ColorMatrix()
        }
        if (enhance) {
            val contrast = 1.05f
            val translate = (-0.5f * contrast + 0.5f) * 255f
            matrix.postConcat(ColorMatrix(floatArrayOf(
                contrast, 0f, 0f, 0f, translate,
                0f, contrast, 0f, 0f, translate,
                0f, 0f, contrast, 0f, translate,
                0f, 0f, 0f, 1f, 0f
            )))
        }
        return matrix
    }

    private fun applyLightSharpen(bitmap: Bitmap) {
        if (bitmap.width < 3 || bitmap.height < 3) return
        val width = bitmap.width
        var previous = IntArray(width)
        var current = IntArray(width)
        var next = IntArray(width)
        val result = IntArray(width)
        bitmap.getPixels(previous, 0, width, 0, 0, width, 1)
        bitmap.getPixels(current, 0, width, 0, 1, width, 1)
        for (y in 1 until bitmap.height - 1) {
            bitmap.getPixels(next, 0, width, 0, y + 1, width, 1)
            result[0] = current[0]
            result[width - 1] = current[width - 1]
            for (x in 1 until width - 1) {
                val center = current[x]
                val left = current[x - 1]
                val right = current[x + 1]
                val up = previous[x]
                val down = next[x]
                fun channel(shift: Int): Int {
                    val c = (center shr shift) and 0xff
                    val around = (((left shr shift) and 0xff) + ((right shr shift) and 0xff) +
                        ((up shr shift) and 0xff) + ((down shr shift) and 0xff)) / 4
                    val denoised = if (kotlin.math.abs(c - around) < 14) {
                        c * 0.78f + around * 0.22f
                    } else {
                        c.toFloat()
                    }
                    return (denoised + (denoised - around) * 0.30f)
                        .toInt().coerceIn(0, 255)
                }
                result[x] = (0xff shl 24) or (channel(16) shl 16) or
                    (channel(8) shl 8) or channel(0)
            }
            bitmap.setPixels(result, 0, width, 0, y, width, 1)
            val swap = previous
            previous = current
            current = next
            next = swap
        }
    }

    private fun applyVignette(bitmap: Bitmap, amount: Float) {
        val canvas = Canvas(bitmap)
        val radius = kotlin.math.max(bitmap.width, bitmap.height) * 0.72f
        val shader = RadialGradient(
            bitmap.width / 2f,
            bitmap.height / 2f,
            radius,
            intArrayOf(android.graphics.Color.TRANSPARENT, android.graphics.Color.TRANSPARENT,
                android.graphics.Color.argb((amount * 255).toInt(), 0, 0, 0)),
            floatArrayOf(0f, 0.58f, 1f),
            Shader.TileMode.CLAMP
        )
        canvas.drawRect(0f, 0f, bitmap.width.toFloat(), bitmap.height.toFloat(),
            Paint(Paint.ANTI_ALIAS_FLAG).apply { this.shader = shader })
    }

    private fun publishToGallery(source: File): String? {
        if (!source.exists()) return null
        val lower = source.name.lowercase()
        val isVideo = lower.endsWith(".mp4") || lower.endsWith(".mov")
        val isPhoto = lower.endsWith(".jpg") || lower.endsWith(".jpeg") || lower.endsWith(".png")
        val mime = when {
            isVideo -> "video/mp4"
            isPhoto -> if (lower.endsWith(".png")) "image/png" else "image/jpeg"
            else -> "application/octet-stream"
        }
        val collection = if (isVideo) {
            MediaStore.Video.Media.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY)
        } else if (isPhoto) {
            MediaStore.Images.Media.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY)
        } else {
            MediaStore.Downloads.getContentUri(MediaStore.VOLUME_EXTERNAL_PRIMARY)
        }
        val relativePath = when {
            isVideo -> "Movies/LUMO"
            isPhoto -> "Pictures/LUMO"
            else -> "Download/LUMO"
        }
        val values = ContentValues().apply {
            put(MediaStore.MediaColumns.DISPLAY_NAME, source.name)
            put(MediaStore.MediaColumns.MIME_TYPE, mime)
            put(MediaStore.MediaColumns.RELATIVE_PATH, relativePath)
            put(MediaStore.MediaColumns.IS_PENDING, 1)
        }
        return try {
            contentResolver.delete(
                collection,
                "${MediaStore.MediaColumns.DISPLAY_NAME}=? AND ${MediaStore.MediaColumns.RELATIVE_PATH}=?",
                arrayOf(source.name, "$relativePath/")
            )
            val uri = contentResolver.insert(collection, values) ?: return null
            contentResolver.openOutputStream(uri)?.use { output ->
                FileInputStream(source).use { input -> input.copyTo(output) }
            }
            values.clear()
            values.put(MediaStore.MediaColumns.IS_PENDING, 0)
            contentResolver.update(uri, values, null, null)
            uri.toString()
        } catch (error: Exception) {
            Log.w("YI4K", "Could not publish download", error)
            null
        }
    }

    private fun command(
        result: MethodChannel.Result,
        action: (ActionCameraCommandCallback, ActionCameraCommandCallback1<YICameraSDKError>) -> Unit
    ) {
        if (cameraOrError(result) == null) return
        action(
            ActionCameraCommandCallback { mainHandler.post { result.success(null) } },
            ActionCameraCommandCallback1 { error -> sdkError(result, error) }
        )
    }

    private fun cameraOrError(result: MethodChannel.Result): ActionCamera? {
        if (!connected || camera == null) {
            result.error("not_connected", "請先連線到 YI 4K 相機", null)
            return null
        }
        return camera
    }

    private fun settingsMap(settings: ActionCameraSettings): Map<String, Any?> = mapOf(
        "productName" to settings.productName,
        "serialNumber" to settings.serialNumber,
        "softwareVersion" to settings.softwareVersion,
        "hardwareVersion" to settings.hardwareVersion,
        "status" to settings.status?.name,
        "systemMode" to settings.systemMode?.name,
        "videoResolution" to settings.videoResolution?.name,
        "photoResolution" to settings.photoResolution?.name,
        "videoQuality" to settings.videoQuality?.name,
        "fieldOfView" to settings.videoFieldOfView?.name,
        "videoWhiteBalance" to settings.videoWhiteBalance?.name,
        "videoISO" to settings.videoISO?.name,
        "videoExposure" to settings.videoExposureValue?.name,
        "stabilization" to settings.electronicImageStabilizationState?.name,
        "microphone" to settings.videoMuteState?.name,
        "sharpness" to settings.videoSharpness?.name,
        "colorMode" to settings.videoColorMode?.name
    )

    private fun emit(type: String, payload: Map<String, Any?>) {
        mainHandler.post { eventSink?.success(payload + ("type" to type)) }
    }

    private fun sdkError(result: MethodChannel.Result, error: YICameraSDKError?) {
        mainHandler.post {
            result.error("yi_${error?.code ?: -1}", errorText(error), error?.detail)
        }
    }

    private fun errorText(error: YICameraSDKError?): String =
        error?.detail?.takeIf { it.isNotBlank() } ?: "相機沒有回應，請確認已連上相機 Wi‑Fi"

    override fun onDestroy() {
        camera?.disconnect()
        sdkThread.quitSafely()
        super.onDestroy()
    }
}
