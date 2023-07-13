package com.skythinker.balancebot;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.webkit.JavascriptInterface;
import android.webkit.WebView;

import androidx.annotation.NonNull;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;

public class CtrlView extends WebView {

    public interface Callback {
        void onUserInput(String type, Object value);
    } //接收用户输入事件的回调接口

    private Bitmap rockerBmp, videoBmp, btOnBmp, btOffBmp, videoOnBmp, videoOffBmp; //各图片的Bitmap对象
    private int width, height; //视图宽高
    private boolean rockerDown = false, sliderDown = false, configDown = false; //各控件是否正在被按下
    private Rect rockerRect = new Rect(), sliderRect = new Rect(), videoRect = new Rect(); //各控件的区域
    private Rect btStateRect = new Rect(), videoStateRect = new Rect(), configRect = new Rect();
    public float rockerValueX = 0.0f, rockerValueY = 0.0f; //摇杆的值(-1.0~1.0)
    public float sliderValue = -1.0f; //滑块的值(-1.0~1.0)
    public boolean bluetoothConnected = false, videoConnected = false; //标记蓝牙和视频是否连接
    public boolean isVideoMode = false; //标记当前是视频模式还是蓝牙模式
    Handler handler = new Handler();
    private String messageText = ""; //显示在左下角的文字提醒
    Callback callback = null;

    @SuppressLint({"JavascriptInterface", "SetJavaScriptEnabled"})
    public CtrlView(Context context) {
        super(context);

        //加载图片
        rockerBmp = BitmapFactory.decodeResource(this.getContext().getResources(),R.drawable.rocker);
        btOnBmp = BitmapFactory.decodeResource(this.getContext().getResources(),R.drawable.bt_on);
        btOffBmp = BitmapFactory.decodeResource(this.getContext().getResources(),R.drawable.bt_off);
        videoOnBmp = BitmapFactory.decodeResource(this.getContext().getResources(),R.drawable.video_on);
        videoOffBmp = BitmapFactory.decodeResource(this.getContext().getResources(),R.drawable.video_off);

        setFocusable(true);
        setFocusableInTouchMode(true);
        setClickable(true);

        getSettings().setJavaScriptEnabled(true); //启用JavaScript
        loadUrl("file:///android_asset/streamview.html"); //加载本地HTML页面
        addJavascriptInterface(this, "webview"); //创建WebView交互接口
    }

    @JavascriptInterface
    public void jsCallback(String type, String message) { //接收来自HTML页面的消息
        if(type.equals("setVideoState")){
            if(message.equals("true"))
                videoConnected = true;
            else
                videoConnected = false;
            invalidate();
        }
    }

    @SuppressLint("DrawAllocation")
    @Override
    protected void onDraw(Canvas canvas) { //在WebView上叠加绘制控件
        super.onDraw(canvas); //绘制WebView原本内容

        Paint paint = new Paint();
        paint.setAntiAlias(true);

        //绘制状态标识&配置按钮
        paint.setColor(Color.argb(configDown?30:60,0xFF,0xFF,0xFF));
        canvas.drawRoundRect(new RectF(configRect),20,20,paint);
        paint.setAlpha(255);
        if(isVideoMode)
            canvas.drawBitmap(videoConnected?videoOnBmp:videoOffBmp,
                    new Rect(0,0,btOnBmp.getWidth(),btOnBmp.getHeight()),
                    new Rect(configRect.left+20,configRect.top+20,configRect.right-20,configRect.bottom-20),paint);
        else
            canvas.drawBitmap(bluetoothConnected?btOnBmp:btOffBmp,
                    new Rect(0,0,btOnBmp.getWidth(),btOnBmp.getHeight()),
                    new Rect(configRect.left+20,configRect.top+20,configRect.right-20,configRect.bottom-20),paint);

        //绘制摇杆
        paint.setAlpha(rockerDown?255:100);
        canvas.drawBitmap(rockerBmp,
                new Rect(0,0,rockerBmp.getWidth(),rockerBmp.getHeight()),
                rockerRect,paint);
        paint.setColor(Color.argb(rockerDown?150:30,0xFF,0xFF,0xFF));
        canvas.drawCircle(rockerValueX*rockerRect.width()/2.0f+rockerRect.centerX(),
                -rockerValueY*rockerRect.height()/2.0f+rockerRect.centerY(),rockerRect.width()/8.0f,paint);

        //绘制滑块
        paint.setColor(Color.argb(sliderDown?150:30,0xFF,0xFF,0xFF));
        canvas.drawRoundRect(sliderRect.left,sliderRect.centerY()-sliderRect.height()/2,sliderRect.right,sliderRect.centerY()+sliderRect.height()/2,
                sliderRect.height()/2,sliderRect.height()/2,paint);
        paint.setColor(Color.argb(sliderDown?150:50,29,145,101));
        float sliderX = sliderValue*(sliderRect.width()-sliderRect.height())/2.0f+sliderRect.centerX();
        canvas.drawCircle(sliderX,sliderRect.centerY(),sliderRect.height()/2,paint);

        //绘制文字提示
        if(messageText.length() > 0) {
            paint.setTextSize(50);
            paint.setColor(Color.argb(100, 0xFF, 0xFF, 0xFF));
            RectF textRect = new RectF(100, height - 50 - 10, (int) paint.measureText(messageText) + 140, height);
            canvas.drawRoundRect(textRect,20,20, paint);
            paint.setColor(Color.argb(255, 0, 0, 0));
            canvas.drawText(messageText, 120, height - 10, paint);
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) { //处理触摸事件
        switch (event.getAction() & MotionEvent.ACTION_MASK)
        {
            //有手指按下
            case MotionEvent.ACTION_DOWN:
            case MotionEvent.ACTION_POINTER_DOWN:
                float x = event.getX();
                float y = event.getY();
                if (distance(x,y,rockerRect.centerX(),rockerRect.centerY()) < rockerRect.width()/2.0f) //在摇杆范围内
                {
                    rockerDown = true;
                    rockerValueX = (x-rockerRect.centerX())/rockerRect.width()*2.0f;
                    rockerValueY = -(y-rockerRect.centerY())/rockerRect.height()*2.0f;
                    if(Math.abs(rockerValueY/rockerValueX) < 0.4)
                        rockerValueY = 0.0f;
                    if(Math.abs(rockerValueX/rockerValueY) < 0.4)
                        rockerValueX = 0.0f;
                    invalidate();
                }
                else if (x>sliderRect.left && x<sliderRect.right && y>sliderRect.top && y<sliderRect.bottom) //在滑块范围内
                {
                    sliderDown = true;
                    sliderValue = (x-sliderRect.centerX())/sliderRect.width()*2.0f;
                    invalidate();
                }
                else if (x>configRect.left && x<configRect.right && y>configRect.top && y<configRect.bottom) //在配置按钮范围内
                {
                    configDown = true;
                    invalidate();
                }
                break;

            //手指正在移动
            case MotionEvent.ACTION_MOVE:
                if (rockerDown) //拖动摇杆
                {
                    float x1 = event.getX();
                    float y1 = event.getY();
                    float dist = distance(x1,y1,rockerRect.centerX(),rockerRect.centerY());
                    if(dist > rockerRect.width()/2.0f)
                    {
                        x1 = rockerRect.centerX() + (x1-rockerRect.centerX())*rockerRect.width()/2.0f/dist;
                        y1 = rockerRect.centerY() + (y1-rockerRect.centerY())*rockerRect.height()/2.0f/dist;
                    }
                    rockerValueX = (x1-rockerRect.centerX())/rockerRect.width()*2.0f;
                    rockerValueY = -(y1-rockerRect.centerY())/rockerRect.height()*2.0f;
                    if(Math.abs(rockerValueX/rockerValueY) < 0.4)
                        rockerValueX = 0.0f;
                    if(Math.abs(rockerValueY/rockerValueX) < 0.4)
                        rockerValueY = 0.0f;
                    invalidate();
                }
                else if (sliderDown) //拖动滑块
                {
                    float x1 = event.getX();
                    float y1 = event.getY();
                    sliderValue = (x1-sliderRect.centerX())/sliderRect.width()*2.0f;
                    if(sliderValue > 1.0f)
                        sliderValue = 1.0f;
                    else if(sliderValue < -1.0f)
                        sliderValue = -1.0f;
                    invalidate();
                }
                break;

            //手指抬起
            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_POINTER_UP:
                if (rockerDown) //摇杆复位
                {
                    rockerDown = false;
                    rockerValueX = 0.0f;
                    rockerValueY = 0.0f;
                    invalidate();
                }
                else if (sliderDown) //松开滑块
                {
                    sliderDown = false;
                    invalidate();
                }
                else if (configDown) //松开配置按钮
                {
                    float x1 = event.getX();
                    float y1 = event.getY();
                    configDown = false;
                    if (x1>configRect.left && x1<configRect.right && y1>configRect.top && y1<configRect.bottom) //在配置按钮范围内松开才判定点击有效
                    {
                        if(callback != null)
                            callback.onUserInput("click","config");
                    }
                    invalidate();
                }
                break;
        }
        return super.onTouchEvent(event);
    }

    //设置图像服务器IP地址
    public void setServerIP(String ip)
    {
        handler.post(new Runnable() {
            @Override
            public void run() {
                loadUrl("javascript:setServerIP('" + ip + "');");
            } //调用HTML页面中的函数
        });
    }

    //设置视图尺寸
    public void setSize(int w, int h)
    {
        width = w;
        height = h;
        rockerRect.set(width*3/4-height/4,height/2-height/4,width*3/4+height/4,height/2+height/4);
        sliderRect.set(width*3/4-height/6,height*5/6-40,width*3/4+height/6,height*5/6+40);
        btStateRect.set(width/2-60,0,width/2,60);
        configRect.set(100,0,100+100,100);
        videoStateRect.set(width/2,0,width/2+60,60);
        if(width*1.0f/height < 320.0f/240)
            videoRect.set(0,height/2-width*240/320/2,width,height/2+width*240/320/2);
        else
            videoRect.set(width/2-height*320/240/2,0,width/2+height*320/240/2,height);
    }

    //计算两点距离
    private float distance(float x1, float y1, float x2, float y2)
    {
        return (float)Math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }

    //设置提示文字
    public void setMessageText(String text)
    {
        messageText = text;
        invalidate();
    }

    //设置事件回调
    public void setCallback(Callback callback) {
        this.callback = callback;
    }
}
