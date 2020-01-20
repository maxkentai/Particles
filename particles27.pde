// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// 1.06 transmodal feedback test rms > thresh -> attract wird repell
// 1.16 input and output edges stored in particle not global array
// 1.24 clean up, performance optimization
// 1.25 lange nacht version
// 1.26 limiter masterpräsi.  distanz kinect 3.5m. wand kinect/beamer 4.27m. ls 7.5m
// 1.27 flick the world

import beads.*;
import org.jaudiolibs.beads.AudioServerIO;

import controlP5.*;
import org.openkinect.freenect2.*;
import org.openkinect.processing.*;
import com.thomasdiewald.pixelflow.java.DwPixelFlow;
import com.thomasdiewald.pixelflow.java.imageprocessing.DwOpticalFlow;
import com.thomasdiewald.pixelflow.java.imageprocessing.filter.DwFilter;

import processing.core.*;
import processing.opengl.PGraphics2D;

import codeanticode.syphon.*;
import java.util.Arrays;

AudioContext ac;
SyphonServer server;

IOAudioFormat audioFormat;
float sampleRate = 44100;
int buffer = 256;
int bitDepth = 16;
int inputs = 2;
int outputs = 4;
boolean stereo = true;  // stereo or quad


ControlP5 cp5;
Kinect2 testKinect;
Kinect2 kinect2a;
Kinect2 kinect2b;

Kinect kinect1a;
Kinect kinect1b;

DwPixelFlow context;
DwOpticalFlow opticalflow;
PGraphics2D pg_cam;  // this graphic will be analyzed by pixelflow
//PGraphics2D pg_oflow;
int   pgImgWidth;    // image size for pg_cam image for flowtracking. equal to flowField dimensions
int   pgImgHeight;    

PImage img1;  // from left cam
PImage img2;  // from right cam
PImage img3;  // combined img
PImage imgBlur; // blurred image of img3
PGraphics imgMouse; // used for mouse input drawing
PGraphics pg;
int skip = 2;      // numbers of pixels to step over, when analyzing the depth image
int cam_w;  // kinect one depth image resolution
int cam_h;
int view_w = 1280;  // if not full screen
int view_h = 720;
int viewYOffset = 100;  // upper black margin, if fullscreen.
//int viewYOffset = 200;  // upper black margin, if fullscreen.
int   imgWidth;
int   imgHeight;    // width of the constructed 2D depth image

int minDepth;    // kinect min = ca. 480mm
int maxDepth;    // max 4500mm
int depthOffset;    // vertical offset of kinect2b right camera

float minX;   // lower x limit as xDimHalf factor 
float maxX;   // upper x limit as xDimHalf factor
float xDimHalf; // half of the converted depthimage real maximum x dimension
float minXDim;
float maxXDim;
float minZ; // cam von oben -> horizontale dimension
float maxZ;
float zDimHalf; // for vertical cam position half of the converted depth image real z dimension (was camera y dimension)
float minY;   // lower y limit as yDimHalf factor 
float maxY;   // upper y limit as yDimHalf factor
float yDimHalf; // for horizontal cam position
float minYDim;
float maxYDim;
// img 1 = links, img 2  = rechts
float img1LeftMargin;  // margin factor * imgWidth
float img1RightMargin;
int img1Width;

float img2LeftMargin;  // margin factor * imgWidth
float img2RightMargin;
int img2Width;

float[] depthLookUp = new float[2048];
// float buffer for pixel transfer from OpenGL to the host application
float[] flow_velocity;
float minCreateDist = 150;  // min dist from other particles for particle creation
boolean debug;
boolean setupMode = false;
boolean freeze;
boolean record;

FlowField flowField;
int flowFieldResolution = 20;  // in pixels
int frameSkip = 2;    // get velocity vectors from pixelflow and update flowField every n frame

ParticleSystem ps;
int maxNumParticles = 30;
Edge[][] edges = new Edge[maxNumParticles][maxNumParticles];  // contains edge references
float maxLife = 600;  // edge survival time in frames, if not intersecting
//Repeller repeller;
boolean moved;
float separation = 1;
float minBgBrightness = 0.01;  // brightness of blurred img
Slider separationSlider;
int particleRadius = 40;
float attractFactor = 0.01;
Slider attractFactorSlider;
float repulsionFactor = 0.5;    // * -attractFactor
Slider repulsionFactorSlider;
int attractRadius = 100;
Slider attractRadiusSlider;
float hpfFreq = 20;
Slider hpfSlider;
float pitchBase = pow(2, (1.0 / 12)); // 1.059460646483;
float pitchRange = 36;
Slider pitchRangeSlider;

float delayFactor = 0.25;
//float delayFactor = 0.78;
Slider delayFactorSlider;
float drag = 1.0;
float rmsForceAmt = 0.0;  // 0-1 , 1: attract/repulse force depends on rms level

float hiThresh = 0.7;  // used for attraction repulsion
Slider hiThreshSlider;
float loThresh = 0.1;
Slider loThreshSlider;
float gateHiThresh = 0.6;  // used for level gate
float gateLoThresh = 0.1;
float attack = 100;    // gate attack
Slider attackSlider;
float decay = 200;    // gate decay
Slider decaySlider;
float levelerMix = 0.71;  // mix between leveler function and 1 as signal factor
Slider levelerMixSlider;
int maxConnex = 10;
Slider maxConnexSlider;

boolean forceElastic;  // true: nodes are connected with elastic forces, false: distance based attraction/repulsion

UGen micIn;
Gain micGain;
Envelope micGainEnv;

PImage texture;

boolean START_FULLSCREEN = true;
boolean kinectConnected = false;
boolean kinectOne = true;  // false = kinect 360
boolean camPositionVertical = false; // false = horizontal
boolean test;
int testX;
int testY;
ArrayList <PVector> testPoints;

PImage canvas;
PImage that;

public void settings() {
  if (START_FULLSCREEN) {
    fullScreen(P2D, 2);
  } else {
    size(view_w, view_h, P2D);
  }
  smooth();
}


void setup() {
  if (!START_FULLSCREEN) {
    //surface.setLocation(view_w, view_h);
  } else {
    view_w = width;
    view_h = height - viewYOffset;
    noCursor();
  } 

  println("view_w: " + view_w + " view_h: " + view_h);

  //surface.setResizable(true);
  //fullScreen(P3D);

  if (stereo) {
    ac = new AudioContext(128);
  } else {
    audioFormat = new IOAudioFormat(sampleRate, bitDepth, inputs, outputs);
    ac = new AudioContext(new AudioServerIO.Jack(), buffer, audioFormat);
  }
  ac.start();

  //micIn = ac.getAudioInput();
  //micGainEnv = new Envelope(ac, 0);
  //micGain = new Gain(ac, 1, micGainEnv);
  //micGain.addInput(micIn);

  texture = loadImage("texture.png");


  // different kinects and positions setup
  if (kinectOne) {
    // set parameters
    cam_w = 512 ;  // kinect one depth image resolution
    cam_h = 424 ;
    imgWidth = cam_w / skip ;
    imgHeight = cam_h / skip ;    // width of the constructed 2D depth image

    if (camPositionVertical) {
      minDepth = 670;    // kinect one min = ca. 480
      maxDepth = 2300;
      depthOffset = 0;    // vertical offset of kinect2b right camera

      minX = 0.2;   // lower x limit as xDimHalf factor 
      maxX = 0.8;   // upper x limit as xDimHalf factor
      xDimHalf = CameraParams.cx * max(maxDepth, maxDepth + depthOffset) / CameraParams.fx; // half of the converted depthimage real maximum x dimension
      minZ = 30; // cam von oben -> horizontale dimension
      maxZ = 57;
      zDimHalf = CameraParams.cy * max(maxDepth, maxDepth + depthOffset) / CameraParams.fy; // half of the converted depth image real z dimension (was camera y dimension)

      // img 1 = links, img 2  = rechts
      img1LeftMargin = 0.15;  // margin factor * imgWidth
      img1RightMargin = 0.15;
      img1Width = imgWidth - (int)(imgWidth * img1LeftMargin) - (int)(imgWidth * img1RightMargin);
      println("img1width "+img1Width);
      img2LeftMargin = 0.16;  // margin factor * imgWidth
      img2RightMargin = 0.13;
      img2Width = int(imgWidth - (imgWidth * img2LeftMargin) - (imgWidth * img2RightMargin));
      println("img2width "+img2Width);
      //
    } else {  // cam position horizontal ------------------------------------

      minDepth = 1000;    // kinect one min = ca. 480
      maxDepth = 2200;

      minX = 0.1;   // lower x limit as xDimHalf factor 
      maxX = 0.9;   // upper x limit as xDimHalf factor
      xDimHalf = CameraParams.cx * max(maxDepth, maxDepth + depthOffset) / CameraParams.fx; // half of the converted depthimage real maximum x dimension

      minY = 0.2;   // from top upper limit
      maxY = 0.9;  // lower limit
      yDimHalf = CameraParams.cy * max(maxDepth, maxDepth + depthOffset) / CameraParams.fy; // half of the converted depth image real y dimension 

      // img 1 = links, img 2  = rechts
      img1LeftMargin = 0.16;  // margin factor * imgWidth
      img1RightMargin = 0.09;
      img1Width = imgWidth - (int)(imgWidth * img1LeftMargin) - (int)(imgWidth * img1RightMargin);
      println("img1width "+img1Width);
      img2LeftMargin = 0.09;  // margin factor * imgWidth
      img2RightMargin = 0.16;
      img2Width = int(imgWidth - (imgWidth * img2LeftMargin) - (imgWidth * img2RightMargin));
      println("img2width "+img2Width);
    }

    // init
    testKinect = new Kinect2(this);
    println("Number of kinects connected: " + testKinect.getNumKinects());

    if (testKinect.getNumKinects() == 0) {
      kinectConnected = false;
    } else {
      kinectConnected = true;
    }

    if (kinectConnected) {
      testKinect.initDepth();
      testKinect.initDevice();

      kinect2a = new Kinect2(this);
      kinect2a.initDepth();

      kinect2b = new Kinect2(this);
      kinect2b.initDepth();

      if (testKinect.getDefaulSerialNum().equals("020624740947")) { // cam 1
        kinect2a = testKinect; //index 0
        kinect2b.initDevice(1); //index 1
      } else {
        kinect2a.initDevice(1); //index 1
        kinect2b = testKinect;
      }
    }
  } // end kinect One


  else {  // kinect 360
    // Lookup table for all possible depth values (0 - 2047)
    for (int i = 0; i < depthLookUp.length; i++) {
      depthLookUp[i] = rawDepthToMeters(i);
    }

    // set parameters
    cam_w = 640 ;  // kinect 360 depth image resolution
    cam_h = 480 ;
    skip = 4;
    imgWidth = cam_w / skip ;
    imgHeight = cam_h / skip ;    // width of the constructed 2D depth image

    if (camPositionVertical) {
      // tbd
    } else {  // cam position horizontal
      minDepth = 800;    // kinect 360 raw depth values : 0 - 2047
      maxDepth = 961;

      minX = 0.2;   // lower x limit as xDimHalf factor 
      maxX = 0.8;   // upper x limit as xDimHalf factor

      xDimHalf = (float)(CameraParams360.cx * depthLookUp[maxDepth] * CameraParams360.fx); // half of the converted depthimage real maximum x dimension at maxdepth
      yDimHalf = (float)(CameraParams360.cy * depthLookUp[maxDepth] * CameraParams360.fy); // half of the converted depthimage real maximum x dimension
      //println("x/y dim " + xDimHalf, yDimHalf);
      minY = 0.1; 
      maxY = 0.8;

      // img 1 = links, img 2  = rechts
      img1LeftMargin = 0.2;  // margin factor * imgWidth
      img1RightMargin = 0.16;
      img1Width = imgWidth - (int)(imgWidth * img1LeftMargin) - (int)(imgWidth * img1RightMargin);

      img2LeftMargin = 0.16;  // margin factor * imgWidth
      img2RightMargin = 0.1;
      img2Width = int(imgWidth - (imgWidth * img2LeftMargin) - (imgWidth * img2RightMargin));
    }


    if (Kinect.countDevices() > 0) kinectConnected = true;
    kinect1a = new Kinect(this);
    kinect1a.activateDevice(1);
    kinect1a.initDepth();
    kinect1b = new Kinect(this);
    kinect1b.activateDevice(0);
    kinect1b.initDepth();
  }  // end kinect 360

  println("kinectConnected: " + kinectConnected);
  minXDim = minX * xDimHalf * 2;  
  maxXDim = maxX * xDimHalf * 2;
  minYDim = minY * yDimHalf * 2;
  maxYDim = maxY * yDimHalf * 2;


  flowField = new FlowField(flowFieldResolution, viewYOffset);
  println(flowField.cols + " "  + flowField.rows);
  pgImgWidth = flowField.cols;
  pgImgHeight = flowField.rows;
  flow_velocity = new float[pgImgWidth * pgImgHeight * 2]; // *2 because x and y velocity
  imgBlur = createImage(pgImgWidth, pgImgHeight, RGB);

  // main library context
  context = new DwPixelFlow(this);
  context.print();
  context.printGL();

  // optical flow
  opticalflow = new DwOpticalFlow(context, pgImgWidth, pgImgHeight);

  pg_cam = (PGraphics2D) createGraphics(pgImgWidth, pgImgHeight, P2D);
  //pg_cam.noSmooth();

  //pg_oflow = (PGraphics2D) createGraphics(width, height, P2D);
  //pg_oflow.smooth(4);

  //flowImg = createImage(width, height, RGB);

  imgMouse = createGraphics(imgWidth, imgHeight, P2D);  // for simulating cam input with the mouse

  ps = new ParticleSystem(maxNumParticles);
  //repeller = new Repeller(width/2, height/2);

  testPoints = new ArrayList();


  background(0);
  frameRate(60);
  //noSmooth();
  //hint(ENABLE_DEPTH_TEST);
  //blendMode(ADD);

  //canvas = createImage(width, height-viewYOffset, RGB); // create the canvas to render to and used to transmit via syphon
  // set up a new sypon (i.e. texture) server
  //server = new SyphonServer(this, "Processing_Syphon");



  cp5 = new ControlP5(this);
  cp5.setVisible(false);

  // add a horizontal sliders, the value of this slider will be linked
  // to variable 'sliderValue' 
  separationSlider = cp5.addSlider("separation")
    .setPosition(10, 20)
    .setRange(0, 1.0)
    ;
  attractRadiusSlider = cp5.addSlider("attractRadius")
    .setPosition(10, 30)
    .setRange(1, 400)
    ;
  attractFactorSlider = cp5.addSlider("attractFactor")
    .setPosition(10, 40)
    .setRange(0, 1)
    ;
  repulsionFactorSlider = cp5.addSlider("repulsionFactor")
    .setPosition(10, 50)
    .setRange(0, 1)
    ;
  cp5.addSlider("drag")
    .setPosition(10, 60)
    .setRange(0, 1)
    ;
  delayFactorSlider = cp5.addSlider("delayFactor")
    .setPosition(10, 70)
    .setRange(0, 1)
    ;
  hiThreshSlider = cp5.addSlider("hiThresh")
    .setPosition(10, 80)
    .setRange(0, 1)
    ;
  loThreshSlider = cp5.addSlider("loThresh")
    .setPosition(10, 90)
    .setRange(0, 1)
    ;
  cp5.addSlider("rmsForceAmt")
    .setPosition(10, 100)
    .setRange(0, 1)
    ;
  cp5.addSlider("gateHiThresh")
    .setPosition(10, 110)
    .setRange(0, 1)
    ;
  cp5.addSlider("gateLoThresh")
    .setPosition(10, 120)
    .setRange(0, 1)
    ;
  attackSlider = cp5.addSlider("attack")
    .setPosition(10, 130)
    .setRange(0, 1000)
    ;
  decaySlider = cp5.addSlider("decay")
    .setPosition(10, 140)
    .setRange(0, 1000)
    ;
  levelerMixSlider = cp5.addSlider("levelerMix")
    .setPosition(10, 150)
    .setRange(0, 1)
    ;
  maxConnexSlider = cp5.addSlider("maxConnex")
    .setPosition(10, 170)
    .setRange(0, maxNumParticles - 1)
    ;
  hpfSlider = cp5.addSlider("hpfFreq")
    .setPosition(10, 180)
    .setRange(20, 400)
    ;
  pitchRangeSlider = cp5.addSlider("pitchRange")
    .setPosition(10, 190)
    .setRange(0, 100)
    ;
  separationSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pSeparation = separationSlider.getValue();
        }
      }
    }
  }
  );
  attractRadiusSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pAttractRadius = attractRadiusSlider.getValue();
        }
      }
    }
  }
  );
  attackSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pAttack = attackSlider.getValue();
        }
      }
    }
  }
  );
  decaySlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pDecay = decaySlider.getValue();
        }
      }
    }
  }
  );
  delayFactorSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pDelayFactor = delayFactorSlider.getValue();
        }
      }
    }
  }
  );
  repulsionFactorSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pRepulsionFactor = repulsionFactorSlider.getValue();
        }
      }
    }
  }
  );
  attractFactorSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pAttractFactor = attractFactorSlider.getValue();
        }
      }
    }
  }
  );
  hiThreshSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pHiThresh = hiThreshSlider.getValue();
        }
      }
    }
  }
  );
  loThreshSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pLoThresh = loThreshSlider.getValue();
        }
      }
    }
  }
  );
  levelerMixSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pLevelerMix = levelerMixSlider.getValue();
        }
      }
    }
  }
  );
  maxConnexSlider.addCallback(new CallbackListener() {
    public void controlEvent(CallbackEvent theEvent) {
      if (theEvent.getAction()==ControlP5.ACTION_BROADCAST) {
        for (Particle p : ps.particles) {
          p.pMaxConnex = (int)maxConnexSlider.getValue();
        }
      }
    }
  }
  );
}  // end setup



void draw() {
  if (setupMode) {
    background(0);
    flowTracking();
    noFill();
    stroke(255);
    rect(0, viewYOffset, width - 1, height-viewYOffset - 1);
    if (debug) {
      if (kinectConnected) {
        if (kinectOne) {
          image(kinect2a.getDepthImage(), 0, imgHeight, imgWidth, imgHeight); 
          image(kinect2b.getDepthImage(), imgWidth, imgHeight, imgWidth, imgHeight);
        } else {
          image(kinect1a.getDepthImage(), 0, imgHeight, imgWidth, imgHeight); 
          image(kinect1b.getDepthImage(), imgWidth, imgHeight, imgWidth, imgHeight);
        }
      }
      stroke(255);
      noFill();
      rect(0, imgHeight, imgWidth, imgHeight);
      rect(imgWidth, imgHeight, imgWidth, imgHeight);
    }
  } 

  // not setup mode
  else {  
    background(0);
    if (mousePressed) {  // mouse input for testing
      if (!test) {
        testY = mouseY;
        if (kinectOne) {
          testX = width - mouseX;
        } else {
          testX = mouseX;
        }
        noStroke();
        fill(255);
        imgMouse.beginDraw();
        imgMouse.clear();
        imgMouse.rectMode(CENTER);
        imgMouse.rect(testX * imgWidth / width, testY * imgHeight / height, 20, 60);
        imgMouse.endDraw();
      } else { // test
        if (kinectOne) {
          testX = width - mouseX;
        } else {
          testX = mouseX;
        }
        testPoints.add(new PVector(testX, mouseY));
      }
    }

    if (test) {
      noStroke();
      fill(255);
      imgMouse.beginDraw();
      imgMouse.clear();
      imgMouse.rectMode(CENTER);
      for (PVector pv : testPoints) {
        imgMouse.rect(pv.x * imgWidth / width, pv.y * imgHeight / height, 10, 10);
      }
      imgMouse.endDraw();
    }


    flowTracking();

    ps.update();
    ps.intersection();
    ps.drag();
    ps.repellFromSides();
    ps.display();

    fill(255);
      flowField.display();
    if (debug) {
      textSize(12);
      fill(255);
      text("frameRate: "+frameRate, 0, height - 10);
      text("v shaped at mouse: "+String.format("%.3f", flowField.vMagsShaped[mouseX*flowField.cols/width][mouseY*flowField.rows/height]), 0, height - 100);
      text("v at mouse: "+String.format("%.3f", flowField.vMags[mouseX*flowField.cols/width][mouseY*flowField.rows/height]), 0, height - 85);
      text("vmax at mouse: "+String.format("%.3f", flowField.magMax[mouseX*flowField.cols/width][mouseY*flowField.rows/height]), 0, height - 70);
    } else {
      if (moved) {
        moved = false;
        noCursor();
      }
    }

    fill(0);
    noStroke();
    rect(0, 0, width, viewYOffset);  // mask upper rect

    if (record) {
      saveFrame("particles-######.tif");
      record = false;
    }
    //that = get();
    //that = get(0, viewYOffset, width, height-viewYOffset);
    //for(int x = 0; x < that.width; x++) {
    //for(int y = 0; y < that.height; y++) {
    //canvas.pixels[((that.height-y-1)*that.width+x)] = that.pixels[y*that.width+x];
    //}
    //}
    //canvas.updatePixels();

    //server.sendImage(canvas); // now send the image across syphon to the mapping software
  } // end not setup mode
}  // end draw

void mouseMoved() {
  if (!debug) {
    cursor();
    moved = true;
  }
}

void mousePressed() {
  //ps.addParticle(mouseX, mouseY);
}

void mouseReleased() {
  imgMouse.beginDraw();
  imgMouse.clear();
  imgMouse.endDraw();
}


void keyPressed() {
  if (key == 'd') {  // debug on/off
    debug = !debug;
    if (debug) {
      cp5.setVisible(true);
      cursor();
    } else {
      cp5.setVisible(false);
      noCursor();
    }
  } else if (key == 'r') {
    record = !record;
  } else if (key == 's') {
    setupMode = !setupMode;
  } else if (key == 'f') {
    freeze = !freeze;
  } else if (key == 't') {
    test = !test;
    if (!test) {
      testPoints.clear();
    }
  } else if (key == ' ') {
    ps.removeAllParticles();
  }
}



void flowTracking() {

  if (kinectConnected) {
    // left camera, kinect 2a: Get the raw depth as array of integers
    int[] depth = new int[cam_w * cam_h];

    int imgOffset = 0;
    PVector point;

    img1 = createImage(imgWidth, imgHeight, RGB);
    img1.loadPixels();

    if (kinectOne) {
      depth = kinect2a.getRawDepth();
      if (camPositionVertical) {
        for (int x = 0; x < kinect2a.depthWidth; x+=skip) {
          for (int y = 0; y < kinect2a.depthHeight * 0.7; y+=skip) {
            int offset = x + y * kinect2a.depthWidth;

            if (depth[offset] > minDepth && depth[offset] < maxDepth) {
              //calculate the x, y, z camera position based on the depth information
              point = depthToPointCloudPos2Dvert(x, y, depth[offset], 0);

              // Draw a point if within limits 
              if (point.x >= minXDim && point.x <= maxXDim && point.z >= minZ && point.z <= maxZ ) {
                // map filtered point's x value to img width
                point.x = map(point.x, minXDim, maxXDim, 0, imgWidth);

                imgOffset = (int)point.y * imgWidth + (int)point.x;
                img1.pixels[imgOffset] = color(255);
              }
            }
          }
        }
      } else { // horizontal

        for (int x = 0; x < kinect2a.depthWidth; x+=skip) {
          for (int y = 0; y < kinect2a.depthHeight; y+=skip) {
            int offset = x + y * kinect2a.depthWidth;

            if (depth[offset] > minDepth && depth[offset] < maxDepth) {
              //calculate the x, y, z camera position based on the depth information
              point = depthToPointCloudPos2Dhor(x, y, depth[offset], 0);

              // Draw a point if within limits 
              if (point.x >= minXDim && point.x <= maxXDim && point.y >= minYDim && point.y <= maxYDim ) {
                // map filtered point's x value to img width
                point.x = map(point.x, minXDim, maxXDim, 0, imgWidth);
                point.y = map(point.y, minYDim, maxYDim, 0, imgHeight);

                imgOffset = (int)point.y * imgWidth + (int)point.x;
                img1.pixels[imgOffset] = color(255);
              }
            }
          }
        }
      }
    } else {  // kinect 360
      depth = kinect1a.getRawDepth();
      if (camPositionVertical) {
        // tbd
      } else {  // horizontal cams
        for (int x = 0; x < kinect1a.width; x+=skip) {
          for (int y = 0; y < kinect1a.height; y+=skip) {
            int offset = x + y * kinect1a.width;

            if (depth[offset] > minDepth && depth[offset] < maxDepth) {
              //calculate the x, y, z camera position based on the depth information
              point = depthToWorld(x, y, depth[offset]);

              // Draw a point if within limits 
              if (point.x >= minXDim && point.x <= maxXDim && point.y >= minYDim && point.y <= maxYDim ) {
                // map filtered point's x value to img width
                point.x = map(point.x, minXDim, maxXDim, 0, imgWidth);
                point.y = map(point.y, minYDim, maxYDim, 0, imgHeight);

                imgOffset = (int)point.y * imgWidth + (int)point.x;
                img1.pixels[imgOffset] = color(255);
              }
            }
          }
        }
      }
    }

    img1.updatePixels();


    // right camera
    imgOffset = 0;
    img2 = createImage(imgWidth, imgHeight, RGB);
    img2.loadPixels();

    if (kinectOne) {
      depth = kinect2b.getRawDepth();    // Get the raw depth as array of integers
      if (camPositionVertical) {
        for (int x = 0; x < kinect2a.depthWidth; x+=skip) {
          for (int y = 0; y < kinect2a.depthHeight * 0.7; y+=skip) {
            int offset = x + y * kinect2a.depthWidth;

            if (depth[offset] > minDepth + depthOffset && depth[offset] < maxDepth + depthOffset) {
              //calculate the x, y, z camera position based on the depth information
              point = depthToPointCloudPos2Dvert(x, y, depth[offset], depthOffset);

              // Draw a point
              if (point.x >= minX * xDimHalf * 2 && point.x <= maxX * xDimHalf * 2 && point.z >= minZ && point.z <= maxZ ) {
                // map filtered point's x value to img width
                point.x = map(point.x, minX * xDimHalf * 2, 2 * xDimHalf * maxX, 0, imgWidth);

                imgOffset = (int)point.y * imgWidth + (int)point.x;
                img2.pixels[imgOffset] = color(255);
              }
            }
          }
        }
      } else { // horizontal

        for (int x = 0; x < kinect2b.depthWidth; x+=skip) {
          for (int y = 0; y < kinect2b.depthHeight; y+=skip) {
            int offset = x + y * kinect2b.depthWidth;

            if (depth[offset] > minDepth && depth[offset] < maxDepth) {
              //calculate the x, y, z camera position based on the depth information
              point = depthToPointCloudPos2Dhor(x, y, depth[offset], 0);

              // Draw a point if within limits 
              if (point.x >= minXDim && point.x <= maxXDim && point.y >= minYDim && point.y <= maxYDim ) {
                // map filtered point's x value to img width
                point.x = map(point.x, minXDim, maxXDim, 0, imgWidth);
                point.y = map(point.y, minYDim, maxYDim, 0, imgHeight);

                imgOffset = (int)point.y * imgWidth + (int)point.x;
                img2.pixels[imgOffset] = color(255);
              }
            }
          }
        }
      }
    } 
    // kinect 360
    else { 
      depth = kinect1b.getRawDepth();
      if (camPositionVertical) {
        // tbd
      } else {  // vertical cams
        for (int x = 0; x < kinect1b.width; x+=skip) {
          for (int y = 0; y < kinect1b.height; y+=skip) {
            int offset = x + y * kinect1b.width;

            if (depth[offset] > minDepth && depth[offset] < maxDepth) {
              //calculate the x, y, z camera position based on the depth information
              point = depthToWorld(x, y, depth[offset]);

              // Draw a point if within limits 
              if (point.x >= minX * xDimHalf * 2 && point.x <= maxX * xDimHalf * 2 && point.y >= minY * yDimHalf * 2 && point.y <= maxY * yDimHalf * 2 ) {
                // map filtered point's x value to img width
                point.x = map(point.x, minX * xDimHalf * 2, 2 * xDimHalf * maxX, 0, imgWidth);
                point.y = map(point.y, minY * yDimHalf * 2, 2 * yDimHalf * maxY, 0, imgHeight);

                imgOffset = (int)point.y * imgWidth + (int)point.x;
                img2.pixels[imgOffset] = color(255);
              }
            }
          }
        }
      }
    }
    img2.updatePixels();
  }  // end kinect connected


  // combine img1 and img2 to img3
  img3 = createImage(img1Width + img2Width, imgHeight, RGB);
  if (kinectConnected) {
    img3.copy(img1, (int)(img1LeftMargin * imgWidth), 0, img1Width, imgHeight, 0, 0, img1Width, imgHeight);
    img3.copy(img2, (int)(img2LeftMargin * imgWidth), 0, img2Width, imgHeight, img1Width, 0, img2Width, imgHeight);
    //PImage mask = createImage(300, 300, RGB);
    PGraphics mask = createGraphics(80, 60);
    mask.beginDraw();
    mask.fill(0);
    mask.rect(0, 0, mask.width, mask.height);
    mask.endDraw();
    img3.copy(mask, 0, 0, mask.width, mask.height, img3.width - mask.width, 0, mask.width, mask.height);
    if (kinectOne) {  // mirror image
    }
  }

  if (mousePressed || test) {
    img3.blend(imgMouse, 0, 0, imgWidth, imgHeight, 0, 0, img1Width + img2Width, imgHeight, ADD);
  }


  if (setupMode) {
    noFill();
    stroke(255);
    if (kinectConnected) {
      image(img1, 0, 0);
      image(img2, imgWidth, 0);
    }
    image(img3, imgWidth / 2, imgHeight);
    rect(0, 0, imgWidth, imgHeight);  // img 1
    rect(imgWidth, 0, imgWidth, imgHeight);  // img 2
    rect(imgWidth / 2, imgHeight, img3.width, imgHeight);  // img3 frame
    stroke(0, 0, 255);
    line(img1LeftMargin * imgWidth, 0, img1LeftMargin * imgWidth, imgHeight);
    line(img1LeftMargin * imgWidth + img1Width, 0, img1LeftMargin * imgWidth + img1Width, imgHeight);
    line(img2LeftMargin * imgWidth + imgWidth, 0, img2LeftMargin * imgWidth + imgWidth, imgHeight);
    line(img2LeftMargin * imgWidth + img2Width + imgWidth, 0, img2LeftMargin * imgWidth + img2Width + imgWidth, imgHeight);  // img2 right margin
    stroke(0, 255, 0);
    line(img1Width + (imgWidth / 2.0), imgHeight, img1Width + (imgWidth / 2.0), imgHeight * 2);  // img3 divsion
  }


  // render to offscreenbuffer
  img3.resize(pgImgWidth, pgImgHeight);
  mirrorVideoImage(img3);
  pg_cam.beginDraw();
  pg_cam.clear();
  pg_cam.image(img3, 0, 0);
  // insert black square because of disturbance
  //stroke(255);
  //fill(250);
  //pg_cam.rect(pg_cam.width - 200, 0, 200, 200);

  pg_cam.endDraw();

  // update Optical Flow
  opticalflow.update(pg_cam); 

  // flow visualizations
  opticalflow.param.display_mode = 0;
  opticalflow.param.blur_input = 5;
  opticalflow.param.temporal_smoothing = 0.5;  // .93

  // Transfer velocity data from the GPU to the host-application
  // This is in general a bad idea because such operations are very slow. So 
  // either do everything in shaders, and avoid memory transfer when possible, 
  // or do it very rarely. however, this is just an example for convenience.
  if (frameCount % frameSkip == 0) {
    flow_velocity = opticalflow.getVelocity(flow_velocity);
    flowField.update(flow_velocity);    // update flowField with current flow velos

    // create particle if velocity greater createVelo
    //if (ps.particles.size() < maxNumParticles && !setupMode) {  
    //  for (int i = 0; i < flowField.createLocations.size(); i++) {
    //    float minDist = 10000;
    //    for (Particle p : ps.particles) {
    //      float dist = PVector.dist(p.location, flowField.createLocations.get(i));
    //      if (dist < minDist) {
    //        minDist = dist;
    //      }
    //    }
    //    if (minDist > minCreateDist && ps.particles.size() < maxNumParticles) {
    //      ps.addParticle(flowField.createLocations.get(i).x, flowField.createLocations.get(i).y, flowField.createVelos.get(i));
    //    }
    //  }
    //}

    for (Particle p : ps.particles) {  
      if (p.bgBrightness > 0.5) p.impulse();
    }
  } // end update flowField

  if (!setupMode) {

    // "analyse image" (img3) convoled to imgBlur then gradients to flowField vectors. same size img3.width == imgBlur.width == flowField.cols
    img3.loadPixels();
    imgBlur.loadPixels();

    for (int x = 1; x < flowField.cols -  1; x++) {
      for (int y = 1; y < flowField.rows - 1; y++) {
        PVector sum = new PVector(0, 0); // Kernel sum for this pixel
        int testPos = y * flowField.cols + x;
        //float ref = red(imgBlur.pixels[testPos]);
        int colSum = 0;

        // get shaped actual velocity vector
        float v = flowField.vMagsShaped[x][y];

        //if (red(img3.pixels[testPos]) > 10) imgBlur.pixels[testPos] = color(255 *(1 - sqrt(v)));
        if (red(img3.pixels[testPos]) > 10 && v <= 0.25) {  // insert white pixel, if no movement
          imgBlur.pixels[testPos] = color(255 *(1 - sqrt(v)));
        } else if (red(img3.pixels[testPos]) == 0 && v > 0.25) {
          //imgBlur.pixels[testPos] = color(0);
          //flowField.field[x][y].mult(0.0001);
        }
        float ref = red(imgBlur.pixels[testPos]);

        // convolve the imgBlur pixel and calculate the sum vector from dark to bright
        for (int kx = -1; kx <= 1; kx++) {
          for (int ky = -1; ky <= 1; ky++) {
            //if (kx != 0 || ky != 0) {
            // Calculate the adjacent pixel for this kernel point
            int pos = (y + ky) * flowField.cols  + (x + kx);
            // Image is grayscale, red/green/blue are identical
            float val = red(imgBlur.pixels[pos]) - ref;
            // Multiply adjacent pixels based on the kernel values
            sum = sum.add(PVector.mult(flowField.kernel[kx+1][ky+1], val));
            colSum += red(imgBlur.pixels[pos]);
            //}
          }
        }
        flowField.field[x][y].add(sum.mult(0.0001)); 
        colSum = (int)(colSum / 9.0);
        float cMix = 1.0;  // 0 == original color
        colSum = (int)(colSum*cMix + ref*(1-cMix));
        imgBlur.pixels[testPos] = color(colSum);
      }
    }  // end loop through flowField

    img3.updatePixels();
    imgBlur.updatePixels();

    //tint(255, 100 - (flowField.pMagMaxStore * 50));
    //img3.resize(80, 50);
    //image(img3, 0, 0, width, height);
    tint(255, 150);
    image(imgBlur, 0, 0, width, height);
    tint(255);
  } // end not setupMode



  // add flowField vectors to particles
  for (Particle particle : ps.particles) {
    // look up blurred image brightness at particle location
    int iX = (int)particle.location.x*imgBlur.width/width;
    int iY = (int)particle.location.y*imgBlur.height/height;
    int pos = iY * imgBlur.width  + iX;
    pos = constrain(pos, 0, imgBlur.pixels.length-1);
    float bright = red(imgBlur.pixels[pos]) / 255.0;
    //if(bright > 1.0 || bright < 0.0) println("bright: " + bright);
    particle.bgBrightness = bright;
    //println("pos: " + pos + " " + iX + " " + iY);

    // flowfield velo vectors are used for particle acceleration
    if (bright < minBgBrightness) { // not in white of blurred image
      //particle.applyForce(flowField.lookup(particle.location).mult(0.1)); // 0.01
      ///*
      // change to steering behavior
      PVector p = flowField.lookup(particle.location);
      //p.normalize();
      //p.mult(maxspeed);
      if (p.mag() > 1.0 && p.mag() < 5.0) { // steering behavior
        PVector steer = PVector.sub(p, particle.velocity);
        //Limit the magnitude of the steering force.
        steer.limit(0.2);
        particle.applyForce(steer);
      } else { // apply flow field
        particle.applyForce(p.mult(0.1)); // 0.01
      }
      //*/
    } else {
      particle.applyForce(flowField.lookup(particle.location).mult(0.1 *(1-particle.bgBrightness) * (1-particle.dragFactor)));
    }
  }
}   // end flow tracking


// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - CameraParams360.cx) * depth * CameraParams360.fx + xDimHalf);
  result.y = (float)((y - CameraParams360.cy) * depth * CameraParams360.fy + yDimHalf);
  result.z = (float)(depth);
  return result;
}

// convert depth cam points to real points with depthValue for y-position
PVector depthToPointCloudPos2Dvert(int x, int y, int depthValue, int depthOffset) {
  PVector point = new PVector();
  point.y = map(depthValue, minDepth + depthOffset, maxDepth + depthOffset, 0, imgHeight);          // convert depth to y value

  point.x = (x - CameraParams.cx) * depthValue / CameraParams.fx + xDimHalf;        // calc x pos with camera specific values + add max offset for 0 - n range
  //point.x = map(point.x + xDimHalf, 0, 2 * xDimHalf, 0, imgWidth);

  point.z = (y - CameraParams.cy) * depthValue / CameraParams.fy;
  point.z = map(point.z + zDimHalf, 0, 2 * zDimHalf, 0, 100);            // map to 0 - 100 range, used to exclude top and bottom area
  return point;
}

// convert depth cam points to real points with depthValue for y-position
PVector depthToPointCloudPos2Dhor(int x, int y, int depthValue, int depthOffset) {
  PVector point = new PVector();
  point.z = map(depthValue, minDepth + depthOffset, maxDepth + depthOffset, 0, imgHeight);          // convert depth to y value

  point.x = (x - CameraParams.cx) * depthValue / CameraParams.fx + xDimHalf;        // calc x pos with camera specific values + add max offset for 0 - n range
  //point.x = map(point.x + xDimHalf, 0, 2 * xDimHalf, 0, imgWidth);

  point.y = (y - CameraParams.cy) * depthValue / CameraParams.fy + yDimHalf;
  //point.y = map(point.y + yDimHalf, 0, 2 * yDimHalf, 0, 100);            // map to 0 - 100 range, used to exclude top and bottom area
  return point;
}

void mirrorVideoImage(PImage video) {  // for kinect one
  PImage videoBuf = video.copy();
  videoBuf.loadPixels();
  video.loadPixels();
  for (int y = 0; y < video.height; y++) {
    for (int x = 0; x < video.width; x++) {
      video.pixels[y*video.width+x] = videoBuf.pixels[(video.width - x - 1) + y*video.width]; // Reversing x to mirror the
    }
  }
  video.updatePixels();
}
