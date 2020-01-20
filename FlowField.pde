// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flow Field Following

class FlowField {

  // A flow field is a two dimensional array of PVectors
  PVector[][] field;  // added velo vectors
  float[][] vMags;      //  actual velocities from pixelflow library passed in with flow_velo[]
  float[][] vMagsShaped;  // shaped
  float[][] magMax;
  boolean[][] magMaxReset;
  float vDecay = 0.93;// 0.2
  PVector[][] bgField;  // background velo vectors
  PVector[][] kernel;  
  color[][] blur; // not used
  int cols, rows; // Columns and Rows
  int resolution; // How large is each "cell" of the flow field
  int kernelSize;
  float vMag;
  float createVelo = 3.0;  // create particle if velo greater this value and nbr of particles lower max
  float minVelo = 1.0;
  int rowOffset;

  FlowField(int r, int viewYOffset) {
    resolution = r;
    kernelSize = 3;
    // Determine the number of columns and rows based on sketch's width and height
    cols = (width - 1) /resolution + 1;
    rows = (height - 1) /resolution + 1;
    rowOffset = viewYOffset/r;
    field = new PVector[cols][rows];
    vMags = new float[cols][rows];
    vMagsShaped = new float[cols][rows];
    magMax = new float[cols][rows];
    magMaxReset = new boolean[cols][rows];
    bgField = new PVector[cols][rows];
    kernel = new PVector[kernelSize][kernelSize];
    blur = new color[cols][rows];
    init();
    println("flowField: ", cols, rows);
  }

  void init() {
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        field[i][j] = new PVector(0, 0);
        //field[i][j] = new PVector(width/2-i*resolution, height/2-j*resolution);
        field[i][j].normalize();
        if (i == 0 || i == cols - 1 || j == 0 || j == rows - 1) {   
          bgField[i][j] = new PVector(width/2-i*resolution, height/2-j*resolution);
          bgField[i][j].mult(0.00001);
        } else {
          bgField[i][j] = new PVector(0, 0);
        }
      }
    }

    for (int i = 0; i < kernelSize; i++) {
      for (int j = 0; j <  kernelSize; j++) {
        //kernel[i][j] = new PVector(1 - i, 1 - j);
        kernel[i][j] = new PVector(1 - i, 1 - j).mult(-1);
        kernel[i][j].normalize();
      }
    }
  }



  void update(float[] flow_velo) {  // flow_velo has 2x size of flowField for separate x and y values
    // update flowField with pixelflow vectors
    for ( int x = 0; x < flowField.cols; x++) {
      for ( int y = rowOffset; y < flowField.rows; y++) {

        int PIDX    = min((flow_velo.length - 1) / 2, (pgImgHeight - 1 - y) * pgImgWidth + x);
        PVector v = new PVector(flow_velo[PIDX * 2], -flow_velo[PIDX * 2 + 1]);  // flow velocity vector
        vMag = v.mag();  // magnitude of current velo works better
        vMags[x][y] = vMag;

        // determine magMax  for decayrate
        if ( vMag >  magMax[x][y]) {
          magMax[x][y] = vMag;  // new max magnitude
        } else if (vMag == 0) {
          magMaxReset[x][y] = true;  // new max magnitude
        } else if (vMag > 0 && magMaxReset[x][y]) {
          magMaxReset[x][y] = false;
          magMax[x][y] = vMag;
        }
        //vDecay = 0.5 + min(0.49, magMax[x][y]/25);

        // shape current velocity
        if (vMag >= vMagsShaped[x][y] && vMag >= 0.1) {
          vMagsShaped[x][y] = min(10, vMag);  // limit and store current velo
        } else {
          vMagsShaped[x][y] = max(0, vMagsShaped[x][y] * vDecay);
          ; // exp decay it
          //vMagsShaped[x][y] = max(0, vMagsShaped[x][y] - vDecay);; // linear decay it
        }

        if (vMag >= minVelo) {
          //flowField.field[x][y] = p.mult(0.2);
          flowField.field[x][y].add(v.mult(0.2));  // 0.2


          //flowField.field[fx][fy].add(PVector.sub(p, flowField.pVelo[fx][fy]));    // test , add velo difference
          //spreadVelo(x, y, p);
          flowField.field[x][y].limit(4);    // limit to max velo
        }
        flowField.field[x][y].mult(0.995);  // velo is decaying
          flowField.field[x][y].limit(4);    // limit to max velo
        //flowField.field[x][y].mult(0.999);  // velo is decaying

        //float vMag = flowField.field[fx][fy].mag();
      }
    }
    // end update flowField



    // update background force
    //for (int i = 0; i < cols; i++) {
    //  for (int j = 0; j < rows; j++) {
    //    field[i][j].add(bgField[i][j]);  // background force
    //    field[i][j].limit(3);
    //  }
    //}

    /*
    // convolve the flowfield
     for (int x = 1; x < flowField.cols -  1; x++) {
     for (int y = 1; y < flowField.rows - 1; y++) {
     PVector sum = new PVector(0, 0); // Kernel sum for this pixel
     
     for (int kx = -1; kx <= 1; kx++) {
     for (int ky = -1; ky <= 1; ky++) {
     if (kx != 0 && ky != 0) {
     sum = sum.add(PVector.mult(flowField.field[x+kx][y+ky], 0.002));
     }
     }
     }
     flowField.field[x][y].add(sum);
     }
     }
     */

    /*
    // spread current flow vectors to adjectant vectors according to the angle
     for (int x = 1; x < flowField.cols -  1; x++) {
     for (int y = 1; y < flowField.rows - 1; y++) {
     //PVector sum = new PVector(0, 0); // Kernel sum for this pixel
     
     for (int kx = -1; kx <= 1; kx++) {
     for (int ky = -1; ky <= 1; ky++) {
     if (kx != 0 && ky != 0) {
     float angle = PVector.angleBetween(flowField.kernel[kx+1][ky+1], flowField.field[x][y]);
     //float angle = PVector.angleBetween(flowField.field[x][y], flowField.field[x+kx][y+ky]);
     //float angle = PVector.angleBetween(flowField.field[x+kx][y+ky], flowField.field[x][y]);
     angle = abs(angle - PI) / PI;
     if (angle > 0.5 ) {
     PVector pv = PVector.mult(flowField.field[x][y], angle * angle * 0.01);
     //PVector pv = PVector.mult(flowField.field[x][y], angle * 0.2);
     //PVector pv = PVector.mult(flowField.field[x][y], pow(angle, 2) * 0.1);
     flowField.field[x+kx][y+ky].add(pv);
     flowField.field[x+kx][y+ky].limit(3);
     //flowField.field[x+kx][y+ky].add(PVector.mult(flowField.field[x][y], angle));
     }
     }
     }
     }
     }
     }
     */
  } // end update

  // Draw every vector
  void display() {
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        //drawVector(currentVelos[i][j], i*resolution, j*resolution, resolution*0.2);
        drawVector(field[i][j], i*resolution, j*resolution, resolution*0.2);
      }
    }
    /*
    for (int i = -1; i < kernelSize-1; i++) {
     for (int j = -1; j <  kernelSize-1; j++) {
     pushMatrix();
     translate(mouseX/resolution*resolution + i*20 + 10, mouseY/resolution*resolution + j*20 + 10);
     //line(0, 0, kernel[i+1][j+1].x * 10, kernel[i+1][j+1].y * 10);
     float angle = PVector.angleBetween(kernel[i+1][j+1], lookup(new PVector(mouseX, mouseY)));
     angle = abs(angle - PI)/PI;
     stroke(255);
     textSize(10);
     text(String.format("%.1f", angle), 0, 0);
     //kernel[i][j].normalize();
     popMatrix();
     }
     }
     */
  }

  void spreadVelo(int x, int y, PVector pv) {
    if (x>0 && y>0 && x<flowField.cols-1 && y<flowField.rows-1) {
      for (int kx = -1; kx <= 1; kx++) {
        for (int ky = -1; ky <= 1; ky++) {
          if (kx != 0 && ky != 0) {
            float angle = PVector.angleBetween(flowField.kernel[kx+1][ky+1], pv);
            angle = abs(angle - PI) / PI;
            if (angle > 0.5 ) {
              pv.mult(angle * angle * 10);
              flowField.field[x+kx][y+ky].add(pv);
            }
          }
        }
      }
    }
  }

  // Renders a vector object 'v' as an arrow and a position 'x,y'
  void drawVector(PVector v, float x, float y, float scayl) {
    pushMatrix();
    //float arrowsize = 4;
    // Translate to position to render vector
    translate(x + resolution * 0.5, y + resolution * 0.5);
    strokeWeight(1);
    // Call vector heading function to get direction (note that pointing up is a heading of 0) and rotate
    rotate(v.heading());
    // Calculate length of vector & scale it to be bigger or smaller if necessary
    float len = max(1, v.mag()*scayl);
    // Draw three lines to make an arrow (draw pointing up since we've rotate to the proper direction)
    //stroke(v.x * 100, v.y * 100, 255);
    stroke(len * 15 + 166, 100);
    line(0, 0, len, 0);

    //line(len,0,len-arrowsize,+arrowsize/2);
    //line(len,0,len-arrowsize,-arrowsize/2);
    popMatrix();
  }

  PVector lookup(PVector lookup) {
    int column = int(constrain(lookup.x/resolution, 0, cols-1));
    int row = int(constrain(lookup.y/resolution, 0, rows-1));
    return field[column][row].copy();
  }

  float lookupMag(PVector location) {
    return field[columnForX(location.x)][rowForY(location.y)].mag();
  }

  int columnForX(float x) {
    int column = int(constrain(x/resolution, 0, cols-1));
    return column;
  }

  int rowForY(float y) {
    int row = int(constrain(y/resolution, 0, rows-1));
    return row;
  }

  float getVelocityMagAtLocation(PVector location) {
    return vMags[columnForX(location.x)][rowForY(location.y)];
  }

  float getShapedVelocityMagAtLocation(PVector location) {
    return vMagsShaped[columnForX(location.x)][rowForY(location.y)];
  }
}
