// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Simple Particle System

class Particle {
  int id;
  PVector location;
  PVector velocity;
  PVector acceleration;
  float lifespan;
  float bgBrightness;  // brightness of imgBlur at particle location 0-1
  float age;
  float mass = 1;
  float diameter = 2*particleRadius;
  float vDiam;
  float range = 1;
  boolean attraction = true;
  float pSeparation;
  float pAttractRadius;
  float pAttack;
  float pDecay;
  float pDelayFactor;
  float pAttractFactor;
  float pRepulsionFactor;
  float pHiThresh;
  float pLoThresh;
  float pLevelerMix;
  float pShaperMix;
  float selfDelTime;
  float masterGain;

  Gain signalInput;
  WavePlayer wp;
  Gain noiseGain;
  Function leveler;
  Function returnRms;
  Function gate;
  Envelope outGainEnv;
  Envelope env;
  TapIn delayIn;
  BiquadFilter hpf;
  RMS rms;
  Panner pan;
  Panner panY;
  Panner panXLow;
  Panner panXHigh;

  PeakDetector peak;
  Glide hpfGlide;
  SignalReporter sigRep;
  WaveShaper shaper;
  Compressor comp;
  TapOut selfDel;
  Glide selfDelGlide;
  Glide levelerGlide;

  float rmsVal;
  float rmsOutVal;
  float[] rmsValHist;
  int rmsValHistWriteIndx;
  int rmsValHistReadIndx;
  int rmsValNbr = 50;
  float rmsValPrev;
  int rmsValCount;
  float levelFact;
  float pHpfFreq;
  float threshold;
  int gateValue;
  boolean separate;  // do separation if true
  float dragFactor;  // brightness dependent

  //float randomFactor;
  int hiCrossings;
  int hiCrossingsPrev;  // previous value for comparison to calc framessince last
  int loCrossings;
  int framesSinceLastHiCross;
  float framesSinceLastHiCrossLimit;
  float overMax = 0;  // cumulated rms value overshoot of 0.95
  int pMaxConnex;
  int inConnex;
  int outConnex;
  float textureOffset;  // for animating the edge texture
  ArrayList <Edge> inputEdges;
  ArrayList <Edge> outputEdges;


  Particle(float x, float y, int id_, PVector velocity_) {
    id = id_;
    pSeparation = separation;
    pAttractRadius = attractRadius;
    pAttack = attack;
    pDecay = decay;
    pDelayFactor = delayFactor;
    pAttractFactor = attractFactor;
    pRepulsionFactor = repulsionFactor;
    pHiThresh = hiThresh;
    pLoThresh = loThresh;
    pLevelerMix = levelerMix;
    rmsValHist = new float[rmsValNbr];
    pMaxConnex = maxConnex;
    acceleration = new PVector();
    velocity = velocity_;
    location = new PVector(x, y);
    lifespan = 1.0;
    framesSinceLastHiCrossLimit = 500;

    //OscMessage myMessage = new OscMessage("/node");
    //myMessage.add(this.id);      /* add ephemer nbr */
    //oscP5.send(myMessage, myRemoteLocation);      /* send the message */
    //println("ended" + id);
    //randomFactor = random(1.0);

    // Beads
    levelFact = 1.17;
    //attack = 40;
    //decay = 1000;
    //calc_pHpfFreq();  // set hpf from location.y
    wp = new WavePlayer(ac, 10, Buffer.NOISE);
    env = new Envelope(ac, 0);
    noiseGain = new Gain(ac, 1, env); // create a Gain object for the node impulse
    noiseGain.addInput(wp);
    signalInput = new Gain(ac, 1, 1);  // signal input
    pHpfFreq = hpfFreq;

    // meassure input signal, dynamically damp input signal
    rms = new RMS(ac, 1, 441);
    rms.addInput(signalInput);

    levelerGlide = new Glide(ac, pLevelerMix, 20);

    leveler = new Function(rms, levelerGlide)    // von philippe
    {
      public float calculate() {
        //if(Float.isNaN(x[0])) {x[0] = 0;}
        //return pow(2, -2.0 * x[0] + 0.1) * lifespan ;  // philippe

        //return pow(2, -2.0 * x[0] + 0.1) ;  // philippe
        //return pow(2, x[0] * -2.0 + 0.1) * pLevelerMix + (1 - pLevelerMix);
        return pow(2, x[0] * -2.0 + 0.1) * x[1] + (1 - x[1]);

        //return lifespan * pow(2, x[0] * -2.0 + 0.1) * pLevelerMix + (1 - pLevelerMix);
        //return (float)Math.atan(x[0]) * 0.63662;  // neukom
      }
    };

    Mult mult = new Mult(ac, signalInput, leveler);
    Clip clip = new Clip(ac);
    clip.setRange(-2, 2);  // safety clip. wegen NaN ?
    clip.addInput(mult);

    // decide attraction / repulsion according to rms
    returnRms = new Function(rms) 
    {
      public float calculate() {
        rmsVal = min(1.0, x[0]);    // limit rms to 0-1
        if (attraction && rmsVal > pHiThresh) {
          attraction = false; // repulsion
          hiCrossings++;
        } else if (attraction == false && rmsVal < pLoThresh) {
          attraction = true;  // attract
          loCrossings++;
        }
        //rmsValHistWriteIndx = (rmsValHistWriteIndx + 1) % rmsValNbr;
        //rmsValHist[rmsValHistWriteIndx] = rmsVal;    // write rms history
        return x[0];  // pro forma, not used
      }
    };
    ac.out.addDependent(returnRms); // let it calculate without using the function explicitly

    // hpf filter
    hpfGlide = new Glide(ac, pHpfFreq, 20);
    hpf = new BiquadFilter(ac, BiquadFilter.HP, hpfGlide, 1);
    //hpf.addInput(mult);
    hpf.addInput(clip);

    // gate for level control
    outGainEnv = new Envelope(ac, 0);
    gate = new Function(rms) 
    {
      public float calculate() {
        if (gateValue == 0) { // gate is closed
          //if (x[0] < gateLoThresh && x[0] < gateHiThresh) {
          if (x[0] < pLoThresh) {
            gateValue = 1;  // open gate
            outGainEnv.clear();
            //outGainEnv.addSegment(lifespan, pAttack);
            outGainEnv.addSegment(1.0, pAttack);
          }
        } else {  // gate is open
          //if (x[0] > gateHiThresh) {
          if (x[0] > pHiThresh) {
            gateValue = 0; // close gate
            outGainEnv.clear();
            // tbd: if x[0] > 0.99 pDecay = 1;
            outGainEnv.addSegment(pLevelerMix, pDecay);  
            
            //outGainEnv.addSegment(0.0, pDecay);
          }
        }
        return gateValue;
      }
    };
    ac.out.addDependent(gate);

    Gain outGain = new Gain(ac, 1, outGainEnv);  // this signal goes into the delay and to the speakers via pan and other gain
    outGain.addInput(hpf);

    RMS   rmsOut = new RMS(ac, 1, 441);
    rmsOut.addInput(outGain);

    Function   returnOutputRms = new Function(rmsOut) 
    {
      public float calculate() {
        rmsOutVal = min(1.0, x[0]);    // limit rms to 0-1
        return x[0];  // pro forma, not used
      }
    };
    ac.out.addDependent(returnOutputRms); // let it calculate without using the function explicitly

    // output panning
    if (stereo) {
      pan = new Panner(ac, x/width * 2 - 1);
      pan.addInput(outGain);
      Gain master = new Gain(ac, 2, 1.0/maxNumParticles);
      master.addInput(pan);

      ac.out.addInput(master);  // connect it to the output
    } else {
      panY = new Panner(ac, y/height * 2 - 1);
      panXHigh = new Panner(ac, x/width * 2 - 1);
      panXLow = new Panner(ac, x/width * 2 - 1);
      panY.addInput(outGain);
      panXHigh.addInput(0, panY, 0);
      panXLow.addInput(0, panY, 1);

      Gain master = new Gain(ac, 4, 1.0/maxNumParticles);
      master.addInput(0, panXHigh, 0);
      master.addInput(1, panXHigh, 1);
      master.addInput(2, panXHigh, 0);
      master.addInput(3, panXHigh, 1);

      ac.out.addInput(master);  // connect it to the output
    }


    // feed node output into tap in
    delayIn = new TapIn(ac, 2000); // create the delay input - the second parameter sets the maximum delay time in milliseconds
    delayIn.addInput(noiseGain); // connect the impulse to the delay
    delayIn.addInput(outGain); // connect the node output to the delay

    //println("connected inputs "+signalInput.getNumberOfConnectedUGens(0));
  }    // end constructor

  void impulse() {
    if (rmsVal < loThresh * 0.01) { 
      env.addSegment(lifespan * 0.95, max(1.0, velocity.mag() * 100));  // noise attack
      env.addSegment(0, 10);  // noise decay
    }
  }

  // test against all other particles if to make, maintain or disconnect a connection
  void intersects(ArrayList<Particle> particles) {
    for (Particle other : particles) {
      if (other != this) {
        PVector dir = PVector.sub(location, other.location); // direction of repulsion for this particle
        PVector p = dir.copy();  
        float dist = dir.mag();
        dist = max(1.0, dist); // lower limit = 1

        if (dist < pAttractRadius * range) {  // intersects with other particle. maybe time to create edge
          //if ((dist < pAttractRadius * range) || (dist < pAttractRadius *  4.0 && (edges[this.id][other.id] != null))) {  // intersects with other particle
          //range = 2;  // test

          if (edges[this.id][other.id] == null) { //no edge -> create a new one

            if (other.inConnex < maxConnex && outConnex < maxConnex) {  
              if (bgBrightness > minBgBrightness || other.bgBrightness > minBgBrightness) {  // new inConnex only if in the light
                edges[this.id][other.id] = new Edge(this.id, other.id, dist);  // create new edge
                //println("connected: " + this.id + " " + other.id);
                other.inConnex++;  // in inConnex
                outConnex++;
                //signalInput.removeAllConnections(selfDel);  // test
                //if (rmsVal < loThresh * 0.01) { // noise burst if level too low
                if ((rmsVal < loThresh * 0.01) && (other.rmsVal < other.pLoThresh * 0.01)) { // noise burst if from and to node level too low
                  impulse();
                  //env.addSegment(0.95, this.velocity.mag() * 100);
                } else {
                  //micGainEnv.addSegment(1.0, this.velocity.mag() * 100);
                  //micGainEnv.addSegment(0, 1000);
                }
              }
            } 
            //
          } else { // edge already exists -> still intersects or intersects again
            if (edges[this.id][other.id].lifespan < maxLife) {  // didn't intersect the frame before
              edges[this.id][other.id].lifespan = maxLife;      // give back full lifespan
              edges[this.id][other.id].setGain(lifespan / maxLife );
            } else {
              // still intersecting -> do nothing
            }
            edges[this.id][other.id].setDelay(dist * pDelayFactor);
          }

          if (edges[this.id][other.id] != null) { // edge exitsts
            if (dist >= particleRadius ) {

              //if (rmsVal > hiThresh) {
              //  levelFact -= 0.01;
              //}
              //if (rmsVal < loThresh) {
              //  levelFact = levelFact + 0.01;
              //}
              //levelFact = constrain(levelFact, 0.1, 2.0);

              float factor;
              if (attraction) { 
                factor = (1 - rmsForceAmt) +  ((1 - rmsVal) * rmsForceAmt);
                dir.setMag(-pAttractFactor * range/(dist * dist) * factor);  // attract
              } else {  // repulsion
                factor = (1 - rmsForceAmt) +  (rmsVal * rmsForceAmt);
                dir.setMag(pRepulsionFactor * range/(dist * dist) * 100 * factor);   // repulse
              }
            } else { // within particle radius
              bounce(dir, other);
            }
          } // end edge exitsts
          else {  // no edge
            separate = true;
            //range = 1;  // test
          }
        } else { // does not intersect, but check if there's still an edge

          //range = 1;  // test
          //dir.setMag(0.0);  // test
          //if (edges[this.id][other.id] != null) edges[this.id][other.id].setDelay(dist * pDelayFactor);  // test

          if (edges[this.id][other.id] != null) { // there still is an edge -> reduce lifespan
            if (bgBrightness < minBgBrightness || other.bgBrightness < minBgBrightness || other.inConnex > maxConnex || outConnex > maxConnex) {
              if (!edges[this.id][other.id].isAlive() ) { // reduce lifespan and check if alive
                edges[this.id][other.id].disconnect(); // disconnect and kill edge beads only if bg dark
                edges[this.id][other.id] = null; // release edge
                other.inConnex--;
                outConnex--;
                //if (signalInput.getNumberOfConnectedUGens(0) == 0) {
                //signalInput.addInput(selfDel);  // test
                //}
                dir.setMag(0.0);  // nothing
              } else {
                edges[this.id][other.id].setDelay(dist * pDelayFactor);
                dir.setMag(pSeparation*50/dist);
              }
            } else {
              edges[this.id][other.id].setDelay(dist * pDelayFactor);
              dir.setMag(pSeparation*50/dist);
            }
          } else { // not intersecting and no edge
            separate = true;
          }
        }    // end intersects


        if (separate) { // not intersecting and no edge -> separation force
          separate = false;
          if (dist >= pAttractRadius) {
            if (dist < 2*pAttractRadius) {
              dist = dist - pAttractRadius;
              dist = max(1.0, dist); // lower limit = 1
              dir.setMag(pSeparation/dist);
            } else { // to far away
              dir.setMag(0.0);  // nothing
            }
          } else {  // < attractRadius 
            if (dist >= particleRadius ) {
              dir.setMag((pAttractRadius - dist + 1)*pSeparation);
              //dir.setMag(0.0);  // nothing
            } else { // bounce
              bounce(dir, other);
            }
          }
        }  // end separate

        applyForce(dir);

        //attract = 1;

        // display edge
        if (edges[this.id][other.id] != null) {

          // cone
          /*
          for (int i = 0; i < rmsValNbr; i++) {
           rmsValHistReadIndx = rmsValHistWriteIndx - i;
           if (rmsValHistReadIndx < 0) rmsValHistReadIndx = rmsValNbr + rmsValHistReadIndx; 
           if (rmsValHist[rmsValHistReadIndx]  <= rmsValHist[rmsValHistWriteIndx]) { // smaller current rms value
           //stroke( sqrt(rmsSave) * 255 * (rmsValNbr - i) / rmsValNbr, 255 * (rmsValNbr - i) / rmsValNbr);
           //ellipse(location.x, location.y, rmsSave * diameter + 2, rmsSave * diameter + 2);
           
           float r = (rmsValHist[rmsValHistReadIndx] * diameter + 2) / 2.0;
           float teta = acos(r / dist);
           PVector q = p.copy();  
           q.setMag(r); 
           q.rotate(teta);
           float x = q.x;
           float y = q.y;
           q.rotate(-2*teta);
           noStroke();
           fill(pow(rmsValHist[rmsValHistReadIndx], 1.5) * 255, 255 * (rmsValNbr - i) / rmsValNbr);
           quad(location.x, location.y, location.x + x, location.y + y, other.location.x, other.location.y, location.x + q.x, location.y + q.y);
           }
           }
           */

          // cone
          float r = (rmsVal * diameter) / 2.0;
          float teta = acos(r / dist);
          //p.mult(0.5);
          //PVector ancor2 = new PVector(location.x + p.x, location.y + p.y);
          p.setMag(r); 
          p.rotate(teta);
          //p.x = max(p.x, 1.0);
          //p.y = max(p.y, 1.0);
          float x = p.x;
          float y = p.y;
          p.rotate(-2*teta);
          fill(rmsOutVal * 120 + 55, 50 * rmsOutVal + 50);
          noStroke();
          quad(location.x, location.y, location.x + x, location.y + y, other.location.x, other.location.y, location.x + p.x, location.y + p.y);

          //stroke(255);
          beginShape(QUADS);
          tint(255, 50 + rmsOutVal * 50);
          texture(texture);
          vertex(location.x, location.y, textureOffset, 5);
          vertex(location.x + x, location.y + y, textureOffset + 1, 0);
          vertex(other.location.x, other.location.y, textureOffset + dist, 5);
          vertex(location.x + p.x, location.y + p.y, textureOffset + 1, 10);
          tint(255, 255);
          endShape();


          //stroke(rmsVal * 255 + 40, 128);
          //noFill();
          //bezier(location.x + x, location.y + y, location.x + p.x, location.y + p.y, ancor2.x , ancor2.y  , other.location.x, other.location.y);

          // line
          stroke(255 * rmsVal, edges[this.id][other.id].lifespan * 255.0 / maxLife);
          strokeWeight(1);
          line(location.x, location.y, other.location.x, other.location.y);
        }
      }
    }  // end particles loop
    //} // test
  }

  void bounce(PVector dir, Particle other) {
    float spring = 0.001;
    float angle = atan2(dir.y, dir.x);
    float targetX = location.x + cos(angle) * 2*particleRadius;
    float targetY = location.y + sin(angle) * 2*particleRadius;
    float ax = (targetX - other.location.x) * spring;
    float ay = (targetY - other.location.y) * spring;
    velocity.x += ax;
    velocity.y += ay;
    other.velocity.x -= ax;
    other.velocity.y -= ay;
    dir.setMag(0);
  }

  void applyForce(PVector force) {
    //PVector f = force.copy();
    //f.div(mass);
    acceleration.add(force);
  }


  // Method to update location and other parameters
  void update() {
    if (!freeze) {
      velocity.add(acceleration);
      location.add(velocity);
      acceleration.mult(0);
    }
    checkBounds();
    if (stereo) {
      pan.setPos(location.x/width * 2 -1);
    } else {
      panY.setPos(location.y/height * 2 -1);
      panXHigh.setPos(location.x/width * 2 -1);
      panXLow.setPos(location.x/width * 2 -1);
    }
    pHpfFreq = calc_pHpfFreq();
    setHpfGlideFreq(pHpfFreq);
    textureOffset -= max(0.1, 2 * (1 - pDelayFactor));
    //selfDelTime = max(1.0, velocity.mag()*100);
    //selfDelGlide.setValue(selfDelTime); 
    //masterGainGlide.setValue(1.0f/maxNumParticles * masterGain);

    if (textureOffset <= 0) textureOffset = 100 + textureOffset;
    if (pLevelerMix > 0.5) framesSinceLastHiCrossLimit -= 0.1;

    if (hiCrossings == hiCrossingsPrev && rmsVal < 0.95) {
      framesSinceLastHiCross++;
    } else {
      hiCrossingsPrev = hiCrossings;
      framesSinceLastHiCross = 0;
    }
    if (framesSinceLastHiCross > framesSinceLastHiCrossLimit) {
      framesSinceLastHiCross = 0;
      framesSinceLastHiCrossLimit = 500;
      if (pLevelerMix > 0.5) pLevelerMix = 0;
      //if(pLevelerMix > 0.5) pLevelerMix *= 0.1;
      //else pLevelerMix = 1.0;
      //pDelayFactor = pDelayFactor/ 2;
      pAttack = pAttack / 2;
      pAttack = max(1, pAttack);

      pDecay = pDecay * 2;
      pDecay = min(1000, pDecay);
    }


    if (rmsVal < pLoThresh) {
      //lifespan -= max(0.0, (1 - rmsVal) * 0.001);
      //pAttractRadius += 0.1;
      //pAttractRadius = min(2 * attractRadius, pAttractRadius);
      //pAttractRadius = min(4 * attractRadius, pAttractRadius);

      pAttack -= 0.2;
      pAttack = max(1, pAttack);
      //pAttack = max(0.01 * attack, pAttack);
      pDecay += 0.18;
      pDecay = min(10 * decay, pDecay);
      pDelayFactor -= 0.001 * (1-bgBrightness) ;  // test 1.15
      //pDelayFactor = max( 0.05, pDelayFactor); // test 1.15
      pDelayFactor = max(delayFactor / 10.0, pDelayFactor); // test 1.15
      pRepulsionFactor -= 0.01;
      pRepulsionFactor = max(repulsionFactor*0.5, pRepulsionFactor);
      pAttractFactor += 0.017; // was 0.013
      //pAttractFactor += 0.73 * (1 - lifespan); // was 0.013
      pAttractFactor = min(1.0, pAttractFactor);
      pLoThresh += 0.0001;
      pLoThresh = min(loThresh * 2.0, pLoThresh); //
      pLoThresh = min(pHiThresh - 0.02, pLoThresh);
      //pShaperMix += 0.001;
      //pShaperMix = min(0.01, pShaperMix);
      //pShaperMix = min(0.05, pShaperMix);
      
      pLevelerMix += 0.001;  // test 14
      
      //if (pLevelerMix > 1.0) pLevelerMix = 0.0;
      pLevelerMix = min(1.0, pLevelerMix);  // test 14
    } else {
      if (gateValue == 1) { // open
        pLoThresh -= 0.0001;
        pLoThresh = max(loThresh, pLoThresh);
      } else { // closed
        pLoThresh += 0.0001;
        pLoThresh = min(pHiThresh-0.01, pLoThresh);
      }
    }
    if (rmsVal > pHiThresh) {
      //lifespan += (1 - rmsVal) * 0.001  ;
      //lifespan = min(1.0, lifespan);
      pAttractRadius -= 1;
      pAttractRadius = max(attractRadius * 0.5, pAttractRadius);
      pAttack += 1;
      //pAttack += 1;
      pAttack = min(5 * attack, pAttack);
      //pAttack = min(5 * attack, pAttack);
      pDecay -= 10;
      pDecay = max(1, pDecay);
      //pDecay = max(0.01 * decay, pDecay);
      //pDecay = max(0.05 * decay, pDecay);
      pDelayFactor += 0.002 * (1-bgBrightness);  // 0.002
      pDelayFactor = min(delayFactor * 4, pDelayFactor); // 2
      pDelayFactor = min(1, pDelayFactor); // test 1.15
      pRepulsionFactor += 0.01;
      pRepulsionFactor = min(1.0, pRepulsionFactor);
      pAttractFactor -= 0.28; // was 0.28
      //pAttractFactor -= 0.68 * lifespan; // was 0.28
      pAttractFactor = max(0.1 * attractFactor, pAttractFactor);

      //pHiThresh -= 0.002;
      //pHiThresh = max(hiThresh * 0.5, pHiThresh); 
      //pHiThresh = max(pLoThresh + 0.01, pHiThresh);

      pShaperMix -= 0.001;
      pShaperMix = max(0.0, pShaperMix);
      //pHpfFreq *= 0.5;
      //pHpfFreq = max(40, pHpfFreq);
      //setHpfGlideFreq(pHpfFreq + (1 - location.y / height) * 40.0);
    } else {
      //pHiThresh += 0.0001;
      //pHiThresh = min(hiThresh, pHiThresh);

      //pHpfFreq *= 1.0001;
      //pHpfFreq *= 1.0001;
      //if (pHpfFreq > 400) pHpfFreq = 40;
      //pHpfFreq = min(2000, pHpfFreq);
      //setHpfGlide(pHpfFreq);
      //setHpfGlideFreq(pHpfFreq + (1 - location.y / height) * 40.0);
    }
    //shaper.setWetMix(pShaperMix);

    if (rmsVal > 0.95) {    // higher than max level
      overMax = overMax + (rmsVal - 0.95);
      pHiThresh -= (rmsVal - 0.95)*0.1;
      //pHiThresh = max(hiThresh * 0.5, pHiThresh); 
      pHiThresh = max(pLoThresh + 0.01, pHiThresh);
      pLevelerMix += overMax;  // test 14
      //pLevelerMix += (rmsVal - 0.95);  // test 14
      //if (pLevelerMix > 1.0) pLevelerMix = 0.0;
      pLevelerMix = min(1.0, pLevelerMix); // test 14
      if (rmsVal > 0.99) {
        //location = location.add(flowField.lookup(location).mult(10));
        pRepulsionFactor = min(1.0, 2*pRepulsionFactor);
        //dragFactor = 0.01;
      }
    } else {  // lower than max level
      overMax *= 0.9;
      pHiThresh += 0.0005;
      pHiThresh = min(hiThresh, pHiThresh);
    }
    if (velocity.mag() > 0.5) {
      pLevelerMix -= 0.0002;  
      pLevelerMix = max(0.0, pLevelerMix);
    } else {
      pLevelerMix += 0.0001;  
      pLevelerMix = min(1.0, pLevelerMix);
    }

    // bgBrightness influence
    //lifespan = bgBrightness;  // test

    if (bgBrightness < minBgBrightness) {
      if (rmsVal > 0.01) {  // there is some input
        //lifespan = min(1.0, lifespan * 1.02);
      } else {
        //masterGain = max(0.0, masterGain - 0.001);
        //lifespan = max(0.001, lifespan * 0.998); // lifespan decay
      }
      pAttractRadius -= 0.1;
      pAttractRadius = max(attractRadius * 0.5, pAttractRadius);
      pSeparation = min(separation, pSeparation + 0.001);
      range = 1;
      dragFactor = max(0.0001, dragFactor * 0.8);
    } else {
      range = 1 + bgBrightness;
      //masterGain = min(1.0, masterGain + 0.1);
      //lifespan = min(1.0, lifespan * 1.05); // lifespann increase
      //lifespan = min(1.0, bgBrightness);
      pAttractRadius += 0.1;
      pAttractRadius = min(attractRadius * 2, pAttractRadius);
      pSeparation = max(0.0, pSeparation - 0.05);
      dragFactor = min(1.0, dragFactor * 1.02);
    }

    if (gateValue == 0) { // closed gate
      outGainEnv.clear();
      outGainEnv.addSegment(pLevelerMix, pDecay);  // tbd outgain mit plevelermix nachführen
    }

    // write rms history
    rmsValHistWriteIndx = (rmsValHistWriteIndx + 1) % rmsValNbr;
    rmsValHist[rmsValHistWriteIndx] = rmsOutVal;

    levelerGlide.setValue(pLevelerMix);
  } // end update


  // Method to  
  void display() {
    float rmsSave = 0;
    if (debug) {
      noFill();
      stroke(155);
      ellipse(location.x, location.y, 2*pAttractRadius*range, 2*pAttractRadius*range);
    }
    noStroke();
    if (bgBrightness < 0.01) fill(55, flowField.lookupMag(location)*20);  // static paricle shape
    //if(bgBrightness < 0.01) fill(55, flowField.getShapedVelocityMagAtLocation(location)*20);  // static paricle shape
    else fill(55, bgBrightness*(1-rmsVal)*255);
    ellipse(location.x, location.y, diameter, diameter);  

    if (attraction) {
      if (rmsVal < rmsValPrev) {
        rmsValPrev = rmsVal;
        rmsValCount = 0;
      } else {
        rmsValCount++;
      }
      rmsSave = rmsValPrev;
      if (rmsValCount >= 20) {
        rmsValCount = 0;
        rmsValPrev = 1.0;
      }
    } else {  // repulsion
      if (rmsVal > rmsValPrev) {
        rmsValPrev = rmsVal;
        rmsValCount = 0;
      } else {
        rmsValCount++;
      }
      rmsSave = rmsValPrev;
      if (rmsValCount >= 20) {
        rmsValCount = 0;
        rmsValPrev = 0.0;
      }
    }

    // visualize attraction / repulsion
    if (attraction) {
      vDiam -= 0.5;
      if (vDiam <= 0 ) vDiam = diameter;
      stroke( sqrt(rmsSave) * 255, 100 + 150 * rmsVal * pAttractFactor);
    } else {
      vDiam += 0.5;
      if (vDiam > diameter) vDiam = 0;
      stroke( sqrt(rmsSave) * 255, 200 + 50 * rmsVal * pRepulsionFactor);
    }
    noFill();  // ----
    strokeWeight(1);
    float cDiam = sqrt(rmsOutVal) * diameter;  // current diameter
    //ellipse(location.x, location.y, cDiam, cDiam); 

    stroke( rmsSave * 255, 25 + 50 * rmsSave);
    for (int i = 0; vDiam + i * 7.0 <= cDiam; i++) { // draw ellipses equal and greater than diameter
      ellipse(location.x, location.y, vDiam + i * 7.0, vDiam + i * 7.0);
    }
    for (int i = 1; vDiam - i * 7.0 > 0; i++) { // draw ellipses smaller than diameter
      if (vDiam - i * 7.0 < cDiam) {
        ellipse(location.x, location.y, vDiam - i * 7.0, vDiam - i * 7.0);
      }
    }

    // display 50 ellipses with radius according to last 50 rmsVals
    for (int i = 0; i < rmsValNbr; i++) {
      rmsValHistReadIndx = rmsValHistWriteIndx - i;
      if (rmsValHistReadIndx < 0) rmsValHistReadIndx = rmsValNbr + rmsValHistReadIndx; 
      rmsSave = rmsValHist[rmsValHistReadIndx];
      if (rmsSave <= rmsValHist[rmsValHistWriteIndx]) { // smaller current rms value
        //if (attraction) {
        noFill();
        stroke( 30 + sqrt(rmsSave) * 255 * (rmsValNbr - i) / rmsValNbr, 255 * (rmsValNbr - i) / rmsValNbr); // ---
        //} else {
        //noStroke();
        //fill( sqrt(rmsSave) * 255 * (rmsValNbr - i) / rmsValNbr, 255 * (rmsValNbr - i) / rmsValNbr); // ---
        //}

        //stroke( 255 * (rmsValNbr - i) / rmsValNbr, 255 * (rmsValNbr - i) / rmsValNbr);

        ellipse(location.x, location.y, rmsSave * diameter, rmsSave * diameter);
      }
    }

    if (debug) {
      textSize(10);
      fill(155);
      text("Id: " + id, location.x + 6, location.y - 60);
      text("vel/field: "+ String.format("%.2f", velocity.mag())+"/"+String.format("%.2f", flowField.lookupMag(location)), location.x + 6, location.y - 50);

      text("Bright: " + String.format("%.2f", bgBrightness), location.x + 6, location.y - 40);
      text("Separat: " + String.format("%.2f", pSeparation), location.x + 6, location.y - 30);
      text("Radius: " + String.format("%.2f", pAttractRadius), location.x + 6, location.y - 20);
      if (gateValue == 1) fill(0, 255, 0);
      else fill(255, 0, 0);
      text("A/D: ", location.x + 6, location.y -10);
      fill(155);
      text(String.format("%.2f", pAttack) + "/"+ String.format("%.2f", pDecay), location.x + 30, location.y -10);
      text("Thresh:" + String.format("%.3f", pLoThresh) + "  " + String.format("%.3f", pHiThresh), location.x + 6, location.y );
      text("Att/Rep:" + String.format("%.3f", pAttractFactor)+ "/" + String.format("%.3f", pRepulsionFactor), location.x + 6, location.y + 10);
      text("Delay: " + String.format("%.3f", pDelayFactor), location.x + 6, location.y +20);
      if (rmsVal > 0.99) fill(255, 0, 0);
      else fill(255);
      text("RMS:" + String.format("%.3f", rmsVal), location.x + 6, location.y + 30);
      fill(155);
      text("Mix:" + String.format("%.2f", pLevelerMix), location.x + 6, location.y + 40);
      text("hCr:" + hiCrossings, location.x + 6, location.y + 50);
      text("lastCross:" + framesSinceLastHiCross+"/"+framesSinceLastHiCrossLimit, location.x + 6, location.y + 60);
      text("hpf:" + pHpfFreq, location.x + 6, location.y + 70);
      text("inConx:" + inConnex, location.x + 6, location.y + 80);
      text("outConx:" + outConnex, location.x + 6, location.y + 90);
      text("drag:" + String.format("%.3f", dragFactor), location.x + 6, location.y + 100);
    }
  }

  // Is the particle still useful?
  boolean isAlive() {
    if (lifespan <= 0.0) {
      return false;
    } else {
      return true;
    }
  }

  void checkBounds() {
    if (location.y < viewYOffset) {
      location.y = viewYOffset;
      if (velocity.y < 0) velocity.y = - velocity.y;
    } else if (location.y >= height) {
      location.y = height - 1;
      if (velocity.y > 0) velocity.y = - velocity.y;
    } else if (location.x >= width) {
      location.x = width - 1;
      if (velocity.x > 0) velocity.x = - velocity.x;
    } else if (location.x < 0) {
      location.x = 0;
      if (velocity.x < 0) velocity.x = - velocity.x;
    }
  }

  void repellFromSides() {
    PVector force = new PVector(0, 0);
    PVector forceCalc = new PVector(0, 0);
    float distance = 0;

    // separation from left side
    distance = max(1, location.x - pAttractRadius);
    if (distance <= pAttractRadius) {
      forceCalc.set(1, 0);
      forceCalc.setMag(pSeparation/distance);
      force.add(forceCalc);
    }
    // separation from right side
    distance = max(1, width - location.x - pAttractRadius);
    if (distance <= pAttractRadius) {
      forceCalc.set(-1, 0);
      forceCalc.setMag(pSeparation/distance);
      force.add(forceCalc);
    }
    // separation from top
    distance = max(1, location.y - viewYOffset - pAttractRadius);
    if (distance <= pAttractRadius) {
      forceCalc.set(0, 1);
      forceCalc.setMag(pSeparation/distance);
      force.add(forceCalc);
    }
    // separation from bottom
    distance = max(1, height - location.y - pAttractRadius);
    if (distance <= pAttractRadius) {
      forceCalc.set(0, -1);
      forceCalc.setMag(pSeparation/distance);
      force.add(forceCalc);
    }

    /*
    if (location.x < pAttractRadius) {
     forceCalc.set(1, 0);
     distanceX = location.x;
     } else if (location.x > width - pAttractRadius) {
     force.set(-1, 0);
     distanceX =  width - location.x ;
     } 
     if (location.y < pAttractRadius) {
     force.add(0, 1);
     distanceY = location.y;
     } else if (location.y > height - pAttractRadius) {
     force.add(0, -1);
     distanceY = height - location.y;
     }
     distance = max(1, sqrt(pow(distanceX, 2) + pow(distanceY, 2)));
     force.setMag(100 * pSeparation/distance);
     */
    force.mult(10);
    applyForce(force);
  }


  void drag() {
    float speed = velocity.mag();
    float dragMagnitude;
    if (bgBrightness < minBgBrightness) dragMagnitude = drag * speed * speed * dragFactor * 100;
    else dragMagnitude = drag * speed * speed * dragFactor * 1000;
    //dragMagnitude = drag * speed * speed * max(0.001, sqrt(bgBrightness) * 2);
    PVector dragForce = velocity.copy();
    dragMagnitude = min(dragMagnitude, dragForce.mag());
    dragForce.mult(-1);
    dragForce.normalize();
    dragForce.mult(dragMagnitude);
    applyForce(dragForce);
  }

  // Beads
  float calc_pHpfFreq() {
    return pow(pitchBase, (1 - location.y / (height - 1)) * pitchRange) * hpfFreq;
  }

  void setHpfGlideFreq(float freq) {
    hpfGlide.setValue(freq);
  }

  void disconnect() {
    for (int i = 0; i < maxNumParticles; i++) {
      if (edges[this.id][i] != null) {
        edges[this.id][i].disconnect(); // disconnect and kill edge beads
        edges[this.id][i] = null; // release edge
      }
      if (edges[i][this.id] != null) {
        edges[i][this.id].disconnect(); // disconnect and kill edge beads
        edges[i][this.id] = null; // release edge
      }
    }
    signalInput.kill();
    wp.kill();
    noiseGain.kill();
    env.kill();

    outGainEnv.kill();
    delayIn.kill();
    hpf.kill();
    rms.kill();
    pan.kill();
    hpfGlide.kill();
    //shaper.kill();
  }
}
