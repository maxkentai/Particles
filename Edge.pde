class Edge {
  int fromNode;
  int toNode;
  float lifespan;
  float eLength;
  float amp;
  TapOut delayOut;
  Envelope env;
  Glide delayTimeGlide;
  Gain outputGain;
  Glide outputGainGlide;
  int keepBits;
  //int reducer = 1;
  //float cutoff = 1000;
  //Glide lpfGlide;
  //BiquadFilter lpf;

  Edge(int fromNode_, int toNode_, float distance) {
    fromNode = fromNode_;
    toNode = toNode_;
    eLength = distance;
    lifespan = maxLife;
    delayTimeGlide = new Glide(ac, eLength, 10);
    delayOut = new TapOut(ac, ps.getParticle(fromNode).delayIn, delayTimeGlide); // create the delay output - the final parameter is the length of the initial delay time in milliseconds
    outputGainGlide = new Glide(ac, 1, 10);
    outputGain = new Gain(ac, 1, outputGainGlide);
    outputGain.addInput(delayOut);

    ps.getParticle(toNode).signalInput.addInput(outputGain); // connect the delay output to the Particle
  }

  void disconnect() {
    setGain(0.0);
    ps.getParticle(toNode).signalInput.removeAllConnections(outputGain);  // remove outputGain from all inputs of signalInput
    outputGain.clearInputConnections();  // clear all input connections
    delayOut.kill();
    delayTimeGlide.kill();
    outputGain.kill();
    outputGainGlide.kill();
    
    //println("disconnected: " + this.fromNode + "  " + this.toNode);
  }

  boolean isAlive() {
    lifespan--;
    setGain(max (0.0, lifespan / maxLife ));
    if (lifespan <= -100.0) {
      return false;
    } else {
      return true;
    }
  }

  void setDelay(float delaytime) {
    if (delaytime < 1) delaytime = 1;
    delayTimeGlide.setValue(delaytime);
  }

  void setGain(float gain) {
    outputGainGlide.setValue(gain);
  }

  void display() {
    //for (int i = 0; i < rmsValNbr; i++) {
    //rmsValHistReadIndx = rmsValHistWriteIndx - i;
    // if(rmsValHistReadIndx < 0) rmsValHistReadIndx = rmsValNbr + rmsValHistReadIndx; 
    //rmsSave = rmsValHist[rmsValHistReadIndx];
    // if(rmsSave <= rmsValHist[rmsValHistWriteIndx]) { // smaller current rms value
    //stroke( sqrt(rmsSave) * 255 * (rmsValNbr - i) / rmsValNbr, 255 * (rmsValNbr - i) / rmsValNbr);
    ////stroke( 255 * (rmsValNbr - i) / rmsValNbr, 255 * (rmsValNbr - i) / rmsValNbr);
    //ellipse(location.x, location.y, rmsSave * diameter + 2, rmsSave * diameter + 2);
    // }
    //    }
    //
  }
}
