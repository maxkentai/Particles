// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Using Generics now!  comment and annotate, etc.

class ParticleSystem {
  ArrayList<Particle> particles;

  ParticleSystem(int numParticles) {
    particles = new ArrayList<Particle>();
    for (int i=0; i< numParticles; i++) {
      this.addParticle(random(width), random(height), new PVector(random(1), random(1)));
    }
  }

  void addParticle(float x, float y, PVector velocity) {
    int id = 0;
    for (id = 0; id < maxNumParticles; id++) {
      boolean found = false;
      for (Particle p : particles) {
        if (id == p.id) {
          found = true;
          break;
        }
      }
      if (!found) break;
    }
    particles.add(new Particle(x, y, id, velocity));
    //Particle part = getParticle(id);
    //part.selfDel = new Edge(part.id, part.id, random(100));
    println("particle added: " + id);
  }

  void removeAllParticles() {
    for (int i = particles.size()-1; i >= 0; i--) {
      Particle p = particles.get(i);
      p.disconnect();
      particles.remove(i);
    }
  }

  void display() {
    for (Particle p : particles) {
      p.display();
    }
  }

  void applyForce(PVector f) {
    for (Particle p : particles) {
      p.applyForce(f);
    }
  }

  void intersection() {
    for (Particle p : particles) {
      p.intersects(particles);
    }
  }

  void drag() {
    for (Particle p : particles) {
      p.drag();
    }
  }

  void repellFromSides() {
    for (Particle p : particles) {
      p.repellFromSides();
    }
  }

  void applyRepeller(Repeller r) {
    for (Particle p : particles) {
      PVector force = r.repel(p);        
      p.applyForce(force);
    }
  }


  void update() {
    for (int i = particles.size()-1; i >= 0; i--) {
      Particle p = particles.get(i);
      p.update();
      //if (!p.isAlive()) {
      //  p.disconnect();
      //  particles.remove(i);
      //  println("dead: " + p.id);
      //}
    }
  }

  Particle getParticle(int id) {
    int i = 0;
    for (i = 0; i < particles.size(); i++) {
      if (particles.get(i).id == id) {
        break;
      }
    }
    return particles.get(i);
  }
}
