$("#boidModal").on('shown.bs.modal', function() {
  if (document.getElementById("boidCanvas")) {
    return;
  }
  let container = document.getElementById("boidContainer");
  let sketch = function(p) {
    let peripheralmode = true;
    let shouldDisplay = true;
    let flock;
    let sliderwalls;
    let sliderview;
    let slidercohesion;
    let sliderseparate;
    let slideralign;
    let grid = [];

    let Boid = function(l, mf, ms) {
      this.location = l.copy();
      this.velocity = p.createVector(0, 0);
      this.acceleration = p.createVector(0, 0);
      this.maxforce = mf;
      this.maxspeed = ms;
      this.r = 15;
      this.peripheral = 240;
      this.visionlength = 10*this.r;
      this.viewmult = 1;
      this.cohesionmult = 1;
      this.separatemult = 1;
      this.wallsmult = 1;
      this.alignmult = 1;
      this.column = p.floor(location.x * 11 / p.width);
      this.row = p.floor(location.y * 11 / p.height);
    };

    Boid.prototype.applyForce = function(force) {
      this.acceleration.add(force);
    }

    Boid.prototype.align = function(boids) {
      let averageVel = p.createVector(0, 0);
      for(var i = 0; i < boids.length; i++) {
        let boid = boids[i]
        let peripherybool = false;
        if (this.peripheralmode) {
          let d = p.p.Vector.sub(this.location, boid.location).mag();
          let angle = 0;
          if (this.velocity.mag() != 0) {
            angle = p.Vector.angleBetween(this.velocity, p.Vector.sub(boid.location, this.location));
          } else {
            angle = p.Vector.angleBetween(p.createVector(1, 0), p.Vector.sub(boid.location, this.location));
          }
          peripherybool = (angle < radians(this.peripheral/2)) && (this.d < this.visionlength);
        } else {
          peripherybool = boid.location.dist(this.location) < this.visionlength;
        }
        if (peripherybool) {
          averageVel.add(boid.velocity);
        }
      }
      if (boids.length > 0) {
        averageVel.div(boids.length);
        averageVel.setMag(this.maxspeed);
        let steer = averageVel.copy()
        steer = steer.sub(this.velocity);
        steer.limit(this.maxforce);
        return steer;
      } else {
        return averageVel;
      }
    }

    Boid.prototype.cohesion = function(boids) {
      let averageLoc = p.createVector(0, 0);
      for(var i = 0; i < boids.length; i++) {
        let boid = boids[i];
        let peripherybool = false;
        if (this.peripheralmode) {
          let d = p.Vector.sub(this.location, boid.location).mag();
          let angle = 0;
          if (this.velocity.mag() != 0) {
            angle = p.Vector.angleBetween(this.velocity, Pp.Vector.sub(boid.location, this.location));
          } else {
            angle = Pp.Vector.angleBetween(p.createVector(1, 0), p.Vector.sub(boid.location, this.location));
          }
          peripherybool = (angle < radians(this.peripheral/2)) && (d < this.visionlength);
        } else {
          peripherybool = boid.location.dist(this.location) < this.visionlength;
        }
        if (peripherybool) {
          averageLoc.add(boid.location);
        }
      }
      if (boids.length > 0) {
        averageLoc.div(boids.length);
        let desiredVelocity = averageLoc.copy();
        desiredVelocity.sub(this.location);
        desiredVelocity.limit(this.maxspeed);
        let steer = desiredVelocity.copy()
        steer.sub(this.velocity);
        steer.limit(this.maxforce);
        return steer;
      } else {
        return averageLoc;
      }
    }

    Boid.prototype.separate = function(boids) {
      let desiredseparation = 2 * this.r;
      let sum = p.createVector(0, 0);
      for (var i = 0; i < boids.length; i++) {
        let boid = boids[i];
        let dv = this.location.copy()
        dv.sub(boid.location);
        let d = dv.mag();
        let peripherybool = false;
        if (this.peripheralmode) {
          let angle = 0;
          if (this.velocity.mag() != 0) {
            angle = p.Vector.angleBetween(this.velocity, p.Vector.sub(boid.location, this.location));
          } else {
            angle = p.Vector.angleBetween(p.createVector(1, 0), p.Vector.sub(boid.location, this.location));
          }
          peripherybool = (angle < p.radians(this.peripheral/2)) && (d < this.visionlength);
        } else {
          peripherybool = d < this.visionlength;
        }
        if (((d > 0) && (d < desiredseparation)) && peripherybool) {
          let diff = this.location.copy();
          diff.sub(boid.location);
          diff.normalize().div(d*d);
          sum.add(diff);
        }
      }
      if (boids.length > 0) {
        sum.div(boids.length);
        sum.setMag(this.maxspeed);
        let steer = sum.copy()
        steer.sub(this.velocity);
        steer.limit(this.maxforce);
        return steer;
      } else {
        return p.createVector(0, 0);
      }
    }

    Boid.prototype.walls = function() {
      let steer = p.createVector(0, 0);
      if (this.location.x < 25) {
        let desired = p.createVector(this.maxspeed,this.velocity.y);
        let steerage = desired.copy();
        steerage.sub(this.velocity);
        steerage.limit(this.maxforce);
        steer.add(this.steerage);
      }
      if (this.location.x > p.width - 25) {
        let desired = p.createVector(-1 * this.maxspeed,this.velocity.y);
        let steerage = desired.copy();
        steerage.sub(this.velocity);
        steerage.limit(this.maxforce);
        steer.add(steerage);
      }
      if (this.location.y < 25) {
        let desired = p.createVector(this.velocity.x,this.maxspeed);
        let steerage = desired.copy()
        steerage.sub(this.velocity);
        steerage.limit(this.maxforce);
        steer.add(steerage);
      }
      if (this.location.y > p.height - 25) {
        let desired = p.createVector(this.velocity.x,-1 * this.maxspeed);
        let steerage = desired.copy();
        steerage.sub(this.velocity);
        steerage.limit(this.maxforce);
        steer.add(steerage);
      }
      steer.limit(this.maxforce);
      return steer;
    }

    Boid.prototype.view = function(boids) {
      let sum = p.createVector(0, 0);
      for (var i = 0; i < boids.length; i++) {
        let boid = boids[i];
        let dist = p.dist(boid.location.x, boid.location.y, location.x, location.y);
        if (dist > 0 && dist < this.visionlength) {
          let angle = 0;
          if (this.velocity.mag() != 0) {
            angle = p.Vector.angleBetween(this.velocity, p.Vector.sub(boid.location, this.location));
          } else {
            angle = p.Vector.angleBetween(p.createVector(1, 0), p.Vector.sub(boid.location, this.location));
          }
          if (angle < this.peripheral/4) {
            let desired = p.Vector.sub(this.velocity.normalize(), p.Vector.sub(boid.location, this.location).normalize());
            desired.normalize();
            desired.mult(this.maxspeed);
            let steer = Pp.Vector.sub(desired, this.velocity);
            steer.limit(this.maxforce);
            sum.add(steer);
          }
        }
      }
      if (boids.length > 0) {
        sum.div(boids.length);
      }
      sum.setMag(this.maxforce);
      return sum;
    }

    Boid.prototype.flock = function(boids) {
      this.column = p.int(location.x * 10 / p.width);
      this.row = p.int(location.y * 10 / p.height);
      let sep = this.separate(boids);
      let ali = this.align(boids);
      let coh = this.cohesion(boids);
      let view = this.view(boids);
      let wall = this.walls();
      let seek = this.seek(p.createVector(p.mouseX, p.mouseY));
      seek.mult(10);
      sep.mult(this.separatemult);
      ali.mult(this.alignmult);
      coh.mult(this.cohesionmult);
      view.mult(this.viewmult);
      wall.mult(this.wallsmult);
      this.applyForce(seek);
      this.applyForce(view);
      this.applyForce(sep);
      this.applyForce(ali);
      this.applyForce(coh);
      this.applyForce(wall);
    }

    Boid.prototype.update = function() {
      this.acceleration.limit(this.maxforce);
      this.velocity.add(this.acceleration);
      this.velocity.setMag(this.maxspeed);
      this.location.add(this.velocity);
      this.acceleration.mult(0.01);
      if (this.location.x < 0) {
        this.location.x = p.width;
        //velocity.x = -velocity.x;
      }
      if (this.location.y < 0) {
        this.location.y = p.height;
        //velocity.y = -velocity.y;
      }
      if (this.location.x > p.width) {
        this.location.x = 0;
        //velocity.x = -velocity.x;
      }
      if (this.location.y > p.height) {
        this.location.y = 0;
        //velocity.y = -velocity.y;
      }
    }

    Boid.prototype.seek = function(target) {
      let desired = target.copy();
      desired.sub(this.location);  // A vector pointing from the position to the target
      // Normalize desired and scale to maximum speed
      desired.normalize();
      desired.mult(this.maxspeed);
      // Steering = Desired minus Velocity
      let steer = desired.copy();
      steer.sub(this.velocity);
      steer.limit(this.maxforce);  // Limit to maximum steering force
      return steer;
    }

    Boid.prototype.run = function(boids) {
      this.flock(boids);
      this.update();
      this.display();
    }

    Boid.prototype.display = function() {
      p.push();
      p.stroke(0);
      if (this.peripheralmode) {
        p.fill(100);
      } else {
        p.fill(0, 255, 0);
      }
      p.translate(this.location.x, this.location.y);
      p.rotate(this.velocity.heading());
      p.triangle(0, 0, -1 * this.r, -1 * this.r*0.25, -1 * this.r, this.r * 0.25);
      if (shouldDisplay) {
        p.stroke(0, 0, 255);
        p.fill(0, 0, 255, 40);
        p.arc(0, 0, this.visionlength*2, this.visionlength*2, -1 * p.radians(this.peripheral/2), p.radians(this.peripheral/2));
        p.line(0, 0, this.visionlength*p.cos(-1 * p.radians(this.peripheral/2)), this.visionlength*p.sin(-1 * p.radians(this.peripheral/2)));
        p.line(0, 0, this.visionlength*p.cos(p.radians(this.peripheral/2)), this.visionlength*p.sin(p.radians(this.peripheral/2)));
        p.rotate(-1 * this.velocity.heading());
      }

      //stroke(255, 0, 0);
      //line(0, 0, 10*acceleration.x, 10*acceleration.y);
      p.pop();
    }

    let Flock = function() {
      this.boids = [];
    }

    Flock.prototype.run = function() {
      for (var i = 0; i < this.boids.length; i++) {
        let boid = this.boids[i];
        boid.run(this.boids)
      }
    }

    Flock.prototype.addBoid = function(boid) {
      this.boids.push(boid)
    }

    p.setup = function() {
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 3 / 4);
      canvas.id("boidCanvas");
      p.background(249);
      p.loop();
      peripheralmode = true;
      shouldDisplay = false;
      flock = new Flock();
      for (var i = 0; i < 11; i++) {
        grid.push([]);
        for (var j = 0; j < 11; j++) {
          grid[i].push([]);
        }
      }
      for (var i = 0; i < 3; i++) {
        let b = new Boid(p.createVector(p.width/2 + p.random(0.001), p.height/2 + p.random(0.001)), 0.1, 1);
        flock.addBoid(b);
      }
      p.stroke(0);
      p.fill(0);
    }

    p.draw = function() {
      if (!document.getElementById("boidCanvas")) {
        p.noLoop();
      }
      p.noStroke();
      p.fill(255, 20);
      p.rect(0, 0, p.width, p.height);
      p.stroke(0);
      for (var i = 0; i < 11; i++) {
        p.line(i*p.width/11, 0, i*p.width/11, p.height);
        p.line(0, i*p.height/11, p.width, i*p.height/11);
      }
      flock.run();
      for (var i = 0; i < 11; i++) {
        for (var j = 0; j < 11; j++) {
          grid[i][j] = [];
        }
      }
      for (var i = 0; i < flock.boids.length; i++) {
        let boid = flock.boids[i];
        boid.alignmult = parseFloat(document.getElementById("align").value) / 10;
        boid.separatemult = parseFloat(document.getElementById("separate").value) / 10;
        boid.cohesionmult = parseFloat(document.getElementById("cohesion").value) / 10;
        boid.viewmult = parseFloat(document.getElementById("view").value) / 10;
        boid.wallsmult = parseFloat(document.getElementById("walls").value) / 10;
        console.log(boid.alignmult, boid.separatemult, boid.cohesionmult, boid.viewmult, boid.wallsmult)
        grid[boid.column][boid.row].push(boid);
      }
    }

    p.keyPressed = function() {
      if (p.key == 'p') {
        peripheralmode = !peripheralmode;
        if (peripheralmode) {
          for (var i = 0; i < flock.boids.length; i++) {
            let boid = flock.boids[i];
            boid.peripheral = 240;
          }
        } else {
          for (var i = 0; i < flock.boids.length; i++) {
            let boid = flock.boids[i];
            boid.peripheral = 360;
          }
        }
      }
      if (p.key == 'd') {
        console.log("D DETECTED")
        if (shouldDisplay) {
          shouldDisplay = false;
        } else {
          shouldDisplay = true;
        }
      }
      if (p.key == 'r') {
        console.log("R detected")
        p.background(249);
        flock = new Flock();
        for (var i = 0; i < 3; i++) {
          let b = new Boid(p.createVector(p.width/2 + p.random(0.001), p.height/2 + p.random(0.001)), 0.1, 1);
          flock.addBoid(b);
        }
      }
    }

    p.mousePressed = function() {
      let b = new Boid(p.createVector(p.mouseX, p.mouseY), 0.1, 1);
      flock.addBoid(b);
    }
  };
  new p5(sketch, container);
});
$("#boidModal").on('hidden.bs.modal', function() {
  document.getElementById("boidCanvas").remove();
});

$("#cellularAutomataModal").on('shown.bs.modal', function() {
  let container = document.getElementById("cellularAutomataContainer");
  if (document.getElementById("cellularAutomataCanvas")) {
    return;
  }
  let sketch = function(p) {
    let CA = function() {
      this.sideLength = 10;
      this.cells = [];
      this.ruleset = [];
      for (var i = 0; i < 8; i++) {
        this.ruleset.push(p.round(p.random(1)));
      }
      for (var i = 0; i < p.height / this.sideLength; i++) {
        this.cells.push([]);
        for (var j = 0; j < p.width/this.sideLength; j++) {
          this.cells[i].push(p.int(p.random(2)));
        }
      }
    };

    CA.prototype.generate = function() {
      let nextgen = [];
      for (var i = 0; i < this.cells.length; i++) {
        let left = this.cells[this.cells.length - 1][(i - 1 + this.cells[0].length) % this.cells[0].length];
        let me = this.cells[this.cells.length - 1][i];
        let right = this.cells[this.cells.length - 1][(i + 1) % this.cells[0].length];
        nextgen.push(this.rules(left, me, right));
      }
      for (var i = 0; i < this.cells.length - 1; i++) {
        this.cells[i] = this.cells[i + 1];
      }
      this.cells[this.cells.length - 1] = nextgen;
    };

    CA.prototype.rules = function(left, me, right) {
      let string = "" + left + me + right;
      let ruleIndex = parseFloat(string, 2);
      return this.ruleset[ruleIndex];
    };

    CA.prototype.drawGen = function() {
      for (var i = 0; i < this.cells.length; i++) {
        for (var j = 0; j < this.cells[0].length; j++) {
          if (this.cells[i][j] == 1) {
            p.fill(0, 255, 0);
          } else {
            p.fill(255, 0, 0);
          }
          let outerradius = (this.cells.length - i - 1)*(p.sqrt(p.height*p.height + p.width*p.width)/2)/this.cells.length;
          let innerradius = (this.cells.length - i)*(p.sqrt(p.height*p.height + p.width*p.width)/2)/this.cells.length;
          let innertheta = (this.cells[0].length - j)*p.TAU/this.cells[0].length;
          let outertheta = (this.cells[0].length - j - 1)*p.TAU/this.cells[0].length;
          p.push();
          p.translate(p.width/2, p.height/2);
          p.quad(innerradius * p.cos(innertheta), innerradius * p.sin(innertheta), innerradius * p.cos(outertheta), innerradius * p.sin(outertheta), outerradius * p.cos(outertheta), outerradius * p.sin(outertheta), outerradius * p.cos(innertheta), outerradius * p.sin(innertheta));
          p.pop();
        }
      }
    };

    let ca;

    p.setup = function() {
      p.noStroke();
      p.loop();
      console.log("CANVAS CREATED");
      canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 3/4);
      canvas.id("cellularAutomataCanvas");
      ca = new CA();
      p.frameRate(20);
    };

    p.draw = function() {
      if (!document.getElementById("cellularAutomataCanvas")) {
        p.noLoop();
      }
      ca.drawGen();
      ca.generate();
    };

    p.keyPressed = function() {
      if(p.key == 'r') {
        for (var i = 0; i < ca.cells.length; i++) {
          for (var j = 0; j < ca.cells[0].length; j++) {
            ca.cells[i][j] = p.int(p.random(2));
          }
        }
        for (var i = 0; i < 8; i++) {
          ca.ruleset[i] = p.round(p.random(1));
        }
      }
    }
  };
  new p5(sketch, container);
});

$("#cellularAutomataModal").on('hidden.bs.modal', function() {
  document.getElementById("cellularAutomataCanvas").remove()
});

$("#coronaModal").on('shown.bs.modal', function() {
  let container = document.getElementById("coronaContainer");
  if (document.getElementById("coronaCanvas")) {
    return;
  }
  let sketch = function(p) {

    let Graph = function() {
      this.current_index = 0;
      this.bar_width = 3;
      this.len = p.floor(p.width / this.bar_width);
      this.infectionData = [];
      this.recoveryData = [];
      this.deadData = [];
      for (var i = 0; i < this.len; i++) {
        this.infectionData.push(0);
        this.recoveryData.push(0);
        this.deadData.push(0);
      }

      this.infectionData[0] = 1;
    };

    Graph.prototype.update_data = function(community) {
      if (this.current_index < this.len - 1) {
        this.current_index += 1;
        this.infectionData[this.current_index] = community.infected;
        this.recoveryData[this.current_index] = community.recovered;
        this.deadData[this.current_index] = community.dead;
      } else {
        for (var i = 0; i < this.len - 1; i++) {
          this.infectionData[i] = this.infectionData[i+1];
          this.recoveryData[i] = this.recoveryData[i+1];
          this.deadData[i] = this.deadData[i+1];
        }
        this.infectionData[this.len - 1] = community.infected;
        this.recoveryData[this.len - 1] = community.recovered;
        this.deadData[this.len - 1] = community.dead;
      }
    };

    Graph.prototype.display = function(community) {
      p.fill(100);
      p.rect(0, 0, p.width, community.upper_margin);
      let total = community.numballs;
      for (var i = 0; i < this.current_index; i++) {
        p.noStroke();
        p.fill(255, 0, 255);
        p.rect(i*p.width/this.len, 0, this.bar_width, community.upper_margin*this.deadData[i]/total);
        p.fill(0, 255, 0);
        p.rect(i*p.width/this.len, community.upper_margin*this.deadData[i]/total, this.bar_width, community.upper_margin*this.recoveryData[i]/total);
        p.fill(0, 0, 255);
        p.rect(i*p.width/this.len, community.upper_margin*(this.recoveryData[i] + this.deadData[i])/total, this.bar_width, community.upper_margin*(total - this.recoveryData[i] - this.infectionData[i] - this.deadData[i])/total);
        p.fill(255, 0, 0);
        p.rect(i*p.width/this.len, community.upper_margin*(total - this.infectionData[i])/total, this.bar_width, community.upper_margin*this.infectionData[i]/total);
      }
    };

    let Community = function(numballs, rad, speed, upper_margin, danger_fraction, death_percentage) {
      this.infected = 1;
      this.recovered = 0;
      this.dead = 0;
      this.numballs = numballs;
      this.rad = rad;
      this.speed = speed;
      this.upper_margin = upper_margin;
      this.danger_fraction = danger_fraction;
      this.death_percentage = death_percentage;
      this.balls = [];
      for (var i = 0; i < this.numballs; i++) {
        this.balls.push(new Ball(this.speed, this.rad, p.random(0, p.width), p.random(this.upper_margin, p.height)));
        this.balls[i].mobile = false;
        if (i % danger_fraction == 0) {
          this.balls[i].mobile = true;
        }
      }
      this.balls[0].infected = true;
    };

    Community.prototype.update = function() {
      for (var i = 0; i < this.numballs; i++) {
        this.balls[i].update();
        if (this.balls[i].frames_infected >= 750/this.speed && this.balls[i].infected) {
          this.balls[i].infected = false;
          if (p.random(0, 100) < this.death_percentage) {
            this.balls[i].dead = true;
            this.dead++;
          } else {
            this.balls[i].recovered = true;
            this.recovered++;
          }
          this.infected--;
        }
      }
    };

    Community.prototype.display = function() {
      for (var i = 0; i < this.numballs; i++) {
        this.balls[i].display();
      }
    };


    Community.prototype.detect_collisions = function() {
      for (var i = 0; i < this.numballs; i++) {
        let balli = this.balls[i];
        if (balli.x < this.rad) {
          this.balls[i].x = this.rad;
          this.balls[i].velx = p.abs(balli.velx);
        } else if (balli.x > p.width - this.rad) {
          this.balls[i].x = p.width - this.rad;
          this.balls[i].velx = (-1) * p.abs(balli.velx)
        }
        if (balli.y < this.rad + this.upper_margin) {
          this.balls[i].y = this.rad + this.upper_margin;
          this.balls[i].vely = p.abs(balli.vely);
        } else if (balli.y > p.height - this.rad) {
          this.balls[i].y = p.height - this.rad;
          this.balls[i].vely = (-1) * p.abs(balli.vely)
        }
        for (var j = 0; j < this.numballs; j++) {
          if (i != j) {
          let ballj = this.balls[j];
          let dist = p.sqrt((ballj.x - balli.x)*(ballj.x - balli.x) + (ballj.y - balli.y)*(ballj.y - balli.y));
            if ((dist < 2 * this.rad) && !(ballj.dead || balli.dead)) {
              let coordvectorx1 = ballj.x - balli.x;
              let coordvectory1 = ballj.y - balli.y;
              let mag = p.sqrt(coordvectorx1*coordvectorx1 + coordvectory1*coordvectory1);
              coordvectorx1 = coordvectorx1/mag;
              coordvectory1 = coordvectory1/mag;
              let coordvectorx2 = -1 * coordvectory1;
              let coordvectory2 = coordvectorx1;
              let cofactor_coeff = (1/(coordvectorx1*coordvectory2 - coordvectorx2*coordvectory1));
              let ballivel1 = cofactor_coeff*(balli.velx*coordvectory2 + balli.vely*coordvectorx2);
              let ballivel2 = cofactor_coeff*(balli.vely*coordvectorx1 - balli.velx*coordvectory1);
              let balljvel1 = cofactor_coeff*(ballj.velx*coordvectory2 + ballj.vely*coordvectorx2);
              let balljvel2 = cofactor_coeff*(ballj.vely*coordvectorx1 - ballj.velx*coordvectory1);
              if (!balli.mobile) {
                balljvel1 = p.abs(balljvel1)*1.01;
              } else if (!ballj.mobile) {
                ballivel1 = -1 * p.abs(ballivel1)*1.01;
              } else {
                ballivel1 = -1 * p.abs(balljvel1);
                balljvel1 = p.abs(ballivel1);
              }
              this.balls[i].velx = coordvectorx1*ballivel1 + coordvectorx2*ballivel2;
              this.balls[i].vely = coordvectory1*ballivel1 + coordvectory2*ballivel2;
              this.balls[j].velx = coordvectorx1*balljvel1 + coordvectorx2*balljvel2;
              this.balls[j].vely = coordvectory1*balljvel1 + coordvectory2*balljvel2;
              if (balli.infected && !ballj.infected && !ballj.recovered) {
                this.balls[j].infected = true;
                this.infected += 1;
              }
              else if (ballj.infected && !balli.infected && !balli.recovered) {
                this.balls[i].infected = true;
                this.infected += 1;
              }
            }
          }
        }
      }
    };

    let Ball = function(speed, rad, x, y) {
      this.infected = false;
      this.recovered = false;
      this.dead = false;
      this.speed = speed;
      this.rad = rad;
      this.x = x;
      this.y = y;
      this.velx = p.random((-1)*this.speed, this.speed);
      this.vely = (p.round(p.random(0,1))*2 - 1)*p.sqrt(this.speed*this.speed - this.velx*this.velx);
      this.frames_infected = 0;
    };

    Ball.prototype.update = function() {
      this.velx = this.velx * this.speed / p.sqrt(this.velx*this.velx + this.vely * this.vely);
      this.vely = this.vely * this.speed / p.sqrt(this.velx*this.velx + this.vely * this.vely);
      if (!this.mobile || this.dead) {
        this.velx = 0;
        this.vely = 0;
      }
      this.x += this.velx;
      this.y += this.vely;
      if (this.infected) {
        this.frames_infected++;
      }
    };

    Ball.prototype.display = function() {
      p.noStroke();
      p.fill(125);
      if (this.infected) {
        p.fill(255, 0, 0);
      } else if (this.recovered) {
        p.fill(0, 255, 0);
      } else if (this.dead) {
        p.fill(255, 0, 255);
      }
    //  if (this.mobile) {
        //console.log(`MOBILE DISPLAYED ${this.x}, ${this.y}`);
    //  }
      p.ellipse(this.x, this.y, 2 * this.rad, 2 * this.rad);
    };

    let community;
    let graph;
    let current_frame;
    let over;
    let totalframes;

    p.setup = function() {
      p.loop();
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 3/4);
      canvas.id("coronaCanvas");
      init();
      over = true;
    }

    function init() {
      p.background(255);
      document.getElementById("speed").disabled = true;
      document.getElementById("danger_fraction").disabled = true;
      document.getElementById("death_percentage").disabled = true;
      document.getElementById("summary").innerHTML = "";
      community = new Community(100, 5, parseFloat(document.getElementById("speed").value), 100, parseFloat(document.getElementById("danger_fraction").value), parseFloat(document.getElementById("death_percentage").value));
      graph = new Graph();
      current_frame = 0;
      over = false;
      totalframes = 0;
    }

    p.draw = function() {
      if (!document.getElementById("coronaCanvas")) {
        p.noLoop();
      }
      if (!over) {
        p.fill(255);
        p.rect(0, community.upper_margin, p.width, p.height - community.upper_margin);
        community.detect_collisions();
        community.update();
        community.display();
        current_frame++;
        totalframes++;
        if (current_frame == (p.ceil(20 / community.speed))) {
          current_frame = 0;
          graph.update_data(community);
          graph.display(community);
          if (community.infected == 0) {
            document.getElementById("summary").innerHTML = `Summary:<br>Duration of infection: ${totalframes} frames<br>Deaths: ${community.dead}<br>Recovered: ${community.recovered}<br>Uninfected: ${community.numballs - community.dead - community.recovered}<br>Speed (0.1-5): ${document.getElementById("speed").value}<br>Responsibility (1-20): ${document.getElementById("danger_fraction").value}<br>Death Rate (0-100): ${document.getElementById("death_percentage").value}`;
            document.getElementById("speed").disabled = false;
            document.getElementById("danger_fraction").disabled = false;
            document.getElementById("death_percentage").disabled = false;
            over = true;
            p.background(255);
            p.fill(0);
            p.text("Click to Start", p.width/2, p.height/2)
            p.noLoop();
          }
        }
      } else {
        p.background(255);
        p.fill(0);
        p.text("Click to Start", p.width/2, p.height/2);
        document.getElementById("speed").disabled = false;
        document.getElementById("danger_fraction").disabled = false;
        document.getElementById("death_percentage").disabled = false;
        p.noLoop();
      }
    }

    p.mouseClicked = function() {
      if (over && p.mouseX > 0 && p.mouseX < p.width && p.mouseY > 0 && p.mouseY < p.height) {
        init();
        over = false;
        p.loop();
      }
    };
  };
  new p5(sketch, container);
});

$("#coronaModal").on('hidden.bs.modal', function() {
  document.getElementById("coronaCanvas").remove();
})

$("#flowModal").on('shown.bs.modal', function() {
  if (document.getElementById("flowCanvas")) {
    return;
  }

  let container = document.getElementById("flowContainer");
  let sketch = function(p) {

    let FlowField = function(r) {
      this.resolution = r;
      this.cols = p.width / r;
      this.rows = p.height / r;
      this.vectors = [];
      this.frames = 0;
      for (var i = 0; i < this.cols; i++) {
        this.vectors.push([]);
        for (var j = 0; j < this.rows; j++) {
          this.vectors[i].push(p.createVector(0, 0));
        }
      }
      this.randomxoffset = p.random(100000);
      this.randomyoffset = p.random(100000);
      this.randomtoffset = p.random(1000000);
    };

    FlowField.prototype.lookup = function(lookup) {
      let column = p.int(p.constrain(lookup.x / this.resolution, 0, this.cols - 1));
      let row = p.int(p.constrain(lookup.y / this.resolution, 0, this.rows - 1));
      return this.vectors[column][row].copy();
    };

    FlowField.prototype.setValues = function() {
      for (var i = 0; i < this.cols; i++) {
        for (var j = 0; j < this.rows; j++) {
          let theta = p.noise(i * 0.01 + this.randomxoffset, j * 0.01 + this.randomyoffset, this.frames * 0.01 + this.randomtoffset) * 2 * p.TAU;
          this.vectors[i][j] = p.createVector(p.cos(theta), p.sin(theta));
        }
      }
      this.frames++;
    };

    FlowField.prototype.render = function() {
      for (var i = 0; i < this.cols; i++) {
        for (var j = 0; j < this.rows; j++) {
          p.stroke(this.lookup(p.createVector(i * this.resolution, j * this.resolution)).x * 255, this.lookup(p.createVector(i * this.resolution, j * this.resolution)).y * 255, (this.lookup(p.createVector(i * this.resolution, j * this.resolution)).y + this.lookup(p.createVector(i * this.resolution, j * this.resolution)).x) * 122.5)
          p.line(i * this.resolution, j * this.resolution, i * this.resolution + 10 * this.lookup(p.createVector(i * this.resolution, j * this.resolution)).x, j * this.resolution + 10 * this.lookup(p.createVector(i * this.resolution, j * this.resolution)).y);
        }
      }
    }

    let Vehicle = function(x, y) {
      this.location = p.createVector(x, y);
      let theta = p.random(p.TAU);
      this.velocity = p.createVector(p.cos(theta), p.sin(theta));
      this.maxSpeed = parseFloat(document.getElementById("maxSpeed").value);
      this.maxForce = parseFloat(document.getElementById("maxForce").value);
      this.velocity.mult(this.maxSpeed)
    }

    Vehicle.prototype.seek = function(flow) {
      let desired = flow.lookup(p5.Vector.add(this.location, p5.Vector.mult(this.velocity, this.velocity.mag())));
      desired.normalize();
      desired.mult(this.maxSpeed);
      p.stroke(0);
      p.fill(255, 0, 255);
      p.ellipse(p5.Vector.add(this.location, p5.Vector.mult(this.velocity, this.velocity.mag())).x, p5.Vector.add(this.location, p5.Vector.mult(this.velocity, this.velocity.mag())).y, 10, 10);
      let steer = p5.Vector.sub(desired, this.velocity);
      steer.limit(this.maxForce);
      this.velocity.add(steer);
      this.velocity.limit(this.maxSpeed);
      this.location.add(this.velocity);
      if (this.location.x > p.width) {
        this.location.x = p.width;
        this.velocity.x = -1 * p.abs(this.velocity.x);
      } else if (this.location.x < 0) {
        this.location.x = 0;
        this.velocity.x = p.abs(this.velocity.x);
      }
      if (this.location.y > p.height) {
        this.location.y = p.height;
        this.velocity.y = -1 * p.abs(this.velocity.y);
      } else if (this.location.y < 0) {
        this.location.y = 0;
        this.velocity.y = p.abs(this.velocity.y);
      }
    }

    Vehicle.prototype.render = function() {
      p.push();
      p.translate(this.location.x, this.location.y);
      p.rotate(this.velocity.heading());
      p.stroke(0);
      p.fill(0, 255, 0);
      p.triangle(0, 0, -20, 5, -20, -5);
      p.pop();
    }

    let flowfield;
    let vehicle;

    p.setup = function() {
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 3/4);
      canvas.id("flowCanvas");
      p.loop();
      p.background(255);
      flowfield = new FlowField(10);
      vehicle = new Vehicle(p.random(p.width), p.random(p.height));
    }

    p.draw = function() {
      if (!document.getElementById("flowCanvas")) {
        p.noLoop();
      }
      p.background(255);
      flowfield.setValues();
      flowfield.render();
      vehicle.seek(flowfield);
      vehicle.render();
    }

    p.keyPressed = function() {
      if (p.key == "r") {
        flowfield = new FlowField(10);
        vehicle = new Vehicle(p.random(p.width), p.random(p.height));
      }
    };
  };

  new p5(sketch, container)
});

$("#flowModal").on('hidden.bs.modal', function() {
  document.getElementById("flowCanvas").remove();
});

$("#gameModal").on('shown.bs.modal', function() {
  if (document.getElementById("gameCanvas")) {
    return;
  }
  console.log("GAME MODAL SHOWN");
  let container = document.getElementById("gameContainer");
  let sketch = function(p) {

    let Cell = function(x, y, w) {
      this.x = x;
      this.y = y;
      this.w = w;
      this.state = p.floor(p.random(6)) / 5;
      this.previous = this.state;
    };

    Cell.prototype.savePrevious = function() {
      this.previous = this.state
    };

    Cell.prototype.newState = function(s) {
      this.state = s;
    };

    Cell.prototype.display = function() {
      p.fill(this.state * 255);
      p.stroke(0);
      p.rect(this.x, this.y, this.w, this.w);
    };

    let GOL = function(w) {
      this.w = w;
      this.columns = p.floor(p.width / w);
      this.rows = p.floor(p.height / w);
      this.board = [];
      this.init();
    };

    GOL.prototype.init = function() {
      this.board = [];
      for (var i = 0; i < this.columns; i++) {
        this.board.push([])
        for (var j = 0; j < this.rows; j++) {
          this.board[i].push(new Cell(i * this.w, j * this.w, this.w));
          this.board[i][j].savePrevious();
        }
      }
    };

    GOL.prototype.generate = function() {
      for (var i = 0; i < this.columns; i++) {
        for (var j = 0; j < this.rows; j++) {
          this.board[i][j].savePrevious();
        }
      }

      for (var x = 0; x < this.columns; x++) {
        for (var y = 0; y < this.rows; y++) {
          let neighbors = 0;
          for (var i = -1; i <=1; i++) {
            for (var j = -1; j <= 1; j++) {
              let neighborcol = (x + i + this.columns) % this.columns;
              let neighborrow = (y + j + this.rows) % this.rows;
              neighbors += this.board[neighborcol][neighborrow].previous;
            }
          }
          neighbors -= this.board[x][y].previous;

          if (this.board[x][y].state > 0.1 && neighbors < 2 && p.random(1) > 0.4) {
              this.board[x][y].newState(this.board[x][y].state - 0.1);
          } else if (this.board[x][y].state > 0.1 && neighbors > 3 && p.random(1) > 0.2) {
            this.board[x][y].newState(this.board[x][y].state - 0.1);
          } else if (this.board[x][y].state > 0 && this.board[x][y].state < 1) {
            this.board[x][y].newState(this.board[x][y].state + 0.1);
          } else if (this.board[x][y].state == 0 && neighbors == 3) {
            this.board[x][y].newState(1);
          }
        }
      }
    };

    GOL.prototype.display = function() {
      for (var i = 0; i < this.columns; i++) {
        for (var j = 0; j < this.rows; j++) {
          this.board[i][j].display();
        }
      }
    };

    let spacePressed;
    let gol;

    p.setup = function() {
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 3/4);
      canvas.id("gameCanvas");
      p.loop();
      gol = new GOL(16);
      spacePressed = false;
    };

    p.draw = function() {
      p.background(255);
      console.log("DRAWN");
      if (!document.getElementById("gameCanvas")) {
        p.noLoop();
      }
      if (!spacePressed) {
        gol.generate();
      }
      gol.display();
    };

    p.keyPressed = function() {
      if (p.key == "p") {
        spacePressed = !spacePressed;
      } else if (p.key == "r") {
        gol.init();
      } else if (p.key == "d") {
        for (var i = 0; i < gol.columns; i++) {
          for (var j = 0; j < gol.rows; j++) {
            gol.board[i][j].state = 0;
            gol.board[i][j].previous = 0;
          }
        }
      } else if (p.key == 'a') {
        for (var i = 0; i < gol.columns; i++) {
          for (var j = 0; j < gol.rows; j++) {
            gol.board[i][j].state = 1;
            gol.board[i][j].previous = 1;
          }
        }
      }
    }

    p.mousePressed = function() {
      if (p.mouseX > 0 && p.mouseX < p.width && p.mouseY > 0 && p.mouseY < p.height) {
        if (spacePressed) {
          let col = p.floor(p.mouseX*gol.columns/p.width);
          let row = p.floor(p.mouseY*gol.rows/p.height);
          gol.board[col][row].state = ((gol.board[col][row].state*10 + 2) % 11)/10;
          gol.board[col][row].previous = gol.board[col][row].state;
        }
      }
    };
  };
  new p5(sketch, container);
});

$("#gameModal").on('hidden.bs.modal', function() {
  document.getElementById("gameCanvas").remove();
});

$("#glassModal").on('shown.bs.modal', function() {
  if (document.getElementById("glassCanvas")) {
    return;
  }
  let container = document.getElementById("glassContainer");
  let sketch = function(p) {

      let systems;
      let glassitems;
      let hammering;
      let hammeringcount;
      let offsetw;

      p.setup = function() {
        p.loop();
        offsetw = container.offsetWidth;
        let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 2 / 3);
        canvas.id("glassCanvas");
        p.background(255);
        p.strokeWeight(2);
        init();
        hammering = false;
        hammeringcount = 0;
      };

      function drawHammer() {
        p.push();
        let thetaoffset = -1 * p.PI / 6;
        if (hammering) {
          thetaoffset = p.abs((hammeringcount - 10)) * (-1 * p.PI / 60);
          hammeringcount++;
          if (hammeringcount == 21) {
            hammering = false;
            hammeringcount = 0;
          }
        }
        p.translate(p.constrain(p.mouseX, 0, p.width), p.constrain(p.mouseY + 60 * offsetw / 1200, 60 * offsetw / 1200, p.height + 60 * offsetw / 1200));
        p.rotate(p.PI/3 + thetaoffset);
        p.stroke(0);
        p.fill(165, 42, 42);
        p.rect(-33 * offsetw / 1200, 5 * offsetw / 1200, 10 * offsetw / 1200, -60 * offsetw / 1200);
        p.fill(175);
        p.rect(-48 * offsetw / 1200, -65 * offsetw / 1200, 40 * offsetw / 1200, 10 * offsetw / 1200);
        p.pop();
        p.fill(0, 255, 255, 120);
      }

      function init() {
        systems = [];
        glassitems = [];
        let numwindows = p.floor(p.random(5, 10));
        for (var i = 0; i < numwindows; i++) {
          glassitems.push(new GlassItem(p.random(p.width - 200 * offsetw / 1200), p.random(p.height - 200 * offsetw / 1200), p.random(100 * offsetw / 1200, 250 * offsetw / 1200), p.random(100 * offsetw / 1200, 250 * offsetw / 1200)));
        }
      }

      p.draw = function() {
        if (!document.getElementById("glassCanvas")) {
          p.noLoop();
        }
        p.background(255);
        for (let system of systems) {
          system.run();
        }
        for (let item of glassitems) {
          item.render();
        }
        let it = systems[Symbol.iterator]();
        let ps = it.next()
        let counter = 0;
        while (!ps.done) {
          if (ps.value.particles.length < 1) {
            systems.splice(counter, 1);
          } else {
            counter++;
          }
          ps = it.next();
        }
        let iterator = glassitems[Symbol.iterator]();
        let item = iterator.next();
        counter = 0;
        while (!item.done) {
          if (!item.value.unclicked) {
            glassitems.splice(counter, 1);
          } else {
            counter++;
          }
          item = iterator.next();
        }
        drawHammer();
      };

      let GlassItem = function(x, y, w, h) {
        this.posx = x;
        this.posy = y;
        this.boxw = w;
        this.boxh = h;
        this.unclicked = true;
      };

      GlassItem.prototype.render = function() {
        if (this.unclicked) {
          p.stroke(0);
          p.fill(0, 255, 255, 120);
          p.rect(this.posx, this.posy, this.boxw, this.boxh);
        }
      };

      let ParticleSystem = function(x, y, boxwidth, boxheight) {
        document.getElementById("glassAudio").play();
        this.particles = [];
        for (var i = 0; i < 15; i++) {
          for (var j = 0; j < 15; j++) {
            let xcoord = i * boxwidth / 15 + x;
            let ycoord = j * boxheight / 15 + y;
            let radius = p.createVector(xcoord - x - boxwidth / 2, ycoord - y - boxheight / 2);
            this.particles.push(new Particle(xcoord, ycoord, radius.x / 20 + p.random(-5, 5), radius.y/20 + p.random(-5, 5)));
          }
        }
      };

      ParticleSystem.prototype.run = function() {
        let it = this.particles[Symbol.iterator]();
        let p = it.next();
        let counter = 0;
        while(!p.done) {
          p.value.run();
          if (p.value.isDead()) {
            this.particles.splice(counter, 1);
          } else {
            counter++;
          }
          p = it.next();
        }
      };

      let Particle = function(x, y, vox, voy) {
        this.position = p.createVector(x, y);
        this.velocity = p.createVector(vox, voy);
        this.acceleration = p.createVector(0, 0.5);
        this.vertices = [];
        let numsides = p.floor(p.random(3, 12));
        for (var i = 0; i < numsides; i++) {
          let r = p.random(5, 15);
          let theta = p.TAU * i / numsides;
          this.vertices.push(p.createVector(r * p.cos(theta), r * p.sin(theta)));
        }
        this.angle = 0;
        this.angularvel = p.random(-0,01, 0.01);
        this.angularacc = p.random(-0.005, 0.005);
        this.lifespan = 255;
      };

      Particle.prototype.run = function() {
        this.update();
        this.render();
      };

      Particle.prototype.update = function() {
        this.velocity.add(this.acceleration);
        this.position.add(this.velocity);
        this.angularvel += this.angularacc;
        this.angle += this.angularvel;
        this.lifespan -= 2;
      };

      Particle.prototype.isDead = function() {
        return (this.lifespan < 0);
      }

      Particle.prototype.render = function() {
        p.stroke(0);
        p.strokeWeight(2);
        p.fill(0, 255, 255, 120);
        p.push();
        p.translate(this.position.x, this.position.y);
        p.rotate(this.angle);
        p.beginShape();
        for (let vertex of this.vertices) {
          p.vertex(vertex.x, vertex.y);
        }
        p.endShape(p.CLOSE);
        p.pop();
      };

      p.mouseClicked = function() {
        hammering = true;
        for (let item of glassitems) {
          if (((p.mouseX > item.posx) && (p.mouseX < item.posx + item.boxw)) && ((p.mouseY > item.posy) && (p.mouseY < item.posy + item.boxh))) {
            item.unclicked = false;
            systems.push(new ParticleSystem(item.posx, item.posy, item.boxw, item.boxh));
          }
        }
      };

      p.keyPressed = function() {
        if (p.key == "r") {
          init();
        }
      };
  };
  new p5(sketch, container);
});

$("#glassModal").on('hidden.bs.modal', function() {
  document.getElementById("glassCanvas").remove();
})

$("#hurricaneModal").on('shown.bs.modal', function() {
  if (document.getElementById("hurricaneCanvas")) {
    return;
  }
  let container = document.getElementById("hurricaneContainer");
  let sketch = function(p) {
    let windParticles;
    let balls;
    let offsetw;

    p.setup = function() {
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth);
      canvas.id("hurricaneCanvas");
      offsetw = container.offsetWidth;
      p.background(255);
      p.loop();
      windParticles = [];
      balls = []
      for (var i = 0; i < 1000; i++) {
        windParticles.push(new WindParticle(20 * offsetw / 500, p.random(p.TAU), i * 10))
      }
      for (var i = 0; i < 20; i++) {
        balls.push(new Ball());
      }
    };

    p.draw = function() {
      if (!document.getElementById("hurricaneCanvas")) {
        p.noLoop();
      }
      p.background(255);
      for (var i = 0; i < windParticles.length; i++) {
        windParticles[i].update();
        windParticles[i].render();
      }
      p.strokeWeight(0.5 * (p.sqrt(2) - 1) * offsetw);
      p.stroke(0, 125);
      p.fill(255, 0);
      p.ellipse(p.width/2, p.height/2, (0.5 * p.sqrt(2) + 0.5) * offsetw, (0.5 * p.sqrt(2) + 0.5) * offsetw);
      for (var i = 0; i < balls.length; i++) {
        balls[i].update();
        balls[i].render();
      }
    }

    let WindParticle = function(r, theta, frameoffset) {
      this.angle = theta;
      this.radius = r;
      this.location = p.createVector(r * p.cos(theta), r * p.sin(theta));
      for (var i = 0; i < frameoffset; i++) {
        this.update();
      }
    }

    WindParticle.prototype.update = function() {
      let dtheta;
      let windspeed = (20 * offsetw / 500 - this.radius) / (100 * offsetw) + 0.02;
      dtheta = p.PI/this.radius;
      let dr = windspeed / dtheta;
      this.angle += dtheta;
      this.radius += dr;
      if (p.abs(this.location.x) > p.width/2 && p.abs(this.location.y) > p.height/2) {
        this.radius = 20 * offsetw / 500;
      }
      this.location.x = this.radius * p.cos(this.angle);
      this.location.y = this.radius * p.sin(this.angle);
    }

    WindParticle.prototype.render = function() {
      p.strokeWeight(2);
      p.stroke(0);
      p.fill(0, 0, 255);
      p.ellipse(p.width / 2 + this.location.x, p.height / 2 + this.location.y, 10 * offsetw / 500, 10 * offsetw / 500)
    }

    let Ball = function() {
      let randomnum = p.random(1);
      if (randomnum > 0.75) {
        this.location = p.createVector((p.random(100) + 20)*offsetw/500, (p.random(100) + 20)*offsetw/500);
      } else if (randomnum > 0.5) {
        this.location = p.createVector(-1 * (p.random(100) + 20)*offsetw/500, -1 * (p.random(100) + 20)*offsetw/500);
      } else if (randomnum > 0.25) {
        this.location = p.createVector(-1 * (p.random(100) + 20)*offsetw/500, (p.random(100) + 20)*offsetw/500);
      } else {
        this.location = p.createVector((p.random(100) + 20)*offsetw/500, -1 * (p.random(100) + 20)*offsetw/500);
      }
      this.velocity = p.createVector(0, 0);
      this.radius = this.location.mag();
      this.angle = p.atan2(this.location.y, this.location.x);
    };

    Ball.prototype.update = function() {
      this.radius = this.location.mag();
      if (this.radius < 20 * offsetw/500) {
        if (this.radius == 0) {
          this.location == p.createVector(20 * offsetw / 500, 0);
        } else {
          this.location.normalize();
          this.location.mult(20 * offsetw / 500);
        }
        this.radius = 20 * offsetw/500;
      }
      this.angle = p.atan2(this.location.y, this.location.x);
      let dtheta = p.PI / this.radius;
      let windspeed = (20 * offsetw / 500 - this.radius) / (100 * offsetw) + 0.02;
      let dr = windspeed / dtheta;
      let rfinal = this.radius + dr;
      let anglefinal = this.angle + dtheta;
      let wind = p.createVector(rfinal * p.cos(anglefinal) - this.radius * p.cos(this.angle), rfinal * p.sin(anglefinal) - this.radius * p.sin(this.angle));
      let vrelative = p.createVector(this.velocity.x - wind.x, this.velocity.y - wind.y);
      let windForce = vrelative.mult(-0.005);
      p.strokeWeight(2);
      p.stroke(0);
      if (this.location.x > p.width / 2) {
        this.location.x = p.width / 2;
        this.velocity.x = -1 * this.velocity.x;
      } else if (this.location.x < -1 * p.width / 2) {
        this.location.x = -1 * p.width / 2;
        this.velocity.x = -1 * this.velocity.x;
      }
      if (this.location.y > p.height / 2) {
        this.location.y = p.height / 2;
        this.velocity.y = -1 * this.velocity.y;
      } else if (this.location.y < -1 * p.height / 2) {
        this.location.y = -1 * p.height / 2;
        this.velocity.y = -1 * this.velocity.y;
      }
      if (this.location.mag() > p.width / 2) {
        this.location.normalize()
        this.location.mult(p.width / 2);
        let velox = this.velocity.x;
        this.velocity.x = this.velocity.x*p.cos(2*this.angle) + p.sin(2*this.angle)*this.velocity.y;
        this.velocity.y = p.sin(2*this.angle)*velox - p.cos(2*this.angle)*this.velocity.y;
        this.velocity.mult(-1);
      }
      this.velocity.add(windForce);
      this.location.add(this.velocity);
    };

    Ball.prototype.render = function() {
      p.stroke(0);
      p.strokeWeight(2);
      p.fill(255, 0, 0);
      p.ellipse(this.location.x + p.width / 2, this.location.y + p.height / 2, 10 * offsetw/500, 10 * offsetw/500);
    };

    p.keyPressed = function() {
      if (p.key == 'r') {
         for (var i = 0; i < balls.length; i++) {
           let angle = p.random(p.TAU);
           balls[i].location.x = 80*offsetw/500*p.cos(angle);
           balls[i].location.y = 80*offsetw/500*p.sin(angle);
           balls[i].velocity.x = 5*offsetw/500*p.cos(angle);
           balls[i].velocity.y = 5*offsetw/500*p.sin(angle);
        }
      }
    };
  };
  new p5(sketch, container);
});

$("#hurricaneModal").on('hidden.bs.modal', function() {
  document.getElementById("hurricaneCanvas").remove();
});

$("#obstacleModal").on('shown.bs.modal', function() {
  if (document.getElementById("obstacleCanvas")) {
    return;
  }
  let container = document.getElementById("obstacleContainer");
  let sketch = function(p) {

    let DNA = function(newgenes) {
      if (newgenes != undefined) {
        console.log("NOT UNDEFINED");
        this.genes = newgenes;
        return;
      }
      console.log("UNDEFINED");
      this.genes = [];
      this.maxforce = 0.1;
      for (var i = 0; i < lifetime; i++) {
        let angle = p.random(2 * p.PI);
        console.log(angle);
        let new_vector = p.createVector(p.cos(angle), p.sin(angle));
        console.log(new_vector);
        this.genes.push(new_vector);
        this.genes[i].mult(p.random(0, this.maxforce));
      }

      this.genes[0].normalize();
      console.log(this.genes);
    }

    DNA.prototype.crossover = function(partner) {
      let child = [];
      for (var i = 0; i < this.genes.length; i++) {
        if (p.random(1) > 0.5) {
          child.push(this.genes[i]);
        } else {
          child.push(partner.genes[i]);
        }
      }
      let newgenes = new DNA(child);
      return newgenes;
    };

    DNA.prototype.mutate = function(m) {
      for (var i = 0; i < this.genes.length; i++) {
        if (p.random(1) < m) {
          let angle = p.random(2 * p.PI);
          this.genes[i] = p.createVector(p.cos(angle), p.sin(angle));
          this.genes[i].mult(p.random(0, this.maxforce));
        }
      }
      this.genes[0].normalize();
    };

    let Rocket = function(l, dna, totalRockets) {
      this.acceleration = p.createVector(0, 0);
      this.velocity = p.createVector(0, 0);
      this.position = l.copy();
      this.r = 4;
      this.dna = dna;
      this.finishTime = 0;
      this.geneCounter = 0;
      this.recordDist = 100000;
      this.hitObstacle = false;
      this.hitTarget = false;
    };

    Rocket.prototype.fitness = function() {
      if (this.recordDist < 1) {
        this.recordDist = 1;
      }
      this.fitness = 1 / (this.finishTime * this.recordDist);
      this.fitness = p.pow(this.fitness, 4);
      if (this.hitObstacle) {
        this.fitness *= 0.01;
      }
      if (this.hitTarget) {
        this.fitness *= 5;
      }
    };

    Rocket.prototype.run = function(os) {
      if (!this.hitObstacle && !this.hitTarget) {
        this.applyForce(this.dna.genes[this.geneCounter]);
        this.geneCounter = (this.geneCounter + 1) % this.dna.genes.length;
        this.update();
        this.obstacles(os);
      }
      if (!this.hitObstacle) {
        this.display();
      }
    };

    Rocket.prototype.checkTarget = function() {
      let d = p.dist(this.position.x, this.position.y, target.position.x, target.position.y);
      if (d < this.recordDist) {
        this.recordDist = d;
      }
      if (target.contains(this.position) && !this.hitTarget) {
        this.hitTarget = true;
      } else if (!this.hitTarget) {
        this.finishTime++;
      }
    };

    Rocket.prototype.obstacles = function(os) {
      for (let obs of os) {
        if (obs.contains(this.position)) {
          this.hitObstacle = true;
        }
      }
    }

    Rocket.prototype.applyForce = function(f) {
      this.acceleration.add(f);
    }

    Rocket.prototype.update = function() {
      this.velocity.add(this.acceleration);
      this.position.add(this.velocity);
      this.acceleration.mult(0);
    }

    Rocket.prototype.display = function() {
      //background(255,0,0);
      let theta = this.velocity.heading() + p.PI/2;
      p.fill(200, 100);
      p.stroke(0);
      p.strokeWeight(1);
      p.push();
      p.translate(this.position.x, this.position.y);
      p.rotate(theta);

      // Thrusters
      p.rectMode(p.CENTER);
      p.fill(0);
      p.rect(-this.r/2, this.r*2, this.r/2, this.r);
      p.rect(this.r/2, this.r*2, this.r/2, this.r);

      // Rocket body
      p.fill(175);
      p.beginShape(p.TRIANGLES);
      p.vertex(0, -this.r*2);
      p.vertex(-this.r, this.r*2);
      p.vertex(this.r, this.r*2);
      p.endShape();

      p.pop();
    };

    Rocket.prototype.getFitness = function() {
      return this.fitness;
    };

    Rocket.prototype.getDNA = function() {
      return this.dna;
    };

    Rocket.prototype.stopped = function() {
      return this.hitObstacle;
    }

    let Population = function(m, num) {
      this.mutationRate = m;
      this.population = [];
      this.matingPool = [];
      this.generations = 0;
      for (var i = 0; i < num; i++) {
        let position = p.createVector(p.width / 2, p.height - 20);
        this.population.push(new Rocket(position, new DNA(), num));
      }
    };

    Population.prototype.live = function(os) {
      for (var i = 0; i < this.population.length; i++) {
        this.population[i].checkTarget();
        this.population[i].run(os);
      }
    };

    Population.prototype.targetReached = function() {
      for (var i = 0; i < this.population.length; i++) {
        return (this.population[i].hitTarget);
      }
    };

    Population.prototype.fitness = function() {
      for (var i = 0; i < this.population.length; i++) {
        this.population[i].fitness();
      }
    }

    Population.prototype.selection = function() {
      this.matingPool = [];
      let maxFitness = this.getMaxFitness();
      for (var i = 0; i < this.population.length; i++) {
        let fitnessNormal = this.population[i].getFitness()/maxFitness;
        let n = p.round(fitnessNormal * 100);
        for (var j = 0; j < n; j++) {
          this.matingPool.push(this.population[i]);
        }
      }
    }

    Population.prototype.reproduction = function() {
      for (var i = 0; i < this.population.length; i++) {
        let m = p.floor(p.random(this.matingPool.length));
        let d = p.floor(p.random(this.matingPool.length));
        let mom = this.matingPool[m];
        let dad = this.matingPool[d];
        let momgenes = mom.getDNA();
        let dadgenes = dad.getDNA();
        let childgenes = momgenes.crossover(dadgenes);
        childgenes.mutate(this.mutationRate);
        let position = p.createVector(p.width/2, p.height - 20);
        this.population[i] = new Rocket(position, childgenes, this.population.length);
      }
      this.generations++;
    }

    Population.prototype.getGenerations = function() {
      return this.generations;
    }

    Population.prototype.getMaxFitness = function() {
      let record = 0;
      for (var i = 0; i < this.population.length; i++) {
        if (this.population[i].getFitness() > record) {
          record = this.population[i].getFitness();
        }
      }
      return record;
    }

    let Obstacle = function(x, y, w, h) {
      this.position = p.createVector(x, y);
      this.w = w;
      this.h = h;
    }

    Obstacle.prototype.display = function() {
      p.stroke(0);
      p.fill(175);
      p.strokeWeight(2);
      p.rectMode(p.CORNER);
      p.rect(this.position.x, this.position.y, this.w, this.h);
    }

    Obstacle.prototype.contains = function(spot) {
      return (spot.x > this.position.x && spot.x < this.position.x + this.w && spot.y > this.position.y && spot.y < this.position.y + this.h);
    }

    let lifetime;  // How long should each generation live

    let population;  // Population

    let lifecycle;          // Timer for cycle of generation
    let recordtime;         // Fastest time to target

    let target;        // Target position

    //int diam = 24;          // Size of target

    let obstacles;  //an array list to keep track of all the obstacles!

    p.setup = function() {
      p.loop();
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 9 / 16);
      canvas.id("obstacleCanvas");
      lifetime = 300;
      lifecycle = 0;
      recordtime = lifetime;
      target = new Obstacle(p.width/2 - 12, 24, 24, 24);
      let mutationRate = 0.01;
      population = new Population(mutationRate, 50);
      obstacles = [];
      obstacles.push(new Obstacle(0, 0, 1, p.height));
      obstacles.push(new Obstacle(p.width - 1, 0, 1, p.height));
      obstacles.push(new Obstacle(0, p.height - 1, p.width, 1));
      obstacles.push(new Obstacle(0, 0, p.width, 1));
    };

    p.draw = function() {
      if (!document.getElementById("obstacleCanvas")) {
        p.noLoop();
      }
      p.background(255);
      target.display();
      if (lifecycle < lifetime) {
        population.live(obstacles);
        if ((population.targetReached()) && lifecycle < recordtime) {
          recordtime = lifecycle;
        }
        lifecycle++;
      } else {
        lifecycle = 0;
        population.fitness();
        population.selection();
        population.reproduction();
      }

      for (let obs of obstacles) {
        obs.display();
      }
      p.fill(0);
      p.strokeWeight(1);
      p.text("Generation #: " + population.getGenerations(), 10, 18);
      p.text("Cycles left: " + (lifetime-lifecycle), 10, 36);
      p.text("Record cycles: " + recordtime, 10, 54);
    };

    p.keyPressed = function() {
      if (p.key == "m") {
        target.position.x = p.mouseX;
        target.position.y = p.mouseY;
        recordtime = lifetime;
      }
    };


    let tempobstacle;
    let dragging = false;

    p.mouseDragged = function() {
      if (!dragging) {
        tempobstacle = new Obstacle(p.mouseX, p.mouseY, 0, 0);
      }
      dragging = true;
    }

    p.mouseReleased = function() {
      if (dragging) {
        tempobstacle.w = p.mouseX - tempobstacle.position.x;
        tempobstacle.h = p.mouseY - tempobstacle.position.y;
        obstacles.push(tempobstacle);
      }
      dragging = false;
    };
  };
  new p5(sketch, container);
});

$("#obstacleModal").on('hidden.bs.modal', function() {
  document.getElementById("obstacleCanvas").remove();
});

$("#othelloModal").on('shown.bs.modal', function() {
  if (document.getElementById("othelloCanvas")) {
    return;
  }
  let container = document.getElementById("othelloContainer");
  let sketch = function(p) {
    let board;
    let player1;
    let player2;

    p.setup = function() {
      p.loop();
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth);
      canvas.id("othelloCanvas");
      p.background(255);
      board = new Board(8);
      player1 = new Player(1, true);
      player2 = new Player(2, false);
      board.render();
    };

    p.draw = function() {
      if (!board.game_done) {
        board.render();
        player1.display_mouse();
        player2.display_mouse();
      }
    };

    let Player = function(id, my_turn) {
      this.id = id;
      this.my_turn = my_turn;
    };

    Player.prototype.display_mouse = function() {
      if (this.my_turn) {
        let box_i = p.constrain(p.floor(p.mouseY * board.size / p.height), 0, board.size - 1);
        let box_j = p.constrain(p.floor(p.mouseX * board.size / p.width), 0, board.size - 1);
        if (board.board_array[box_i][box_j] == 0) {
          if (this.id == 1) {
            p.fill(255, 0, 0);
          } else {
            p.fill(0, 255, 0);
          }
          p.ellipse(p.width * (2 * box_j + 1) / (2 * board.size), p.height * (2 * box_i + 1) / (2 * board.size), p.width / board.size, p.height / board.size);
        }
      }
    };

    Player.prototype.attempt_move = function() {
      if (this.my_turn) {
        let box_i = p.constrain(p.floor(p.mouseY * board.size / p.height), 0, board.size - 1);
        let box_j = p.constrain(p.floor(p.mouseX * board.size / p.width), 0, board.size - 1);
        if (board.is_valid(box_i, box_j, this.id)) {
          board.update(box_i, box_j, this.id);
        }
      }
    };

    let Board = function(size) {
      this.size = size;
      this.game_done = false;
      this.num_moves = 0;
      this.board_array = [];
      for (var i = 0; i < this.size; i++) {
        this.board_array.push([]);
        for (var j = 0; j < this.size; j++) {
          this.board_array[i].push(0);
        }
      }
    };

    Board.prototype.valid_moves_remain = function(id) {
      if (this.num_moves < 4) {
        return true;
      }
      for (var i = 0; i < this.size; i++) {
        for (var j = 0; j < this.size; j++) {
          if (this.is_valid(i, j, id)) {
            return true;
          }
        }
      }
      return false;
    };

    Board.prototype.update = function(index_i, index_j, id) {
      this.board_array[index_i][index_j] = id;
      if (index_i != this.size - 1) {
        if (this.board_array[index_i + 1][index_j] == id % 2 + 1) {
          for(var i = 2; index_i + i < this.size; i++) {
            if (this.board_array[index_i + i][index_j] == id) {
              for(var j = 1; j < i; j++) {
                this.board_array[index_i + j][index_j] = id;
              }
              break;
            }
          }
        }
        if (index_j != 0) {
          if (this.board_array[index_i + 1][index_j - 1] == id % 2 + 1) {
            for(var i = 2; index_i + i < this.size && index_j - i >= 0; i++) {
              if (this.board_array[index_i + i][index_j - i] == id) {
                for(var j = 1; j < i; j++) {
                  this.board_array[index_i + j][index_j - j] = id;
                }
                break;
              }
            }
          }
        }
        if (index_j != this.size - 1) {
          if (this.board_array[index_i + 1][index_j + 1] == id % 2 + 1) {
            for(var i = 2; index_i + i < this.size && index_j + i < this.size; i++) {
              if (this.board_array[index_i + i][index_j + i] == id) {
                for(var j = 1; j < i; j++) {
                  this.board_array[index_i + j][index_j + j] = id;
                }
                break;
              }
            }
          }
        }
      }
      if (index_i != 0) {
        if (this.board_array[index_i - 1][index_j] == id % 2 + 1) {
          for(var i = 2; index_i - i >= 0; i++) {
            if (this.board_array[index_i - i][index_j] == id) {
              for(var j = 1; j < i; j++) {
                this.board_array[index_i - j][index_j] = id;
              }
              break;
            }
          }
        }
        if (index_j != 0) {
          if (this.board_array[index_i - 1][index_j - 1] == id % 2 + 1) {
            for(var i = 2; index_i - i >= 0 && index_j - i >= 0; i++) {
              if (this.board_array[index_i - i][index_j - i] == id) {
                for(var j = 1; j < i; j++) {
                  this.board_array[index_i - j][index_j- j] = id;
                }
                break;
              }
            }
          }
        }
        if (index_j != this.size - 1) {
          if (this.board_array[index_i - 1][index_j + 1] == id % 2 + 1) {
            for(var i = 2; index_i - i >= 0 && index_j + i < this.size; i++) {
              if (this.board_array[index_i - i][index_j + i] == id) {
                for(var j = 1; j < i; j++) {
                  this.board_array[index_i - j][index_j + j] = id;
                }
                break;
              }
            }
          }
        }
      }
      if (index_j != this.size - 1) {
         if (this.board_array[index_i][index_j + 1] == id % 2 + 1) {
           for(var i = 2; index_j + i < this.size; i++) {
             if (this.board_array[index_i][index_j + i] == id) {
               for(var j = 1; j < i; j++) {
                 this.board_array[index_i][index_j + j] = id;
               }
               break;
             }
           }
         }
      }
      if (index_j != 0) {
         if (this.board_array[index_i][index_j - 1] == id % 2 + 1) {
           for(var i = 2; index_j - i >= 0; i++) {
             if (this.board_array[index_i][index_j - i] == id) {
               for(var j = 1; j < i; j++) {
                 this.board_array[index_i][index_j - j] = id;
               }
               break;
             }
           }
         }
      }
      this.num_moves++;
      if (id == 1) {
        if (this.valid_moves_remain(2)) {
          player1.my_turn = false;
          player2.my_turn = true;
        } else {
          this.game_done = true
        }
      } else {
        if (this.valid_moves_remain(1)) {
          player1.my_turn = true;
          player2.my_turn = false;
        } else {
          this.game_done = true;
        }
      }
      if (this.game_done) {
        let num_green = 0;
        let num_red = 0;
        for (var i = 0; i < this.size; i++) {
          for (var j = 0; j < this.size; j++) {
            if (this.board_array[i][j] == 1) {
              num_red++;
            } else if (this.board_array[i][j] == 2) {
              num_green++;
            }
          }
        }
        p.textSize(32);
        p.fill(255);
        p.textAlign(p.CENTER);
        if (num_red > num_green) {
          p.background(255, 0, 0);
          p.text("RED WINS", p.width/2, p.eight/2);
        } else if (num_green > num_red) {
          p.background(0, 255, 0);
          p.text("GREEN WINS", p.width/2, p.height/2);
        } else {
          p.background(0, 0, 255);
          p.text("IT'S A TIE", p.width/2, p.height/2);
        }
      }
    };

    Board.prototype.is_valid = function(index_i, index_j, id) {
      if (this.board_array[index_i][index_j] != 0) {
        return false;
      }
      if (this.num_moves < 4) {
        return ((index_i == 3 || index_i == 4) && (index_j == 3 || index_j == 4));
      }
      if (index_i != this.size - 1) {
        if (this.board_array[index_i + 1][index_j] == id % 2 + 1) {
          for(var i = 2; index_i + i < this.size; i++) {
            if (this.board_array[index_i + i][index_j] == id) {
              return true;
            }
          }
        }
        if (index_j != 0) {
          if (this.board_array[index_i + 1][index_j - 1] == id % 2 + 1) {
            for(var i = 2; index_i + i < this.size && index_j - i >= 0; i++) {
              if (this.board_array[index_i + i][index_j - i] == id) {
                return true;
              }
            }
          }
        }
        if (index_j != this.size - 1) {
          if (this.board_array[index_i + 1][index_j + 1] == id % 2 + 1) {
            for(var i = 2; index_i + i < this.size && index_j + i < this.size; i++) {
              if (this.board_array[index_i + i][index_j + i] == id) {
                return true;
              }
            }
          }
        }
      }
      if (index_i != 0) {
        if (this.board_array[index_i - 1][index_j] == id % 2 + 1) {
          for(var i = 2; index_i - i >= 0; i++) {
            if (this.board_array[index_i - i][index_j] == id) {
              return true;
            }
          }
        }
        if (index_j != 0) {
          if (this.board_array[index_i - 1][index_j - 1] == id % 2 + 1) {
            for(var i = 2; index_i - i >= 0 && index_j - i >= 0; i++) {
              if (this.board_array[index_i - i][index_j - i] == id) {
                return true;
              }
            }
          }
        }
        if (index_j != this.size - 1) {
          if (this.board_array[index_i - 1][index_j + 1] == id % 2 + 1) {
            for(var i = 2; index_i - i >= 0 && index_j + i < this.size; i++) {
              if (this.board_array[index_i - i][index_j + i] == id) {
                return true;
              }
            }
          }
        }
      }
      if (index_j != this.size - 1) {
         if (this.board_array[index_i][index_j + 1] == id % 2 + 1) {
           for(var i = 2; index_j + i < this.size; i++) {
             if (this.board_array[index_i][index_j + i] == id) {
               return true;
             }
           }
         }
      }
      if (index_j != 0) {
         if (this.board_array[index_i][index_j - 1] == id % 2 + 1) {
           for(var i = 2; index_j - i >= 0; i++) {
             if (this.board_array[index_i][index_j - i] == id) {
               return true;
             }
           }
         }
      }
      return false;
    }

    Board.prototype.render = function() {
      p.background(255);
      for (var i = 0; i < this.size + 1; i++) {
        p.stroke(0);
        p.line(i*p.width / this.size, 0, i * p.width / this.size, p.height);
        p.line(0, i* p.height / this.size, p.width, i * p.height / this.size);
      }
      for (var i = 0; i < this.size; i++) {
        for (var j = 0; j < this.size; j++) {
          if (this.board_array[i][j] == 1) {
            p.noStroke();
            p.fill(255, 0, 0);
            p.ellipse(p.width * (2 * j + 1) / (2 * this.size), p.height * (2 * i + 1) / (2 * this.size), p.width / this.size, p.height / this.size);
          }
          else if (this.board_array[i][j] == 2) {
            p.noStroke();
            p.fill(0, 255, 0);
            p.ellipse(p.width * (2 * j + 1) / (2 * this.size), p.height * (2 * i + 1) / (2 * this.size), p.width / this.size, p.height / this.size);
          }
        }
      }
    };

    p.mousePressed = function() {
      let box_i = p.constrain(p.floor(p.mouseY * board.size / p.height), 0, board.size - 1);
      let box_j = p.constrain(p.floor(p.mouseX * board.size / p.width), 0, board.size - 1);
      if (board.board_array[box_i][box_j] == 0) {
        player1.attempt_move();
        player2.attempt_move();
      }
    }
  };
  new p5(sketch, container);
});

$("#othelloModal").on('hidden.bs.modal', function() {
  document.getElementById("othelloCanvas").remove();
});

$("#pathModal").on('shown.bs.modal', function() {
  if (document.getElementById("pathCanvas")) {
    return;
  }
  let container = document.getElementById("pathContainer");
  let sketch = function(p) {
    let debug = true;

    let path;

    let followers;

    p.setup = function() {
      p.loop();
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth * 3/4);
      canvas.id("pathCanvas");
      path = new Path();
      followers = [];
      for (var i = 0; i < 5; i++) {
        followers.push(new Follower(p.createVector(p.random(p.width), p.random(p.height)), 2, 0.1));
      }
    };

    p.draw = function() {
      if (!document.getElementById("pathCanvas")) {
        p.noLoop();
      }
      p.background(255);
      path.display();
      for (let follower of followers) {
        follower.separate(followers);
        follower.follow(path);
        follower.run();
        follower.borders(path);
      }
    };

    p.keyPressed = function() {
      if (p.key == "d") {
        debug = !debug;
      } else if (p.key == "r") {
        path = new Path();
      }
    };

    p.mousePressed = function() {
      if (p.mouseX > 0 && p.mouseY > 0 && p.mouseX < p.width && p.mouseY < p.height) {
        followers.push(new Follower(p.createVector(p.mouseX, p.mouseY), 2, 0.1));
      }
    };

    let Path = function() {
      this.radius= 20;
      this.horizrad = p.width / 2 - p.random(10) * p.width / 40;
      this.vertrad = p.height / 2 - p.random(10) * p.height / 40;
      this.numpointsminusone = 60;
      this.points = [];
      for (var i = 0; i < this.numpointsminusone + 1; i++) {
        let t = i * p.TAU/(this.numpointsminusone);
        let xcoord = this.horizrad * p.cos(t) / (p.sq(p.sin(t)) + 1) + p.width/2;
        let ycoord = p.height/2 + this.vertrad * p.sin(t)*p.cos(t)/(1 + p.sq(p.sin(t)))
        console.log(i, t, xcoord, ycoord);
        this.addPoint(xcoord, ycoord);
      }
    }

    Path.prototype.addPoint = function(x, y) {
      this.points.push(p.createVector(x, y));
    }

    Path.prototype.display = function() {
      p.stroke(0, 100);
      p.strokeWeight(this.radius*2);
      p.noFill();
      p.beginShape();
      let counter = 0;
      for (let v of this.points) {
        p.vertex(v.x, v.y);
        if (v.x < p.width/100) {
          console.log(counter, v.x);
        }
        counter++;
      }
      p.endShape();
      p.stroke(0);
      p.strokeWeight(1);
      p.noFill();
      p.beginShape()
      for (let v of this.points) {
        p.vertex(v.x, v.y);
      }
      p.endShape();
    }

    let Follower = function(l, ms, mf) {
      this.location = l.copy();
      this.acceleration = p.createVector(0, 0);
      this.maxspeed = ms;
      this.maxforce = mf;
      this.trumaxsteer = mf;
      this.velocity = p.createVector(p.random(0.01), p.random(0.01));
      this.r = 4;
    }

    Follower.prototype.separate = function(vehicles) {
      let desiredseparation = 5 * this.r;
      let sum = p.createVector(0, 0);
      let count = 0;
      for (let other of vehicles) {
        let d = p5.Vector.dist(this.location, other.location);
        if ((d > 0) && (d  < desiredseparation)) {
          let diff = p5.Vector.sub(this.location, other.location);
          diff.normalize();
          diff.mult(desiredseparation/d);
          sum.add(diff);
          count++;
        }
      }
      if (count > 0) {
        sum.div(count);
        sum.setMag(this.maxspeed);
        let steer = p5.Vector.sub(sum, this.velocity);
        steer.limit(2 * this.maxforce);
        this.acceleration.add(steer);
      }
    };

    Follower.prototype.run = function() {
      this.update();
      this.render();
    }

    Follower.prototype.follow = function(path) {
      let future = this.velocity.copy();
      future.setMag(50);
      future.add(this.location);
      let normal = null;
      let target = null;
      let worldrecord = 1000000;

      for (var i = 0; i < path.numpointsminusone; i++) {
        let a = path.points[i].copy();
        let b = path.points[i + 1].copy();
        let normalPoint = this.getNormalPoint(future, a, b);
        if (normalPoint.x < a.x || normalPoint.x > b.x) {
          normalPoint = b.copy();
        }
        let distance = p5.Vector.dist(future, normalPoint);
        if (distance <= worldrecord) {
          worldrecord = distance;
          normal = normalPoint.copy();
          let dir = p5.Vector.sub(b, a);
          dir.setMag(worldrecord/this.velocity.mag());
          target = normalPoint.copy();
          target.add(dir);
        }
      }
      let distance = p5.Vector.dist(future, normal);
      if (distance > path.radius) {
        this.seek(target, this.maxforce);
      } else {
        let maxsteer = this.trumaxsteer * worldrecord / path.radius;
        this.seek(target, maxsteer);
      }
      if (debug) {
        p.fill(0);
        p.stroke(0);
        p.line(this.location.x, this.location.y, future.x, future.y);
        p.ellipse(future.x, future.y, 4, 4);

        p.fill(0);
        p.stroke(0);
        p.line(future.x, future.y, normal.x, normal.y);
        p.ellipse(normal.x, normal.y, 4, 4);
        p.stroke(0);
        if (distance > path.radius) {
          p.fill(255, 0, 0);
        }
        p.noStroke();
        p.ellipse(target.x, target.y, 8, 8);
      }
    };

    Follower.prototype.getNormalPoint = function(point, starting, ending) {
      let startToPoint = p5.Vector.sub(point, starting);
      let startToEnd = p5.Vector.sub(ending, starting);
      startToEnd.normalize();
      let dotproduct = startToPoint.dot(startToEnd);
      startToEnd.mult(dotproduct);
      let normalP = p5.Vector.add(starting, startToEnd);
      return normalP;
    };

    Follower.prototype.update = function() {
      this.velocity.add(this.acceleration);
      this.velocity.limit(this.maxspeed);
      this.location.add(this.velocity);
      this.acceleration.mult(0);
    };

    Follower.prototype.applyForce = function(force) {
      this.acceleration.add(force);
    };

    Follower.prototype.seek = function(destination, maxf) {
      let desiredvel = p5.Vector.sub(destination, this.location);
      if (desiredvel.mag() == 0) {
        return;
      }
      desiredvel.setMag(this.maxspeed);
      let steer = p5.Vector.sub(desiredvel, this.velocity);
      steer.limit(maxf);
      this.applyForce(steer);
    }

    Follower.prototype.render = function() {
      let theta = this.velocity.heading() + p.HALF_PI;
      p.fill(175);
      p.stroke(0);
      p.push();
      p.translate(this.location.x, this.location.y);
      p.rotate(theta);
      p.beginShape(p.TRIANGLES);
      p.vertex(0, -this.r*2);
      p.vertex(-this.r, this.r*2);
      p.vertex(this.r, 2*this.r);
      p.endShape();
      p.pop();
    };

    Follower.prototype.borders = function(path) {
      if (this.location.x > path.points[path.numpointsminusone].x + this.r) {
        this.location.x = path.points[0].x + this.r;
        this.location.y = path.points[0].y + (this.location.y - path.points[path.numpointsminusone].y);
      }
    }
  };
  new p5(sketch, container);
});

$("#pathModal").on('hidden.bs.modal', function() {
  document.getElementById("pathCanvas").remove();
});

$("#perceptronModal").on('shown.bs.modal', function() {
  if (document.getElementById("perceptronCanvas")) {
    return;
  }
  let container = document.getElementById("perceptronContainer");
  let sketch = function(p) {

    let Perceptron = function(n) {
      this.weights = [];
      for (var i = 0; i < n; i++) {
        this.weights.push(p.random(-1, 1));
      }
      this.c = 0.01;
    }

    Perceptron.prototype.train = function(inputs, desired) {
      let guess = this.feedforward(inputs);
      let error = desired - guess;
      for (var i = 0; i < this.weights.length; i++) {
        this.weights[i] += this.c * error * inputs[i];
      }
    };

    Perceptron.prototype.activate = function(num) {
      if (num > 0) {
        return 1;
      }
      return -1;
    };

    Perceptron.prototype.feedforward = function(inputs) {
      let sum = 0;
      for (var i = 0; i < this.weights.length; i++) {
        sum += inputs[i] * this.weights[i];
      }
      return this.activate(sum);
    };

    let Trainer = function(x, y, a) {
      this.inputs = [x, y, 1];
      this.answer = a;
    }

    let ptron;
    let training;
    let count;

    function f(x) {
      return 1.5*x;
    }

    p.setup = function() {
      p.loop();
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth*9/16);
      canvas.id("perceptronCanvas");
      ptron = new Perceptron(3);
      training = [];
      count = 0;
      for (var i = 0; i < 2000; i++) {
        let x = p.random(-p.width/2, p.width/2);
        let y = p.random(-p.height/2, p.height/2);
        let answer = 1;
        if (y < f(x)) {
          answer = -1;
        }
        training.push(new Trainer(x, y, answer));
      }
    };

    p.draw = function() {
      if (!document.getElementById("perceptronCanvas")) {
        p.noLoop();
      }
      p.background(255);
      p.translate(p.width/2, p.height/2);
      ptron.train(training[count].inputs, training[count].answer);
      count = (count + 1) % training.length;
      for (var i = 0; i < count; i++) {
        p.stroke(0);
        let guess = ptron.feedforward(training[i].inputs);
        p.fill(0);
        if (guess > 0) {
          p.noFill();
        }
        p.ellipse(training[i].inputs[0], training[i].inputs[1], 8, 8);
      }
    }

  };
  new p5(sketch, container);
});

$("#perceptronModal").on('hidden.bs.modal', function() {
  document.getElementById("perceptronCanvas").remove();
});

$("#perlinModal").on("shown.bs.modal", function() {
  console.log("PERLIN SHOWN");
  if (document.getElementById("perlinCanvas")) {
    return;
  }
  let container = document.getElementById("perlinContainer");
  let sketch = function(p) {

    let Landscape = function(scl, w, h) {
      this.scl = scl;
      this.w = w;
      this.h = h;
      this.cols = w/scl;
      this.rows = h/scl;
      this.zoff = 0;
      this.z = [];
      for (var i = 0; i < this.cols; i++) {
        this.z.push([]);
        for (var j = 0; j < this.rows; j++) {
          this.z[i].push(0);
        }
      }
    };

    Landscape.prototype.calculate = function() {
      let xoff = 0;
      for (var i = 0; i < this.cols; i++) {
        let yoff = 0;
        for (var j = 0; j < this.rows; j++) {
          this.z[i][j] = p.noise(xoff, yoff, this.zoff) * 240 - 120;
          yoff += 0.1;
        }
        xoff += 0.1;
      }
      this.zoff += 0.01;
      // console.log(this.z);
    };

    Landscape.prototype.render = function() {
      for (var x = 0; x < this.z.length - 1; x++) {
        p.beginShape(p.QUAD_STRIP);
        for (var y = 0; y < this.z[x].length; y++) {
          // p.strokeWeight(3);
          let currentElevation = this.z[x][y];
          let currentShade = (currentElevation/120 + 1) * 255;
          let colorVector = p.createVector(x/2/this.z.length + p.noise(coloroffset)/2, y/2/this.z[x].length + p.noise(coloroffset + 100000)/2, p.noise(coloroffset + 1000));
          colorVector.setMag(currentShade);
          p.stroke(colorVector.x, colorVector.y, colorVector.z);
          let xcoord = x * this.scl - this.w/2;
          let ycoord = y * this.scl - this.h/2;
          p.vertex(xcoord, ycoord, this.z[x][y]);
          p.vertex(xcoord + this.scl, ycoord, this.z[x+1][y]);
        }
        p.endShape();
      }
    };

    let land;
    let theta;
    let coloroffset;

    p.setup = function() {
      p.loop()
      // let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth, p.WEBGL);
      let canvas = p.createCanvas(container.offsetWidth, container.offsetWidth, p.WEBGL);
      canvas.id("perlinCanvas");
      theta = 0;
      coloroffset = 0;
      land = new Landscape(5, p.width, p.height);
    }

    p.draw = function() {
      if (!document.getElementById("perlinCanvas")) {
        p.noLoop();
      }
      p.background(255);
      p.translate(0, 20,-160);
      p.rotateX(p.PI/3);
      p.rotateZ(theta);
      land.calculate();
      land.render();
      coloroffset += 0.1;
      theta += 0.0025;
    };

  };
  new p5(sketch, container);
});

$("#perlinModal").on('hidden.bs.modal', function() {
  document.getElementById("perlinCanvas").remove();
});
