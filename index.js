/* Main application to simulate a constrained system 
of 2D ppoints connected via elastic massless rods (links)
together forming a 2D block of a solid material.

The links are removed/deactivated from the system,
once their change in length is less than a predefined small number.
Thus, the block losses its mass in those regions, where
the construction have low values of elastic strain.

The links or rods are rendered and color-coded w.r.t. their value of strain. 

(c) Tsivilskiy Ilya, 2019 */

/* RGB colormap */
class ColorMap {
    constructor() {
        this.sharp = 10;
        this.power = 2;
        this.ff = 255;
    }

    gaussian(value, offset) {
        return parseInt(this.ff / (1 + Math.pow((value - offset), this.power) * this.sharp));
    };

    red(value) {
        return this.gaussian(value, 1);
    };

    green(value) {
        return this.gaussian(value, .5);
    };

    blue(value) {
        return this.gaussian(value, 0);
    };

    rgbString(value) {
        return 'rgb(' + this.red(value) + ', ' + this.green(value) + ', ' + this.blue(value) + ')';
    };
}

/* Vector2 */
class Vector2 {
    constructor(x = 0.0, y = 0.0) {
        this.x = x;
        this.y = y;
    }

    add(b) {
        return new Vector2(this.x + b.x, this.y + b.y);
    };

    subtract(b) {
        return new Vector2(this.x - b.x, this.y - b.y);
    };

    scaleBy(s) {
        return new Vector2(this.x * s, this.y * s);
    };

    get magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    };

    get clone() {
        return new Vector2(this.x, this.y);
    };
}

/* Vpoint - a 2D point that */
class VPoint {
    constructor(pos, index, acceleration = new Vector2(), fixed = false) {
        this.pos = this.oldPos = pos;
        this.index = index;
        this.acceleration = acceleration;
        this.fixed = fixed;
    }

    update(dt) {
        // pos = 2*pos - oldPos + a*dt^2
        let tmp = this.pos.clone;
        // Verlet's integrator
        this.pos = tmp.scaleBy(2).subtract(this.oldPos).add(this.acceleration.scaleBy(dt * dt));
        if (this.fixed) {
            this.pos = this.oldPos.clone;
        }
        this.oldPos = tmp.clone;
    }

    resolveCollisions(minv, maxv) {
        if (this.pos.x > maxv.x) {
           this.pos.x = maxv.x * 1.0;
        }

        if (this.pos.y > maxv.y) {
           this.pos.y = maxv.y * 1.0;
        }

        if (this.pos.x < minv.x) {
            this.pos.x = minv.x * 1.0;
        }

        if (this.pos.y < minv.y) {
            this.pos.y = minv.y * 1.0;
        }
    }
}


/* VLink - a bound connecting two instances of VPoint */
class VLink {
    constructor(p1, p2, stiffness = 0.5) {
        this.p1 = p1;
        this.p2 = p2;
        if (typeof this.p1 === undefined) {
            throw 'VLink: p1 is undefined';
        }
        if (typeof this.p2 === undefined) {
            throw 'VLink: p2 is undefined';
        }
        this.originalLength = this.currentLength * 1.0;
        this.stiffness = stiffness;
        this.active = true;
    }

    get currentLength() {
        return (this.direction).magnitude;
    };

    get direction() {
        return (this.p2.pos).subtract(this.p1.pos);
    };

    get deltaLength() {
        return Math.abs(this.currentLength - this.originalLength) / this.originalLength;
    };

    updateOriginalLength() {
        this.originalLength = this.currentLength * 1.0;
    };

    update() {
        if (!this.active) return;

        let len = this.currentLength;
        let diff = (this.originalLength - len);
        let offset = this.direction.clone.scaleBy(diff*0.5*this.stiffness);

        this.p1.pos = this.p1.pos.subtract(offset);
        this.p2.pos = this.p2.pos.add(offset);

        if (this.p1.fixed) {
            this.p1.pos = this.p1.oldPos.clone;
        }
        if (this.p2.fixed) {
            this.p2.pos = this.p2.oldPos.clone;
        }
    };

    containsPoint(p) {
        return ((this.p1.hasEqualIndex(p)) || (this.p2.hasEqualIndex(p)));
    };
}

/* VSolver - physics solver operating on VPoints and VLinks */
class VSolver {
    constructor(timestep = 0.01, iterations = 20) {
        this.points = [];
        this.links = [];
        this.ITERATIONS = iterations;
        this.timestep = timestep;
    }

    update(minv, maxv) {
        let i, j;
        let np = this.points.length;
        let nc = this.linksCount;

        //console.log('upd pts');
        for (i = 0; i < np; i++) {
            this.points[i].update(this.timestep);
        }

        //console.log('upd lnks');
        // relax the links
        for (j = 0; j < this.ITERATIONS; j++) {
            for (i = 0; i < nc; i++) {
                this.links[i].update();
            }
        }

        //console.log('upd coll');
        for (i = 0; i < np; i++) {
            this.points[i].resolveCollisions(minv, maxv);
        }
    };

    // find min and max delta-lengths over all links
    get deltaLengthsRange() {
        let i, dl, lnk;
        let lenMax = 0;
        let lenMin = 1000;

        for (i = 0; i < this.links.length; i++) {
            lnk = this.links[i];
            if (lnk.active) {
                dl = lnk.deltaLength;
                if (dl < lenMin) {
                    lenMin = dl;
                }
                if (dl > lenMax) {
                    lenMax = dl;
                }
            }
        }

        return {'min': lenMin, 'max': lenMax};
    };

    get activeLinksCount() {
        let i;
        let activeLinks = 0;
        for (i = 0; i < this.links.length; i++) {
            activeLinks += this.links[i].active * 1.0;
        }
        return activeLinks;
    };

    get linksCount() {
        return this.links.length;
    }
}

/* Renderer - visualizes the VSolver's output */
class Renderer2D {
    constructor(canvasId) {
        let canvas = document.getElementById(canvasId);

        this.width = parseFloat(canvas.width);
        this.height = parseFloat(canvas.height);
        this.ctx = canvas.getContext('2d');
    }

    clear() {
        this.ctx.beginPath();
        this.ctx.rect(0, 0, this.width, this.height);
        this.ctx.fillStyle = '#ffffff';
        this.ctx.fill();
        this.ctx.closePath();
        this.ctx.clearRect(0, 0, this.width, this.height);
    };

    screenToCartesian(vec2, sc = 10000) {
        return new Vector2(vec2.x/sc - this.width/2, this.height/2  - vec2.y/sc);
    };

    cartesianToScreen(vec2, sc = 10000) {
        return new Vector2(sc*vec2.x + this.width/2, this.height/2 - sc*vec2.y);
    };

    setLineStyle(width=2, color = '#555555') {
        this.ctx.lineWidth = width;
        this.ctx.strokeStyle = color;
    };

    moveTo(vec2) {
        this.ctx.moveTo(vec2.x, vec2.y);
    };

    lineTo(vec2) {
        this.ctx.lineTo(vec2.x, vec2.y);
    };

    startDrawing() {
        this.ctx.beginPath();
    };

    finishDrawing() {
        this.ctx.stroke();
    };

    drawPoint(center, radius = 5, fill = '#000000', stroke = '#cccccc') {
        this.ctx.beginPath();
        this.ctx.arc(center.x, center.y, radius, 0, 2*Math.PI, false);
        this.ctx.fillStyle = fill;
        this.ctx.fill();
        this.ctx.lineWidth = 1;
        this.ctx.strokeStyle = stroke;
        this.ctx.stroke();
    };
}

/* Main application */
class Main {
    constructor() {
        this.txt = document.getElementById('txtlabel');

        let L = 0.025;
        let sc = 4;
        this.minv = new Vector2(-sc*L/2, -sc*L/2);
        this.maxv = new Vector2(sc*L/2, sc*L/2 );

        this.vsol = null;
        this.drawing_scale = 30000;
        this.renderer = new Renderer2D('canvas');

        this.cmap = new ColorMap();

        this.t = 0.0;
        this.t_max = 50;
        this.dt = 0.002;
        this.solverIterations = 300;

        let stiffness = 2000; // stiffness constant of all rods (that is proportional to the Young's modulus)
        let pointsAndLinks = this.initMesh(55, L, L * 0.25, stiffness, -9.8);

        this.vsol = new VSolver(this.dt, this.solverIterations);
        this.vsol.points = pointsAndLinks.points;
        this.vsol.links = pointsAndLinks.links;

        this.vsol.update(this.minv, this.maxv);

        this.log('Initial geometry');
    }

    log(text) {
        this.txt.innerText = text;
    }

    // Init a 2D bridge-like block, then mesh it
    initMesh(res = 3, W = 0.025, H = 0.01, st = 0.9, g = -9.8) {
        let iy, ix;
        let vp, id;
        let tl, tr;
        let br, bl;
        let base;
        let resx = res;
        let resy = Math.round(res * (H/W));
        console.log(resx + ' ' + resy);
        let hx = W/(resx - 1);
        let hy = H/(resy - 1);
        let points = [];
        let links = [];
        id = 0;

        // fill up points array
        for (iy = 0; iy < resy; iy++) {
            for (ix = 0; ix < resx; ix++) {
                vp = new VPoint(new Vector2(hx * ix - W / 2, hy * iy - H / 2),
                    id,
                    new Vector2(0.0, g), ((ix === 0) || (ix === (resx-1))) && (iy == 0));
                points.push(vp);
                id++;
            }
        }

        /*

        tl _ tr
        |  x  |
        bl _ br

        */

        // fill up links array
        for (iy = 0; iy < (resy-1); iy++) {
            for (ix = 0; ix < (resx - 1); ix++) {
                base = iy * resx + ix;
                tl = base;
                tr = base + 1;
                bl = tl + resx;
                br = tr + resx;

                links.push(new VLink(points[tl], points[tr], st));
                links.push(new VLink(points[tl], points[br], st));
                links.push(new VLink(points[tl], points[bl], st));
                links.push(new VLink(points[tr], points[br], st));
                links.push(new VLink(points[tr], points[bl], st));
                links.push(new VLink(points[bl], points[br], st));
            }
        }

        return {'points': points, 'links': links};
    };

    drawMesh() {
        let sp, i, vp, lnk, dlNorm;
        let dl = this.vsol.deltaLengthsRange;

        // draw all links colored by their delta-lengths
        for (i = 0; i < this.vsol.linksCount; i++) {
            lnk = this.vsol.links[i];

            dlNorm = (lnk.deltaLength - dl.min) / (dl.max - dl.min);
            let colstr = this.cmap.rgbString(Math.pow(dlNorm, 0.5));

            if (lnk.active) {
                this.renderer.setLineStyle(5, colstr);
                this.renderer.startDrawing();
                sp = this.renderer.cartesianToScreen(this.vsol.links[i].p1.pos, this.drawing_scale);
                this.renderer.moveTo(sp);
                sp = this.renderer.cartesianToScreen(this.vsol.links[i].p2.pos, this.drawing_scale);
                this.renderer.lineTo(sp);
                this.renderer.finishDrawing();
            }
        }

        for (i=0; i<this.vsol.points.length; i++) {
            vp = this.vsol.points[i];
            if (vp.fixed) {
                sp = this.renderer.cartesianToScreen(vp.pos, this.drawing_scale);
                this.renderer.drawPoint(sp);
            }
        }
    };

    updateLinksActivity() {
        console.log('upd');

        let dl = this.vsol.deltaLengthsRange;
        let i, lnk, dlNorm;
        for (i = 0; i < this.vsol.linksCount; i++) {
            lnk = this.vsol.links[i];
            dlNorm = (lnk.deltaLength - dl.min) / (dl.max - dl.min);

            // deactivate a link if its length changes a little bit
            if (lnk.active) {
                this.vsol.links[i].active = (dlNorm > 0.003);
                //this.vsol.links[i].updateOriginalLength();
            }
        }

        let total = this.vsol.linksCount;
        let active = this.vsol.activeLinksCount;
        let suppressed = total - active;
        this.log('Mass loss = ' + Math.round(100 * suppressed / total) + ' %');
    }

    updateAll() {
        this.vsol.update(this.minv, this.maxv);
        this.renderer.clear();
        this.drawMesh();

        this.t++;
        if (this.t > this.t_max) {
            this.t = 0.0;
            this.updateLinksActivity();
        }
    };
}

/* Main entry point */
let app;

function init() {
   // console.log('init');

    app = new Main();

    animate();
}

function animate() {
    app.updateAll();
    requestAnimationFrame(animate);
}