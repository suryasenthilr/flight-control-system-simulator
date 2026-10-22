/**
 * AeroSim-Control: 3D Aircraft Flight Visualizer (Three.js)
 * 
 * Features:
 *  - Procedural 3D Jet Aircraft model with fuselage, swept wings, vertical fin
 *  - Physically articulated elevator control surfaces that deflect in real-time
 *  - Moving atmospheric cloud layer and ground grid
 *  - Interactive Orbit Camera (Chase Cam, Free Cam, Side Cam)
 *  - High-precision canvas 2D fallback if WebGL is unavailable
 */

class Aircraft3DView {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) return;

        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.aircraftGroup = null;
        this.elevatorLeft = null;
        this.elevatorRight = null;
        this.grid = null;
        this.particles = null;
        this.trailPoints = [];
        this.trailLine = null;

        this.camMode = 'chase'; // 'chase', 'side', 'free'
        this.isMouseDown = false;
        this.mouseX = 0;
        this.mouseY = 0;
        this.camRotX = 0.2;
        this.camRotY = -0.5;

        this.init();
    }

    init() {
        if (typeof THREE === 'undefined') {
            this.init2DFallback();
            return;
        }

        try {
            const w = this.container.clientWidth || 600;
            const h = this.container.clientHeight || 350;

            // Scene
            this.scene = new THREE.Scene();
            this.scene.background = new THREE.Color(0x0a0f1d);
            this.scene.fog = new THREE.FogExp2(0x0a0f1d, 0.008);

            // Camera
            this.camera = new THREE.PerspectiveCamera(45, w / h, 0.5, 1000);
            this.camera.position.set(18, 8, 22);

            // Renderer
            this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
            this.renderer.setSize(w, h);
            this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
            this.renderer.shadowMap.enabled = true;
            this.container.appendChild(this.renderer.domElement);

            // Lighting
            const ambLight = new THREE.AmbientLight(0xffffff, 0.7);
            this.scene.add(ambLight);

            const dirLight = new THREE.DirectionalLight(0x38bdf8, 1.4);
            dirLight.position.set(30, 45, 25);
            dirLight.castShadow = true;
            this.scene.add(dirLight);

            const sunLight = new THREE.DirectionalLight(0xffedd5, 0.8);
            sunLight.position.set(-20, -10, -20);
            this.scene.add(sunLight);

            // Infinite Ground Grid
            this.grid = new THREE.GridHelper(400, 80, 0x1e293b, 0x0f172a);
            this.grid.position.y = -8;
            this.scene.add(this.grid);

            // Build Aircraft
            this.buildAircraft();

            // Atmospheric Streamlines (Wind Particles)
            this.buildWindParticles();

            // Flight Path Ribbon
            this.buildFlightPathTrail();

            // Event Listeners for Interaction
            this.setupInteraction();

            window.addEventListener('resize', () => this.onResize());
        } catch (e) {
            console.warn('WebGL initialization failed, falling back to 2D:', e);
            this.init2DFallback();
        }
    }

    buildAircraft() {
        this.aircraftGroup = new THREE.Group();

        const fuselageMat = new THREE.MeshStandardMaterial({
            color: 0xe2e8f0,
            roughness: 0.25,
            metalness: 0.65
        });

        const accentMat = new THREE.MeshStandardMaterial({
            color: 0x0284c7, // Aerospace cyan-blue accent
            roughness: 0.3,
            metalness: 0.4
        });

        const cockpitGlassMat = new THREE.MeshPhysicalMaterial({
            color: 0x0f172a,
            roughness: 0.1,
            metalness: 0.9,
            transparent: true,
            opacity: 0.85
        });

        const controlSurfaceMat = new THREE.MeshStandardMaterial({
            color: 0xf59e0b, // Amber for moving control surfaces
            roughness: 0.3,
            metalness: 0.5
        });

        // 1. Fuselage
        const fuseGeom = new THREE.ConeGeometry(0.9, 8.5, 32);
        fuseGeom.rotateX(Math.PI / 2);
        const fuselage = new THREE.Mesh(fuseGeom, fuselageMat);
        fuselage.position.z = 0;
        this.aircraftGroup.add(fuselage);

        // Cockpit canopy
        const canopyGeom = new THREE.SphereGeometry(0.75, 16, 16);
        canopyGeom.scale(0.8, 0.7, 2.2);
        const canopy = new THREE.Mesh(canopyGeom, cockpitGlassMat);
        canopy.position.set(0, 0.65, 1.2);
        this.aircraftGroup.add(canopy);

        // 2. Main Wings (Swept)
        const wingShape = new THREE.Shape();
        wingShape.moveTo(0, 1.2);
        wingShape.lineTo(7.0, -1.8);
        wingShape.lineTo(6.5, -2.4);
        wingShape.lineTo(0, -1.2);
        wingShape.closePath();

        const extrudeSettings = { depth: 0.12, bevelEnabled: true, bevelSegments: 2, steps: 1, bevelSize: 0.04, bevelThickness: 0.04 };
        const wingGeom = new THREE.ExtrudeGeometry(wingShape, extrudeSettings);
        wingGeom.rotateX(-Math.PI / 2);

        // Right Wing
        const rightWing = new THREE.Mesh(wingGeom, fuselageMat);
        rightWing.position.set(0, 0, 0);
        this.aircraftGroup.add(rightWing);

        // Left Wing
        const leftWing = rightWing.clone();
        leftWing.scale.x = -1;
        this.aircraftGroup.add(leftWing);

        // Wingtips
        const tipGeom = new THREE.BoxGeometry(0.1, 0.8, 1.0);
        const rightTip = new THREE.Mesh(tipGeom, accentMat);
        rightTip.position.set(6.8, 0.3, -2.1);
        this.aircraftGroup.add(rightTip);

        const leftTip = rightTip.clone();
        leftTip.position.x = -6.8;
        this.aircraftGroup.add(leftTip);

        // 3. Vertical Stabilizer (Fin)
        const finShape = new THREE.Shape();
        finShape.moveTo(0, 0);
        finShape.lineTo(-1.6, 2.8);
        finShape.lineTo(-2.2, 2.8);
        finShape.lineTo(-2.8, 0);
        finShape.closePath();

        const finGeom = new THREE.ExtrudeGeometry(finShape, { depth: 0.1, bevelEnabled: true, bevelSize: 0.02, bevelThickness: 0.02 });
        finGeom.rotateY(Math.PI / 2);
        const fin = new THREE.Mesh(finGeom, accentMat);
        fin.position.set(0.05, 0.8, -2.5);
        this.aircraftGroup.add(fin);

        // 4. Horizontal Stabilizer & Moving Elevators
        // Left horizontal stabilizer base
        const hStabGeom = new THREE.BoxGeometry(2.4, 0.08, 1.2);
        const leftHStab = new THREE.Mesh(hStabGeom, fuselageMat);
        leftHStab.position.set(-1.8, 0.2, -4.0);
        this.aircraftGroup.add(leftHStab);

        const rightHStab = new THREE.Mesh(hStabGeom, fuselageMat);
        rightHStab.position.set(1.8, 0.2, -4.0);
        this.aircraftGroup.add(rightHStab);

        // Articulated Left Elevator (hinged at front of surface)
        const elevGeom = new THREE.BoxGeometry(2.3, 0.06, 0.6);
        elevGeom.translate(0, 0, -0.3); // pivot at hinge line

        this.elevatorLeft = new THREE.Mesh(elevGeom, controlSurfaceMat);
        this.elevatorLeft.position.set(-1.8, 0.2, -4.6);
        this.aircraftGroup.add(this.elevatorLeft);

        // Articulated Right Elevator
        this.elevatorRight = new THREE.Mesh(elevGeom, controlSurfaceMat);
        this.elevatorRight.position.set(1.8, 0.2, -4.6);
        this.aircraftGroup.add(this.elevatorRight);

        // Engine Thruster Nozzle Glow
        const nozzleGeom = new THREE.CylinderGeometry(0.5, 0.6, 0.8, 16);
        nozzleGeom.rotateX(Math.PI / 2);
        const nozzleMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8 });
        const nozzle = new THREE.Mesh(nozzleGeom, nozzleMat);
        nozzle.position.set(0, 0, -4.4);
        this.aircraftGroup.add(nozzle);

        this.scene.add(this.aircraftGroup);
    }

    buildWindParticles() {
        const particleCount = 180;
        const geom = new THREE.BufferGeometry();
        const positions = new Float32Array(particleCount * 3);

        for (let i = 0; i < particleCount; i++) {
            positions[i * 3 + 0] = (Math.random() - 0.5) * 40;
            positions[i * 3 + 1] = (Math.random() - 0.5) * 20;
            positions[i * 3 + 2] = (Math.random() - 0.5) * 80;
        }

        geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        const mat = new THREE.PointsMaterial({
            color: 0x38bdf8,
            size: 0.25,
            transparent: true,
            opacity: 0.65
        });

        this.particles = new THREE.Points(geom, mat);
        this.scene.add(this.particles);
    }

    buildFlightPathTrail() {
        const maxTrailPoints = 120;
        const trailGeom = new THREE.BufferGeometry();
        const positions = new Float32Array(maxTrailPoints * 3);
        trailGeom.setAttribute('position', new THREE.BufferAttribute(positions, 3));

        const trailMat = new THREE.LineBasicMaterial({
            color: 0x4ade80,
            transparent: true,
            opacity: 0.7,
            linewidth: 2
        });

        this.trailLine = new THREE.Line(trailGeom, trailMat);
        this.trailLine.frustumCulled = false;
        this.scene.add(this.trailLine);
    }

    setupInteraction() {
        const dom = this.renderer.domElement;

        dom.addEventListener('mousedown', (e) => {
            this.isMouseDown = true;
            this.mouseX = e.clientX;
            this.mouseY = e.clientY;
        });

        window.addEventListener('mouseup', () => {
            this.isMouseDown = false;
        });

        window.addEventListener('mousemove', (e) => {
            if (!this.isMouseDown) return;
            const dx = e.clientX - this.mouseX;
            const dy = e.clientY - this.mouseY;
            this.mouseX = e.clientX;
            this.mouseY = e.clientY;

            this.camRotY -= dx * 0.008;
            this.camRotX = Math.max(Math.min(this.camRotX + dy * 0.008, 1.2), -0.8);
        });

        dom.addEventListener('wheel', (e) => {
            e.preventDefault();
            if (this.camera) {
                const zoomFactor = e.deltaY * 0.02;
                this.camera.fov = Math.max(Math.min(this.camera.fov + zoomFactor, 75), 20);
                this.camera.updateProjectionMatrix();
            }
        }, { passive: false });
    }

    setCameraMode(mode) {
        this.camMode = mode;
    }

    reset() {
        if (this.aircraftGroup) {
            this.aircraftGroup.rotation.set(0, 0, 0);
        }
        if (this.elevatorLeft && this.elevatorRight) {
            this.elevatorLeft.rotation.set(0, 0, 0);
            this.elevatorRight.rotation.set(0, 0, 0);
        }
        this.trailPoints = [];
        if (this.trailLine) {
            this.trailLine.geometry.setDrawRange(0, 0);
        }
    }

    update(flightState) {
        if (!this.aircraftGroup || !this.renderer) return;

        // 1. Aircraft Attitude (Pitch theta in radians, yaw, roll)
        const theta = flightState.theta;
        const delta_e = flightState.delta_e;

        // Note: In 3D, pitching up corresponds to positive X rotation
        this.aircraftGroup.rotation.x = theta;
        this.aircraftGroup.rotation.y = 0.0;
        this.aircraftGroup.rotation.z = 0.0;

        // 2. Animate Elevator Control Surface Deflection
        // Elevator deflects relative to horizontal stabilizer (rotation about local X axis)
        if (this.elevatorLeft && this.elevatorRight) {
            // Negative delta_e is trailing edge UP (aero convention for pitch up)
            this.elevatorLeft.rotation.x = -delta_e;
            this.elevatorRight.rotation.x = -delta_e;
        }

        // 3. Move wind particles toward the back to simulate forward airspeed
        if (this.particles) {
            const positions = this.particles.geometry.attributes.position.array;
            const speed = (flightState.airspeed / 55.0) * 1.5;
            for (let i = 0; i < positions.length / 3; i++) {
                positions[i * 3 + 2] -= speed;
                if (positions[i * 3 + 2] < -40) {
                    positions[i * 3 + 2] = 40;
                    positions[i * 3 + 0] = (Math.random() - 0.5) * 40;
                    positions[i * 3 + 1] = (Math.random() - 0.5) * 20;
                }
            }
            this.particles.geometry.attributes.position.needsUpdate = true;
        }

        // 4. Update Flight Path Ribbon Trail
        if (this.trailLine) {
            const pos = new THREE.Vector3(0, flightState.altitude * 0.05, 0);
            this.trailPoints.push(pos);
            if (this.trailPoints.length > 120) this.trailPoints.shift();

            const posArray = this.trailLine.geometry.attributes.position.array;
            for (let i = 0; i < this.trailPoints.length; i++) {
                const zOffset = (this.trailPoints.length - 1 - i) * -0.8 - 4.5;
                const p = this.trailPoints[i];
                posArray[i * 3 + 0] = 0;
                posArray[i * 3 + 1] = p.y - flightState.altitude * 0.05;
                posArray[i * 3 + 2] = zOffset;
            }
            this.trailLine.geometry.setDrawRange(0, this.trailPoints.length);
            this.trailLine.geometry.attributes.position.needsUpdate = true;
        }

        // 5. Update Camera Position based on Selected Mode
        if (this.camMode === 'chase') {
            const radius = 24.0;
            const camX = radius * Math.sin(this.camRotY) * Math.cos(this.camRotX);
            const camZ = radius * Math.cos(this.camRotY) * Math.cos(this.camRotX);
            const camY = radius * Math.sin(this.camRotX) + 4.0;

            this.camera.position.set(camX, camY, camZ);
            this.camera.lookAt(0, 0.5, 0);
        } else if (this.camMode === 'side') {
            this.camera.position.set(22, 2.5, -0.5);
            this.camera.lookAt(0, 0, 0);
        } else if (this.camMode === 'cockpit') {
            this.camera.position.set(0, 0.9, 1.4);
            this.camera.rotation.set(theta, 0, 0);
        }

        // Render Frame
        this.renderer.render(this.scene, this.camera);
    }

    onResize() {
        if (!this.container || !this.renderer || !this.camera) return;
        const w = this.container.clientWidth;
        const h = this.container.clientHeight;
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
    }

    init2DFallback() {
        this.container.innerHTML = '<canvas id="aircraft2DFallback" style="width:100%; height:100%;"></canvas>';
        const canvas = document.getElementById('aircraft2DFallback');
        const ctx = canvas.getContext('2d');

        this.update = (flightState) => {
            const w = canvas.width = canvas.clientWidth;
            const h = canvas.height = canvas.clientHeight;
            ctx.clearRect(0, 0, w, h);

            // Draw clean side-profile aircraft
            ctx.save();
            ctx.translate(w * 0.5, h * 0.5);
            ctx.rotate(-flightState.theta);

            // Fuselage
            ctx.fillStyle = '#e2e8f0';
            ctx.beginPath();
            ctx.ellipse(0, 0, 90, 16, 0, 0, 2 * Math.PI);
            ctx.fill();

            // Wing
            ctx.fillStyle = '#0284c7';
            ctx.fillRect(-20, -5, 45, 10);

            // Tail & Elevator
            ctx.save();
            ctx.translate(-75, -8);
            ctx.rotate(-flightState.delta_e);
            ctx.fillStyle = '#f59e0b';
            ctx.fillRect(-25, -2, 25, 4);
            ctx.restore();

            ctx.restore();
        };
    }
}

if (typeof window !== 'undefined') {
    window.Aircraft3DView = Aircraft3DView;
}
