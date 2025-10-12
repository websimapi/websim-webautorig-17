import * as THREE from 'three';

// Helper: Shortest distance between two 3D line segments
// Returns { distanceSq, a, b } where a and b are the closest points on seg1 and seg2
function closestPointsBetweenSegments(p1, q1, p2, q2) {
    const d1 = q1.clone().sub(p1);
    const d2 = q2.clone().sub(p2);
    const r = p1.clone().sub(p2);

    const a = d1.dot(d1); // Squared length of segment 1
    const e = d2.dot(d2); // Squared length of segment 2
    const f = d2.dot(r);

    let t1 = 0, t2 = 0;

    if (a <= 1e-6 && e <= 1e-6) { // Both segments are points
        t1 = t2 = 0;
    } else if (a <= 1e-6) { // First segment is a point
        t1 = 0;
        t2 = f / e;
        t2 = THREE.MathUtils.clamp(t2, 0, 1);
    } else {
        const c = d1.dot(r);
        if (e <= 1e-6) { // Second segment is a point
            t2 = 0;
            t1 = -c / a;
            t1 = THREE.MathUtils.clamp(t1, 0, 1);
        } else { // General case
            const b = d1.dot(d2);
            const denom = a * e - b * b;

            if (denom !== 0) {
                t1 = (b * f - c * e) / denom;
                t1 = THREE.MathUtils.clamp(t1, 0, 1);
            }

            t2 = (b * t1 + f) / e;

            if (t2 < 0) {
                t2 = 0;
                t1 = -c / a;
                t1 = THREE.MathUtils.clamp(t1, 0, 1);
            } else if (t2 > 1) {
                t2 = 1;
                t1 = (b - c) / a;
                t1 = THREE.MathUtils.clamp(t1, 0, 1);
            }
        }
    }

    const c1 = p1.clone().add(d1.multiplyScalar(t1));
    const c2 = p2.clone().add(d2.multiplyScalar(t2));
    return { distanceSq: c1.distanceToSquared(c2), a: c1, b: c2 };
}

class Capsule {
    constructor(name, bone, start, end, radius) {
        this.name = name;
        this.bone = bone;
        this.segment = new THREE.Line3(start, end); // Local space
        this.radius = Math.max(0.01, radius); // Ensure non-zero radius
        this.worldSegment = new THREE.Line3();
    }

    updateWorldMatrix() {
        if (!this.bone.parent) return;
        this.bone.parent.updateWorldMatrix(true, false);
        this.worldSegment.copy(this.segment).applyMatrix4(this.bone.parent.matrixWorld);
    }
}

export class CollisionSystem {
    constructor(skeleton, markers, modelInverseMatrix) {
        this.skeleton = skeleton;
        this.capsules = new Map();
        this.collisionPairs = [];
        this.margin = 0.02; // Collision buffer

        this.createCapsules(markers, modelInverseMatrix);
        this.defineCollisionPairs();
    }

    createCapsules(markers, modelInverseMatrix) {
        const localMarkers = new Map();
        for (const [id, worldPos] of markers.entries()) {
            localMarkers.set(id, worldPos.clone().applyMatrix4(modelInverseMatrix));
        }

        const getBone = (name) => this.skeleton.getBoneByName(name);
        const getMarker = (name) => localMarkers.get(name);

        const definitions = [
            // Torso
            { name: 'torso', start: 'hips', end: 'neck', radius: getMarker('left_hip').distanceTo(getMarker('right_hip')) / 2.5 },
            // Arms
            { name: 'l_upperarm', start: 'left_shoulder', end: 'left_elbow', radiusKey: 'left_shoulder' },
            { name: 'r_upperarm', start: 'right_shoulder', end: 'right_elbow', radiusKey: 'right_shoulder' },
            { name: 'l_forearm', start: 'left_elbow', end: 'left_wrist', radiusKey: 'left_elbow' },
            { name: 'r_forearm', start: 'right_elbow', end: 'right_wrist', radiusKey: 'right_elbow' },
            // Legs
            { name: 'l_thigh', start: 'left_hip', end: 'left_knee', radiusKey: 'left_hip' },
            { name: 'r_thigh', start: 'right_hip', end: 'right_knee', radiusKey: 'right_hip' },
            { name: 'l_calf', start: 'left_knee', end: 'left_ankle', radiusKey: 'left_knee' },
            { name: 'r_calf', start: 'right_knee', end: 'right_ankle', radiusKey: 'right_knee' }
        ];

        for (const def of definitions) {
            const bone = getBone(def.start);
            const p1 = getMarker(def.start);
            const p2 = getMarker(def.end);

            if (!bone || !p1 || !p2) continue;

            const startPos = bone.position.clone();
            const endPos = p2.clone().sub(p1).add(startPos);

            let radius = def.radius;
            if (def.radiusKey) {
                const clavicleL = getMarker('left_clavicle');
                const clavicleR = getMarker('right_clavicle');
                const hipL = getMarker('left_hip');
                const hipR = getMarker('right_hip');

                // Estimate radius based on shoulder/hip width as a proxy for thickness
                const shoulderWidth = clavicleL && clavicleR ? clavicleL.distanceTo(clavicleR) : 0.2;
                const hipWidth = hipL && hipR ? hipL.distanceTo(hipR) : 0.2;
                radius = (shoulderWidth + hipWidth) / 4 * 0.4; // Average width, scaled down
            }

            const capsule = new Capsule(def.name, bone, startPos, endPos, radius);
            this.capsules.set(def.name, capsule);
        }
    }

    defineCollisionPairs() {
        const getCap = (name) => this.capsules.get(name);
        const pairs = [
            ['l_upperarm', 'torso'], ['r_upperarm', 'torso'],
            ['l_forearm', 'torso'], ['r_forearm', 'torso'],
            ['l_thigh', 'torso'], ['r_thigh', 'torso'],
            ['l_thigh', 'r_thigh'],
        ];
        for (const [nameA, nameB] of pairs) {
            const capA = getCap(nameA);
            const capB = getCap(nameB);
            if (capA && capB) {
                this.collisionPairs.push([capA, capB]);
            }
        }
    }

    update() {
        this.capsules.forEach(cap => cap.updateWorldMatrix());

        for (const [capA, capB] of this.collisionPairs) {
            const { distanceSq, a, b } = closestPointsBetweenSegments(
                capA.worldSegment.start, capA.worldSegment.end,
                capB.worldSegment.start, capB.worldSegment.end
            );

            const minDistance = capA.radius + capB.radius + this.margin;
            if (distanceSq < minDistance * minDistance) {
                this.resolveCollision(capA, capB, Math.sqrt(distanceSq), minDistance, a, b);
            }
        }
    }

    resolveCollision(capA, capB, distance, minDistance, pA, pB) {
        const penetration = minDistance - distance;
        if (penetration <= 0) return;

        // Push limb (capA) away from torso/other limb (capB)
        const pushVector = pA.clone().sub(pB).normalize();

        // The bone we want to rotate is capA's bone (e.g., shoulder for upperarm)
        const boneToAdjust = capA.bone;

        // The rotation should be around an axis perpendicular to both the push vector
        // and the limb's direction.
        const limbDir = capA.worldSegment.end.clone().sub(capA.worldSegment.start).normalize();
        let rotationAxis = new THREE.Vector3().crossVectors(limbDir, pushVector).normalize();

        if (rotationAxis.lengthSq() < 0.001) {
            // Fallback if vectors are parallel
            rotationAxis = new THREE.Vector3(0, 1, 0);
        }

        // Apply a small rotation. The amount is proportional to penetration.
        // A factor of 0.1 makes it a soft, gradual correction.
        const correctionAngle = (penetration / capA.radius) * 0.1;

        const correctionQuat = new THREE.Quaternion().setFromAxisAngle(rotationAxis, correctionAngle);

        // We need to apply this rotation in the bone's local space.
        // Transform the world-space quaternion to local space.
        const parentWorld = new THREE.Matrix4();
        boneToAdjust.parent.updateWorldMatrix(true, false);
        parentWorld.copy(boneToAdjust.parent.matrixWorld).invert();

        const parentWorldQuat = new THREE.Quaternion().setFromRotationMatrix(parentWorld);

        const localCorrectionQuat = correctionQuat.premultiply(parentWorldQuat).normalize();

        boneToAdjust.quaternion.premultiply(localCorrectionQuat);
        boneToAdjust.updateWorldMatrix(true, false);
    }
}