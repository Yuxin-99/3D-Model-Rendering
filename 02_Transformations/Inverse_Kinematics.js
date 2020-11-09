/*
 * Applying Inverse Kinematics
 */

//*****************************TEMPLATE CODE DO NOT MODIFY********************************//
// ASSIGNMENT-SPECIFIC API EXTENSION
THREE.Object3D.prototype.setMatrix = function(a) {
    this.matrix=a;
    this.matrix.decompose(this.position,this.quaternion,this.scale);
}
// SETUP RENDERER AND SCENE
var scene = new THREE.Scene();
var renderer = new THREE.WebGLRenderer();
renderer.setClearColor(0xffffff);
document.body.appendChild(renderer.domElement);
// SETUP CAMERA
var camera = new THREE.PerspectiveCamera(30, 1, 0.1, 1000);
camera.position.set(-28,10,28);
camera.lookAt(scene.position);
scene.add(camera);
// SETUP ORBIT CONTROL OF THE CAMERA
var controls = new THREE.OrbitControls(camera);
controls.damping = 0.2;
// ADAPT TO WINDOW RESIZE
function resize() {
    renderer.setSize(window.innerWidth, window.innerHeight);
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();
// FLOOR WITH CHECKERBOARD
var floorTexture = new THREE.ImageUtils.loadTexture('images/checkerboard.jpg');
floorTexture.wrapS = floorTexture.wrapT = THREE.RepeatWrapping;
floorTexture.repeat.set(4, 4);
var floorMaterial = new THREE.MeshBasicMaterial({ map: floorTexture, side: THREE.DoubleSide });
var floorGeometry = new THREE.PlaneBufferGeometry(30, 30);
var floor = new THREE.Mesh(floorGeometry, floorMaterial);
floor.position.y = 0;
floor.rotation.x = Math.PI / 2;
scene.add(floor);
//****************************************************************************************//

var octopusMatrix = {type: 'm4', value: new THREE.Matrix4().set(
        1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,3.0,
        0.0,0.0,1.0,0.0,
        0.0,0.0,0.0,1.0
    )};

//*****************************TEMPLATE CODE DO NOT MODIFY********************************//
// MATERIALS
var normalMaterial = new THREE.MeshNormalMaterial();
var octopusMaterial = new THREE.ShaderMaterial({
    uniforms:{
        octopusMatrix: octopusMatrix,
    },
});
var shaderFiles = [
    'glsl/octopus.vs.glsl',
    'glsl/octopus.fs.glsl'
];
new THREE.SourceLoader().load(shaderFiles, function(shaders) {
    octopusMaterial.vertexShader = shaders['glsl/octopus.vs.glsl'];
    octopusMaterial.fragmentShader = shaders['glsl/octopus.fs.glsl'];
})
// GEOMETRY
function loadOBJ(file, material, scale, xOff, yOff, zOff, xRot, yRot, zRot) {
    var onProgress = function(query) {
        if ( query.lengthComputable ) {
            var percentComplete = query.loaded / query.total * 100;
            console.log( Math.round(percentComplete, 2) + '% downloaded' );
        }
    };
    var onError = function() {
        console.log('Failed to load ' + file);
    };
    var loader = new THREE.OBJLoader();
    loader.load(file, function(object) {
        object.traverse(function(child) {
            if (child instanceof THREE.Mesh) {
                child.material = material;
            }
        });
        object.position.set(xOff,yOff,zOff);
        object.rotation.x= xRot;
        object.rotation.y = yRot;
        object.rotation.z = zRot;
        object.scale.set(scale,scale,scale);
        scene.add(object);
    }, onProgress, onError);

}
// We set octopus on (0,0,0) without scaling
// so we can change these values with transformation matrices.
loadOBJ('obj/Octopus_08_A.obj',octopusMaterial,1.0,0,0,0,0,0,0);

//***** Rotation helper functions ******//
function defineRotation_X(theta) {
    var cosTheta = Math.cos(theta);
    var sinTheta = Math.sin(theta);
    var mtx = new THREE.Matrix4().set(
        1.0,       0.0,      0.0,       0.0,
        0.0,       cosTheta, -sinTheta, 0.0,
        0.0,       sinTheta, cosTheta,  0.0,
        0.0,       0.0,      0.0,       1.0
    );
    return mtx;
}
function defineRotation_Y(theta) {
    var cosTheta = Math.cos(theta);
    var sinTheta = Math.sin(theta);
    var mtx = new THREE.Matrix4().set(
        cosTheta,  0.0,      sinTheta,  0.0,
        0.0,       1.0,      0.0,       0.0,
        -sinTheta, 0.0,      cosTheta,  0.0,
        0.0,       0.0,      0.0,       1.0
    );
    return mtx;
}
function defineRotation_Z(theta) {
    var cosTheta = Math.cos(theta);
    var sinTheta = Math.sin(theta);
    var mtx = new THREE.Matrix4().set(
        cosTheta,  -sinTheta, 0.0,       0.0,
        sinTheta,  cosTheta,  0.0,       0.0,
        0.0,       0.0,       1.0,       0.0,
        0.0,       0.0,       0.0,       1.0
    );
    return mtx;
}
//************************************************//
function addEyeAndPupil(material, eyeballTS, pupilTS, pupilTheta) {
    var eyegeo = new THREE.SphereGeometry(1.0,64,64);
    // Eyeball
    var eyeball = new THREE.Mesh(eyegeo, material);
    var eyeballMtx = new THREE.Matrix4().multiplyMatrices(
        octopusMatrix.value,
        eyeballTS
    );
    eyeball.setMatrix(eyeballMtx);
    scene.add(eyeball);
    // Pupil
    var pupilRT = defineRotation_Y(pupilTheta);
    var pupilTSR = new THREE.Matrix4().multiplyMatrices(
        pupilRT,
        pupilTS
    );
    var pupilMtx = new THREE.Matrix4().multiplyMatrices(
        eyeballMtx,
        pupilTSR
    );
    var pupil = new THREE.Mesh(eyegeo, material);
    pupil.setMatrix(pupilMtx);
    scene.add(pupil);
    return [eyeball, pupil];
}

// Left eye
var eyeballTS_L = new THREE.Matrix4().set(
    0.5,0.0,0.0,-0.2,
    0.0,0.5,0.0,4.1,
    0.0,0.0,0.5,0.92,
    0.0,0.0,0.0,1.0
);
var pupilTS_L = new THREE.Matrix4().set(
    0.35,0.0,0.0,0.0,
    0.0,0.35,0.0,0.0,
    0.0,0.0,0.15,-0.9,
    0.0,0.0,0.0,1.0
);
var theta_L = Math.PI * (130 /180.0);
// Right eye
var eyeballTS_R = new THREE.Matrix4().set(
    0.5,0.0,0.0,-0.2,
    0.0,0.5,0.0,4.1,
    0.0,0.0,0.5,-0.92,
    0.0,0.0,0.0,1.0
);
var pupilTS_R = new THREE.Matrix4().set(
    0.35,0.0,0.0,0.0,
    0.0,0.35,0.0,0.0,
    0.0,0.0,0.15,-0.9,
    0.0,0.0,0.0,1.0
);
var theta_R = Math.PI * (50 /180.0);
lefteye = addEyeAndPupil(normalMaterial, eyeballTS_L, pupilTS_L, theta_L);
eyeball_L = lefteye[0];
pupil_L = lefteye[1];
righteye = addEyeAndPupil(normalMaterial, eyeballTS_R, pupilTS_R, theta_R);
eyeball_R = righteye[0];
pupil_R = righteye[1];
//****************************************************************************************//

// Geometries of the arm
var j1 = new THREE.SphereGeometry(0.5,64,64);
var l1 = new THREE.CylinderGeometry(0.35, 0.45, 2, 64);
var j2 = new THREE.SphereGeometry(0.4, 64, 64);
var l2 = new THREE.CylinderGeometry(0.25, 0.35, 2, 64);
var j3 = new THREE.SphereGeometry(0.3, 64, 64);
var l3 = new THREE.CylinderGeometry(0.1, 0.25, 2, 64);

// ***** Q1 *****//
function addOneArm(angle_Y, angle_Z, socketPosition) {
    /* angle_Y, angle_Z determines the direction of the enire arm
     * i.e. you create a arm on world scene's origin, rotate along
     * y-axis, and z-axis by these angles will let you insert your
     * arm into the socket
    */
    var rotY = defineRotation_Y(angle_Y);
    var rotZ = defineRotation_Z(angle_Z);
    var rotMtx = new THREE.Matrix4().multiplyMatrices(rotY, rotZ);

    // Add joint1
    var joint1 = new THREE.Mesh(j1, normalMaterial);
    var joint1TrsMtx = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, socketPosition[0],
        0.0, 1.0, 0.0, socketPosition[1],
        0.0, 0.0, 1.0, socketPosition[2],
        0.0, 0.0, 0.0, 1.0
    );

    var joint1LocMtx = new THREE.Matrix4().multiplyMatrices(joint1TrsMtx, rotMtx,);
    var joint1Mtx = new THREE.Matrix4().multiplyMatrices(
        octopusMatrix.value,
        joint1LocMtx
    );
    joint1.setMatrix(joint1Mtx);
    scene.add(joint1);

    // Add link1
    var link1 = new THREE.Mesh(l1, normalMaterial);
    var link1Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 1.25,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    var link1Mtx = new THREE.Matrix4().multiplyMatrices(joint1.matrix, link1Trs);
    link1.setMatrix(link1Mtx);
    scene.add(link1);

    // Add joint2
    var joint2 = new THREE.Mesh(j2, normalMaterial);
    var joint2Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.8,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    var joint2Mtx = new THREE.Matrix4().multiplyMatrices(link1.matrix, joint2Trs);
    joint2.setMatrix(joint2Mtx);
    scene.add(joint2);

    // Add link2
    var link2 = new THREE.Mesh(l2, normalMaterial);
    var link2Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.8,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    var link2Mtx = new THREE.Matrix4().multiplyMatrices(joint2.matrix, link2Trs);
    link2.setMatrix(link2Mtx);
    scene.add(link2);

    // Add joint3
    var joint3 = new THREE.Mesh(j3, normalMaterial);
    var joint3Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.9,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    var joint3Mtx = new THREE.Matrix4().multiplyMatrices(link2.matrix, joint3Trs);
    joint3.setMatrix(joint3Mtx);
    scene.add(joint3);

    // Add link3
    var link3 = new THREE.Mesh(l3, normalMaterial);
    var link3Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.9,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    var link3Mtx = new THREE.Matrix4().multiplyMatrices(joint3.matrix, link3Trs);
    link3.setMatrix(link3Mtx);
    scene.add(link3);

    return [joint1, link1, joint2, link2, joint3, link3];
}

/* Now, call addOneArm() 4 times with 4 directions and
 * and 4 socket positions, you will add 4 arms on octopus
 * We return a tuple of joints and links, use them to
 * animate the octopus
*/

// Socket positions
socketPos1 = [-3.1, -0.35, 1.3];
socketPos2 = [-1.3, -0.35, 3.1];
socketPos3 = [1.3, -0.35, 3.1];
socketPos4 = [3.1, -0.35, 1.3];
socketPos5 = [3.1, -0.35, -1.3];
socketPos6 = [1.3, -0.35, -3.1];
socketPos7 = [-1.3, -0.35, -3.1];
socketPos8 = [-3.1, -0.35, -1.3];
//***** add 8 arms *****//
var arm1 = addOneArm(Math.PI*(-156.5/180), Math.PI*(-0.5), socketPos1);
var arm2 = addOneArm(Math.PI*(-110/180), Math.PI*(-0.5), socketPos2);
var arm3 = addOneArm(Math.PI*(-65/180), Math.PI*(-0.5), socketPos3);
var arm4 = addOneArm(Math.PI*(-18.5/180), Math.PI*(-0.5), socketPos4);
var arm5 = addOneArm(Math.PI*(18.5/180), Math.PI*(-0.5), socketPos5);
var arm6 = addOneArm(Math.PI*(65/180), Math.PI*(-0.5), socketPos6);
var arm7 = addOneArm(Math.PI*(110/180), Math.PI*(-0.5), socketPos7);
var arm8 = addOneArm(Math.PI*(156.5/180), Math.PI*(-0.5), socketPos8);

//***** animate arms *****/
function animateArm(arm, angle_Y, angle_Z, socketPosition) {
    joint1 = arm[0];
    link1 = arm[1];
    joint2 = arm[2];
    link2 = arm[3];
    joint3 = arm[4];
    link3 = arm[5];
    // *********************************** solve IK *********************************** //
    var upVector = new THREE.Vector3(0,-1,0);
    var end = [-3.1, -1.9, 2.3];

    var dist = Math.sqrt((socketPosition[0] - end[0])^2 + (end[1] + 0.35)^2 + (socketPosition[2] - end[2])^2);
    var fakeLegLen = 0.7 * dist;
    var fakeLeg = {pos: socketPosition, height: (fakeLegLen + j1.parameters.radius)};
    var joint3Pos = intersectPos(fakeLeg, (l3.parameters.height + j3.parameters.radius), end, upVector);
    var firstLink = {pos: socketPosition, height: l1.parameters.height + j1.parameters.radius};
    var joint2Pos = intersectPos(firstLink, (l2.parameters.height + j2.parameters.radius), joint3Pos, upVector);

    var link3Transform = lookAtTransformation(joint3Pos, end, upVector);
    link3.setMatrix(link3Transform);
    // var joint3Mtx = new THREE.Matrix4().multiplyMatrices(link3.matrix, new THREE.Matrix4().getInverse(link3Trs));
    // joint3.setMatrix(joint3Mtx);

    var link2Transform = lookAtTransformation(joint2Pos, joint3Pos, upVector);
    // link2.setMatrix(link2Transform);
    // link2.setMatrix(new THREE.Matrix4().multiplyMatrices(link2Transform, joint3.matrix));
    // var joint2Mtx = new THREE.Matrix4().multiplyMatrices(link2.matrix, new THREE.Matrix4().getInverse(link2Trs));
    // joint2.setMatrix(joint2Mtx);

    var link1Transform = lookAtTransformation(socketPosition, joint2Pos, upVector);
    // *********************************** solve IK *********************************** //

    var rotMtx = new THREE.Matrix4().multiplyMatrices(defineRotation_Y(angle_Y), defineRotation_Z(angle_Z));

    // Add joint1
    var joint1TrsMtx = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, socketPosition[0],
        0.0, 1.0, 0.0, socketPosition[1],
        0.0, 0.0, 1.0, socketPosition[2],
        0.0, 0.0, 0.0, 1.0
    );
    var joint1LocMtx = new THREE.Matrix4().multiplyMatrices(joint1TrsMtx, rotMtx);
    var joint1Mtx = new THREE.Matrix4().multiplyMatrices(
        octopusMatrix.value,
        joint1LocMtx
    );
    joint1.setMatrix(joint1Mtx);

    // Add link1
    var link1Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 1.25,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    var link1Mtx = new THREE.Matrix4().multiplyMatrices(joint1.matrix, link1Trs);
    link1.setMatrix(link1Mtx);

    // Add joint2
    // var joint2Trs = new THREE.Matrix4().set(
    //     1.0, 0.0, 0.0, 0.0,
    //     0.0, 1.0, 0.0, 0.8,
    //     0.0, 0.0, 1.0, 0.0,
    //     0.0, 0.0, 0.0, 1.0
    // );
    var joint2Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, joint2Pos[0],
        0.0, 1.0, 0.0, joint2Pos[1],
        0.0, 0.0, 1.0, joint2Pos[2],
        0.0, 0.0, 0.0, 1.0
    );


    // Add link2
    var link2Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.8,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );


    // Add joint3
    var joint3Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.9,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    // var joint3Trs = new THREE.Matrix4().set(
    //     1.0, 0.0, 0.0, joint3Pos[0],
    //     0.0, 1.0, 0.0, joint3Pos[1],
    //     0.0, 0.0, 1.0, joint3Pos[2],
    //     0.0, 0.0, 0.0, 1.0
    // );
    // var joint3Mtx = new THREE.Matrix4().multiplyMatrices(joint3Trs);


    // Add link3
    var link3Trs = new THREE.Matrix4().set(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.9,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
    var link3Mtx = new THREE.Matrix4().multiplyMatrices(joint3.matrix, link3Trs);
    // link3.setMatrix(link3Mtx);
    joint3.setMatrix(new THREE.Matrix4().multiplyMatrices(link3.matrix, new THREE.Matrix4().getInverse(link3Trs)));

    var link2Mtx = new THREE.Matrix4().multiplyMatrices(joint3.matrix, new THREE.Matrix4().getInverse(joint3Trs));
    link2.setMatrix(link2Mtx);

    var joint2Mtx = new THREE.Matrix4().multiplyMatrices(link2.matrix, new THREE.Matrix4().getInverse(link2Trs));
    joint2.setMatrix(joint2Mtx);

    // *********************************** solve IK *********************************** //
    // link1.setMatrix(new THREE.Matrix4().multiplyMatrices(link1Transform, joint1.matrix));

    // link1.setMatrix(new THREE.Matrix4().multiplyMatrices(link1Transform, joint2.matrix));
    // var joint1Mtx = new THREE.Matrix4().multiplyMatrices(link1.matrix, new THREE.Matrix4().getInverse(link1Trs));
    // joint1.setMatrix(joint1Mtx);
    // joint1.setMatrix(link1Transform);
}

//***** calculate the intersection points of two links *****//
//***** l1 and l2 are the two links with their origins and length *****/
function intersectPos(l1, l2, endEffector, upVector) {
    var vecA = new THREE.Vector3(endEffector[0] - l1.pos[0], endEffector[1] - l1.pos[1], endEffector[2] - l1.pos[2]);
    var lenA = Math.sqrt((Math.pow(vecA.x, 2) + Math.pow(vecA.y, 2) + Math.pow(vecA.z, 2)));
    var o_S2 = lenA - l2;
    var lenC = l1.height - o_S2;
    var lenB = o_S2 + (lenC * 0.5);
    var lenD = (lenB * lenC) / lenA;

    var posJ0 = [(vecA.x / lenA) * (o_S2 + lenD), (vecA.y / lenA) * (o_S2 + lenD), (vecA.z / lenA) * (o_S2 + lenD)];
    var posJ = [posJ0[0] + l1.pos[0], posJ0[1] + l1.pos[1], posJ0[2] + l1.pos[2]];
    // planeNormal = vecA X upVector
    var planeNorm = new THREE.Vector3(
        vecA.y * upVector.z - vecA.z * upVector.y,
        vecA.z * upVector.x - vecA.x * upVector.z,
        vecA.x * upVector.y - vecA.y * upVector.x);
    // JVector = normalize (planeNormal X vecA)
    var vecJ0 = new THREE.Vector3(
        planeNorm.y * vecA.z - planeNorm.z * vecA.y,
        planeNorm.z * vecA.x - planeNorm.x * vecA.z,
        planeNorm.x * vecA.y - planeNorm.y * vecA.x);
    var lenJ = Math.sqrt((Math.pow(vecJ0.x, 2) + Math.pow(vecJ0.y, 2) + Math.pow(vecJ0.z, 2)));
    var vecJ = new THREE.Vector3(vecJ0.x/lenJ, vecJ0.y/lenJ, vecJ0.z/lenJ);

    var oppositeLeg = Math.sqrt((Math.pow(l1.height, 2) - Math.pow(o_S2 + lenD,2)));

    var intersection = [vecJ.x * oppositeLeg + posJ[0], vecJ.y * oppositeLeg + posJ[1], vecJ.z * oppositeLeg + posJ[2]];
    return intersection;
}

//***** calculate the intersection points of two links *****//
function lookAtTransformation(objPos, targetPos, upVector){
    var transformMatrix = new THREE.Matrix4();
    // lookAtVector = normalize (targetPos - objPos)
    var lookAtVec0 = new THREE.Vector3(targetPos[0] - objPos[0], targetPos[1] - objPos[1], targetPos[2] - objPos[2]);
    var lenLookAtVec = Math.sqrt((Math.pow(lookAtVec0.x, 2) + Math.pow(lookAtVec0.y, 2) + Math.pow(lookAtVec0.z, 2)));
    var lookAtVec = new THREE.Vector3(lookAtVec0.x/lenLookAtVec, lookAtVec0.y/lenLookAtVec, lookAtVec0.z/lenLookAtVec);

    // column1 = normalize (lookAtVec X upVector)
    var col10 = new THREE.Vector3(
        lookAtVec.y * upVector.z - lookAtVec.z * upVector.y,
        lookAtVec.z * upVector.x - lookAtVec.x * upVector.z,
        lookAtVec.x * upVector.y - lookAtVec.y * upVector.x);
    var lenCol1 = Math.sqrt((Math.pow(col10.x, 2) + Math.pow(col10.y, 2) + Math.pow(col10.z, 2)));
    var col1 = new THREE.Vector3(col10.x/lenCol1, col10.y/lenCol1, col10.z/lenCol1);

    // column2 = normalize (col1 X lookAtVec)
    var col20 = new THREE.Vector3(
        col1.y * lookAtVec.z - col1.z * lookAtVec.y,
        col1.z * lookAtVec.x - col1.x * lookAtVec.z,
        col1.x * lookAtVec.y - col1.y * lookAtVec.x);
    var lenCol2 = Math.sqrt((Math.pow(col20.x, 2) + Math.pow(col20.y, 2) + Math.pow(col20.z, 2)));
    var col2 = new THREE.Vector3(col20.x/lenCol2, col20.y/lenCol2, col20.z/lenCol2);

    transformMatrix.set(
        col1.x, col2.x, lookAtVec.x, objPos[0],
        col1.y, col2.y, lookAtVec.y, objPos[1],
        col1.z, col2.z, lookAtVec.z, objPos[2],
        0.0, 0.0, 0.0, 1.0
    );
    return transformMatrix;
}

var time = 0;
var x_pos = 0;
var y_pos = 0;
var initalMtx = octopusMatrix.value;
function updateBody() {
    switch(channel)
    {
        case 0:
            break;

        case 1: {
            // function for the x_pos & y_pos of the endEffector
            if (time < 5) {
                // movement along the parabola
                x_pos = -3.1 - 0.6 * time;
                y_pos = -(x_pos - 4.6)^2 - 1.5;
            } else {
                // movement along the line on the floor
                x_pos = -6.1 + 0.6 * time;
                y_pos = -3.0;
            }

            animateArm(arm1, Math.PI*(-156.5/180), Math.PI*(-0.5), socketPos1);

            // Animate Right Eye (eyeball and pupil)
            eyeball_R.setMatrix(new THREE.Matrix4().multiplyMatrices(
                octopusMatrix.value,
                eyeballTS_R
            ));
            pupil_R.setMatrix(new THREE.Matrix4().multiplyMatrices(
                new THREE.Matrix4().multiplyMatrices(
                    octopusMatrix.value,
                    eyeballTS_R
                ),
                new THREE.Matrix4().multiplyMatrices(
                    defineRotation_Y(theta_R),
                    pupilTS_R
                )
            ));
            scene.add(eyeball_R);
            scene.add(pupil_R);
            // Animate Left Eye (eyeball and pupil)
            oct_eye_L = new THREE.Matrix4().multiplyMatrices(
                octopusMatrix.value,
                eyeballTS_L
            );
            pupil_L_TSR = new THREE.Matrix4().multiplyMatrices(
                defineRotation_Y(theta_L),
                pupilTS_L
            );
            oct_pupil = new THREE.Matrix4().multiplyMatrices(
                oct_eye_L,
                pupil_L_TSR
            );
            eyeball_L.setMatrix(oct_eye_L);
            pupil_L.setMatrix(oct_pupil);
        }

        case 2: {

        };

        default:
            break;
    }
}


// LISTEN TO KEYBOARD
var keyboard = new THREEx.KeyboardState();
var channel = 0;
function checkKeyboard() {
    for (var i=0; i<6; i++)
    {
        if (keyboard.pressed(i.toString()))
        {
            channel = i;
            break;
        }
    }
}


// SETUP UPDATE CALL-BACK
function update() {
    checkKeyboard();
    if (time < 10) {
        time = time + 1;
    } else {
        time = 0;
    }
    updateBody();
    requestAnimationFrame(update);
    renderer.render(scene, camera);
}

update();