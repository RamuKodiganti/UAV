let path = [];
let obstacles = [
  { type: "sphere", x: 200, y: 200, z: 0, radius: 50 },
  { type: "cuboid", x: 400, y: 300, z: 100, width: 100, height: 200, depth: 50 },
];
let bounds = {
  min_x: 0, max_x: 800,
  min_y: 0, max_y: 600,
  min_z: -100, max_z: 100
};
let dynamic_obstacles = [
  { type: "sphere", x: 400, y: 500, z: 50, radius: 40, speedX: 1, speedY: 0.7, speedZ: 0.5 },
];
let droneX = 100, droneY = 100, droneZ = 0;

function setup() {
  createCanvas(800, 600, WEBGL);
  perspective(PI / 3.0, width / height, 0.1, 1000);
}

function draw() {
  background(220);
  lights();
  orbitControl();  // Allows mouse interaction with 3D scene

  // Move dynamic obstacles
  for (let obs of dynamic_obstacles) {
    obs.x += obs.speedX;
    obs.y += obs.speedY;
    obs.z += obs.speedZ;
    if (obs.x + obs.radius > bounds.max_x || obs.x - obs.radius < bounds.min_x) obs.speedX *= -1;
    if (obs.y + obs.radius > bounds.max_y || obs.y - obs.radius < bounds.min_y) obs.speedY *= -1;
    if (obs.z + obs.radius > bounds.max_z || obs.z - obs.radius < bounds.min_z) obs.speedZ *= -1;
  }

  // Draw static obstacles
  for (let obs of obstacles) {
    push();
    translate(obs.x - width / 2, obs.y - height / 2, obs.z);
    if (obs.type === "sphere") {
      sphere(obs.radius);
    } else if (obs.type === "cuboid") {
      box(obs.width, obs.height, obs.depth);
    }
    pop();
  }

  // Draw dynamic obstacles
  for (let obs of dynamic_obstacles) {
    push();
    fill(255, 165, 0);  // Orange color
    translate(obs.x - width / 2, obs.y - height / 2, obs.z);
    sphere(obs.radius);
    pop();
  }

  // Update drone position
  if (path.length > 0) {
    droneX = path[0][0];
    droneY = path[0][1];
    droneZ = path[0][2];
    path.shift();  // Move drone along path
  }

  // Draw drone
  push();
  fill(0, 255, 0);
  translate(droneX - width / 2, droneY - height / 2, droneZ);
  rotateY(frameCount * 0.01);
  cone(20, 40);
  pop();

  // Replan every 30 frames
  if (frameCount % 30 === 0) {
    planPath();
  }
}

function planPath() {
  const data = {
    start: [100, 100, 0],  // 3D start
    goal: [700, 500, 50],  // 3D goal
    obstacles: obstacles.concat(dynamic_obstacles),
    bounds: bounds
  };

  httpPost(
    "http://127.0.0.1:5000/plan_path",
    "json",
    data,
    (response) => {
      path = response.path;
      console.log("Path received:", path);
    },
    (error) => {
      console.error("Error:", error);
    }
  );
}