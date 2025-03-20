// let path = [];
// let obstacles = [
//   { type: "sphere", x: 200, y: 200, z: 0, radius: 50 },
//   { type: "cuboid", x: 400, y: 300, z: 100, width: 100, height: 200, depth: 50 },
// ];
// let bounds = {
//   min_x: 0, max_x: 800,
//   min_y: 0, max_y: 600,
//   min_z: -100, max_z: 100
// };
// let dynamic_obstacles = [
//   { type: "sphere", x: 400, y: 500, z: 50, radius: 40, speedX: 1, speedY: 0.7, speedZ: 0.5 },
// ];
// let droneX = 100, droneY = 100, droneZ = 0;

// function setup() {
//   createCanvas(800, 600, WEBGL);
//   perspective(PI / 3.0, width / height, 0.1, 1000);
// }

// function draw() {
//   background(220);
//   lights();
//   orbitControl();  // Allows mouse interaction with 3D scene

//   // Move dynamic obstacles
//   for (let obs of dynamic_obstacles) {
//     obs.x += obs.speedX;
//     obs.y += obs.speedY;
//     obs.z += obs.speedZ;
//     if (obs.x + obs.radius > bounds.max_x || obs.x - obs.radius < bounds.min_x) obs.speedX *= -1;
//     if (obs.y + obs.radius > bounds.max_y || obs.y - obs.radius < bounds.min_y) obs.speedY *= -1;
//     if (obs.z + obs.radius > bounds.max_z || obs.z - obs.radius < bounds.min_z) obs.speedZ *= -1;
//   }

//   // Draw static obstacles
//   for (let obs of obstacles) {
//     push();
//     translate(obs.x - width / 2, obs.y - height / 2, obs.z);
//     if (obs.type === "sphere") {
//       sphere(obs.radius);
//     } else if (obs.type === "cuboid") {
//       box(obs.width, obs.height, obs.depth);
//     }
//     pop();
//   }

//   // Draw dynamic obstacles
//   for (let obs of dynamic_obstacles) {
//     push();
//     fill(255, 165, 0);  // Orange color
//     translate(obs.x - width / 2, obs.y - height / 2, obs.z);
//     sphere(obs.radius);
//     pop();
//   }

//   // Update drone position
//   if (path.length > 0) {
//     droneX = path[0][0];
//     droneY = path[0][1];
//     droneZ = path[0][2];
//     path.shift();  // Move drone along path
//   }

//   // Draw drone
//   push();
//   fill(0, 255, 0);
//   translate(droneX - width / 2, droneY - height / 2, droneZ);
//   rotateY(frameCount * 0.01);
//   cone(20, 40);
//   pop();

//   // Replan every 30 frames
//   if (frameCount % 30 === 0) {
//     planPath();
//   }
// }

// function planPath() {
//   const data = {
//     start: [100, 100, 0],  // 3D start
//     goal: [700, 500, 50],  // 3D goal
//     obstacles: obstacles.concat(dynamic_obstacles),
//     bounds: bounds
//   };

//   httpPost(
//     "http://127.0.0.1:5000/plan_path",
//     "json",
//     data,
//     (response) => {
//       path = response.path;
//       console.log("Path received:", path);
//     },
//     (error) => {
//       console.error("Error:", error);
//     }
//   );
// }


let path = [];
let obstacles = [
  { type: "sphere", x: 200, y: 200, z: 0, radius: 30 },  // Reduced radius
  { type: "cuboid", x: 400, y: 300, z: 50, width: 80, height: 150, depth: 40 }  // Reduced size
];
let bounds = {
  min_x: 0, max_x: 800,
  min_y: 0, max_y: 600,
  min_z: -100, max_z: 100
};
let dynamic_obstacles = [];  // Temporarily disabled for debugging
let droneX = 100, droneY = 100, droneZ = 0;
let start = [100, 100, 0];
let goal = [700, 500, 50];

function setup() {
  createCanvas(800, 600, WEBGL);
  perspective(PI / 3.0, width / height, 0.1, 1000);
  // Initial path planning
  planPath();
}

function draw() {
  background(220);
  lights();
  orbitControl();

  // Draw 3D grid
  drawGrid();

  // Move dynamic obstacles (if any)
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
    fill(255, 165, 0);
    translate(obs.x - width / 2, obs.y - height / 2, obs.z);
    sphere(obs.radius);
    pop();
  }

  // Draw path
  if (path.length > 0) {
    beginShape();
    stroke(0, 0, 255); // Blue path
    strokeWeight(2);
    noFill();
    for (let point of path) {
      vertex(point[0] - width / 2, point[1] - height / 2, point[2]);
    }
    endShape();
  }

  // Update drone position
  if (path.length > 0) {
    let nextPoint = path[0];
    droneX = nextPoint[0];
    droneY = nextPoint[1];
    droneZ = nextPoint[2];
    // Move to the next point only if the drone is close enough
    let dist = Math.sqrt(
      Math.pow(droneX - nextPoint[0], 2) +
      Math.pow(droneY - nextPoint[1], 2) +
      Math.pow(droneZ - nextPoint[2], 2)
    );
    if (dist < 5) {  // Threshold to consider the point reached
      path.shift();
    }
  }

  // Draw drone
  push();
  fill(0, 255, 0);
  translate(droneX - width / 2, droneY - height / 2, droneZ);
  rotateY(frameCount * 0.01);
  cone(20, 40);
  pop();

  // Draw goal marker
  push();
  fill(255, 0, 0);
  translate(goal[0] - width / 2, goal[1] - height / 2, goal[2]);
  sphere(10);
  pop();

  // Replan every 60 frames (slower to reduce load)
  if (frameCount % 60 === 0) {
    planPath();
  }
}

// Function to draw a 3D grid
function drawGrid() {
  stroke(150);
  strokeWeight(1);
  let gridSize = 100;
  // X-Y plane grid at z = min_z
  for (let x = bounds.min_x; x <= bounds.max_x; x += gridSize) {
    line(x - width / 2, bounds.min_y - height / 2, bounds.min_z, x - width / 2, bounds.max_y - height / 2, bounds.min_z);
  }
  for (let y = bounds.min_y; y <= bounds.max_y; y += gridSize) {
    line(bounds.min_x - width / 2, y - height / 2, bounds.min_z, bounds.max_x - width / 2, y - height / 2, bounds.min_z);
  }
}

function planPath() {
  const data = {
    start: start,
    goal: goal,
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
      if (path.length === 0) {
        console.warn("No path found. Check obstacles, bounds, or planner parameters.");
      }
    },
    (error) => {
      console.error("Error:", error);
    }
  );
}