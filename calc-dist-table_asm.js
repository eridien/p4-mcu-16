// js utility to calculate decelerations distance table
// and prepare the .asm file for inclusion in the mcu-motors mcu code

const CLK_TICKS_PER_SEC = 25000;
const stepsPerPulse = 1;  // assumes ustep of 1/8 (3)

let speed = 4000        // assumes 100 mm/sec 
let accel = 40000;      // assumes 1000 mm/sec/sec 

let dist = 0;

for (; speed > 0; dist += stepsPerPulse, speed -= Math.floor(accel/speed)) {
  console.log(speed, dist);
}

console.log(dist);
