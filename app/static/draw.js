var canvas;
var context;

var init = [5, 5];
var goal = [395, 395];

var x0 = 0;
var y0 = 0;
var x1 = 0;
var y1 = 0;
var paint = 0;

const CLEAR_COLOR = "#FFFFFF";
const STROKE_COLOR = "#000000";
const LINE_WIDTH = 16;

function clear(ctx = context, fill = true) {
    ctx.clearRect(0, 0, context.canvas.width, context.canvas.height);
    if (fill) {
        ctx.fillStyle = CLEAR_COLOR;
        ctx.fillRect(0, 0, context.canvas.width, context.canvas.height);
    }
}

function drawEnds() {
    // redraw init/goal
    ctx = document.getElementById('overlay').getContext("2d");
    clear(ctx, false);
    drawCircle(init[0], init[1], 16, '#FF0000', 4, ctx);
    drawCircle(goal[0], goal[1], 16, '#0000FF', 4, ctx);
}

/**
- Preparing the Canvas: Basic functions
**/
function setup_canvas() {
    canvas = document.getElementById('canvas');
    context = document.getElementById('canvas').getContext("2d");

    clear();
    drawEnds();
    $('#canvas').mousedown(function(e) {
        const rect = canvas.getBoundingClientRect();
        const scaleX = canvas.width / rect.width;
        const scaleY = canvas.height / rect.height;
        const mouseX = scaleX * (e.clientX - rect.left);
        const mouseY = scaleY * (e.clientY - rect.top);

        console.log(e);
        if (e.ctrlKey === true) {
            init = [mouseX, mouseY];
            drawEnds();
            return;
        }

        if (e.altKey === true) {
            goal = [mouseX, mouseY];
            drawEnds();
            return;
        }

        x0 = mouseX;
        y0 = mouseY;
        x1 = mouseX;
        y1 = mouseY;

        if (e.button == 0) {
            paint = 1;
        } else if (e.button == 1) {
            paint = -1;
        }
        if (paint != 0) {
            let col = (paint === -1 ? "#FFFFFF" : "#000000");
            drawLine(x0, y0, x1, y1, col);
        }
    })

    $('#canvas').mousemove(function(e) {
        if (paint != 0) {
            // roll
            x0 = x1;
            y0 = y1;

            const rect = canvas.getBoundingClientRect();
            const scaleX = canvas.width / rect.width;
            const scaleY = canvas.height / rect.height;
            const mouseX = scaleX * (e.clientX - rect.left);
            const mouseY = scaleY * (e.clientY - rect.top);

            x1 = mouseX;
            y1 = mouseY;

            let col = (paint === -1 ? "#FFFFFF" : "#000000");
            drawLine(x0, y0, x1, y1, col);
        }
    })

    $('#canvas').mouseup(function(e) {
        paint = 0;
    })
}

function drawCircle(x, y, radius, color, width, ctx) {
    // Configure context (again)
    ctx.strokeStyle = color;
    ctx.lineJoin = "round";
    ctx.lineWidth = width;

    ctx.beginPath();
    ctx.arc(x, y, radius, 0, 2 * Math.PI);
    ctx.stroke();
}

function drawLine(src_x, src_y, dst_x, dst_y, color = STROKE_COLOR, width = LINE_WIDTH, ctx = context) {
    // Configure context (again)
    ctx.strokeStyle = color;
    ctx.lineJoin = "round";
    ctx.lineWidth = width;

    ctx.beginPath();
    ctx.moveTo(src_x, src_y);
    ctx.lineTo(dst_x, dst_y);
    ctx.closePath();
    ctx.stroke();
}
