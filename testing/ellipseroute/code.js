function circle(ctx, x, y, color) {
	ctx.strokeStyle = color;
	ctx.fillStyle = color;
	ctx.beginPath();
	ctx.arc(x, y, 5, 0, 2*Math.PI, 0);
	ctx.fill();
}

// only axis-aligned so far, i.e. facing horizontally/vertically at points
// no straight paths supported, each node is a turn
// dir: where we're facing at starting point
// 0=up, 1=left, 2=down, 3=right
function ellipse(ctx, ax, ay, bx, by, dir) {
	// TODO: generalize this monster
	var quarter = Math.PI/2;
	var ox, oy, a, b, dir;
	if (dir == 0) {
		ox = bx;
		oy = ay;

		a = ax - ox;
		b = by - oy;

		dir = 1;
	} else if (dir == 1) {
		ox = ax;
		oy = by;

		a = bx - ox;
		b = ay - by;

		dir = 2;
	} else if (dir == 2) {
		ox = bx;
		oy = ay;

		a = ax - bx;
		b = by - ay;

		dir = 3;
	} else if (dir == 3) {
		ox = ax;
		oy = by;

		a = bx - ax;
		b = ay - by;

		dir = 0;
	}

	ctx.beginPath();
	for (var i = 0; i <= quarter; i += quarter / 10) {
		var x = a * Math.cos(i);
		var y = b * Math.sin(i);
		ctx.lineTo(ox + x, oy + y);
	}
	ctx.stroke();
	return dir;
}

function route(ctx, pts) {
	var ax = pts[0].x;
	var ay = pts[0].y;
	var dir = 3; // robot starts facing right
	circle(ctx, ax, ay, 'red');
	for (var i = 1; i < pts.length; i++) {
		var bx = pts[i].x;
		var by = pts[i].y;
		circle(ctx, bx, by, 'red');
		dir = ellipse(ctx, ax, ay, bx, by, dir);
		ax = bx;
		ay = by;
	}
}

// http://stackoverflow.com/questions/55677/how-do-i-get-the-coordinates-of-a-mouse-click-on-a-canvas-element
function relMouseCoords(event) {
	var totalOffsetX = 0;
	var totalOffsetY = 0;
	var canvasX = 0;
	var canvasY = 0;
	var currentElement = canvas; // this didn't work, lol

	do {
		totalOffsetX += currentElement.offsetLeft - currentElement.scrollLeft;
		totalOffsetY += currentElement.offsetTop - currentElement.scrollTop;
	}
	while(currentElement = currentElement.offsetParent);

	canvasX = event.pageX - totalOffsetX;
	canvasY = event.pageY - totalOffsetY;

	return {x:canvasX, y:canvasY}
}

// "parsed" from the xml file
function poles(ctx) {
	var coords = [
		3, 3.1,
		3, 2.3,

		3.2, 1.2,
		4, 1.2,

		2.2, 1.9,
		2.2, 1.1,

		1, 0.3,
		1, 1.1,

		1.5, 2.8,
		1.5, 2
	];
	for (var i = 0; i < coords.length; i += 2) {
		var x = coords[i];
		var y = coords[i + 1];
		circle(ctx, x * scale, h - y * scale, 'blue');
	}
}

function draw(ctx) {
	ctx.fillStyle = 'black';
	ctx.fillRect(0, 0, w, h);
	poles(ctx);
	route(ctx, pts);
}

function clickEvent(ev) {
	pos = relMouseCoords(ev);
	pts.push(pos);
	draw(ctx);
}

var pts = [];
var ctx;
var canvas;
var w, h;
var scale = 150;

window.onload = function() {
	 canvas = document.getElementById('renderwindow');

	w = canvas.width;
	h = canvas.height;

	// robot start location
	pts.push({x: 3 * scale, y: h - 2.7 *  scale});

	ctx = canvas.getContext('2d');
	draw(ctx);

	canvas.addEventListener('click', clickEvent);
}
