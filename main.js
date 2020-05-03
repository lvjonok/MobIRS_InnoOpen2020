Robot = function(leftMotor, rightMotor, leftLineSensor, rightLineSensor, leftSideSensor, rightSideSensor, middleLineSensor){
	this.motorLeft = brick.motor(leftMotor);
	this.motorRight = brick.motor(rightMotor);
	this.encoderLeft = brick.encoder("E" + leftMotor[1]);
	this.encoderRight = brick.encoder("E" + rightMotor[1]);
	this.llS = brick.sensor(leftLineSensor).read;
	this.rlS = brick.sensor(rightLineSensor).read;
	this.lsS = brick.sensor(leftSideSensor).read;
	this.rsS = brick.sensor(rightSideSensor).read;
	this.mlS = brick.sensor(middleLineSensor).read;
	this.sensors = [this.lsS, this.llS, this.mlS, this.rlS, this.rsS];
	this.track_sm = 17.5;
	this.wheelD_sm = 5.6;
	this.resetEncoders = function(){
		this.encoderLeft.reset();
		this.encoderRight.reset();
	};
	this.resetEncoders();
	this.moveEncoders = function(enc, speed){
		var seL = this.encoderLeft.read();
		var seR = this.encoderRight.read();
		var tL = seL + enc;
		var tR = seR + enc;
		
		var eL = this.encoderLeft.read();
		var eR = this.encoderRight.read();
		while (eL - seL < enc || eR - seR < enc){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed(speed, speed);
		}
		var st = Date.now();
		while (Date.now() - st < 250){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((tL - eL) * 1.4, (tR - eR) * 1.4);
		}
		this.setSpeed(0, 0);
	};
	this.setSpeed = function(lS, rS){
		if (rS == undefined) rS = lS;
		this.motorLeft.setPower(lS);
		this.motorRight.setPower(rS);
	};
	this.getSensors = function(){
		return [this.llS(), this.rlS(), this.lsS(), this.rsS()];
	};
	this.driveLine2 = function(){
		var error = this.llS() - this.rlS();
		var last_error = error;
		var kP = 0.65;
		var kD = 1.5;
		var speed = 90;
		print(this.llS() + " " + this.rlS());
		print(this.lsS() + " " + this.rsS());
		// return;

		while (true){
			print(this.getSensors()); 
			errorLeft = this.llS() - 50;
			errorRight = this.rlS() - 50;
			if (this.lsS() < 50){ // left side sensor is on white
				// noth
			} else {
				errorLeft = errorLeft * -1;
			}
			if (this.rsS() < 50){ // right side sensor is on white
				// noth
			} else {
				errorRight = errorRight * -1;
			}
			error = errorLeft - errorRight;
			this.setSpeed(speed - error * kP - (error - last_error) * kD, speed + error * kP + (error - last_error) * kD);
			wait(5);
			last_error = error;
		}			
	};
	this.driveSector = function(){
		// var const_enc = 750; // encoders to reach next cell
		var const_enc = (52.5 * (2 / 3) / (this.wheelD_sm * Math.PI) * 360);
		var speed = 90;
		var map = [1, undefined, undefined, undefined]; // contains available movements in this cell, direction is regarding moving
		/*
			map descriptor:
				1 - way available
				-1 - no way
		*/
		var seL = this.encoderLeft.read();
		var seR = this.encoderRight.read();
		var teL = seL + const_enc;
		var teR = seR + const_enc;

		var surface_color = [undefined, undefined, undefined, undefined, undefined];
		var surface_history = [[],[],[],[],[]];

		for (var i = 0; i < 5; i++){
			var sens = this.sensors[i]()
			surface_color[i] = sens < 50 ? 1 : -1;
			surface_history[i].push(surface_color[i]);
		}

		var llS = this.llS();
		var rlS = this.rlS();
		var lsS = this.lsS();
		var rsS = this.rsS();
		
		var last_lsS = lsS;
		var last_rsS = rsS;

		var surface_left_color = lsS < 50 ? 1 : -1; // 1 - white, -1 - black
		var surface_left_history = [surface_left_color];

		var surface_right_color = rsS < 50 ? 1 : -1;
		var surface_right_history = [surface_right_color];

		var surface_left_in_color = llS < 50 ? 1 : -1;
		var surface_left_in_history = [surface_left_in_color];

		var surface_right_in_color = rlS < 50 ? 1 : -1;
		var surface_right_in_history = [surface_right_in_color];

		var ll = this.encoderLeft.read() - seL;
		var lr = this.encoderRight.read() - seR;
		while (ll < const_enc || eR - seR < const_enc){
			var llS = this.llS();
			var rlS = this.rlS();
			var lsS = this.lsS();
			var rsS = this.rsS();
			var lSurfaceS = this.lSurfaceS();
			var rSurfaceS = this.rSurfaceS();

			for (var i = 0; i < 5; i++){
				var sens = this.sensors[i]();
				
			}


			if (lsS < 50 && surface_left_color == -1){
				surface_left_color = 1;
				surface_left_history.push(surface_left_color);
			} else if (lsS > 50 && surface_left_color == 1){
				surface_left_color = -1;
				surface_left_history.push(surface_left_color);
			}

			if (rsS < 50 && surface_right_color == -1){
				surface_right_color = 1;
				surface_right_history.push(surface_right_color);
			} else if (rsS > 50 && surface_right_color == 1){
				surface_right_color = -1;
				surface_right_history.push(surface_right_color);
			}

			if (llS < 50 && surface_left_in_color == -1){
				surface_left_in_color = 1;
				surface_left_in_history.push(surface_left_in_color);
			} else if (llS > 50 && surface_left_in_color == 1){
				surface_left_in_color = -1;
				surface_left_in_history.push(surface_left_in_color);
			}

			if (rlS < 50 && surface_right_in_color == -1){
				surface_right_in_color = 1;
				surface_right_in_history.push(surface_right_in_color);
			} else if (rlS > 50 && surface_right_in_color == 1){
				surface_right_in_color = -1;
				surface_right_in_history.push(surface_right_in_color);
			}

			ll = this.encoderLeft.read() - seL;
			lr = this.encoderRight.read() - seR;
			this.setSpeed(speed - (ll - lr) * 2, speed + (ll - lr) * 2);
			last_lsS = lsS;
			last_rsS = rsS;
			// wait(1);
		}
		surface_left_history.push(surface_left_color);
		surface_right_history.push(surface_right_color);
		print(surface_left_history);
		print(surface_right_history);
		print(surface_left_in_history);
		print(surface_right_in_history);
		var st = Date.now();
		while (Date.now() - st < 250){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((teL - eL) * 3, (teR - eR) * 3);
		}
		this.setSpeed(0, 0);
	};
};

var wait = script.wait;
var abs = Math.abs;
var max = Math.max;
var min = Math.min;

var main = function(){
	robot = new Robot("M4", "M3", "A1", "A2", "A3", "A4", "A5");

	robot.driveSector();
}

main();