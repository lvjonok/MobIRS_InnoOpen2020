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
	this.target_encoders = {'left' : 0, 'right' : 0};
	this.resetEncoders();
	this.sm2cpr = function(sm){
		return sm / (this.wheelD_sm * Math.PI) * 360;
	};
	this.sign = function(number){
		if (number > 0) return 1;
		if (number < 0) return -1;
		return 0;
	}
	this.moveEncoders = function(enc, speed){
		// var seL = this.encoderLeft.read();
		// var seR = this.encoderRight.read();
		var tL = this.target_encoders['left'] + enc;
		var tR = this.target_encoders['right'] + enc;

		this.target_encoders['left'] += enc;
		this.target_encoders['right'] += enc;
		
		var eL = this.encoderLeft.read();
		var eR = this.encoderRight.read();
		while ((tL > eL && tR > eR && enc > 0)||(tL < eL && tR < eR && enc < 0)){ // (eL - seL < enc || eR - seR < enc){
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
	this.turnDegrees = function(degrees){
		var const_enc = this.sm2cpr(this.track_sm * Math.PI * (degrees / 360));
		var direction = this.sign(const_enc);
		// print(const_enc);
		// print(direction);
		this.target_encoders['left'] += const_enc;
		this.target_encoders['right'] -= const_enc;

		var eL = this.encoderLeft.read();
		var eR = this.encoderRight.read();
		
		while ((this.target_encoders['left'] > eL && this.target_encoders['right'] < eR && direction == 1) || (this.target_encoders['left'] < eL && this.target_encoders['right'] > eR && direction == -1)){
			eL = this.encoderLeft.read();
			eR = this.encoderRight.read();
			this.setSpeed(80 * direction, -80 * direction);
		}
		var st = Date.now();
		while (Date.now() - st < 500){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((this.target_encoders['left'] - eL) * 1.4, (this.target_encoders['right'] - eR) * 1.4);
		}
		this.setSpeed(0, 0);
	};
	this.turnDegreesOneWheel = function(degrees){
		var const_enc = this.sm2cpr(this.track_sm * Math.PI * (degrees / 180));
		var direction = this.sign(const_enc);
		// print(const_enc);
		// print(direction);

		const_enc = abs(const_enc);

		if (direction == 1){
			this.target_encoders['left'] += const_enc;
		} else {
			this.target_encoders['right'] += const_enc;
		}
		
		var eL = this.encoderLeft.read();
		var eR = this.encoderRight.read();
		
		while ((this.target_encoders['left'] > eL && direction == 1) || (this.target_encoders['right'] > eR && direction == -1)){
			eL = this.encoderLeft.read();
			eR = this.encoderRight.read();
			if (direction == 1){
				this.setSpeed(70, 0);
			} else {
				this.setSpeed(0, 70);
			}
		}
		// this.setSpeed(0, 0);
		// wait(3000);
		var st = Date.now();
		while (Date.now() - st < 500){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((this.target_encoders['left'] - eL) * 10, (this.target_encoders['right'] - eR) * 10);
		}
		this.setSpeed(0, 0);
	}
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
		var map = [undefined, undefined, 1, undefined]; // contains available movements in this cell, direction is regarding local start direction
		/*
			map descriptor:
				1 - way available
				-1 - no way
		*/
		var teL = this.target_encoders['left'] + const_enc;
		var teR = this.target_encoders['right'] + const_enc;

		this.target_encoders['left'] += const_enc;
		this.target_encoders['right'] += const_enc;

		var surface_color = [undefined, undefined, undefined, undefined, undefined];
		var surface_history = [[],[],[],[],[]];

		var ll = const_enc - this.target_encoders['left'] + this.encoderLeft.read(); //  this.encoderLeft.read() - seL;
		var lr = const_enc - this.target_encoders['right'] + this.encoderRight.read();

		for (var i = 0; i < 5; i++){
			var sens = this.sensors[i]()
			surface_color[i] = sens < 50 ? 1 : -1;
			surface_history[i].push([surface_color[i], Math.round(ll)]);
		}

		var flr = false; // true - if we are passing round turn

		var output = {'direction' : 0, 'map' : map, 'movement' : undefined};

		while (ll < const_enc || lr < const_enc){
			for (var i = 0; i < 5; i++){
				var sens = this.sensors[i]();
				if (((sens < 50 && surface_color[i] == -1) || (sens > 50 && surface_color[i] == 1)) && ll < 600){
					surface_color[i] *= -1;
					surface_history[i].push([surface_color[i], Math.round(ll)]);
					if (i == 2 && ll > 120 && ll < 260) flr = true;
					else if (i != 2 && flr == true){
						// print(surface_history[2]);
						this.target_encoders['left'] = teL - 170;
						this.target_encoders['right'] = teR - 170; // this.encoderRight.read();
						var st = Date.now();
						while (Date.now() - st < 1000){
							var eL = this.encoderLeft.read();
							var eR = this.encoderRight.read();
							this.setSpeed((this.target_encoders['left'] - eL) * 8, (this.target_encoders['right'] - eR) * 8);
						}
						this.setSpeed(0, 0);
						// robot.moveEncoders(305, 50);
						if (i == 1){
							output['movement'] = 'FL';
							output['direction'] = -1;
							output['map'][3] = 1;
							robot.turnDegreesOneWheel(-90);
						} else if (i == 3){
							output['movement'] = 'FR';
							output['direction'] = 1;
							output['map'][1] = 1;
							robot.turnDegreesOneWheel(90);
						}						
						robot.moveEncoders(-184, -30);
						smartPrint(output);
						return output;
					}
				}
			}

			ll = const_enc - this.target_encoders['left'] + this.encoderLeft.read(); //  this.encoderLeft.read() - seL;
			lr = const_enc - this.target_encoders['right'] + this.encoderRight.read();
			this.setSpeed(speed - (ll - lr) * 2, speed + (ll - lr) * 2);
		}
		// print(surface_history);
		var st = Date.now();
		while (Date.now() - st < 250){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((teL - eL) * 3, (teR - eR) * 3);
		}
		this.setSpeed(0, 0);

		output['direction'] = 0;
		output['movement'] = 'F';
		var end_surface = surface_history[0][surface_history[0].length - 1][0];
		if (end_surface != surface_history[2][surface_history[2].length - 1][0]){
			output['map'][0] = 1;
		} else {
			output['map'][0] = 0;
		}
		if (surface_history[0].length > 2){
			output['map'][3] = 1;
		} else {
			output['map'][3] = 0;
		}
		if (surface_history[4].length > 2){
			output['map'][1] = 1;
		} else {
			output['map'][1] = 0;
		}
		smartPrint(output);
		return output;
	};
};

var smartPrint = function(obj){print(JSON.stringify(obj, null, 4));};
var wait = script.wait;
var abs = Math.abs;
var max = Math.max;
var min = Math.min;

var main = function(){
	robot = new Robot("M4", "M3", "A1", "A2", "A3", "A4", "A5");

	// robot.turnDegrees(90);
	robot.driveSector();
	
	robot.turnDegrees(90);
	robot.driveSector();
	robot.turnDegrees(-90);
	robot.driveSector();
	robot.driveSector();
	robot.driveSector();
	robot.turnDegrees(-90);
	robot.driveSector();
	robot.driveSector();
	robot.driveSector();
	robot.driveSector();
	robot.driveSector();
	robot.turnDegrees(90);
	robot.driveSector();
	robot.turnDegrees(-90);
	robot.driveSector();
}

main();