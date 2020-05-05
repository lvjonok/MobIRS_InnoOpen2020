Robot = function(leftMotor, rightMotor, leftLineSensor, rightLineSensor, leftSideSensor, rightSideSensor, middleLineSensor){
	this.motorLeft = brick.motor(leftMotor);								// robot's left motor object
	this.motorRight = brick.motor(rightMotor);								// robot's right motor object
	this.encoderLeft = brick.encoder("E" + leftMotor[1]);					// robot's left encoder object
	this.encoderRight = brick.encoder("E" + rightMotor[1]);					// robot's right encoder object
	this.llS = brick.sensor(leftLineSensor).read;							// robot's left line sensor object
	this.rlS = brick.sensor(rightLineSensor).read;							// robot's right line sensor object
	this.lsS = brick.sensor(leftSideSensor).read;							// robot's left side sensor object
	this.rsS = brick.sensor(rightSideSensor).read;							// robot's right side sensor object
	this.mlS = brick.sensor(middleLineSensor).read;							// robot's middle line sensor object
	this.sensors = [this.lsS, this.llS, this.mlS, this.rlS, this.rsS];		// containter for sensor objects
	this.track_sm = 17.5;													// robot's track length
    this.wheelD_sm = 5.6;													// robot's wheel diameter
    
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
	};
	this.getSpeedSmooth = function(encDist, Dist, speed_start, speed_end){
		/*
		*/
		return speed_start + (encDist / (Dist / (speed_end - speed_start) ) );
	};
	this.moveEncoders = function(enc, speed){
		if (speed == undefined) speed = 0;
		// var seL = this.encoderLeft.read();
		// var seR = this.encoderRight.read();
		var tL = this.target_encoders['left'] + enc;
		var tR = this.target_encoders['right'] + enc;

		this.target_encoders['left'] += enc;
		this.target_encoders['right'] += enc;
		
		var eL = this.encoderLeft.read();
		var eR = this.encoderRight.read();

		var acceleration_start = 0.05;
		var acceleration_end = 0.95;

		var eL = this.encoderLeft.read();
		var eR = this.encoderRight.read();
		var ll = abs(enc - tL + eL);
		var rl = abs(enc - tR + eR);

		var error = (ll - rl);
		var last_error = error;

		var kP = 2 * this.sign(enc);
		var kD = 0;

		while ((tL > eL && tR > eR && enc > 0)||(tL < eL && tR < eR && enc < 0)){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			var ll = abs(enc - tL + eL);
			var rl = abs(enc - tR + eR);
			error = (ll - rl);
			// print(ll, ' ', rl);
			if (ll + rl < abs(enc) * 2 * acceleration_start){
				// print('accelerating');
				speed = this.getSpeedSmooth(ll, abs(enc) * acceleration_start, 5, 100);
			} else if (ll + rl > abs(enc) * 2 * acceleration_end){
				// print('slowing');
				speed = this.getSpeedSmooth(ll - abs(enc) * acceleration_end, abs(enc) - abs(enc) * acceleration_end, 100, 10);
			} else {
				// print('const')
				speed = 100;
			}
			// speed = 30;
			// print(error);
			// print('affection ', error * kP + (error - last_error) * kD);
			this.setSpeed(	speed * this.sign(enc) - error * kP, 
							speed * this.sign(enc) + error * kP);
			last_error = error;
			// wait(10);
		}
		var st = Date.now();
		while (Date.now() - st < 100){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((tL - eL) * 5, (tR - eR) * 5);
		}
		this.setSpeed(0, 0);
	};
	this.turnDegrees = function(degrees){
		var const_enc = this.sm2cpr(this.track_sm * Math.PI * (degrees / 360));
		var direction = this.sign(const_enc);
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
		while (Date.now() - st < 200){
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
		
		var speed = 0;
		var l = 0;
		var acceleration_start = 0.05;
		var acceleration_end = 0.93;

		while ((this.target_encoders['left'] > eL && direction == 1) || (this.target_encoders['right'] > eR && direction == -1)){
			eL = this.encoderLeft.read();
			eR = this.encoderRight.read();
			if (direction == 1){
				l = const_enc - this.target_encoders['left'] + eL;
				if (l < const_enc * acceleration_start){
					speed = this.getSpeedSmooth(l, const_enc, 30, 100);
				} else if (l > const_enc * acceleration_end){
					speed = this.getSpeedSmooth(l, const_enc, 100, 10);
				} else {
					speed = 100;
				}
				this.setSpeed(speed, 0);
			} else {
				l = const_enc - this.target_encoders['right'] + eR;
				if (l < const_enc * acceleration_start){
					speed = this.getSpeedSmooth(l, const_enc, 30, 100);
				} else if (l > const_enc * acceleration_end){
					speed = this.getSpeedSmooth(l, const_enc, 100, 10);
				} else {
					speed = 100;
				}
				this.setSpeed(0, speed);
			}
		}
		var st = Date.now();
		while (Date.now() - st < 400 && false){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((this.target_encoders['left'] - eL) * 6, (this.target_encoders['right'] - eR) * 6);
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
};

var bw = function(){
	brick.playTone(2000, 300);
	while (!brick.keys().wasPressed(KeysEnum.Up)) { script.wait(100)}
};
var smartPrint = function(obj){print(JSON.stringify(obj, null, 4));};
var wait = script.wait;
var abs = Math.abs;
var max = Math.max;
var min = Math.min;
var floor = Math.floor;
var ceil = Math.ceil;
var round = Math.round;

var main = function(){
    robot = new Robot("M4", "M3", "A1", "A2", "A3", "A4", "A5");
    
    s = brick.lineSensor("video2");

    s.init(true);

    val = s.read();
    print(val);
    while (robot.sensors[2]() > 50){ // val[1] < 60){
        robot.setSpeed(40 + val[0] * 0.3, 40 - val[0] * 0.3);
        val = s.read();
        print(val);
    }
    robot.setSpeed(0);
    

	// robot.driveSector();
	// robot.turnDegreesOneWheel(90);
	// robot.turnDegreesOneWheel(-	90);
	return;

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
	robot.turnDegrees(-90);
};
main();