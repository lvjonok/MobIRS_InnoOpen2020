Robot = function(leftMotor, rightMotor, leftLineSensor, rightLineSensor, leftSideSensor, rightSideSensor){
	this.motorLeft = brick.motor(leftMotor);
	this.motorRight = brick.motor(rightMotor);
	this.encoderLeft = brick.encoder("E" + leftMotor[1]);
	this.encoderRight = brick.encoder("E" + rightMotor[1]);
	this.llS = brick.sensor(leftLineSensor).read;
	this.rlS = brick.sensor(rightLineSensor).read;
	this.lsS = brick.sensor(leftSideSensor).read;
	this.rsS = brick.sensor(rightSideSensor).read;
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
		throw "kek1";	
	};
	this.setSpeed = function(lS, rS){
		if (rS == undefined) rS = lS;
		this.motorLeft.setPower(lS);
		this.motorRight.setPower(rS);
	};
	this.getSensors = function(){
		return [this.llS(), this.rlS(), this.lsS(), this.rsS()];
	};
	this.driveLine = function(){
		var error = this.llS() - this.rlS();
		var last_error = error;
		var kP = 0.7;
		var kD = 3;
		var speed = 40;
		print(this.llS() + " " + this.rlS());
		print(this.lsS() + " " + this.rsS());
		// return;

		while (true){
			// 0 - white, 100 -black
			// white part
			while (this.lsS() < 30 && this.rsS() < 30){
				error = this.llS() - this.rlS();
				this.setSpeed(speed - error * kP, speed + error * kP);
				wait(10);
			}
			print(this.getSensors());
			wait(10);
			if (this.rsS() < 20){ // right sensor didnt reach
				while (this.rsS() < 20){
					this.setSpeed(10, 20);
					wait(10);		
				}
			} else if (this.lsS() < 20){ // left sensor didnt reach
				while (this.lsS() < 20){
					this.setSpeed(20, 10);
					wait(10);		
				}
			}
			print(this.getSensors()); 
			// this.setSpeed(0,0);
			// return;
			// wait(2000);	
			// inverse part
			while (this.lsS() > 30 && this.rsS() > 30){
				error = -(this.llS() - this.rlS());
				this.setSpeed(speed - error * kP, speed + error * kP);
				wait(10);
			}
			wait(10);
			if (this.rsS() < 20){ // right sensor didnt reach
				while (this.rsS() < 20){
					this.setSpeed(0, 20);
					wait(10);		
				}
			} else if (this.lsS() < 20){ // left sensor didnt reach
				while (this.lsS() < 20){
					this.setSpeed(20, 0);
					wait(10);		
				}
			}
			// this.setSpeed(0,0);
			// wait(2000);
		}

		
		while (this.lsS() > 20 && this.rsS() > 20){
			error = -(this.llS() - this.rlS());
			this.setSpeed(speed - error * kP, speed + error * kP);
			wait(10);
		}
		while (this.lsS() > 20 || this.rsS() > 20){
			wait(10);
		}
		while (this.lsS() < 20 && this.rsS() < 20){
			error = this.llS() - this.rlS();
			this.setSpeed(speed - error * kP, speed + error * kP);
			wait(10);
		}
		while (this.lsS() < 20 || this.rsS() < 20){
			wait(10);
		}
	}
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
	}
}

var wait = script.wait;
var abs = Math.abs;

var main = function(){
	robot = new Robot("M4", "M3", "A1", "A2", "A3", "A4");
	robot.driveLine2()
}

main();