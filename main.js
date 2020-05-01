Robot = function(leftMotor, rightMotor, leftLineSensor, rightLineSensor, leftSideSensor, rightSideSensor){
	this.motorLeft = brick.motor(leftMotor);
	this.motorRight = brick.motor(rightMotor);
	this.encoderLeft = brick.encoder("E" + leftMotor[1]);
	this.encoderRight = brick.encoder("E" + rightMotor[1]);
	this.llS = brick.sensor(leftLineSensor);
	this.rlS = brick.sensor(rightLineSensor);
	this.lsS = brick.sensor(leftSideSensor);
	this.rsS = brick.sensor(rightSideSensor);
	this.track_sm = 17.5;
	this.wheelD_sm = 5.6;
	this.resetEncoders = function(){
		this.encoderLeft.reset();
		this.encoderRight.reset();
	};
}

var main = function(){
	new robot = Robot("M4", "M4", "A1", "A2", "A3", "A4");

}