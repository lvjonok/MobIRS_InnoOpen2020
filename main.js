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
	this.camera = brick.lineSensor("video2")								// robot's camera
	this.camera.init(true);
	this.sensors = [this.lsS, this.llS, this.mlS, this.rlS, this.rsS];		// containter for sensor objects
	this.track_sm = 17.5;													// robot's track length
	this.wheelD_sm = 5.6;													// robot's wheel diameter
	this.resetEncoders = function(){
		this.encoderLeft.reset();
		this.encoderRight.reset();
		this.target_encoders['left'] = 0;
		this.target_encoders['right'] = 0;
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
		
		var l = abs(this.target_encoders['left'] - eL);
		var acceleration_start = 0.05;
		var acceleration_end = 0.8;

		var speed = 20;

		while ((this.target_encoders['left'] > eL && this.target_encoders['right'] < eR && direction == 1) || (this.target_encoders['left'] < eL && this.target_encoders['right'] > eR && direction == -1)){
			eL = this.encoderLeft.read();
			eR = this.encoderRight.read();
			var l = abs(const_enc - this.target_encoders['left'] + eL);
			if (l < abs(const_enc) * acceleration_start){
				speed = this.getSpeedSmooth(l, abs(const_enc), 15, 100);
			} else if (l > abs(const_enc) * acceleration_end){
				speed = this.getSpeedSmooth(abs(const_enc) - l, abs(const_enc) - abs(const_enc) * acceleration_end, 5, 100);
				// print(speed);
			} else {
				speed = 100;
			}
			// speed = 30;
			this.setSpeed(speed * direction, -speed * direction);
		}
		var st = Date.now();
		while (Date.now() - st < 150){
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
	this.driveSector2 = function(){
		// var const_enc = 750; // encoders to reach next cell
		var const_enc = (52.5 * (2 / 3) / (this.wheelD_sm * Math.PI) * 360);
		var speed = 40;
		var map = [-1, -1, 1, -1]; // contains available movements in this cell, direction is regarding local start direction
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
				if (((sens < 50 && surface_color[i] == -1) || (sens > 50 && surface_color[i] == 1)) && ll < 650){
					surface_color[i] *= -1;
					surface_history[i].push([surface_color[i], Math.round(ll)]);
					if (i == 2 && ll > 100 && ll < 300) flr = true;
					else if (i != 2 && flr == true){
						var enc_less = floor(this.sm2cpr(this.track_sm / 2));
						// smartPrint(surface_history);
						this.target_encoders['left'] = teL - enc_less;
						this.target_encoders['right'] = teR - enc_less; // this.encoderRight.read();
						var st = Date.now();
						while (Date.now() - st < 2000){
							var eL = this.encoderLeft.read();
							var eR = this.encoderRight.read();
							this.setSpeed((this.target_encoders['left'] - eL) * 6, (this.target_encoders['right'] - eR) * 6);
						}
						this.setSpeed(0, 0);
						// robot.moveEncoders(305, 50);
						if (i == 1){						// we detected left turn
							output['movement'] = 'FL';
							output['direction'] = -1;
							output['map'] = [1, -1, -1, 1];
							robot.turnDegreesOneWheel(-90);
						} else if (i == 3){					// we detected right turn
							output['movement'] = 'FR';
							output['direction'] = 1;
							output['map'] = [1, 1, -1, -1];
							robot.turnDegreesOneWheel(90);
						}						
						robot.moveEncoders(-enc_less, -20);
						// smartPrint(output);
						return output;
					}
				}
			}
			ll = const_enc - this.target_encoders['left'] + this.encoderLeft.read(); //  this.encoderLeft.read() - seL;
			lr = const_enc - this.target_encoders['right'] + this.encoderRight.read();
			this.setSpeed(speed - (ll - lr) * 3, speed + (ll - lr) * 3);
		}
		var st = Date.now();
		while (Date.now() - st < 750){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((teL - eL) * 6, (teR - eR) * 6);
		}
		this.setSpeed(0, 0);
		// smartPrint(surface_history);
		output['direction'] = 0;
		output['movement'] = 'F';
		var end_surface = surface_history[0][surface_history[0].length - 1][0];
		if (end_surface != surface_history[2][surface_history[2].length - 1][0]){
			output['map'][0] = 1;
		}
		if (surface_history[0].length > 2){
			output['map'][3] = 1;
		}
		if (surface_history[4].length > 2){
			output['map'][1] = 1;
		}
		// smartPrint(output);
		return output;
	};
	this.driveSector = function(){
		// var const_enc = 750; // encoders to reach next cell

		// 4 secs with speed 40, end encoder is 716
		// 2.6 secs with speed 70, and errors after are -7 and -4
		// this.resetEncoders();
		var const_enc = (52.5 * (2 / 3) / (this.wheelD_sm * Math.PI) * 360);
		var speed = 5;
		var map = [-1, -1, 1, -1]; // contains available movements in this cell, direction is regarding local start direction
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

		var start_acc = 0.05;
		var end_acc = 0.9;
		// print('bef all')
		while (ll < const_enc || lr < const_enc){
			for (var i = 0; i < 5; i++){
				var sens = this.sensors[i]();
				if (((sens < 50 && surface_color[i] == -1) || (sens > 50 && surface_color[i] == 1)) && ll < 650){
					surface_color[i] *= -1;
					surface_history[i].push([surface_color[i], Math.round(ll)]);
					if (i == 2 && ll > 100 && ll < 300) flr = true;
					else if ((i == 3 || i == 1) && flr == true){
						var enc_less = floor(this.sm2cpr(this.track_sm / 2));
						// smartPrint(surface_history);
						this.target_encoders['left'] = teL - enc_less;
						this.target_encoders['right'] = teR - enc_less; // this.encoderRight.read();
						print(this.target_encoders['left'] - this.encoderLeft.read());
						var st = Date.now();
						while (Date.now() - st < 1300){
							var eL = this.encoderLeft.read();
							var eR = this.encoderRight.read();
							this.setSpeed((this.target_encoders['left'] - eL) * 6, (this.target_encoders['right'] - eR) * 6);
						}
						this.setSpeed(0, 0);
						// print(this.target_encoders['left'] - this.encoderLeft.read());
						// wait(3000);
						// this.target_encoders['left'] = this.encoderLeft.read();
						// this.target_encoders['right'] = this.encoderRight.read();
						// this.moveEncoders(teL - enc_less - this.encoderLeft.read());
						// throw "END";
						if (i == 1){						// we detected left turn
							output['movement'] = 'FL';
							output['direction'] = -1;
							output['map'] = [1, -1, -1, 1];
							robot.turnDegreesOneWheel(-90);
						} else if (i == 3){					// we detected right turn
							output['movement'] = 'FR';
							output['direction'] = 1;
							output['map'] = [1, 1, -1, -1];
							robot.turnDegreesOneWheel(90);
						}
						// print('bef -enc less')
						robot.moveEncoders(-enc_less);
						// print('after -enc')
						// smartPrint(output);
						return output;
					}
				}
			}
			ll = const_enc - this.target_encoders['left'] + this.encoderLeft.read(); //  this.encoderLeft.read() - seL;
			lr = const_enc - this.target_encoders['right'] + this.encoderRight.read();
			if (ll < const_enc * start_acc){
				ll = max(1, ll);
				speed = this.getSpeedSmooth(ll, const_enc * start_acc, 5, 100);	
			} else if (ll > const_enc * end_acc){
				speed = this.getSpeedSmooth(ll - const_enc * end_acc, const_enc - const_enc * end_acc, 100, 10);
			} else {
				speed = 100;
			}
			cam = this.camera.read()[0];
			print(cam * 0.3); 
			this.setSpeed(speed - (ll - lr) * 3 + cam * 0.6, speed + (ll - lr) * 3 - cam * 0.6);
		}
		var eL = this.encoderLeft.read();
		var eR = this.encoderRight.read();
		print('error left after ', teL - eL);
		print('error right after is ', teR - eR);
		var st = Date.now();
		while (Date.now() - st < 150){
			var eL = this.encoderLeft.read();
			var eR = this.encoderRight.read();
			this.setSpeed((teL - eL) * 6, (teR - eR) * 6);
		}
		this.setSpeed(0, 0);
		smartPrint(surface_history);
		output['direction'] = 0;
		output['movement'] = 'F';
		var end_surface = surface_history[0][surface_history[0].length - 1][0];
		if (end_surface != surface_history[2][surface_history[2].length - 1][0]){
			output['map'][0] = 1;
		}
		if (surface_history[0].length > 2){
			output['map'][3] = 1;
		}
		if (surface_history[4].length > 2){
			output['map'][1] = 1;
		}
		// smartPrint(output);
		return output;
	}
	this.getCellMap = function(){											// returns array with 4 elements: 1 - way exists, -1 - no way
		var map = [];
		var surface = this.sensors[0]() < 50 ? 1 : -1;	// surface color: 1 - white, -1 - black
		for (var i = 0; i < 4; i++){
			var middle_value = this.sensors[2]() < 50 ? 1 : -1;	// middle sensor color: 1 - white, -1 - black
			if (middle_value == 1){
				map.push(surface == 1 ? -1 : 1);
			} else if (middle_value == -1){
				map.push(surface == 1 ? 1 : -1);
			}
			robot.turnDegrees(90);
		}
		return map;
	};
};

Field = function(robot){
	this.robot = robot;							// link to our robot
	this.map = [];								// real map
	this.height = 6;							// real map y height
	this.width = 6;								// real map x width
	this.x = -1;								// real robot x coordinate
	this.y = -1;								// real robot y coordinate
	this.direction = 1;							// real robot direction
	this.localization_map = [];					// virtual map
	this.localization_height = 20;				// virtual map y height
	this.localization_width = 20;				// virtual map x width
	this.localization_x = 10;					// virtual robot x coordinate
	this.localization_y = 10;					// virtual robot y coordinate
	this.adjustDirection = function(direction){
		if (direction < 0) direction += 4;
		if (direction > 3) direction -= 4;
		return direction;
	}
	this.getNextVertex = function(cur_v, dir, width, height){					// returns next vertex in custom map
		switch (dir){
			case 0:
				cur_v -= width;
			break;
			case 1:
				cur_v += 1;
				if ( (cur_v < width * height) && (cur_v % width != 0) ) return cur_v;
				else return -1;
			break;
			case 2:
				cur_v += width;
			break;
			case 3:
				cur_v -= 1;
				if (cur_v >= 0 && cur_v % width != width - 1) return cur_v;
				else return -1;
			break;
			default:
				throw "bad direction";
			break;
		}
		if (cur_v >= 0 && cur_v < width * height) return cur_v;
		return -1;
	};
	this.generateMap = function(map, width, height){							// generates blank map with all roads
		if (typeof(map) != "object") throw "ERROR this.generateMap = function(map, width, height){";
		for (var vertex = 0; vertex < width * height; vertex++){
			map.push([]);
			for (var direction = 0; direction < 4; direction++){
				map[vertex].push(this.getNextVertex(vertex, direction, width, height));
			}
			// print(vertex, " ", map[vertex]);
		}
	};
	this.getVertexFromCoor = function(x, y, width, height){						// converts coordinates to vertex
		return y * width + x;
	};
	this.getCoorsFromVertex = function(vertex){									// converts vertex to coordinates
		return [vertex - floor(vertex / 6) * 6, floor(vertex / 6)]
	};
	this.localization = function(){												// localizes robot in map
		var self = this;
		min_x = this.localization_x;
		max_x = min_x;
		min_y = this.localization_y;
		max_y = min_y;

		var cv_directions = [[0, 1, 2, 3], [1, 2, 3, 0], [2, 3, 0, 1], [3, 0, 1, 2]];

		function changeCoors(direction){
			switch (direction){
				case 0:
					self.localization_y -= 1;
					min_y = min(min_y, self.localization_y);
				break;
				case 1:
					self.localization_x += 1;
					max_x = max(max_x, self.localization_x);
				break;
				case 2:
					self.localization_y += 1;
					max_y = max(max_y, self.localization_y);
				break;
				case 3:
					self.localization_x -= 1;
					min_x = min(min_x, self.localization_x);
				break;
			}
		};
		function moveRobot(cur_v, last_v){
			if (cur_v != last_v){
				// print('moving from ', cur_v, ' ', last_v);
				turn = self.getTurn(self.direction, self.localization_map[cur_v].indexOf(last_v));
				switch (turn){
					case 0:
						out = self.robot.driveSector();
					break;
					case 1:
						self.direction += 1;
						self.robot.turnDegrees(90);
						out = self.robot.driveSector();
					break;
					case -1:
						self.direction -= 1;
						self.robot.turnDegrees(-90);
						out = self.robot.driveSector();
					break;
					case 2:
						self.direction += 2;
						self.robot.turnDegrees(180);
						out = self.robot.driveSector();
						
					break;
				}
				self.direction = self.adjustDirection(self.direction);
				changeCoors(self.direction);
				self.direction += out['direction'];
				self.direction = self.adjustDirection(self.direction);
				return out;
			}
			return false;
		};

		var current_vertex = this.getVertexFromCoor(this.localization_x, this.localization_y, this.localization_width, this.localization_height);
		var last_vertex = current_vertex;

		var visited = [current_vertex];
		var queue = [current_vertex];

		while (max_x - min_x < 5 || max_y - min_y < 5){
			current_vertex = queue[queue.length - 1];
			var moved = moveRobot(last_vertex, current_vertex);
			print(max_x, ' ', min_x, ' ', max_y, ' ', min_y);
			// print(queue);
			// print('moved');
			// smartPrint(moved);
			if (moved){
				// we can take map after moving
				var cell_map = moved['map'];
			} else {
				// we should turn around and read map
				// print('kek');
				var cell_map = this.robot.getCellMap();
			}
			visited.push(current_vertex);
			// print('current map ', cell_map);
			var av_v = this.updateVertexAdjacency(current_vertex, this.direction, this.localization_map, cell_map);
			for (var j = 0; j < av_v.length; j++){
				if ( visited.indexOf(av_v[j]) == -1 ){
					// print('adding ', av_v[j]);
					queue.push(av_v[j]);
					break;
				}
			}
			if (queue[queue.length - 1] == current_vertex){
				from = queue.pop();
			}
			// bw();

			last_vertex = current_vertex;
		}

	};
	this.getTurn = function(s_d, e_d){											// returns turn from one to another direction	
		var turn_array = [[0,1,2,-1],[-1,0,1,2],[2,-1,0,1],[1,2,-1,0]];
		return turn_array[s_d][e_d];
	};
	this.updateVertexAdjacency = function(vertex, direction, map, local_map){	// updates cell map vertex
		var l_i = 0;
		var available_vertices = [];
		// print('update adjacency ', map[vertex], ' ', vertex);
		for (var c_d_raw = direction; c_d_raw < direction + 4; c_d_raw++){
			var c_d = c_d_raw > 3 ? c_d_raw - 4 : c_d_raw;
			var r_d = c_d + 2;
			print(c_d, ' ', local_map[l_i]);
			r_d = r_d > 3 ? r_d - 4 : r_d;
			next_vertex = map[vertex][	c_d];
			if (local_map[l_i] == -1){		
				// print('next vertex ', next_vertex);
				if (next_vertex != -1) map[next_vertex][r_d] = -1;
				map[vertex][c_d] = -1;
			} else {
				if (l_i != 2 || true){
					available_vertices.push(next_vertex);
					// print('added av ', c_d, ' ', next_vertex);
				}
			}
			l_i += 1;
		}
		print('candidates for adding ', available_vertices);
		// bw();
		return available_vertices;
	};
	this.generateMap(this.map, 6, 6);
	this.generateMap(this.localization_map, 20, 20);
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
	field = new Field(robot);

	// robot.moveEncoders(1000);
	// robot.turnDegrees(-90);
	// robot.moveEncoders(1000);
	// robot.turnDegrees(-90);
	// robot.moveEncoders(1000);
	// robot.turnDegrees(-90);
	// robot.moveEncoders(1000);
	// robot.turnDegrees(-90);


	// robot.turnLine(90);
	// robot.driveSector();
	field.localization();
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