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
		var acceleration_start = 0.02;
		var acceleration_end = 0.97; // 0.8

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
		var acceleration_start = 0.02;
		var acceleration_end = 0.97;

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
		while (Date.now() - st < 50){
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
		var const_enc = (52.5 * (2 / 3) / (this.wheelD_sm * Math.PI) * 360);
		var speed = 5;
		var map = [-1, -1, 1, -1]; // contains available movements in this cell, direction is regarding local start direction
		/*
			map descriptor:
				1	 - 	way available
				-1	 - 	no way
		*/
		var teL = this.target_encoders['left'] + const_enc;
		var teR = this.target_encoders['right'] + const_enc;

		this.target_encoders['left'] += const_enc;
		this.target_encoders['right'] += const_enc;

		var surface_color = [undefined, undefined, undefined, undefined, undefined];
		var surface_history = [[],[],[],[],[]];

		var ll = const_enc - this.target_encoders['left'] + this.encoderLeft.read();
		var lr = const_enc - this.target_encoders['right'] + this.encoderRight.read();

		for (var i = 0; i < 5; i++){
			var sens = this.sensors[i]()
			surface_color[i] = sens < 50 ? 1 : -1;							// 1 - white surface, -1 - black surface
			surface_history[i].push([surface_color[i], Math.round(ll)]);
		}

		var flr = false; // true - if we are passing round turn
		var output = {'direction' : 0, 'map' : map, 'movement' : undefined, 'sector type' : 0};

		var start_acc = 0.03;
		var end_acc = 0.97;
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
						this.target_encoders['right'] = teR - enc_less;
						// print(this.target_encoders['left'] - this.encoderLeft.read());
						var st = Date.now();
						while (Date.now() - st < 1000){
							var eL = this.encoderLeft.read();
							var eR = this.encoderRight.read();
							this.setSpeed((this.target_encoders['left'] - eL) * 6, (this.target_encoders['right'] - eR) * 6);
						}
						this.setSpeed(0, 0);
						if (i == 1){								// we detected left turn
							output['movement'] = 'FL';
							output['direction'] = -1;
							output['map'] = [1, -1, -1, 1];
							output['sector type'] = 4;
							robot.turnDegreesOneWheel(-90);
						} else if (i == 3){							// we detected right turn
							output['movement'] = 'FR';
							output['direction'] = 1;
							output['map'] = [1, 1, -1, -1];
							output['sector type'] = 4;
							robot.turnDegreesOneWheel(90);
						}
						robot.moveEncoders(-enc_less);
						return output;
					}
				}
			}
			ll = const_enc - this.target_encoders['left'] + this.encoderLeft.read(); //  this.encoderLeft.read() - seL;
			lr = const_enc - this.target_encoders['right'] + this.encoderRight.read();
			if (ll < const_enc * start_acc){
				ll = max(1, ll);
				speed = this.getSpeedSmooth(ll, const_enc * start_acc, 40, 100);	
			} else if (ll > const_enc * end_acc){
				speed = this.getSpeedSmooth(ll - const_enc * end_acc, const_enc - const_enc * end_acc, 100, 10);
			} else {
				speed = 100;
			}
			cam = this.camera.read()[0];
			this.setSpeed(speed - (ll - lr) * 3 + cam * 0.6, speed + (ll - lr) * 3 - cam * 0.6);
		}
		// var eL = this.encoderLeft.read();
		// var eR = this.encoderRight.read();
		// var st = Date.now();
		// while (Date.now() - st < 150){
		// 	var eL = this.encoderLeft.read();
		// 	var eR = this.encoderRight.read();
		// 	this.setSpeed((teL - eL) * 6, (teR - eR) * 6);
		// }
		// this.setSpeed(0, 0);
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
		/*
			sector types:
				0 - blank
				1 - straight line 						white
				2 - 90 degrees corner 					white
				3 - deadend 							white
				4 - 90 degrees round corner				white
				5 - T-type intersection					white
				6 - X-type intersection					white
				-1 - straight line						black
				-2 - 90 degrees corner					black
				-3 - deadend							black
				-5 - T-type intersection				black
				-6 - X-type intersection				black
		*/
		var lines_count = count(output['map'], 1);
		switch (lines_count){
			case 0:
				throw "it should not happen";
			break;
			case 1:
				// one line available means that it is a deadend
				output['sector type'] = 3 * end_surface;
			break;
			case 2:
				// it could be straight line or 90 degrees corner
				if (output['map'][0] == output['map'][2]){
					output['sector type'] = 1 * end_surface;
				} else {
					output['sector type'] = 2 * end_surface;
				}
			break;
			case 3:
				// T - type intersection
				output['sector type'] = 5 * end_surface;
			break;
			case 4:
				// X - type intersection
				output['sector type'] = 6 * end_surface;
			break;
		}
		// print('this cell was ', output['sector type']);
		return output;
	};
	this.getCellMap = function(){							// returns array with 4 elements: 1 - way exists, -1 - no way and cell type
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
		var lines_count = count(map, 1);
		var sector_type = 0;
		end_surface = surface;
		switch (lines_count){
			case 0:
				throw "it should not happen";
			break;
			case 1:
				// one line available means that it is a deadend
				sector_type = 3 * end_surface;
			break;
			case 2:
				// it could be straight line or 90 degrees corner
				if (map[0] == map[2]){
					sector_type = 1 * end_surface;
				} else {
					sector_type = 2 * end_surface;
				}
			break;
			case 3:
				// T - type intersection
				sector_type = 5 * end_surface;
			break;
			case 4:
				// X - type intersection
				sector_type = 6 * end_surface;
			break;
		}
		return {'map': map, 'sector type': sector_type};
	};
	this.printBase = function(){
		brick.display().clear();
		brick.display().addLabel('BASE', 1, 1);
		brick.display().redraw();
	};
	this.printPoint = function(){
		brick.display().clear();
		brick.display().addLabel('POINT', 1, 1);
		brick.display().redraw();
	};
	this.printFinish = function(){
		brick.display().clear();
		brick.display().addLabel('FINISH', 1, 1);
		brick.display().redraw();
	};
};

Field = function(robot){
	this.robot = robot;							// link to our robot
	this.map = [];								// real map
	this.height = 6;							// real map y height
	this.width = 6;								// real map x width
	this.x = -1;								// real robot x coordinate
	this.y = -1;								// real robot y coordinate
	this.robot_vertex = -1;						// real robot current vertex
	this.direction = 1;							// real robot direction
	this.localization_map = [];					// virtual map
	this.localization_height = 20;				// virtual map y height
	this.localization_width = 20;				// virtual map x width
	this.localization_x = 10;					// virtual robot x coordinate
	this.localization_y = 10;					// virtual robot y coordinate
	this.start_x = -1;							// real robot start x coordinate
	this.start_y = -1;							// real robot start y coordinate
	this.base_x = -1;							// base point x coordinate
	this.base_y = -1;							// base point y coordinate
	this.access_point_candidates = [];			// contains known vertices with their areas where we can put an access point
	this.unknown_access_point_candidates = [];	// contains unvisited cells that might have bigger area than known
	this.access_point_area_goal = 5;			// describes area of point we have chosen, need to make a recursion
	this.adjustDirection = function(direction){
		if (direction < 0) direction += 4;
		if (direction > 3) direction -= 4;
		return direction;
	};
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
			map.push({'adjacency' : [], 'type' : -1});
			for (var direction = 0; direction < 4; direction++){
				map[vertex]['adjacency'].push(this.getNextVertex(vertex, direction, width, height));
			}
		}
	};
	this.getVertexFromCoor = function(x, y, width, height){						// converts coordinates to vertex
		return y * width + x;
	};
	this.getCoorsFromVertex = function(vertex, width, height){					// converts vertex to coordinates
		if (!width) width = 6;
		if (!height) height = 6;
		return [vertex - floor(vertex / width) * height, floor(vertex / width)]
	};
	this.localization = function(){												// localizes robot in map
		var self = this;
		min_x = this.localization_x;
		max_x = min_x;
		min_y = this.localization_y;
		max_y = min_y;
		var firstCell_b = true;

		var avs = {}; // map: key - vertex, value - available vertices

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
				if (self.localization_map[cur_v]['adjacency'].indexOf(last_v) != -1){
					turn = self.getTurn(self.direction, self.localization_map[cur_v]['adjacency'].indexOf(last_v));
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
			}
			return false;
		};
		function updateCoors(vertex){
			coors = self.getCoorsFromVertex(vertex, 20, 20);
			min_x = min(min_x, coors[0]);
			max_x = max(max_x, coors[0]);
			min_y = min(min_y, coors[1]);
			max_y = max(max_y, coors[1]);
			self.localization_x = coors[0];
			self.localization_y = coors[1];
		};

		var current_vertex = this.getVertexFromCoor(this.localization_x, this.localization_y, this.localization_width, this.localization_height);
		var loc_start_vertex = current_vertex;
		var last_vertex = current_vertex;

		var visited = [current_vertex];
		var stack = [current_vertex];

		var changedPriority = false;

		while (max_x - min_x < 5 || max_y - min_y < 5){ // (stack.length > 0){	// 
			current_vertex = stack[stack.length - 1];
			keys__ = Object.keys(avs);
			for (var jj = 0; jj < keys__.length; jj++){
				remove(avs[keys__[jj]], current_vertex);
			}
			if (last_vertex != current_vertex){													// robot moved from one cell to another
				moved = moveRobot(last_vertex, current_vertex);
			} else if (this.localization_x == 10 && this.localization_y == 10 && firstCell_b) { // at first cell we should turn around
				firstCell_b = false;
				moved = this.robot.getCellMap();
			} else {																			// we shouldn't update if we followed path
			}
			cell_map = moved['map'];
			this.localization_map[current_vertex]['type'] = moved['sector type'];
			visited.push(current_vertex);
			var av_d = this.updateVertexAdjacency(current_vertex, this.direction, this.localization_map, cell_map);
			var av_v = [];
			

			if (max_x - min_x == 5 || max_y - min_y == 5){
				if (!changedPriority) print('changed priority');
				changedPriority = true;
				// we have this priority: left -> up -> down -> right || 3 -> 0 -> 2 -> 1
				if (av_d.indexOf(3) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][3]);
				}
				if (av_d.indexOf(0) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][0]);
				}
				if (av_d.indexOf(2) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][2]);
				}
				if (av_d.indexOf(1) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][1]);
				}
			} else {
				// we have this priority: right -> down -> up -> left || 1 -> 2 -> 0 -> 3
				if (av_d.indexOf(1) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][1]);
				}
				if (av_d.indexOf(2) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][2]);
				}
				if (av_d.indexOf(0) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][0]);
				}
				if (av_d.indexOf(3) != -1){
					av_v.push(this.localization_map[current_vertex]['adjacency'][3]);
				}	
			}
			var local_avs = [];
			var local_vis = [];
			var added = false;
			for (var j = 0; j < av_v.length; j++){
				if ( visited.indexOf(av_v[j]) == -1 && !added){		// add first available cell and add to stack
					// print('adding ', av_v[j]);
					stack.push(av_v[j]);
					added = true;
				} else if (visited.indexOf(av_v[j]) == -1){			// add available cells
					// print('already visited ', av_v[j]);
					local_avs.push(av_v[j]);
				} else if (av_v[j] != last_vertex){					// add already visited cells close to current
					// print('it is last vertex ', av_v[j]);
					local_vis.push(av_v[j]);
				}
			}
			avs[current_vertex] = copyObj(local_avs);
			// if (stack[stack.length - 1] == current_vertex) from = stack.pop();
			if (stack[stack.length - 1] == current_vertex){
				if (current_vertex == loc_start_vertex){
					break;
				}
				var found_vertex_with_unvisited_vertices = false;
				for (var i = stack.length - 1; i >= 0; i--){
					if (avs[stack[i]].length > 0 || stack[i] == loc_start_vertex) {
						found_vertex_with_unvisited_vertices = true;
						break;
					}
				}
				if (found_vertex_with_unvisited_vertices){
					for (var i = stack.length - 1; i >= 0; i--){
						if (avs[stack[i]].length > 0 || stack[i] == loc_start_vertex) {
							j = stack.length - 1;
							while (stack[j] != stack[i]){
								stack.pop();
								j--;
							}
							moved = this.moveFromV1ToV2_knownMap(this.localization_map, current_vertex, stack[i], this.direction);
							current_vertex = stack[i];
							updateCoors(current_vertex);
							break;
						}
					}
				} else {
					from = stack.pop();
				}
			}
			last_vertex = current_vertex;
		}
		this.x = this.localization_x - min_x;
		this.y = this.localization_y - min_y;
		var real_x = 0;
		var real_y = 0;
		for (var virtual_y = min_y; virtual_y <= max_y; virtual_y++){
			real_x = 0;
			for (var virtual_x = min_x; virtual_x <= max_x; virtual_x++){
				var virtual_vertex = this.getVertexFromCoor(virtual_x, virtual_y, 20, 20);
				var real_vertex = this.getVertexFromCoor(real_x, real_y, 6, 6);
				var old_map = this.localization_map[virtual_vertex]['adjacency'];
				var new_map = [];
				for (var d = 0; d < 4; d++){
					if (old_map[d] != -1){
						new_map.push(this.getNextVertex(real_vertex, d, 6, 6));
					} else {
						new_map.push(-1);
					}					
				}
				this.map[real_vertex]['adjacency'] = new_map;
				this.map[real_vertex]['type'] = this.localization_map[virtual_vertex]['type'];
				real_x++;
			}
			real_y++;
		}
		// smartPrint(this.map);
		// print('now you are here: ', this.x , ' ', this.y);
		this.start_x = 10 - min_x;
		this.start_y = 10 - min_y;
		this.base_x = this.start_y;
		this.base_y = this.start_x;
	};
	this.BFS = function(map, start_vertex, end_vertex){							// returns list of vertices from start node to end
		var current_vertex = start_vertex;
		var queue = [current_vertex];
		var visited = [current_vertex];
		var from = {};
		while (queue.length > 0){
			current_vertex = queue.shift();
			// print(current_vertex);
			visited.push(current_vertex);
			if (current_vertex == end_vertex) break;
			for (var d = 0; d < 4; d++){
				if (map[current_vertex]['adjacency'][d] != -1 && visited.indexOf(map[current_vertex]['adjacency'][d]) == -1){
					queue.push(map[current_vertex]['adjacency'][d]);
					from[map[current_vertex]['adjacency'][d]] = current_vertex;
				}
			}
		}
		var path = [end_vertex];
		read_vertex = from[end_vertex];
		// print(read_vertex);
		while (read_vertex != start_vertex){
			path.push(read_vertex)
			read_vertex = from[read_vertex];
		}
		path.push(start_vertex);
		return path.reverse();
	};
	this.moveFromV1ToV2_knownMap = function(map, start_vertex, end_vertex, current_direction){
		var path = this.BFS(map, start_vertex, end_vertex);
		for (var i = 0; i < path.length - 1; i++){
			var cur_v = path[i];
			var next_v = path[i+1];
			var needed_direction = map[cur_v]['adjacency'].indexOf(next_v);
			this.vertex = next_v;
			var turn = this.getTurn(current_direction, needed_direction);
			this.robot.turnDegrees(turn * 90);
			out = this.robot.driveSector();
			if (map == this.localization_map){
				coors = this.getCoorsFromVertex(next_v, 20, 20);
				this.localization_x = coors[0];
				this.localization_y = coors[1];
			} else {
				coors = this.getCoorsFromVertex(next_v, 6, 6);
				this.x = coors[0];
				this.y = coors[1];
			}
			current_direction = this.adjustDirection(needed_direction + out['direction']);
		}
		this.direction = current_direction;
		// print('returned');
		// smartPrint(out);
		this.vertex = end_vertex;
		return out;
	};
	this.moveFromV1ToV2_unknownMap = function(map, start_vertex, end_vertex, current_direction){
		var current_vertex = start_vertex;
		while (current_vertex != end_vertex){
			var path = this.BFS(map, current_vertex, end_vertex);
			// print('iteration path ' , path);
			for (var i = 0; i < path.length - 1; i++){
				var cur_v = path[i];
				var next_v = path[i+1];
				var needed_direction = map[cur_v]['adjacency'].indexOf(next_v);
				var turn = this.getTurn(current_direction, needed_direction);
				this.robot.turnDegrees(turn * 90);
				out = this.robot.driveSector();
				current_direction = this.adjustDirection(needed_direction + out['direction']);
				this.direction = current_direction;
				cell_map = out['map'];
				cell_type = out['sector type'];
				current_vertex = next_v;
				// print('verex ', current_vertex, ' map is ', cell_map);
				this.updateVertexAdjacency(current_vertex, this.direction, map, cell_map);
				this.vertex = current_vertex;
				map[current_vertex]['type'] = cell_type;
				
				if (map == this.localization_map){
					coors = this.getCoorsFromVertex(next_v, 20, 20);
					this.localization_x = coors[0];
					this.localization_y = coors[1];
				} else {
					coors = this.getCoorsFromVertex(next_v, 6, 6);
					this.x = coors[0];
					this.y = coors[1];
				}
				break;
			}
			this.direction = current_direction;
		}
		this.vertex = end_vertex;
		return out;
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
			// print(c_d, ' ', local_map[l_i]);
			r_d = r_d > 3 ? r_d - 4 : r_d;
			next_vertex = map[vertex]['adjacency'][c_d];
			if (local_map[l_i] == -1){		
				// print('next vertex ', next_vertex);
				if (next_vertex != -1) map[next_vertex]['adjacency'][r_d] = -1;
				map[vertex]['adjacency'][c_d] = -1;
			} else {
				if (l_i != 2 || true){
					available_vertices.push(c_d);
					// print('added av ', c_d, ' ', next_vertex);
				}
			}
			l_i += 1;
		}
		// print('candidates for adding ', available_vertices);
		return available_vertices;
	};
	this.getPointCandidates = function(){										// returns list of available points
		var point_min_x = max(this.base_x - 2, 0);
		var point_max_x = min(this.base_x + 2, 5);
		var point_min_y = max(this.base_y - 2, 0);
		var point_max_y = min(this.base_y + 2, 5);
		var candidates = [];
		var unknown_cells = {0:[], 1:[], 2:[], 3:[], 4: [], 5:[]};
		for (var y = point_min_y; y <= point_max_y; y++){
			for (var x = point_min_x; x <= point_max_x; x++){
				vertex = this.getVertexFromCoor(x, y, 6, 6);
				if (abs(this.map[vertex]['type']) == 5 || abs(this.map[vertex]['type']) == 6){
					// print('added ', x, ' ', y);
					candidates.push({'vertex': vertex, 'area': this.getPointArea(x, y)});
				} else if (this.map[vertex]['type'] == -1 && this.getPointArea(x, y) > 1){
					// unknown_cells.push({'vertex': vertex, 'area': this.getPointArea(x, y)});
					unknown_cells[this.getPointArea(x, y)].push(vertex);
					// print('discarded ', x, ' ', y, ' cause its type is unknown ');
				}
			}
		}
		this.access_point_candidates = copyObj(candidates);
		this.unknown_access_point_candidates = copyObj(unknown_cells);
		// smartPrint(candidates);
		smartPrint(unknown_cells);
		return this.access_point_candidates;
	};
	this.getPointArea = function(x, y){											// returns area for every point
		var point_min_x = max(this.base_x - 2, 0);
		var point_max_x = min(this.base_x + 2, 5);
		var point_min_y = max(this.base_y - 2, 0);
		var point_max_y = min(this.base_y + 2, 5);
		var area = 0;
		if (x == point_min_x && x != 0){
			area += 3;
			if (y == 0 || y == 5) area -= 1;
		}
		if (x == point_max_x && x != 5){
			area += 3;
			if (y == 0 || y == 5) area -= 1;
		}
		if (y == point_min_y && y != 0){
			area += 3;
			if (x == 0 || x == 5) area -= 1;
		}
		if (y == point_max_y && y != 5){
			area += 3;
			if (x == 0 || x == 5) area -= 1;
		}
		if (area == 6) area -= 1;
		return area;
	};
	this.moveToAccessPoint = function(){										// moves robot to access point
		var max_area = -1;
		var max_ei = -1;
		for (var e_i = 0; e_i < this.access_point_candidates.length; e_i++){
			if (this.access_point_candidates[e_i]['area'] > max_area){
				max_area = this.access_point_candidates[e_i]['area'];
				max_ei = e_i;
			}
		}
		print('\n\n\n\n\n\n\n');
		smartPrint(this.unknown_access_point_candidates);
		print('---------------------------------------');
		smartPrint(this.access_point_candidates);
		print('---------------------------------------');
		print('goal is ',this.access_point_area_goal);
		// bw();

		if (max_area == this.access_point_area_goal){
			print('move from ', this.vertex, ' ', this.access_point_candidates[max_ei]['vertex']);
			this.moveFromV1ToV2_unknownMap(this.map, this.vertex, this.access_point_candidates[max_ei]['vertex'], this.direction);
			print('we got it');
			return true;
		} else {
			while (this.unknown_access_point_candidates[this.access_point_area_goal].length == 0) {this.access_point_area_goal--;}
			print('rec ', this.access_point_area_goal);
			current_goal = this.unknown_access_point_candidates[this.access_point_area_goal].pop();
			this.moveFromV1ToV2_unknownMap(this.map, this.vertex, current_goal, this.direction);
			this.getPointCandidates();
			this.moveToAccessPoint();
			print('first layer');
			// throw "We should search more and maybe check any others";
		}
	};
	this.CompleteTask = function(){												// completes basic task
		this.localization();
		this.moveFromV1ToV2_unknownMap(this.map, this.getVertexFromCoor(this.x, this.y, 6, 6), this.getVertexFromCoor(this.base_x, this.base_y, 6, 6), this.direction);
		this.robot.moveEncoders(-200);
		this.robot.printBase();
		wait(5000);
		this.robot.moveEncoders(200);
		this.getPointCandidates();
		this.moveToAccessPoint();
		this.robot.moveEncoders(-200);
		this.robot.printPoint();
		wait(5000);
		this.robot.moveEncoders(200);
		this.moveFromV1ToV2_unknownMap(this.map, this.getVertexFromCoor(this.x, this.y, 6, 6), this.getVertexFromCoor(this.start_x, this.start_y, 6, 6), this.direction);
		this.robot.moveEncoders(-200);
		this.robot.printFinish();
		wait(5000);
	};
	this.generateMap(this.map, 6, 6);
	this.generateMap(this.localization_map, 20, 20);
};

var copyObj = function(obj){
	return JSON.parse(JSON.stringify(obj));
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
var count = function(arr, value){
	var c = 0;
	for (var k = 0; k < arr.length; k++){
		if (arr[k] == value) c+=1;
	}
	return c;
};
var remove = function(arr, value){
	for( var i = 0; i < arr.length; i++){ if ( arr[i] === value) { arr.splice(i, 1); i--; }}
};



var main = function(){
	robot = new Robot("M4", "M3", "A1", "A2", "A3", "A4", "A5");
	field = new Field(robot);
	// field.localization();
	field.CompleteTask();
	return;
};
main();