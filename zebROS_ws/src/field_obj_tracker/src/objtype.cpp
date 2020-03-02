// A class for defining objects we're trying to
// detect.  The class stores information about shape
// and size of the objects in real-world measurements
#include "field_obj_tracker/objtype.hpp"
#include "field_obj_tracker/convert_coords.h"

using namespace std;
using namespace cv;

ObjectType::ObjectType(ObjectNum contour_type_id=UNINITIALIZED) {
	switch(contour_type_id) {
		//loads one of the preset shapes into the
		//object

		case BALL_2017: //a ball!
			depth_ = 0.2476; // meters
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0, depth_));
			contour_.push_back(Point2f(depth_, depth_));
			contour_.push_back(Point2f(depth_,0));
			name_="ball";
			break;

		case BIN_2016: //a bin (just because)
			depth_ = 0.5588;
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.5842));
			contour_.push_back(Point2f(0.5842,0.5842));
			contour_.push_back(Point2f(0.5842,0));
			name_="bin";
			break;

		case GOAL_2016: //2016 Goal
			{
				depth_ = 0;
				float max_y = .3048;
				contour_.push_back(Point2f(0, max_y - 0));
				contour_.push_back(Point2f(0, max_y - 0.3048));
				contour_.push_back(Point2f(0.0508, max_y - 0.3048));
				contour_.push_back(Point2f(0.0508, max_y - 0.0508));
				contour_.push_back(Point2f(0.508-0.0508, max_y - 0.0508));
				contour_.push_back(Point2f(0.508-0.0508, max_y - 0.3048));
				contour_.push_back(Point2f(0.508, max_y - 0.3048));
				contour_.push_back(Point2f(0.508, max_y - 0));
				name_="goal";
			}
			break;
		case TOP_TAPE_2017: //top piece of tape (2017)
			depth_ = 0;
			real_height_ = 1.9812;  //76 inches + 4 * 1/2 height
			real_height_ -= .22225; // 8.75 in camera height
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0, 0.1010));
			contour_.push_back(Point2f(0,0.118954963068));
			contour_.push_back(Point2f(0.01272380885,0.1230581348));
			contour_.push_back(Point2f(0.0255358792974,0.126876796969));
			contour_.push_back(Point2f(0.0384298504578,0.130409053701));
			contour_.push_back(Point2f(0.0513993207851,0.133653151317));
			contour_.push_back(Point2f(0.0644378512496,0.136607479199));
			contour_.push_back(Point2f(0.0775389685348,0.139270570596));
			contour_.push_back(Point2f(0.0906961682517,0.141641103346));
			contour_.push_back(Point2f(0.103902918167,0.143717900537));
			contour_.push_back(Point2f(0.117152661449,0.145499931089));
			contour_.push_back(Point2f(0.130438819917,0.146986310268));
			contour_.push_back(Point2f(0.143754797315,0.148176300121));
			contour_.push_back(Point2f(0.15709398258,0.149069309847));
			contour_.push_back(Point2f(0.170449753128,0.149664896088));
			contour_.push_back(Point2f(0.183815478141,0.149962763151));
			contour_.push_back(Point2f(0.197184521859,0.149962763151));
			contour_.push_back(Point2f(0.210550246872,0.149664896088));
			contour_.push_back(Point2f(0.22390601742,0.149069309847));
			contour_.push_back(Point2f(0.237245202685,0.148176300121));
			contour_.push_back(Point2f(0.250561180083,0.146986310268));
			contour_.push_back(Point2f(0.263847338551,0.145499931089));
			contour_.push_back(Point2f(0.277097081833,0.143717900537));
			contour_.push_back(Point2f(0.290303831748,0.141641103346));
			contour_.push_back(Point2f(0.303461031465,0.139270570596));
			contour_.push_back(Point2f(0.31656214875,0.136607479199));
			contour_.push_back(Point2f(0.329600679215,0.133653151317));
			contour_.push_back(Point2f(0.342570149542,0.130409053701));
			contour_.push_back(Point2f(0.355464120703,0.126876796969));
			contour_.push_back(Point2f(0.36827619115,0.1230581348));
			contour_.push_back(Point2f(0.381,0.118954963068));
			contour_.push_back(Point2f(0.381, 0));
			contour_.push_back(Point2f(0.381,0.0122875187586));
			contour_.push_back(Point2f(0.368470376004,0.0172542619015));
			contour_.push_back(Point2f(0.355811431603,0.0218814793796));
			contour_.push_back(Point2f(0.343032365293,0.0261658088713));
			contour_.push_back(Point2f(0.330142462857,0.0301041372113));
			contour_.push_back(Point2f(0.317151090612,0.0336936026522));
			contour_.push_back(Point2f(0.304067688612,0.0369315969449));
			contour_.push_back(Point2f(0.29090176378,0.0398157672328));
			contour_.push_back(Point2f(0.277662883004,0.0423440177624));
			contour_.push_back(Point2f(0.264360666186,0.0445145114055));
			contour_.push_back(Point2f(0.251004779249,0.0463256709944));
			contour_.push_back(Point2f(0.237604927115,0.0477761804682));
			contour_.push_back(Point2f(0.224170846654,0.0488649858284));
			contour_.push_back(Point2f(0.210712299607,0.0495912959055));
			contour_.push_back(Point2f(0.197239065494,0.0499545829336));
			contour_.push_back(Point2f(0.183760934506,0.0499545829336));
			contour_.push_back(Point2f(0.170287700393,0.0495912959055));
			contour_.push_back(Point2f(0.156829153346,0.0488649858284));
			contour_.push_back(Point2f(0.143395072885,0.0477761804682));
			contour_.push_back(Point2f(0.129995220751,0.0463256709944));
			contour_.push_back(Point2f(0.116639333814,0.0445145114055));
			contour_.push_back(Point2f(0.103337116996,0.0423440177624));
			contour_.push_back(Point2f(0.09009823622,0.0398157672328));
			contour_.push_back(Point2f(0.0769323113878,0.0369315969449));
			contour_.push_back(Point2f(0.0638489093876,0.0336936026522));
			contour_.push_back(Point2f(0.0508575371435,0.0301041372113));
			contour_.push_back(Point2f(0.0379676347069,0.0261658088713));
			contour_.push_back(Point2f(0.0251885683974,0.0218814793796));
			contour_.push_back(Point2f(0.0125296239964,0.0172542619015));
			contour_.push_back(Point2f(0,0.0122875187586));
			name_ = "top_boiler_tape";
			break;
		case BOTTOM_TAPE_2017: //bottom piece of tape (2017)
			depth_ = 0;
			real_height_ = 1.7272; //5ft 7inches + 1/2 * 2in height
			real_height_ -= .22225; // 8.75 in camera height
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.0489549630683));
			contour_.push_back(Point2f(0.01272380885,0.0530581347998));
			contour_.push_back(Point2f(0.0255358792974,0.0568767969687));
			contour_.push_back(Point2f(0.0384298504578,0.0604090537011));
			contour_.push_back(Point2f(0.0513993207851,0.0636531513167));
			contour_.push_back(Point2f(0.0644378512496,0.0666074791992));
			contour_.push_back(Point2f(0.0775389685348,0.0692705705958));
			contour_.push_back(Point2f(0.0906961682517,0.0716411033459));
			contour_.push_back(Point2f(0.103902918167,0.073717900537));
			contour_.push_back(Point2f(0.117152661449,0.0754999310894));
			contour_.push_back(Point2f(0.130438819917,0.0769863102677));
			contour_.push_back(Point2f(0.143754797315,0.0781763001206));
			contour_.push_back(Point2f(0.15709398258,0.0790693098467));
			contour_.push_back(Point2f(0.170449753128,0.0796648960881));
			contour_.push_back(Point2f(0.183815478141,0.0799627631508));
			contour_.push_back(Point2f(0.197184521859,0.0799627631508));
			contour_.push_back(Point2f(0.210550246872,0.0796648960881));
			contour_.push_back(Point2f(0.22390601742,0.0790693098467));
			contour_.push_back(Point2f(0.237245202685,0.0781763001206));
			contour_.push_back(Point2f(0.250561180083,0.0769863102677));
			contour_.push_back(Point2f(0.263847338551,0.0754999310894));
			contour_.push_back(Point2f(0.277097081833,0.073717900537));
			contour_.push_back(Point2f(0.290303831748,0.0716411033459));
			contour_.push_back(Point2f(0.303461031465,0.0692705705958));
			contour_.push_back(Point2f(0.31656214875,0.0666074791992));
			contour_.push_back(Point2f(0.329600679215,0.0636531513167));
			contour_.push_back(Point2f(0.342570149542,0.0604090537011));
			contour_.push_back(Point2f(0.355464120703,0.0568767969687));
			contour_.push_back(Point2f(0.36827619115,0.0530581347998));
			contour_.push_back(Point2f(0.381,0.0489549630683));
			contour_.push_back(Point2f(0.381, 0));
			contour_.push_back(Point2f(0.381,0.0122875187586));
			contour_.push_back(Point2f(0.368470376004,0.0172542619015));
			contour_.push_back(Point2f(0.355811431603,0.0218814793796));
			contour_.push_back(Point2f(0.343032365293,0.0261658088713));
			contour_.push_back(Point2f(0.330142462857,0.0301041372113));
			contour_.push_back(Point2f(0.317151090612,0.0336936026522));
			contour_.push_back(Point2f(0.304067688612,0.0369315969449));
			contour_.push_back(Point2f(0.29090176378,0.0398157672328));
			contour_.push_back(Point2f(0.277662883004,0.0423440177624));
			contour_.push_back(Point2f(0.264360666186,0.0445145114055));
			contour_.push_back(Point2f(0.251004779249,0.0463256709944));
			contour_.push_back(Point2f(0.237604927115,0.0477761804682));
			contour_.push_back(Point2f(0.224170846654,0.0488649858284));
			contour_.push_back(Point2f(0.210712299607,0.0495912959055));
			contour_.push_back(Point2f(0.197239065494,0.0499545829336));
			contour_.push_back(Point2f(0.183760934506,0.0499545829336));
			contour_.push_back(Point2f(0.170287700393,0.0495912959055));
			contour_.push_back(Point2f(0.156829153346,0.0488649858284));
			contour_.push_back(Point2f(0.143395072885,0.0477761804682));
			contour_.push_back(Point2f(0.129995220751,0.0463256709944));
			contour_.push_back(Point2f(0.116639333814,0.0445145114055));
			contour_.push_back(Point2f(0.103337116996,0.0423440177624));
			contour_.push_back(Point2f(0.09009823622,0.0398157672328));
			contour_.push_back(Point2f(0.0769323113878,0.0369315969449));
			contour_.push_back(Point2f(0.0638489093876,0.0336936026522));
			contour_.push_back(Point2f(0.0508575371435,0.0301041372113));
			contour_.push_back(Point2f(0.0379676347069,0.0261658088713));
			contour_.push_back(Point2f(0.0251885683974,0.0218814793796));
			contour_.push_back(Point2f(0.0125296239964,0.0172542619015));
			contour_.push_back(Point2f(0,0.0122875187586));
			name_ = "bottom_boiler_tape";
			break;
		case SWITCH_2018: //target on the switch fence (2018)
			depth_ = 0;
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.41));
			contour_.push_back(Point2f(0.0508,0.41));
			contour_.push_back(Point2f(0.0508,0));
			name_ = "plate_location_tape";
			break;
		case CUBE_2018: //Cube (2018)
			depth_ = 0.3048;
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.2794));
			contour_.push_back(Point2f(0.3302,0.2794));
			contour_.push_back(Point2f(0.2794,0));
			name_ = "cube";
			break;
		case LEFT_CARGO_2019: //left target on the CARGO SHIP (2019)
			depth_ = 0;
			real_height_ = -.4; // TODO : subtract camera height
			contour_.push_back(Point2f(0,0.01251900023));
			contour_.push_back(Point2f(0.035,0.148019));
			contour_.push_back(Point2f(0.083407382019,0.1355));
			contour_.push_back(Point2f(0.048407382019,0.0));
			name_ = "left_cargo_tape";
			break;
		case RIGHT_CARGO_2019: //right target on the CARGO SHIP (2019)
			depth_ = 0;
			real_height_ = -.4; // TODO : subtract camera height
			contour_.push_back(Point2f(0.083407382,0.012519));
			contour_.push_back(Point2f(0.048407382,0.148019));
			contour_.push_back(Point2f(0.0,0.1355));
			contour_.push_back(Point2f(0.035,0.0));
			name_ = "right_cargo_tape";
			break;
		case POWER_PORT_2020: //target on the POWER PORT (2020)
			depth_ = 0;
			real_height_ = 1.4796516; // TODO: Fix this using actual height
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0.2492375,0.4318));
			contour_.push_back(Point2f(0.7477125,0.4318));
			contour_.push_back(Point2f(0.99695,0));
			contour_.push_back(Point2f(0.9382912,0));
			contour_.push_back(Point2f(0.7183831,0.381));
			contour_.push_back(Point2f(0.2785669,0.381));
			contour_.push_back(Point2f(0.0586588,0));
			name_ = "power_port";
			break;
		case LOADING_BAY_2020: //target on the LOADING BAY (2020)
			depth_ = 0;
			real_height_ = -.3046984; // TODO: fix this using actual height
			contour_.push_back(Point2f(0.0508,0.2286));
			contour_.push_back(Point2f(0.0508,0));
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.2794));
			contour_.push_back(Point2f(0.1778,0.2794));
			contour_.push_back(Point2f(0.1778,0));
			contour_.push_back(Point2f(0.0508,0));
			contour_.push_back(Point2f(0.0508,0.0508));
			contour_.push_back(Point2f(0.127,0.0508));
			contour_.push_back(Point2f(0.127,0.2286));
			name_ = "loading_bay";
			break;
		case TEST_TARGET_2020:
			depth_ = 0;
			real_height_ = 0; // TODO: fix this using actual height
			contour_.push_back(Point2f(0, 0));
			contour_.push_back(Point2f(0, 0.102));
			contour_.push_back(Point2f(0.102, 0));
			contour_.push_back(Point2f(0.102, 0.102));
			name_ = "test_target";
			break;
		case red_power_port_high_goal:
			depth_ = 0;
			height_ = .889;
			//height_ = 2.49555;			commented out heights will be the z-position
			width_ = 1.02;
			positions_.push_back(Point2f(15.98295,5.806186));
			name_ = "red_power_port_high_goal";
			break;
		case blue_power_port_high_goal:
			depth_ = 0;
			height_ = .889;
			//height_ = 2.49555;
			width_ = 1.02;
			positions_.push_back(Point2f(0,2.404364));
			name_ = "blue_power_port_high_goal";
			break;
		case red_power_port_low_goal:
			depth_=0;
			width_=.86;
			height_ =.254;
			//height_ = 0.5842;
			name_="red_power_port_high_goal";
			positions_.push_back(Point2f(0,2.404364));
			break;
		case blue_power_port_low_goal:
			depth_=0;
			width_=.86;
			height_=.254;
			//height_ = 0.5842;
			name_="blue_power_port_low_goal";
			positions_.push_back(Point2f(15.98295,5.806186));
			break;
		case power_cell:
			height_ = 0.18;
			width_ = 0.18;
			depth_ = 0.18;		// need to ask what to do if no set position (power cell) or size (tape)
			name_="power_cell";
			break;
		case power_port_yellow_graphics:
			height_=.4572;
			//height_ = 1.412875;
			width_=.6604;
			depth_=0;
			name_="power_port_yellow_graphics";
			positions_.push_back(Point2f(0,2.404364));
			positions_.push_back(Point2f(15.98295,5.806186));
			break;
		case red_power_port_first_logo:
			height_=.26;
			//height_ = 2.2098;
			width_ = .355;
			depth_ = 0;
			name_ = "red_power_port_first_logo";
			positions_.push_back(Point2f(0,4.840986));
			positions_.push_back(Point2f(15.80515,7.198614));
			break;
		case blue_power_port_first_logo:
			height_=.26;
			//height_ = 2.2098;
                        width_ = .355;
                        depth_ = 0;
			name_ = "blue_power_port_first_logo";
			positions_.push_back(Point2f(15.98295,3.369564));
                        positions_.push_back(Point2f(.1778, 1.011936));
			break;
		case red_loading_bay_tape:
			height_ = .533;
			//height_ = .4572;
			width_ = 0.62;
			depth_ = 0;
			name_ = "red_loading_bay_tape";
			positions_.push_back(Point2f(0,6.128766));
			break;
		case blue_loading_bay_tape:
			height_ = .533;
			//height_ = .4572;
			width_ = .62;
			depth_ = 0;
			name_ = "blue_loading_bay_tape";
			positions_.push_back(Point2f(15.98295,2.081784));
			break;
		case red_loading_bay_left_graphics:
			height_ = 0.4318;
			//height_ = .508;
			width_ = 0.1651;
			depth_ = 0;
			name_ = "red_loading_bay_left_graphics";
			positions_.push_back(Point2f(0,6.263386));
			break;
		case red_loading_bay_right_graphics:
			height_ = 0.654;
			//height_ = .6096;
			width_ = .1651;
			depth_ = 0;
			name_ = "red_loading_bay_right_graphics";
			positions_.push_back(Point2f(0,5.044186));
			break;
		case blue_loading_bay_left_graphics:
			height_ = .4318;
			//height_ = .508;
			width_ = .1651;
			depth_ = 0;
			name_ = "blue_loading_bay_left_graphics";
			positions_.push_back(Point2f(15.983204,1.947164));
			break;
		case blue_loading_bay_right_graphics:
			height_ = 0.654;
			//height_ = .6096;
			width_ = .1651;
			depth_ = 0;
			name_ = "blue_loading_bay_right_graphics";
			positions_.push_back(Point2f(15.983204,3.166364));
			break;
		case red_tape_corner:
			height_ = 0;			//need the other dimensions?
			name_ = "red_tape_corner";
			positions_.push_back(Point2f(.762,5.65404));
			positions_.push_back(Point2f(15.221204,5.80644));
			positions_.push_back(Point2f(5.246878,6.80085));
			positions_.push_back(Point2f(10.73658,6.80085));
			break;
		case blue_tape_corner:
			height_ = 0;			//need the other dimensions?
			name_ = "blue_tape_corner";
			positions_.push_back(Point2f(.762,2.40436));
			positions_.push_back(Point2f(15.221204,2.556764));
			positions_.push_back(Point2f(10.73658,1.4097));
			positions_.push_back(Point2f(5.246878,1.4097));
			break;
		case red_ds_light:
			height_ = .214;
			//height_ = 1.9304;
			width_ = 0.0762;
			depth_ = 0.0762;
			positions_.push_back(Point2f(.381,.659892));
			positions_.push_back(Point2f(0,3.952875));
			positions_.push_back(Point2f(.381,7.550658));
			name_ = "red_ds_light";
			break;
		case blue_ds_light:
			height_ = .214;
			//height_ = 1.9304;
			width_ = 0.0762;
			depth_ = 0.0762;
			name_ = "blue_ds_light";
			positions_.push_back(Point2f(15.60195,.659892));
			positions_.push_back(Point2f(15.98295,4.257675));
			positions_.push_back(Point2f(15.60195,7.550658));
			break;
		case ds_light:
			height_ = .214;
                        //height_ = 1.9304;
                        width_ = 0.0762;
                        depth_ = 0.0762;
                        positions_.push_back(Point2f(.381,.659892));
                        positions_.push_back(Point2f(0,3.952875));
                        positions_.push_back(Point2f(.381,7.550658));
                        name_ = "ds_light";
			positions_.push_back(Point2f(15.60195,.659892));
                        positions_.push_back(Point2f(15.98295,4.257675));
                        positions_.push_back(Point2f(15.60195,7.550658));
                        break;
		case control_panel_light:
			height_ = .214;
			//height = 1.003;
			width_ = 0.0762;
			depth_ = .0762;
			name_ = "control_panel_light";
			positions_.push_back(Point2f(6.88975,6.85165));
			positions_.push_back(Point2f(9.09828,1.3716));
			break;
		case yellow_control_panel_light:
			height_ = .214;
                        //height = 1.003;
                        width_ = 0.0762;
                        depth_ = .0762;
                        name_ = "yellow_control_panel_light";
                        positions_.push_back(Point2f(6.88975,6.85165));
                        positions_.push_back(Point2f(9.09828,1.3716));
                        break;
		case red_shield_generator_light:
			height_ = .214;
			//height_ = 2.39395
			width_ = .0762;
			depth_ = .0762;
			name_ = "red_shield_generator_light";
			positions_.push_back(Point2f(10.103993,3.230245));
			break;
		case blue_shield_generator_light:
			height_ = .214;
			//height_ = 2.39395;
			width_ = .0762;
			depth_ = .0762;
			name_ = "blue_shield_generator_light";
			positions_.push_back(Point2f(5.879719,4.980305));
			break;
		case shield_generator_light:
			height_ = .214;
                        //height_ = 2.39395;
                        width_ = .0762;
                        depth_ = .0762;
                        name_ = "shield_generator_light";
                        positions_.push_back(Point2f(5.879719,4.980305));
                        positions_.push_back(Point2f(10.103993,3.230245));
			break;
		case shield_generator_backstop:
			height_ = .1651;
			//height_ = 2.220722;
			width_ = .2286;
			depth_ = .0635;
			name_ = "shield_generator_backstop";
			positions_.push_back(Point2f(10.753852,4.1459912));
			positions_.push_back(Point2f(7.241286,5.8960512));
			positions_.push_back(Point2f(8.741918,2.314498));
			positions_.push_back(Point2f(5.230368,4.0645588));
			break;
		case shield_generator_first_logo:
			height_ = .203;
			//height_ = 2.67335;
			width_ = .203;
			depth_ = 0;
			name_ = "shield_generator_first_logo";
			positions_.push_back(Point2f(8.17702,1.867912));
			positions_.push_back(Point2f(9.480042,2.406904));
			positions_.push_back(Point2f(5.795264,2.853944));
			positions_.push_back(Point2f(5.256276,4.156964));
			positions_.push_back(Point2f(6.503162,5.803646));
			positions_.push_back(Point2f(7.806182,6.342634));
			positions_.push_back(Point2f(10.188956,5.356606));
			positions_.push_back(Point2f(10.727944,4.053586));
		case shield_generator_yellow_stripe:
			height_ = .5334;
			//height_ = 2.54635
			width_ = .508;
			depth_ = 0;
			name_ = "shield_generator_yellow_stripe";
			positions_.push_back(Point2f(8.86333,1.583182));
			positions_.push_back(Point2f(9.195308,1.720596));
			positions_.push_back(Point2f(5.1089306,3.138678));
			positions_.push_back(Point2f(4.971542,3.4706306));
			positions_.push_back(Point2f(6.787896,6.4899794));
			positions_.push_back(Point2f(7.119874,6.627368));
			positions_.push_back(Point2f(10.8752894,5.071872));
			positions_.push_back(Point2f(11.012678,4.739894));
			break;
		case shield_generator_floor_center_intersection:
			height_ = 0;
			//no object dimensions... can we fix this?
			name_ = "shield_generator_floor_center_intersection";
			positions_.push_back(Point2f(7.991856,4.105275));
			break;
		case red_blue_black_shield_generator_floor_intersection:
			height_ = 0;
			//no object dimensions
			name_ = "red_blue_black_shield_generator_floor_intersection";
			positions_.push_back(Point2f(5.879719,4.980305));
			positions_.push_back(Point2f(10.103993,3.230245));
			break;
		case blue_black_shield_generator_floor_intersection:
			height_ = 0;
			// no object dimensions
			name_ = "blue_black_shield_generator_floor_intersection";
			positions_.push_back(Point2f(6.986143,2.36093));
			break;
		case red_black_shield_generator_floor_intersection:
			height_ = 0;
			// no object dimensions
			name_ = "red_black_shield_generator_floor_intersection";
			positions_.push_back(Point2f(8.997569,5.84962));
			break;
		case red_shield_pillar_intersection:
			height_ = 0;
			// no object dimensions
			name_ = "red_shield_pillar_intersection";
			positions_.push_back(Point2f(11.10996,4.97459));
			positions_.push_back(Point2f(6.885178,6.72465));
			break;
		case blue_shield_pillar_intersection:
			height_ = 0;
			// no object dimensions
			name_ = "blue_shield_pillar_intersection";
			positions_.push_back(Point2f(9.098026,1.4859));
			positions_.push_back(Point2f(4.87426,3.23596));
			break;
		case blue_shield_pillar_intersection:
			height_ = 0;
			// no object dimensions
			name_ = "blue_shield_pillar_intersection";
			positions_.push_back(Point2f(4.87426,3.23596));
			positions_.push_back(Point2f(9.098026,1.4859));
			break;
		case control_panel:
			height_ = 0.05;
			//height_ = .9144;
			width_ = .81;
			depth_ = .81;
			name_ = "control_panel";
			positions_.push_back(Point2f(9.26465,.70485));
			positions_.push_back(Point2f(6.7183,7.5057));
			break;
		case ds_numbers:
			//needs dimensions
			//height_ = 1.9304;
			name_ = "ds_numbers";
			positions_.push_back(Point2f(15.79245,7.220712));
			positions_.push_back(Point2f(15.98298,4.964557));
			positions_.push_back(Point2f(15.98298,3.550793));
			positions_.push_back(Point2f(15.79245,.989838));
			positions_.push_back(Point2f(.1905,7.220712));
			positions_.push_back(Point2f(0,4.65963));
			positions_.push_back(Point2f(0,3.245993));
			positions_.push_back(Point2f(.1905,.989838));
			break;
		default:
			cerr << "error initializing object!" << endl;
	}

	computeProperties();
}

ObjectType::ObjectType(const vector< Point2f > &contour_in, const string &name_in, const float &depth_in) :
	contour_(contour_in),
	depth_(depth_in),
	name_(name_in)
{
	if(contour_in.size() == 0 || name_in.length() == 0 || depth_in < 0)
		throw std::invalid_argument("bad argument to ObjectType Point2f");
	computeProperties();
}

ObjectType::ObjectType(const vector< Point > &contour_in, const string &name_in, const float &depth_in):
	depth_(depth_in),
	name_(name_in)
{
	if(contour_in.size() == 0 || name_in.length() == 0 || depth_in < 0)
		throw std::invalid_argument("bad argument to ObjectType Point");
	for (auto it = contour_in.cbegin(); it != contour_in.cend(); ++it)
		contour_.push_back(Point2f(it->x, it->y));
	computeProperties();
}

void ObjectType::computeProperties()
{
	float min_x = numeric_limits<float>::max();
	float min_y = numeric_limits<float>::max();
	float max_x = -min_x;
	float max_y = -min_y;
	for (auto it = contour_.cbegin(); it != contour_.cend(); ++it)
	{
		min_x = min(min_x, it->x);
		min_y = min(min_y, it->y);
		max_x = max(max_x, it->x);
		max_y = max(max_y, it->y);
	}
	width_ = max_x - min_x;
	height_ = max_y - min_y;
	area_ = contourArea(contour_);

	//compute moments and use them to find center of mass
	Moments mu = moments(contour_, false);
	com_ = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
}

// Given a rectangular bounding box and measured (or estimated) depth, return the world coords of the object
Point3f ObjectType::screenToWorldCoords(const Rect &screen_position, double avg_depth, const image_geometry::PinholeCameraModel &model) const
{
	// avg_depth is to front of object.  Add in half the
	// object's depth to move to the center of it
	avg_depth += depth_ / 2.;
	return ConvertCoords(model).screen_to_world(screen_position, name_, avg_depth);
}

// Given the real-world position of the object, build a rectangle bounding box in screen coords for it
Rect ObjectType::worldToScreenCoords(const Point3f &position, const image_geometry::PinholeCameraModel &model) const
{
	// TODO - handle object depth?
	const Point2f screen_center = ConvertCoords(model).world_to_screen(position, name_);

	// Object distance
	const float r = sqrtf(position.x * position.x + position.y * position.y + position.z * position.z) - depth_ / 2.;
	const Point topLeft(
			cvRound(screen_center.x - model.getDeltaU(width_  / 2.0, r)),
			cvRound(screen_center.y - model.getDeltaV(height_ / 2.0, r)));

	return Rect(topLeft.x, topLeft.y, cvRound(model.getDeltaU(width_, r)), cvRound(model.getDeltaV(height_, r)));
}

float ObjectType::expectedDepth(const Rect &screen_position, const image_geometry::PinholeCameraModel &model) const
{
	// du = fx * dX / Z ==> Z = fx * dX / du
	// where du = size in screen coords
	//       dx = size in real life
	//       fx = focal length in X
	//       Z  = depth
	if (screen_position.width > screen_position.height) // use larger the values for better resolution
		return model.fx() * width_ / screen_position.width;
	return model.fy() * height_ / screen_position.height;
}

bool ObjectType::operator== (const ObjectType &t1) const
{
	return this->shape() == t1.shape();
}
