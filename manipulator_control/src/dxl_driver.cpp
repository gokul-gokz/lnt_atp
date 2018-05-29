#include "../include/manipulator_control/dxl_driver.h"


 
void dxl_driver::sleep(unsigned int mseconds)
{
    clock_t goal = mseconds + clock();
    while (goal > clock());
}

	
dxl_driver::dxl_driver(ros::NodeHandle &nh):nh_(nh){
	if(nh_.getParam("/manipulator/baudrate",baudrate_)){}
	else ROS_INFO("/manipulator/baudrate parameter cou#include <inttypes.h>ld not be loaded ");
	
	if(nh_.getParam("/manipulator/num_of_joints",num_of_joints_)){}
	else ROS_INFO("/manipulator/num_of_joints parameter could not be loaded ");

	if(nh_.getParam("/manipulator/devicename",devicename_)){}
	else ROS_INFO("/manipulator/devicename parameter could not be loaded ");

	if(nh_.getParam("/manipulator/joint_ids",joint_ids_)){}
	else ROS_INFO("/manipulator/joint_ids parameter could not be loaded ");

	if(nh_.getParam("/manipulator/joint_nums",joint_nums_)){}
	else ROS_INFO("/manipulator/joint_nums parameter could not be loaded ");
	if(nh_.getParam("/manipulator/joint_speed",joint_speed_)){}
	else ROS_INFO("/manipulator/joint_speed parameter could not be loaded ");

	if(nh_.getParam("/manipulator/joint_pos_min",joint_pos_min_)){}
	else ROS_INFO("/manipulator/joint_pos_min parameter could not be loaded ");

	if(nh_.getParam("/manipulator/joint_pos_max",joint_pos_max_)){}
	else ROS_INFO("/manipulator/joint_pos_max parameter could not be loaded ");

	if(nh_.getParam("/manipulator/joint_pos_init",joint_pos_init_)){}
	else ROS_INFO("/manipulator/joint_pos_init parameter could not be loaded "); 

	dxl_addparam_result = false;
	dxl_getdata_result = false;
	dxl_comm_result=COMM_TX_FAIL;
}

dxl_driver::~dxl_driver(){
	disableTorque(); 
	closePort();

}



void dxl_driver::init(){
	
	portHandler = dynamixel::PortHandler::getPortHandler(devicename_.c_str());
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	groupBulkWrite = new dynamixel::GroupBulkWrite(portHandler, packetHandler);
	groupBulkRead = new dynamixel::GroupBulkRead(portHandler, packetHandler);
	
	openPort();
	setBaudRate();
	enableTorque();
	//sleep(5000000);
	add_read_param();
    


}

int dxl_driver::getch(){
	#if defined(__linux__) || defined(__APPLE__)
		struct termios oldt, newt;
		int ch;
		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);
		ch = getchar();
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		return ch;
	#elif defined(_WIN32) || defined(_WIN64)
		return _getch();
	#endif
}

int dxl_driver::kbhit(void){
	#if defined(__linux__) || defined(__APPLE__)
		struct termios oldt, newt;
		int ch;
		int oldf;

		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);
		oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
		fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

		ch = getchar();

		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		fcntl(STDIN_FILENO, F_SETFL, oldf);

		if (ch != EOF)
		{
		ungetc(ch, stdin);
		return 1;
		}

	 	return 0;
	#elif defined(_WIN32) || defined(_WIN64)
  		return _kbhit();
	#endif

}

void dxl_driver::printmap(std::map<std::string,int> &m){
	for(std::map<std::string, int>::const_iterator it = m.begin();	it != m.end(); ++it)
	{
	std::cout << it->first << " " << it->second<<"\n";
	}
}

void dxl_driver::printvector(std::vector<double> &vec){
	for (int i = 0; i < vec.size(); ++i){
		printf("%G ",vec[i]);
	}
}


void dxl_driver::printvariables(){
	ROS_INFO("Baudrate is %d",baudrate_);
	ROS_INFO("Number of joints is %d",num_of_joints_);
	ROS_INFO("Device name %s",devicename_.c_str());
	ROS_INFO("Joint IDs");printmap(joint_ids_);
	ROS_INFO("Joint numbers");printmap(joint_nums_);
	ROS_INFO("Joint speeds");printmap(joint_speed_);
	ROS_INFO("Joint position min limit");printmap(joint_pos_min_);
	ROS_INFO("Joint position max limit");printmap(joint_pos_max_);
	ROS_INFO("Joint position init values");printmap(joint_pos_init_);
}



void dxl_driver::read(std::vector<double> &joint_position,std::vector<double> &joint_velocity,std::vector<double> &joint_effort){
	
	bool groupBulkReadSucess=false;
	while(!groupBulkReadSucess){
	
	dxl_comm_result = groupBulkRead->txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS) {
		
		printf("groupBulkRead : %s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}
	
	else{ 
		groupBulkReadSucess=true;
		printf("\n groupBulkRead is success \n");}
		//sleep(50000);
	}
	
	for(std::map<std::string, int>::const_iterator it = joint_nums_.begin();it != joint_nums_.end(); ++it)
	{
		dxl_getdata_result=false;
		int32_t present_pos;
		
		//std::cout << it->first << " " << it->second<<"\n";
		
		if(it->first=="left_grip_joint"){//These are MX64 2.0 series motors ||it->first=="right_grip_joint"
		dxl_getdata_result = groupBulkRead->isAvailable(joint_ids_[it->first], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION );}
		
		else if (it->first=="right_grip_joint"){//do nothing
			dxl_getdata_result=true; //otherwise triggers " groupBulkRead getdata failed " error
			}
		
		else{//These are Pro series motors
			dxl_getdata_result = groupBulkRead->isAvailable(joint_ids_[it->first], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);}
		
		if (dxl_getdata_result != true){//data is not available
			fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed : %s   ",joint_ids_[it->first],it->first.c_str());}
			
			
			
		else{//data is available
			index=(it->second)-1;
			if (it->second<4){//These are Pro H54 series motor
				 
				present_pos=groupBulkRead->getData(joint_ids_[it->first], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
				//printf("Present Position %s : %G \t ", it->first.c_str(),static_cast<double>(present_pos)*(double)(1.0*0.00001251187));
				joint_position[index] = static_cast<double>(present_pos)*(double)(1.0*0.00001251187);}
			
			else if(it->second<7){//These are Pro H42 series motor
				present_pos=groupBulkRead->getData(joint_ids_[it->first], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
				//printf("Present Position %s : %G \t ", it->first.c_str(),static_cast<double>(present_pos)*(double)(1.0*0.00002067489));
				joint_position[index] = static_cast<double>(present_pos)*(double)(1.0*0.00002067489);}
				
			else{//These are MX64 2.0 series motors
				present_pos=groupBulkRead->getData(joint_ids_[it->first], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
				//printf("Present Position %s : %G \t ", it->first.c_str(),static_cast<double>(present_pos)*(double)(1.0*0.00002067489));
				joint_position[index] = static_cast<double>(present_pos)*(double)(1.0*0.001533);
				}
			}
			
	}
	
}

void dxl_driver::write(std::vector<double> &joint_position_command,std::vector<double> &joint_velocity_command,std::vector<double> &joint_effort_command){
	
	for(std::map<std::string, int>::const_iterator it = joint_nums_.begin();it != joint_nums_.end(); ++it)
	{
		//std::cout << it->first << " " << it->second<<"\n";
		index=(it->second)-1;//index in the vector is (joint_num-1)
		printf("%G \n",joint_position_command[index]);
		
		if(it->second<4){//These are Pro H54 series motor
			dxl_goal_position=(joint_position_command[index])*(double)(250961.5/3.14);
			//printf("%i \t", dxl_goal_position);static_cast<int>
			//dxl_goal_position=joint_position_command[index]*(250961.5/3.14);
			}
		else if (it->second<7){//These are Pro H42 series motor
			dxl_goal_position=(joint_position_command[index])*(double)(151875/3.14);
			//printf("%i \t", dxl_goal_position);static_cast<int>
			//dxl_goal_position=joint_position_command[index]*(151875/3.14);
			}
		else{//These are MX64 2.0 series motors
			dxl_goal_position=(joint_position_command[index])*(double)(4095/3.14);
			//printf("%i \t", dxl_goal_position);static_cast<int>
			}

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));
		
		//printing some debug info
		printf("%i \t",dxl_goal_position);
		for(int i=0;i<4;i++){
		printf("%u ",param_goal_position[i]);}
		printf("\n");
		
		
		
		if(it->first=="right_grip_joint"){//do nothing
			}
		
		else if(it->first=="left_grip_joint"){//These are MX64 2.0 series motors ||it->first=="left_grip_joint"
			
			 dxl_addparam_result = groupBulkWrite->addParam(joint_ids_[it->first], ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION, param_goal_position);
		    if (dxl_addparam_result != true){
		      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", joint_ids_[it->first]);}
				
			}
		else{//These are Pro series motor
			
			dxl_addparam_result = groupBulkWrite->addParam(joint_ids_[it->first], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
		    if (dxl_addparam_result != true){
		      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", joint_ids_[it->first]);}
						
			}
		
		 
	}
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS){
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));}
	
	if (dxl_comm_result == COMM_SUCCESS){
		printf("groupBulkWrite is success ! ");
		}
	
	//ROS_INFO("joint_position_command");
	//printvector(joint_position_command);
	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();

}	




void dxl_driver::openPort(){
	if (portHandler->openPort()){
		ROS_INFO("Succeeded to open the port!\n");
		

		}
	else{
		ROS_INFO("Failed to open the port!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		
		}
}

void dxl_driver::setBaudRate(){
	if (portHandler->setBaudRate(baudrate_)){
		ROS_INFO("Succeeded to change the baudrate!\n");
		

		}
	else{
		ROS_INFO("Failed to change the baudrate!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		
		}
}


void dxl_driver::add_read_param(){
	for(std::map<std::string, int>::const_iterator it = joint_ids_.begin();it != joint_ids_.end(); ++it)
	{
		//printf("Adding param for %i \n",it->second);
		//std::cout << it->first << " " << it->second<<"\n";
		dxl_addparam_result=false;
		if(it->first=="right_grip_joint"){//do nothing
			}
		
		else if(it->first=="left_grip_joint"){//These are MX 64 2.0 series motors   || it->first=="left_grip_joint"
			
			 while(dxl_addparam_result==false){
		dxl_addparam_result = groupBulkRead->addParam(it->second, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		  if (dxl_addparam_result != true){
		    fprintf(stderr, "[ID:%03d] : %s grouBulkRead addparam failed \n", it->second,it->first.c_str());}
		   }
	   }
	   
	   else{//These are PRO series motors
		  while(dxl_addparam_result==false){
		dxl_addparam_result = groupBulkRead->addParam(it->second, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
		  if (dxl_addparam_result != true){
		    fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed %s \n",it->second,it->first.c_str());}
		   }
		   }
	}
	//printf("Addparam successful \n");
	//sleep(2000000);
}



void dxl_driver::enableTorque(){
		
	// Enable Dynamixel Torque
	bool result;
	bool grip_encountered=false;
	for(std::map<std::string, int>::const_iterator it = joint_ids_.begin();it != joint_ids_.end(); ++it){
		result=false;
		
		if(it->first=="right_grip_joint"){//do nothing
			}
		
		
		else if(it->first=="left_grip_joint"){//these are MX  serires motors ||it->first=="left_grip_joint"
			
			while(!result){ //This is MX64 2.0 series motor
					
						dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, it->second, ADDR_MX_TORQUE_ENABLE , TORQUE_ENABLE, &dxl_error);
						if (dxl_comm_result != COMM_SUCCESS){
							printf("Torque enable %s : %s\n", it->first.c_str(),packetHandler->getTxRxResult(dxl_comm_result));
							result=false;
							}

						else if (dxl_error != 0){
							printf("Torque enable %s : %s \n", it->first.c_str(),packetHandler->getRxPacketError(dxl_error));
							result=false;
							}

						else{
							ROS_INFO("Dynamixel ID : %d / %s has been successfully connected \n",it->second,it->first.c_str());
							result=true;
							//sleep(500000);
							}
						}
					
			
			}
		else{
			while(!result){ //These are PRO series motors
			
				dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, it->second, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS){
					printf("Torque enable %s : %s \n", it->first.c_str(),packetHandler->getTxRxResult(dxl_comm_result));
					result=false;
					}

				else if (dxl_error != 0){
					printf("Torque enable %s : %s \n",it->first.c_str(), packetHandler->getRxPacketError(dxl_error));
					result=false;
					}

				else{
					ROS_INFO("Dynamixel ID : %d / %s has been successfully connected \n",it->second,it->first.c_str());
					result=true;
					//sleep(500000);
					}
				}
			
		}
	}


}

void dxl_driver::disableTorque(){
		
	// Disable Dynamixel Torque
	
	for(std::map<std::string, int>::const_iterator it = joint_ids_.begin();it != joint_ids_.end(); ++it){
		
		if(it->first=="left_grip_joint"){//do nothing
			}
		
		if(it->first=="right_grip_joint"){//MX64 motor
	
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, it->second, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS){
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}

			else if (dxl_error != 0){
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
				}

			else{		
			ROS_INFO("Dynamixel ID : %d / %s has been dis-connected \n",it->second,it->first.c_str());
			}
			}
		else{//Pro series motor
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, it->second, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS){
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}

			else if (dxl_error != 0){
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
				}

			else{		
			ROS_INFO("Dynamixel ID : %d / %s has been dis-connected \n",it->second,it->first.c_str());
			}
			
		}
	}


}


void dxl_driver::closePort(){
	portHandler->closePort();
}


