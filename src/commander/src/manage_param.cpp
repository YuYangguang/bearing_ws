
/*****************************************************************************************
// Author:      Chang Yuan
// Date:        2017.06.09
// Description: Manage parameter related works
*****************************************************************************************/

#include <commander/commander.h>

using namespace smarteye;


// Summary: read param form host_param.txt and send to the parameter server. 
//          note that the param file is located in install folder
void commander::init_parameter_server()
{
    std::fstream f;
    f.open("../../include/commander/commander/host_param.txt",std::ios::in);

    if(f.is_open())
    {
    	std::string name;
    	float value;
    	int now = f.tellg();
    	f.seekg (0, f.end);
    	int length = f.tellg();
    	f.seekg (0, f.beg);
    	while((now>=0) && (now<length-2))
    	{
    		f>>name>>value;
    		paramMap.insert(std::pair<std::string, float>(name.c_str(),value));
    		ros::param::set(name, value);
    		now = f.tellg();
    	}
    	ROS_INFO_STREAM("there are "<<paramMap.size()<<" parameters in the parameter file");
    	f.close();
    }
    else
    {
        ROS_ERROR("open parameter file failed");
    }
}

// Summary: check which node the parameter belongs to, according to the first 3 letters of the parameter name.
// Parameters: parameter name
// Return: which node the parameter belongs to
int commander::checkHParamBelongs(std::string name)
{
    std::string param_head  = name.substr(0,3);
    ROS_INFO("the parameter belongs to: %s", param_head.c_str()); 
    if(param_head == "CTL")        // control node
        return CONTROL_NODE; 
    else if(param_head == "NAV")   // formation node: navigation layer
        return FORMATION_NODE;
    else if(param_head == "COP")   // formation node: corporation layer
        return FORMATION_NODE; 
    else if(param_head == "FC_")
    	return FORMATION_NODE;
    else if(param_head == "CMD")
    	return COMMANDER_NODE;
    else if(param_head == "TAR")
    	return TRACKING_NODE;
    else
        {
            ROS_WARN("no parameter listed start with %s", param_head.c_str()); 
            return 0;
        }
}

// Summary: change parameter value in the paramMap, and in file as well
// Parameters: parameter name , new parameter value
// Return: 0:success 1:fail
int commander::changeHParam(std::string name, float value)
{
    //Zeroun
    if(name == "CMD_CLOSE_FLT")
    {
        return 0;
    }
    try{
    	paramMap.at(name) = value;
    }
    catch (const std::out_of_range& oor) {
    	ROS_ERROR("there is no such parameter!");
    	return 1;
    }
	
    // write host_param.txt in src folder
    std::fstream f;
    f.open("../../../src/smarteye/commander/include/commander/host_param.txt",std::ios::out|std::ios::in);
    if(f.is_open())
    {
        for(auto it = paramMap.begin(); it != paramMap.end(); it++)
        {
    		f<<it->first<<std::endl<<it->second<<std::endl;
    	}
    	f.close();
    }
    else
    {
        ROS_WARN("open /src parameter file failed");
    }

    // write host_param.txt in install folder
    f.open("../../include/commander/commander/host_param.txt",std::ios::out|std::ios::trunc);
    if(f.is_open())
    {
        for(auto it = paramMap.begin(); it != paramMap.end(); it++)
        {
    		f<<it->first<<std::endl<<it->second<<std::endl;
    	}
    	f.close();
    }
    else
    {
        ROS_ERROR("open /install parameter file failed");
    }

    return 0;
}
