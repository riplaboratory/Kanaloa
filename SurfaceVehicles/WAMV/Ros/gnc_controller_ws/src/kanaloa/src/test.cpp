#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>

#include "yaml-cpp/yaml.h"



// struct Thruster
// {
//   // Thruster Variables
//   std::string name, topic;
//   int position[3], orientation[3];
//   float current_thrust;
//
//
//   // Constructor + Destructor
//   Thruster(ros::NodeHandle *nh, string yaml_file)
//   {
//
//   }
//   ~Thruster(){}
//
//   // Thruster Setup Methods
//   void init_yaml_config(string yaml_file)
//   {
//     YAML::Node config = YAML::LoadFile(yaml_file);
//
//   }
//
//   void set_thrust_sub_cb()
//   {}
//
// }



int main(int argc, char **argv)
{
  std::cout << "Working" << std::endl;

  // YAML::Node config;

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  //
  std::string thruster_param;
  // std::map<std::string, std::string> thruster_param_map;
  // std::vector<std::string> thruster_param_list;
  // // std::vector<std::map<std::string, std::string>> thruster_param_list_2;
  //
  //
  //
  if (nh.getParam("thruster_config_yaml", thruster_param))
  {
    std::cout << thruster_param << std::endl;
    YAML::Node config = YAML::LoadFile(thruster_param);
    std::cout << config << std::endl;
    std::cout << config["thrusters"].size() << std::endl;

    std::cout << typeid(config["thrusters"]).name() << std::endl;
    std::cout << config["thrusters"] << std::endl;

    YAML::Node test_yaml_list = config["thrusters"];

    std::cout << test_yaml_list << std::endl;

    for (int i=0; i < config["thrusters"].size(); i++)
    {

      // std::vector<float> test_vec {config["thrusters"][i]["orientation"]};


      // std::cout << test_vec << std::endl;


      std::cout << config["thrusters"][i]["name"] << std::endl;
      std::cout << config["thrusters"][i]["position"] << std::endl;
      std::cout << config["thrusters"][i]["orientation"] << std::endl;
      std::cout << config["thrusters"][i]["topic"] << std::endl;

      // std::cout << config["thrusters"][i]["orientation"].size() << std::endl; // 0

      // std::vector<float> myNumbers;


      std::string orientation =  config["thrusters"][i]["orientation"].as<std::string>();
      std::cout << orientation << std::endl;

      // std::istringstream is( orientation );
      // std::vector<int> myNumbers( std::istream_iterator<int>( is ), std::istream_iterator<int>() );

      // std::string myString = "10 15 20 23";
      std::stringstream iss( orientation );


      float number;
      std::vector<float> myNumbers;
      while ( iss >> number )
        myNumbers.push_back( number );


      std::stringstream iss2( orientation );
      float num_array[3];
      // for (int j=0; j<3; j++)
      // {
      int j = 0;
      while ( iss2 >> number )
      {
        std::cout << j << " " << number << " --" <<std::endl;
        num_array[j] = number;
        j++;
      }

        // iss2 >> number;
        // num_array[j]=number;
      // }

      // std::cout << myNumbers.at(0) << std::endl;
      // std::cout << myNumbers.at(1) << std::endl;
      // std::cout << myNumbers.at(2) << std::endl;
      //
      // std::cout << "---" << std::endl;

      std::cout << num_array[0] << std::endl;
      std::cout << num_array[1] << std::endl;
      std::cout << num_array[2] << std::endl;


      // YAML::Node primes = YAML::Load(orientation);
      // for (std::size_t j=0;j<primes.size();j++) {
      //   std::cout << primes[j].as<float>() << " ...\n";
      // }


      // std::istringstream is( orientation );
      // std::vector<float> myNumbers( std::istream_iterator<float>( is ), std::istream_iterator<float>());


      // for (int j=0; j < myNumbers.size(); j++)
      // {
      //   std::cout << myNumbers[j] << std::endl;
      // }

      // std::string orientation =  config["thrusters"][i]["orientation"].as<std::string>();;
      //
      // std::stringstream ss(orientation);
      // std::istream_iterator<std::string> begin(ss);
      // std::istream_iterator<std::string> end;
      // std::vector<std::string> vstrings(begin, end);
      // std::copy(vstrings.begin(), vstrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));


    }

  }
  else
  {
    std::cout << "Error" << std::endl;
  }

  ros::spin();



  return 0;
}
