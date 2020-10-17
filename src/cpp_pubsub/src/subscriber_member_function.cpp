// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <regex>
#include <math.h>
#include <map> 

using namespace std;
//#include "Auxillary.h"

class Auxillary{
  map<int, std::string> toGate;


  public:
    Auxillary(){
        for(int box=1;box<10;box++){
            int j = 2 - ceil(box/3.0);
            int i = 2 - box - (j-1)*3;
            std::string directions = "";

            (i<0)? directions.insert(0, abs(i), 'L') : directions.insert(0, abs(i), 'R');

            (j>0)? directions.insert(0, abs(j), 'D') : directions.insert(0, abs(j), 'U');

            toGate.insert( { box, directions} );
        }
    }

    int checkRegex(string input) const{
        std::regex rx("The gate is in square number ([1-9])"); // Declare the regex with a raw string literal
        std::smatch m;
        int out = 0;
        while (regex_search(input, m, rx)) {
            out = stoi(m[1]);
            //std::cout << "Number found: " << out << std::endl; // Get Captured Group 1 text
            input = m.suffix().str(); // Proceed to the next match
        } 
        return out;      
    }

    std::string getPath(int gateBox) const{
        //cout << toGate.find(gateBox)->second << endl;
        return toGate.find(gateBox)->second;
    }

};


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public: Auxillary* AUX = new Auxillary();

public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    std::string gotGate = msg->data.c_str();
    int g = AUX->checkRegex(gotGate);
    if (g>0 && g<10){
      gotGate = AUX->getPath(g);
      RCLCPP_INFO(this->get_logger(), "Subscribed %s", gotGate.c_str());

      auto message = std_msgs::msg::String();
      message.data = gotGate;
      publisher_->publish(message);
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  //AUX.initiate();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
