#include <iostream>
#include <fstream>
#include <iostream>
#include "Pose.h"
using namespace std;

class Logger{
    public: 
        string log_file;
        Logger(string filename){
            this->log_file = filename;
        }
        void log(string object_type, Pose location);
};

void Logger::log(string object_type, Pose location){
    ofstream file;
    file.open(this->log_file);
    file << object_type << "Data: " << location.toString();
    std::cout << object_type << "Data: " << location.toString();
    file.close();
}