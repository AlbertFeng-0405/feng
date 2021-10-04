#include <iostream>
#include <cstdlib>

int main()
{
    srand(8);
    float a = (float)40*rand()/float(RAND_MAX)-20;
    float b = (float)40*rand()/float(RAND_MAX)-20;
    std::cout<<a<<std::endl;
    std::cout<<b<<std::endl;
    
}